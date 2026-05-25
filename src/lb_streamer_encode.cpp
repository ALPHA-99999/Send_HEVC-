#include "lb_streamer.h"
#include "lb_streamer_utils.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>

bool LowBandwidthStreamer::encodeFrame(const cv::Mat& frame, uint32_t frameIndex) {
    if (!m_codecContext || !m_packet) {
        return false;
    }

    // 把 BGR 画面转成编码器需要的像素格式，再送入 FFmpeg。
    AVFrame* submitFrame = nullptr;
    if (m_useHardwareEncoder) {
        if (!m_encodeSwFrame || !m_encodeHwFrame || !m_encodeHwFramesCtx) {
            return false;
        }

        int ret = av_frame_make_writable(m_encodeSwFrame);
        if (ret < 0) {
            logAvError("Frame not writable", ret);
            return false;
        }

        uint8_t* srcSlice[] = {const_cast<uint8_t*>(frame.data)};
        int srcStride[] = {static_cast<int>(frame.step)};
        sws_scale(m_swsContext,
                  srcSlice,
                  srcStride,
                  0,
                  frame.rows,
                  m_encodeSwFrame->data,
                  m_encodeSwFrame->linesize);

        av_frame_unref(m_encodeHwFrame);
        ret = av_hwframe_get_buffer(m_encodeHwFramesCtx, m_encodeHwFrame, 0);
        if (ret < 0) {
            logAvError("Could not get VAAPI frame buffer", ret);
            return false;
        }

        ret = av_hwframe_transfer_data(m_encodeHwFrame, m_encodeSwFrame, 0);
        if (ret < 0) {
            logAvError("Could not transfer frame to VAAPI", ret);
            return false;
        }

        m_encodeHwFrame->pts = frameIndex;
        submitFrame = m_encodeHwFrame;
    } else {
        if (!m_yuvFrame) {
            return false;
        }

        int ret = av_frame_make_writable(m_yuvFrame);
        if (ret < 0) {
            logAvError("Frame not writable", ret);
            return false;
        }

        uint8_t* srcSlice[] = {const_cast<uint8_t*>(frame.data)};
        int srcStride[] = {static_cast<int>(frame.step)};
        sws_scale(m_swsContext,
                  srcSlice,
                  srcStride,
                  0,
                  frame.rows,
                  m_yuvFrame->data,
                  m_yuvFrame->linesize);

        m_yuvFrame->pts = frameIndex;
        submitFrame = m_yuvFrame;
    }

    const auto encodeStart = std::chrono::steady_clock::now();
    int ret = avcodec_send_frame(m_codecContext, submitFrame);
    if (ret < 0) {
        logAvError("Error sending frame to encoder", ret);
        return false;
    }

    while (ret >= 0) {
        ret = avcodec_receive_packet(m_codecContext, m_packet);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            break;
        }
        if (ret < 0) {
            logAvError("Error during encoding", ret);
            return false;
        }

        const bool keyframe = (m_packet->flags & AV_PKT_FLAG_KEY) != 0;
        if (!sendEncodedPayload(m_packet->data, m_packet->size, keyframe)) {
            av_packet_unref(m_packet);
            return false;
        }
        av_packet_unref(m_packet);
    }

    const auto encodeEnd = std::chrono::steady_clock::now();
    const auto encodeMicros = std::chrono::duration_cast<std::chrono::microseconds>(encodeEnd - encodeStart).count();
    if (encodeMicros > 0) {
        m_totalEncodeMicros.fetch_add(static_cast<uint64_t>(encodeMicros));
    }

    return true;
}

bool LowBandwidthStreamer::flushEncoder() {
    if (!m_codecContext || !m_packet) {
        return true;
    }

    // 结束时把编码器内部还没吐完的包冲出来。
    int ret = avcodec_send_frame(m_codecContext, nullptr);
    if (ret < 0 && ret != AVERROR_EOF) {
        logAvError("Error flushing encoder", ret);
        return false;
    }

    while (true) {
        ret = avcodec_receive_packet(m_codecContext, m_packet);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            break;
        }
        if (ret < 0) {
            logAvError("Error receiving flushed packet", ret);
            return false;
        }

        const bool keyframe = (m_packet->flags & AV_PKT_FLAG_KEY) != 0;
        if (!sendEncodedPayload(m_packet->data, m_packet->size, keyframe)) {
            av_packet_unref(m_packet);
            return false;
        }
        av_packet_unref(m_packet);
    }

    return true;
}

bool LowBandwidthStreamer::sendEncodedPayload(const uint8_t* data, int size, bool keyframe) {
    if (!data || size <= 0) {
        return true;
    }

    EncodedChunk chunk;
    chunk.data.assign(data, data + size);
    chunk.keyframe = keyframe;
    m_encodedChunks.push_back(std::move(chunk));

    return flushPendingWirePackets(false);
}

bool LowBandwidthStreamer::flushPendingWirePackets(bool finalFlush) {
    const std::size_t wirePacketSize = static_cast<std::size_t>(std::max(1, m_transportConfig.wirePacketSize));
    if (wirePacketSize < lb_streamer_detail::kStreamHeaderSize) {
        std::cerr << "wirePacketSize is smaller than the stream header" << std::endl;
        return false;
    }

    const std::size_t payloadCapacity = wirePacketSize - lb_streamer_detail::kStreamHeaderSize;
    uint64_t bytesSentForFlush = 0;
    uint64_t packetsSentForFlush = 0;

    while (true) {
        std::size_t bufferedBytes = 0;
        bool firstChunk = true;
        for (const auto& chunk : m_encodedChunks) {
            if (firstChunk) {
                const std::size_t consumed = std::min(m_encodedChunkOffset, chunk.data.size());
                bufferedBytes += chunk.data.size() - consumed;
                firstChunk = false;
            } else {
                bufferedBytes += chunk.data.size();
            }
        }
        if (!finalFlush && bufferedBytes < payloadCapacity) {
            break;
        }
        if (bufferedBytes == 0 && !finalFlush) {
            break;
        }

        const std::size_t bytesToCopy = finalFlush
            ? std::min(payloadCapacity, bufferedBytes)
            : payloadCapacity;

        std::vector<uint8_t> payload(bytesToCopy);
        std::size_t copied = 0;
        bool syncPacket = false;
        uint16_t syncOffset = 0xFFFF;

        while (copied < bytesToCopy && !m_encodedChunks.empty()) {
            EncodedChunk& chunk = m_encodedChunks.front();
            if (m_encodedChunkOffset >= chunk.data.size()) {
                m_encodedChunks.pop_front();
                m_encodedChunkOffset = 0;
                continue;
            }

            const std::size_t chunkOffset = m_encodedChunkOffset;
            const std::size_t chunkRemaining = chunk.data.size() - chunkOffset;
            const std::size_t toCopy = std::min(bytesToCopy - copied, chunkRemaining);
            if (chunk.keyframe && chunkOffset == 0 && !syncPacket) {
                syncPacket = true;
                syncOffset = static_cast<uint16_t>(copied);
            }

            std::memcpy(payload.data() + copied,
                        chunk.data.data() + chunkOffset,
                        toCopy);
            copied += toCopy;
            m_encodedChunkOffset += toCopy;

            if (m_encodedChunkOffset >= chunk.data.size()) {
                m_encodedChunks.pop_front();
                m_encodedChunkOffset = 0;
            }
        }

        if (copied == 0) {
            if (!finalFlush) {
                break;
            }
        }

        uint8_t flags = 0;
        if (syncPacket) {
            flags |= lb_streamer_detail::kStreamFlagKeyframe;
        }
        if (finalFlush && m_encodedChunks.empty()) {
            flags |= lb_streamer_detail::kStreamFlagEndOfStream;
        }

        if (!sendWirePacket(payload.data(), copied, flags, syncOffset)) {
            return false;
        }

        bytesSentForFlush += wirePacketSize;
        ++packetsSentForFlush;

        if (!finalFlush && copied < payloadCapacity) {
            break;
        }

        if (finalFlush && m_encodedChunks.empty()) {
            break;
        }
    }

    if (finalFlush && bytesSentForFlush == 0) {
        if (!sendWirePacket(nullptr, 0, lb_streamer_detail::kStreamFlagEndOfStream, 0xFFFF)) {
            return false;
        }
        bytesSentForFlush += wirePacketSize;
        ++packetsSentForFlush;
    }

    m_totalBytesSent.fetch_add(bytesSentForFlush);
    m_totalPacketsSent.fetch_add(packetsSentForFlush);
    return true;
}

bool LowBandwidthStreamer::sendWirePacket(const uint8_t* payload, size_t payloadLen, uint8_t flags, uint16_t syncOffset) {
    const std::size_t wirePacketSize = static_cast<std::size_t>(std::max(1, m_transportConfig.wirePacketSize));
    if (wirePacketSize < lb_streamer_detail::kStreamHeaderSize) {
        return false;
    }

    std::array<uint8_t, lb_streamer_detail::kWirePacketSize> packet{};
    if (wirePacketSize != packet.size()) {
        std::cerr << "wirePacketSize must be " << packet.size() << " bytes" << std::endl;
        return false;
    }

    packet[0] = static_cast<uint8_t>((lb_streamer_detail::kStreamMagic >> 24) & 0xFF);
    packet[1] = static_cast<uint8_t>((lb_streamer_detail::kStreamMagic >> 16) & 0xFF);
    packet[2] = static_cast<uint8_t>((lb_streamer_detail::kStreamMagic >> 8) & 0xFF);
    packet[3] = static_cast<uint8_t>(lb_streamer_detail::kStreamMagic & 0xFF);
    packet[4] = lb_streamer_detail::kStreamVersion;
    packet[5] = flags;
    packet[6] = static_cast<uint8_t>((lb_streamer_detail::kStreamHeaderSize >> 8) & 0xFF);
    packet[7] = static_cast<uint8_t>(lb_streamer_detail::kStreamHeaderSize & 0xFF);
    packet[8] = static_cast<uint8_t>((m_nextPacketSeq >> 24) & 0xFF);
    packet[9] = static_cast<uint8_t>((m_nextPacketSeq >> 16) & 0xFF);
    packet[10] = static_cast<uint8_t>((m_nextPacketSeq >> 8) & 0xFF);
    packet[11] = static_cast<uint8_t>(m_nextPacketSeq & 0xFF);
    packet[12] = static_cast<uint8_t>((payloadLen >> 8) & 0xFF);
    packet[13] = static_cast<uint8_t>(payloadLen & 0xFF);
    packet[14] = static_cast<uint8_t>((syncOffset >> 8) & 0xFF);
    packet[15] = static_cast<uint8_t>(syncOffset & 0xFF);

    if (payloadLen > 0 && payload != nullptr) {
        std::memcpy(packet.data() + lb_streamer_detail::kStreamHeaderSize, payload, payloadLen);
    }

    if (payloadLen < lb_streamer_detail::kWirePacketSize - lb_streamer_detail::kStreamHeaderSize) {
        std::memset(packet.data() + lb_streamer_detail::kStreamHeaderSize + payloadLen,
                    0,
                    lb_streamer_detail::kWirePacketSize - lb_streamer_detail::kStreamHeaderSize - payloadLen);
    }

    using clock = std::chrono::steady_clock;
    static clock::time_point s_nextAllowedPacketSendTime = clock::now();
    const bool throttlePackets = m_transportConfig.maxSendHz > 0;
    const int maxPacketHz = std::max(1, m_transportConfig.maxSendHz);
    const auto minPacketInterval = std::chrono::microseconds(1000000 / maxPacketHz);
    if (throttlePackets) {
        const auto now = clock::now();
        if (now < s_nextAllowedPacketSendTime) {
            std::this_thread::sleep_until(s_nextAllowedPacketSendTime);
        }
        s_nextAllowedPacketSendTime = clock::now() + minPacketInterval;
    }

#if LOW_BANDWIDTH_TRANSPORT_MODE == LOW_BANDWIDTH_TRANSPORT_SERIAL
    // 串口链路要节流，避免一下子把链路打满。
    if (!lb_streamer_detail::writeSerialAll(m_serialHandle, packet.data(), packet.size())) {
        return false;
    }
    if (m_transportConfig.serialInterPacketDelayMs > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(m_transportConfig.serialInterPacketDelayMs));
    }
#else
    const int sent = sendto(m_udpSocket,
                            reinterpret_cast<const char*>(packet.data()),
                            static_cast<int>(packet.size()),
                            0,
                            reinterpret_cast<sockaddr*>(&m_serverAddr),
                            sizeof(m_serverAddr));
    if (sent < 0) {
        std::cerr << "sendto failed: " << std::strerror(errno) << std::endl;
        return false;
    }
#endif

    ++m_nextPacketSeq;
    return true;
}

bool LowBandwidthStreamer::displayDecodedPacket(const AVPacket* packet) {
    return true;
}

void LowBandwidthStreamer::logAvError(const std::string& prefix, int errnum) const {
    char buffer[AV_ERROR_MAX_STRING_SIZE] = {};
    av_strerror(errnum, buffer, sizeof(buffer));
    std::cerr << prefix << ": " << buffer << std::endl;
}
