#include "lb_streamer.h"
#include "lb_streamer_utils.h"

#include <algorithm>
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

        if (!sendEncodedPayload(m_packet->data, m_packet->size, frameIndex)) {
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

        const uint32_t frameIndex = m_outputFrameCounter.load();
        if (!sendEncodedPayload(m_packet->data, m_packet->size, frameIndex)) {
            av_packet_unref(m_packet);
            return false;
        }
        av_packet_unref(m_packet);
    }

    return true;
}

bool LowBandwidthStreamer::sendEncodedPayload(const uint8_t* data, int size, uint32_t frameIndex) {
    if (!data || size <= 0) {
        return true;
    }

    // 按统一包头拆包，接收端可以用 frameIndex + packetIndex 重组。
#if LOW_BANDWIDTH_TRANSPORT_MODE == LOW_BANDWIDTH_TRANSPORT_SERIAL
    using clock = std::chrono::steady_clock;
    static clock::time_point s_nextAllowedPacketSendTime = clock::now();
    const bool throttlePackets = m_transportConfig.maxSendHz > 0;
    const int maxPacketHz = std::max(1, m_transportConfig.maxSendHz);
    const auto minPacketInterval = std::chrono::microseconds(1000000 / maxPacketHz);

    auto waitForPacketSlot = [&]() {
        if (!throttlePackets) {
            return;
        }
        const auto now = clock::now();
        if (now < s_nextAllowedPacketSendTime) {
            std::this_thread::sleep_until(s_nextAllowedPacketSendTime);
        }
        s_nextAllowedPacketSendTime = clock::now() + minPacketInterval;
    };
#endif

#if LOW_BANDWIDTH_TRANSPORT_MODE == LOW_BANDWIDTH_TRANSPORT_UDP
    using clock = std::chrono::steady_clock;
    static clock::time_point s_nextAllowedPacketSendTime = clock::now();
    const bool throttlePackets = m_transportConfig.maxSendHz > 0;
    const int maxPacketHz = std::max(1, m_transportConfig.maxSendHz);
    const auto minPacketInterval = std::chrono::microseconds(1000000 / maxPacketHz);

    auto waitForPacketSlot = [&]() {
        if (!throttlePackets) {
            return;
        }
        const auto now = clock::now();
        if (now < s_nextAllowedPacketSendTime) {
            std::this_thread::sleep_until(s_nextAllowedPacketSendTime);
        }
        s_nextAllowedPacketSendTime = clock::now() + minPacketInterval;
    };
#endif

    const int payloadSize = std::max(1, m_transportConfig.payloadSize);
    const int chunkCount = (size + payloadSize - 1) / payloadSize;
    uint64_t bytesSentForFrame = 0;

    for (int packetIndex = 0; packetIndex < chunkCount; ++packetIndex) {
        const int offset = packetIndex * payloadSize;
        const int chunkSize = std::min(payloadSize, size - offset);

        std::vector<uint8_t> packet(lb_streamer_detail::kHeaderSize + chunkSize);
        packet[0] = static_cast<uint8_t>((frameIndex >> 8) & 0xFF);
        packet[1] = static_cast<uint8_t>(frameIndex & 0xFF);
        packet[2] = static_cast<uint8_t>((packetIndex >> 8) & 0xFF);
        packet[3] = static_cast<uint8_t>(packetIndex & 0xFF);
        packet[4] = static_cast<uint8_t>((size >> 24) & 0xFF);
        packet[5] = static_cast<uint8_t>((size >> 16) & 0xFF);
        packet[6] = static_cast<uint8_t>((size >> 8) & 0xFF);
        packet[7] = static_cast<uint8_t>(size & 0xFF);
        memcpy(packet.data() + lb_streamer_detail::kHeaderSize, data + offset, chunkSize);

#if LOW_BANDWIDTH_TRANSPORT_MODE == LOW_BANDWIDTH_TRANSPORT_SERIAL
        // 串口链路要节流，避免一下子把链路打满。
        waitForPacketSlot();
        if (!lb_streamer_detail::writeSerialAll(m_serialHandle, packet.data(), packet.size())) {
            return false;
        }
        if (m_transportConfig.serialInterPacketDelayMs > 0 && packetIndex + 1 < chunkCount) {
            std::this_thread::sleep_for(std::chrono::milliseconds(m_transportConfig.serialInterPacketDelayMs));
        }
#else
        waitForPacketSlot();
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
        bytesSentForFrame += static_cast<uint64_t>(packet.size());
    }

    m_totalBytesSent.fetch_add(bytesSentForFrame);
    m_totalPacketsSent.fetch_add(static_cast<uint64_t>(chunkCount));
    m_totalFramesSent.fetch_add(1);
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
