#include "lb_streamer.h"
#include "lb_streamer_utils.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

cv::Rect LowBandwidthStreamer::computeEffectiveRoi(const cv::Size& frameSize) const {
    if (!m_preprocessConfig.enableRoi) {
        return cv::Rect(0, 0, frameSize.width, frameSize.height);
    }

    // 没有手动指定时，默认取画面中心正方形作为 ROI。
    cv::Rect roi = m_preprocessConfig.roi;
    if (roi.width <= 0 || roi.height <= 0) {
        const int side = std::min(frameSize.width, frameSize.height);
        roi = cv::Rect((frameSize.width - side) / 2, (frameSize.height - side) / 2, side, side);
    }

    roi &= cv::Rect(0, 0, frameSize.width, frameSize.height);
    if (roi.width <= 0 || roi.height <= 0) {
        return cv::Rect(0, 0, frameSize.width, frameSize.height);
    }
    return roi;
}

cv::Rect LowBandwidthStreamer::currentRuntimeRoi(const cv::Size& frameSize) {
    // 第一次进入时用配置初始化，之后保持方向键移动后的运行时位置。
    if (!m_runtimeRoiInitialized || m_runtimeRoi.width <= 0 || m_runtimeRoi.height <= 0) {
        m_runtimeRoi = computeEffectiveRoi(frameSize);
        m_runtimeRoiInitialized = true;
        return m_runtimeRoi;
    }

    m_runtimeRoi = clampRoiToFrame(m_runtimeRoi, frameSize);
    m_runtimeRoiInitialized = true;
    return m_runtimeRoi;
}

void LowBandwidthStreamer::moveRuntimeRoi(const cv::Size& frameSize, int dx, int dy) {
    // 方向键只改位置，不改 ROI 大小。
    cv::Rect roi = currentRuntimeRoi(frameSize);
    roi.x += dx;
    roi.y += dy;
    m_runtimeRoi = clampRoiToFrame(roi, frameSize);
    m_runtimeRoiInitialized = true;
}

cv::Rect LowBandwidthStreamer::clampRoiToFrame(const cv::Rect& roi, const cv::Size& frameSize) const {
    cv::Rect bounded = roi;
    if (bounded.width <= 0 || bounded.height <= 0) {
        return computeEffectiveRoi(frameSize);
    }

    bounded.width = std::min(bounded.width, frameSize.width);
    bounded.height = std::min(bounded.height, frameSize.height);
    bounded.x = std::max(0, std::min(bounded.x, frameSize.width - bounded.width));
    bounded.y = std::max(0, std::min(bounded.y, frameSize.height - bounded.height));
    bounded &= cv::Rect(0, 0, frameSize.width, frameSize.height);
    if (bounded.width <= 0 || bounded.height <= 0) {
        return computeEffectiveRoi(frameSize);
    }
    return bounded;
}

cv::Mat LowBandwidthStreamer::preprocessFrame(const cv::Mat& frame, const cv::Rect& roi) {
    // 先裁 ROI，再缩放到编码目标大小。
    cv::Mat roiFrame = frame(roi).clone();

    cv::Mat resized;
    cv::resize(roiFrame, resized,
               cv::Size(m_preprocessConfig.outputWidth, m_preprocessConfig.outputHeight),
               0.0, 0.0, cv::INTER_AREA);

    cv::Mat grayFrame;
    cv::cvtColor(resized, grayFrame, cv::COLOR_BGR2GRAY);

    double motionRatio = 0.0;
    cv::Mat motionMask = buildMotionMask(grayFrame, motionRatio);

    cv::Mat output;
    applyStaticSuppression(resized, grayFrame, motionMask, output);
    applyCenterProtection(resized, output);
    applyTrail(grayFrame, motionMask, motionRatio, output);
    applyCenterProtection(resized, output);

    return output;
}

cv::Mat LowBandwidthStreamer::makePreviewFrame(const cv::Mat& originalFrame,
                                               const cv::Mat& processedFrame,
                                               const cv::Rect& roi) const {
    if (originalFrame.empty() || processedFrame.empty()) {
        return processedFrame;
    }

    // 左边保留原图预览，右边显示处理后的压缩图。
    const cv::Size originalPreviewTarget(960, 540);
    double scale = 1.0;
    cv::Point offset;
    cv::Mat originalPreview = lb_streamer_detail::fitIntoCanvas(originalFrame, originalPreviewTarget, scale, offset);

    const double roiScaleX = scale;
    const double roiScaleY = scale;
    cv::Rect previewRoi(
        static_cast<int>(std::lround(roi.x * roiScaleX)) + offset.x,
        static_cast<int>(std::lround(roi.y * roiScaleY)) + offset.y,
        static_cast<int>(roi.width * roiScaleX),
        static_cast<int>(roi.height * roiScaleY));
    previewRoi &= cv::Rect(0, 0, originalPreview.cols, originalPreview.rows);
    if (previewRoi.width > 0 && previewRoi.height > 0) {
        cv::rectangle(originalPreview, previewRoi, cv::Scalar(0, 255, 255), 2);
    }
    cv::putText(originalPreview, "Original",
                cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::putText(originalPreview, "Arrows move ROI, ESC quit",
                cv::Point(12, 58), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    cv::putText(originalPreview,
                "ROI x=" + std::to_string(roi.x) + " y=" + std::to_string(roi.y) +
                    " w=" + std::to_string(roi.width) + " h=" + std::to_string(roi.height),
                cv::Point(12, 86), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(0, 255, 255), 1, cv::LINE_AA);

    double processedScale = 1.0;
    cv::Point processedOffset;
    cv::Mat processedPreview = lb_streamer_detail::fitIntoCanvas(processedFrame, originalPreview.size(), processedScale, processedOffset);
    cv::putText(processedPreview, "Processed",
                cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

    const int canvasWidth = originalPreview.cols + processedPreview.cols;
    const int canvasHeight = std::max(originalPreview.rows, processedPreview.rows);
    cv::Mat canvas(canvasHeight, canvasWidth, originalPreview.type(), cv::Scalar::all(0));
    originalPreview.copyTo(canvas(cv::Rect(0, 0, originalPreview.cols, originalPreview.rows)));
    processedPreview.copyTo(canvas(cv::Rect(originalPreview.cols, 0,
                                            processedPreview.cols, processedPreview.rows)));

    // 预览窗口太大时，整体再缩小一档，方便单屏查看。
    constexpr int kMaxPreviewWidth = 1600;
    constexpr int kMaxPreviewHeight = 900;
    if (canvas.cols > kMaxPreviewWidth || canvas.rows > kMaxPreviewHeight) {
        const double shrink = std::min(static_cast<double>(kMaxPreviewWidth) / static_cast<double>(canvas.cols),
                                       static_cast<double>(kMaxPreviewHeight) / static_cast<double>(canvas.rows));
        cv::Mat scaled;
        cv::resize(canvas, scaled,
                   cv::Size(std::max(1, static_cast<int>(std::lround(canvas.cols * shrink))),
                            std::max(1, static_cast<int>(std::lround(canvas.rows * shrink)))),
                   0.0, 0.0, cv::INTER_AREA);
        return scaled;
    }

    return canvas;
}

cv::Mat LowBandwidthStreamer::buildMotionMask(const cv::Mat& grayFrame, double& motionRatio) {
    // 差分出运动区域，静态背景尽量变灰。
    cv::Mat motionMask(grayFrame.size(), CV_8UC1, cv::Scalar(0));
    if (m_previousGray.empty()) {
        m_previousGray = grayFrame.clone();
        motionRatio = 0.0;
        return motionMask;
    }

    cv::Mat diff;
    cv::absdiff(grayFrame, m_previousGray, diff);
    cv::threshold(diff, motionMask, m_preprocessConfig.motionThreshold, 255, cv::THRESH_BINARY);

    if (m_preprocessConfig.erodeSize > 0) {
        const cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_RECT,
            cv::Size(m_preprocessConfig.erodeSize * 2 + 1, m_preprocessConfig.erodeSize * 2 + 1));
        cv::erode(motionMask, motionMask, kernel);
    }

    if (m_preprocessConfig.dilateSize > 0) {
        const cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_RECT,
            cv::Size(m_preprocessConfig.dilateSize * 2 + 1, m_preprocessConfig.dilateSize * 2 + 1));
        cv::dilate(motionMask, motionMask, kernel);
    }

    motionRatio = static_cast<double>(cv::countNonZero(motionMask)) /
                  static_cast<double>(motionMask.rows * motionMask.cols);
    m_previousGray = grayFrame.clone();
    return motionMask;
}

void LowBandwidthStreamer::applyStaticSuppression(const cv::Mat& source,
                                                  const cv::Mat& grayFrame,
                                                  const cv::Mat& motionMask,
                                                  cv::Mat& output) const {
    // 先铺灰度背景，再把运动区域保留成彩色。
    cv::Mat grayBgr;
    cv::cvtColor(grayFrame, grayBgr, cv::COLOR_GRAY2BGR);

    int blurKernel = std::max(1, m_preprocessConfig.blurKernel);
    if (blurKernel % 2 == 0) {
        ++blurKernel;
    }
    cv::GaussianBlur(grayBgr, output, cv::Size(blurKernel, blurKernel), 0.0);

    source.copyTo(output, motionMask);
}

void LowBandwidthStreamer::applyCenterProtection(const cv::Mat& source, cv::Mat& output) const {
    if (m_preprocessConfig.centerProtectSize <= 0) {
        return;
    }

    // 中间保护区始终保留原图，避免主体被压成灰色。
    const int protectWidth = std::min(m_preprocessConfig.centerProtectSize, source.cols);
    const int protectHeight = std::min(m_preprocessConfig.centerProtectSize, source.rows);
    const cv::Rect centerRect((source.cols - protectWidth) / 2,
                              (source.rows - protectHeight) / 2,
                              protectWidth,
                              protectHeight);
    source(centerRect).copyTo(output(centerRect));
}

void LowBandwidthStreamer::applyTrail(const cv::Mat& grayFrame,
                                      const cv::Mat& motionMask,
                                      double motionRatio,
                                      cv::Mat& output) {
    if (m_preprocessConfig.trailLength <= 0) {
        return;
    }

    // 低运动时保留轨迹，运动过强就清空，防止拖影堆积。
    if (motionRatio > m_preprocessConfig.trailDisableRatio) {
        m_trailHistory.clear();
        return;
    }

    cv::Mat trailCandidate(grayFrame.size(), CV_8UC1, cv::Scalar(0));
    grayFrame.copyTo(trailCandidate, motionMask);

    if (cv::countNonZero(trailCandidate) == 0) {
        return;
    }

    m_trailHistory.push_back(trailCandidate);
    while (static_cast<int>(m_trailHistory.size()) > m_preprocessConfig.trailLength) {
        m_trailHistory.pop_front();
    }

    cv::Mat trail = m_trailHistory.front().clone();
    for (size_t i = 1; i < m_trailHistory.size(); ++i) {
        cv::max(trail, m_trailHistory[i], trail);
    }

    cv::Mat trailBgr;
    cv::cvtColor(trail, trailBgr, cv::COLOR_GRAY2BGR);
    cv::max(output, trailBgr, output);
}
