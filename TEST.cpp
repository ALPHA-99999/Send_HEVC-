
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

int main() {
cv::Mat frame;
cv::VideoCapture cap(0); 
if (cap.isOpened()) {
    // 先尝试设置最高分辨率（可选）
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    //cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    //cap.set(cv::CAP_PROP_BUFFERSIZE, 1);  // 最小缓冲区


        bool success = cap.set(cv::CAP_PROP_FPS, 30);

    while (1)
    {

     auto lastFrameTime = std::chrono::steady_clock::now();
        if (cap.grab()) {
           auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - lastFrameTime).count();
            std::cout <<elapsed<<"ms"<<  std::endl;
        }else std::cerr << "Failed to capture frame" << std::endl;
    }

     }else std::cout << "open false"<<std::endl;
    return 0;
}