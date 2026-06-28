#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

#include <iostream>
#include <vector>
#include <fstream>
#include <algorithm>
#include <iomanip>
#include <sstream>
// ================= 参数结构体 =================
struct Params {
    int h_min, h_max;
    int s_min;
    int v_min;
    int glare_v_min;
    int glare_s_max;
};

// ================= 固定检测参数 =================
const int AREA_MIN = 10;
const int AREA_MAX = 500;
const int W_MIN = 4, W_MAX = 60;
const int H_MIN = 4, H_MAX = 30;
const float AR_MIN = 1.0f, AR_MAX = 6.0f;
const float RECT_MIN = 0.4f;

// ================= 检测函数 =================
std::vector<cv::Rect> detectRects(
    const cv::Mat& img,
    const Params& p,
    cv::Mat& vis_colorMask,
    cv::Mat& vis_glare,
    cv::Mat& vis_mask,
    cv::Mat& vis_morph
) {
    std::vector<cv::Rect> result;
    if (img.empty() || img.type() != CV_8UC3) return result;

    cv::Mat blur, hsv;
    cv::GaussianBlur(img, blur, cv::Size(3, 3), 0);
    cv::cvtColor(blur, hsv, cv::COLOR_BGR2HSV);

    std::vector<cv::Mat> hsvCh;
    cv::split(hsv, hsvCh);
    const cv::Mat& S = hsvCh[1];
    const cv::Mat& V = hsvCh[2];

    // HSV 分割
    cv::inRange(
        hsv,
        cv::Scalar(p.h_min, p.s_min, p.v_min),
        cv::Scalar(p.h_max, 255, 255),
        vis_colorMask
    );

    // 高光剔除
    cv::Mat glareV, glareS;
    cv::threshold(V, glareV, p.glare_v_min, 255, cv::THRESH_BINARY);
    cv::threshold(S, glareS, p.glare_s_max, 255, cv::THRESH_BINARY_INV);
    cv::bitwise_and(glareV, glareS, vis_glare);

    // 最终掩码
    cv::bitwise_and(vis_colorMask, ~vis_glare, vis_mask);

    // 形态学
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(3, 3)
    );
    cv::morphologyEx(vis_mask, vis_morph, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(vis_morph, vis_morph, cv::MORPH_CLOSE, kernel);

    // 连通域
    cv::Mat labels, stats, centroids;
    int n = cv::connectedComponentsWithStats(vis_morph, labels, stats, centroids);

    for (int i = 1; i < n; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area < AREA_MIN || area > AREA_MAX) continue;

        int x = stats.at<int>(i, cv::CC_STAT_LEFT);
        int y = stats.at<int>(i, cv::CC_STAT_TOP);
        int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);

        if (w < W_MIN || w > W_MAX) continue;
        if (h < H_MIN || h > H_MAX) continue;

        float ar = (float)std::max(w, h) / std::min(w, h);
        if (ar < AR_MIN || ar > AR_MAX) continue;

        float rect = area / float(w * h + 1);
        if (rect < RECT_MIN) continue;

        result.emplace_back(x, y, w, h);
    }

    return result;
}

// ================= 主程序 =================
int main() {
    // -------- ZED 初始化 --------
    sl::Camera zed;
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.depth_mode = sl::DEPTH_MODE::NONE;

    if (zed.open(init_params) != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "ZED 打开失败\n";
        return 1;
    }

    auto cam_info = zed.getCameraInformation();
    auto res = cam_info.camera_configuration.resolution;
    sl::Mat zed_left(res.width, res.height, sl::MAT_TYPE::U8_C4);

    // -------- HSV 参数 --------
    Params params = { 30, 85, 40, 100, 230, 65 };

    // -------- Trackbar --------
    cv::namedWindow("Trackbars", cv::WINDOW_NORMAL);
    cv::createTrackbar("H Min", "Trackbars", &params.h_min, 179);
    cv::createTrackbar("H Max", "Trackbars", &params.h_max, 179);
    cv::createTrackbar("S Min", "Trackbars", &params.s_min, 255);
    cv::createTrackbar("V Min", "Trackbars", &params.v_min, 255);
    cv::createTrackbar("Glare V Min", "Trackbars", &params.glare_v_min, 255);
    cv::createTrackbar("Glare S Max", "Trackbars", &params.glare_s_max, 255);

    int frame_index = 1;

    std::cout << "s=保存  q/ESC=退出\n";

    while (true) {
        if (zed.grab() != sl::ERROR_CODE::SUCCESS) continue;

        zed.retrieveImage(zed_left, sl::VIEW::LEFT, sl::MEM::CPU);
        cv::Mat rgba(zed_left.getHeight(), zed_left.getWidth(), CV_8UC4, zed_left.getPtr<uchar>());
        cv::Mat frame;
        cv::cvtColor(rgba, frame, cv::COLOR_BGRA2BGR);

        params.h_min = std::min(params.h_min, params.h_max - 1);
        params.h_max = std::max(params.h_max, params.h_min + 1);

        cv::Mat colorMask, glareMask, finalMask, morphMask;
        auto boxes = detectRects(frame, params, colorMask, glareMask, finalMask, morphMask);

        cv::Mat vis = frame.clone();
        for (auto& b : boxes)
            cv::rectangle(vis, b, cv::Scalar(0, 255, 255), 2);

        // 显示
        cv::imshow("Original", frame);
        cv::imshow("1 - HSV Mask", colorMask);
        cv::imshow("2 - Glare Mask", glareMask);
        cv::imshow("3 - Final Mask", finalMask);
        cv::imshow("4 - Morph", morphMask);
        cv::imshow("5 - Final Boxes", vis);

        int key = cv::waitKey(10);
        if (key == 27 || key == 'q') break;

        // -------- 保存 --------
        if (key == 's' || key == 'S') {
            std::ostringstream ss;
            ss << std::setw(3) << std::setfill('0') << frame_index;
            std::string id = ss.str();

            cv::imwrite("frame_" + id + ".jpg", vis);
            cv::imwrite("color_mask_" + id + ".jpg", colorMask);
            cv::imwrite("glare_mask_" + id + ".jpg", glareMask);
            cv::imwrite("final_mask_" + id + ".jpg", finalMask);
            cv::imwrite("morph_" + id + ".jpg", morphMask);

            std::ofstream csv("detections_" + id + ".csv");
            csv << "center_x,center_y,x,y,width,height\n";
            for (auto& b : boxes) {
                float cx = b.x + b.width * 0.5f;
                float cy = b.y + b.height * 0.5f;
                csv << cx << "," << cy << "," << b.x << "," << b.y << ","
                    << b.width << "," << b.height << "\n";
            }
            csv.close();

            std::cout << "[Saved] frame_" << id << "\n";
            frame_index++;
        }
    }

    zed.close();
    cv::destroyAllWindows();
    return 0;
}
