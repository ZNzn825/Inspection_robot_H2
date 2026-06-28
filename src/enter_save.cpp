#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>

int main() {
    // 创建输出文件夹
    std::string output_dir = "output";
    std::filesystem::create_directories(output_dir + "/left_image");
    std::filesystem::create_directories(output_dir + "/right_image");

    // 初始化 ZED 相机
    sl::Camera zed;
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD1080;
    init_params.depth_mode = sl::DEPTH_MODE::NEURAL;
    init_params.depth_minimum_distance = 500; // mm
    init_params.depth_maximum_distance = 1000; // mm
    init_params.coordinate_units = sl::UNIT::MILLIMETER;

    sl::ERROR_CODE status = zed.open(init_params);
    if (status != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "无法打开相机：" << status << std::endl;
        return -1;
    }

    sl::CameraInformation cam_info = zed.getCameraInformation();
    sl::Resolution image_size = cam_info.camera_configuration.resolution;

    sl::Mat left_image(image_size.width, image_size.height, sl::MAT_TYPE::U8_C4);
    sl::Mat right_image(image_size.width, image_size.height, sl::MAT_TYPE::U8_C4);

    int save_index = 0;
    std::cout << "按 [Enter] 保存左右图，按 [q] 退出" << std::endl;

    while (true) {
        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
            // 获取左右图像
            zed.retrieveImage(left_image, sl::VIEW::LEFT, sl::MEM::CPU);
            zed.retrieveImage(right_image, sl::VIEW::RIGHT, sl::MEM::CPU);

            // 转换为 OpenCV 格式
            cv::Mat left_bgr(left_image.getHeight(), left_image.getWidth(), CV_8UC4, left_image.getPtr<uchar>());
            cv::Mat right_bgr(right_image.getHeight(), right_image.getWidth(), CV_8UC4, right_image.getPtr<uchar>());

            // 拼接左右图像
            cv::Mat combined;
            cv::hconcat(left_bgr, right_bgr, combined);

            if (combined.type() != CV_8UC3) {
                cv::cvtColor(combined, combined, cv::COLOR_BGRA2BGR);
            }

            // 显示拼接图
            cv::imshow("ZED 左图 + 右图", combined);
            int key = cv::waitKey(10) & 0xFF;
            if (key == 'q') {
                break;
            }
            else if (key == 13) { // Enter 键
                std::string left_image_filename = "output/left_image/left_" + std::to_string(save_index) + ".png";
                std::string right_image_filename = "output/right_image/right_" + std::to_string(save_index) + ".png";

                cv::imwrite(left_image_filename, left_bgr);
                cv::imwrite(right_image_filename, right_bgr);

                std::cout << "已保存左右图像：" << save_index << std::endl;
                save_index++;
            }
        }
    }

    zed.close();
    cv::destroyAllWindows();
    std::cout << "程序已退出" << std::endl;

    return 0;
}
