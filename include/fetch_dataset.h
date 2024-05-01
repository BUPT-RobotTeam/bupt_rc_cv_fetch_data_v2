#pragma once
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include "bupt_rc_cv_interfaces/msg/cv_camera_array.hpp"
#include <yaml-cpp/yaml.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <filesystem>
#include <fstream>
#include <ctime>
#include <algorithm>

struct FetchData {
    std::string video_path_;
    std::string camera_name_;
};


class FetchDataset : public rclcpp::Node {
public:
    FetchDataset(int argc, char* argv[]);
    ~FetchDataset();
    void spin();

private:
    int kbhit();
    void CVCameraArray_topic_callback(const bupt_rc_cv_interfaces::msg::CVCameraArray::SharedPtr msg);
    void param_parser(int argc, char* argv[]);
    bool is_video_path_useful(std::string& video_path);
    void create_folder(std::string& folder_path);
    std::string get_current_time_as_string();

private:
    rclcpp::Subscription<bupt_rc_cv_interfaces::msg::CVCameraArray>::SharedPtr subscription_;

    std::map<std::string, cv::VideoWriter> video_saver_;
    FetchData fetch_data_;

    std::string video_save_folder_;
    std::string dataset_save_folder_;

    bool video_record_flag_;
    bool fetch_data_flag_;

    YAML::Node config_;
    const std::string config_file_path_ = "/home/bupt-rc/ros2_ws/bupt_rc_cv_ws/src/bupt_rc_cv_fetch_dataset_v2/config.yaml";
};
