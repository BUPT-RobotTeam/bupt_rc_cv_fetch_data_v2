#include "fetch_dataset.h"

static bool is_person_opt = true;
cv::Mat video_frame;
FetchDataset::FetchDataset(int argc, char* argv[]) : Node("fetch_dataset"){
    try{
        this->config_ = YAML::LoadFile(this->config_file_path_);
    }
    catch (const YAML::Exception& e) {
        std::cerr << "[BUPT_RC] Error while parsing YAML: " << e.what() << std::endl;
    }

    try{
        this->param_parser(argc, argv);


        //------------------------------创建文件夹------------------------------
        assert(this->config_["video_save_folder"].IsDefined());
        assert(this->config_["dataset_save_folder"].IsDefined());
        this->video_save_folder_ = this->config_["video_save_folder"].as<std::string>();
        this->dataset_save_folder_ = this->config_["dataset_save_folder"].as<std::string>();
        this->create_folder(this->video_save_folder_);
        this->create_folder(this->dataset_save_folder_);

        //------------------------------视频消息订阅------------------------------
        if (this->video_record_flag_) {
             this->subscription_ = this->create_subscription<bupt_rc_cv_interfaces::msg::CVCameraArray>("bupt_rc_cv/cameras", 1, std::bind(&FetchDataset::CVCameraArray_topic_callback, this, std::placeholders::_1));
        }
        else if (this->fetch_data_flag_)
            cv::namedWindow("FetchData", cv::WINDOW_NORMAL);
    }
    catch (const std::exception& e) {
        std::cout << "[BUPT_RC]Exception caught: " << e.what() << std::endl;
    }

}

FetchDataset::~FetchDataset() {
    std::cout << "All resources have been released" << std::endl;
    for (auto& pair : this->video_saver_)
        pair.second.release();
    cv::destroyAllWindows();
}


void FetchDataset::CVCameraArray_topic_callback(const bupt_rc_cv_interfaces::msg::CVCameraArray::SharedPtr msg) {
    for (auto& camera : msg->cameras) {
        if (this->video_saver_.count(camera.cam_name) == 0) {
            std::cout << "wirter create : " << camera.cam_name << std::endl; 
            std::string output_folder = this->video_save_folder_ + "/" + camera.cam_name.c_str();
            this->create_folder(output_folder);

            std::string output_file = output_folder + "/" + this->get_current_time_as_string() + ".avi";
            cv::VideoWriter writer = cv::VideoWriter(output_file, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(camera.img.frame_width, camera.img.frame_height));
            this->video_saver_[camera.cam_name] = writer;
            cv::namedWindow("[fetch_data]" + camera.cam_name, cv::WINDOW_NORMAL);
        }

        cv::Mat frame(camera.img.frame_height, camera.img.frame_width, CV_8UC3, camera.img.frame_data.data());
        cv::imshow("[fetch_data]" + camera.cam_name, frame);
        cv::waitKey(1);
        this->video_saver_[camera.cam_name].write(frame);
    }
}

void FetchDataset::create_folder(std::string& folder_path) {
    if (!std::filesystem::exists(folder_path)) {
        std::filesystem::create_directory(folder_path);
    }
}

void FetchDataset::param_parser(int argc, char* argv[]) {
    this->video_record_flag_ = false;
    this->fetch_data_flag_ = false;

    if (argc == 2) {
        if (std::string(argv[1]) == "--record_video") {
            this->video_record_flag_ = true;
        }
        else 
            throw std::runtime_error("[BUPT_RC] mode is not useful");
    }
    else if (argc == 5) {
        if (std::string(argv[1]) == "--fetch_data") {
            std::string video_path(argv[2]);
            if (!this->is_video_path_useful(video_path)) {
                throw std::runtime_error("[BUPT_RC] mode is not useful");
            }
            this->fetch_data_.video_path_ = std::string(argv[2]);

            if (std::string(argv[3]) == "--cam_name") {
                this->fetch_data_.camera_name_ = std::string(argv[4]);
                this->fetch_data_flag_ = true;
            }
            else 
                throw std::runtime_error("[BUPT_RC] mode is not useful");
        }
        else
            throw std::runtime_error("[BUPT_RC] mode is not useful");
    }
    else
        throw std::runtime_error("[BUPT_RC] mode is not useful");
}

bool FetchDataset::is_video_path_useful(std::string& video_path) {
    bool is_useful = true;
    cv::VideoCapture cap(video_path);

    // 判断文件是否打开
    if (!cap.isOpened())
        is_useful = false;
    else {
        cv::Mat frame;
        // 判断是否可以读取视频帧
        if (!cap.read(frame))
            is_useful = false;

        // 判断视频帧是否为空
        if (frame.empty())
            is_useful = false;
    }

    // 资源释放
    cap.release();
    return is_useful;
}

std::string FetchDataset::get_current_time_as_string() {
    std::time_t now = std::time(nullptr);
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", std::localtime(&now));
    return std::string(buffer);
}

/**
 * @brief opencv 滑动条回调函数
 *
 * @param pos 指定滑动条的位置坐标
 * @param userdata 用户数据类型
 */
static void onTrackbarSlideCallback(int pos, void *userdata) {;
    if (is_person_opt) {
        cv::VideoCapture *cap = static_cast<cv::VideoCapture *>(userdata);
        cap->set(cv::CAP_PROP_POS_FRAMES, pos);
        *cap >> video_frame;
        cv::imshow("FetchData", video_frame);
        cv::waitKey(1);
    }
}

/**
 * @brief 如果是record_videoo 或者 fetch_data_directly,
 * 则用来确保节点能够实时响应节点信息 如果是fetch_data_from_video,
 * 用来处理fetch_data相关操作
 */
void FetchDataset::spin() {
    int key;
    struct termios old_settings, new_settings;

    tcgetattr(STDIN_FILENO, &old_settings);
    new_settings = old_settings;
    new_settings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

    //----------------------------------------
    // 图像处理相关资源
    cv::VideoCapture cap;
    int delay = 1;
    bool is_paused = false;
    int skip_frames = 1;
    int total_frames = 0;
    std::string dataset_folder = this->dataset_save_folder_ + "/" + this->fetch_data_.camera_name_;

    if (this->fetch_data_flag_) {
        //----------------------------------------
        // 基本数据处理
        cap = cv::VideoCapture(this->fetch_data_.video_path_);
        delay = static_cast<int>(2000 / cap.get(cv::CAP_PROP_FPS));
        is_paused = false;
        skip_frames = 1;
        total_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
        cap >> video_frame;

        //----------------------------------------
        // 创建滑动条
        cv::createTrackbar("Position", "FetchData", nullptr, total_frames, onTrackbarSlideCallback, &cap);

        //------------------------------定义数据集保存路径------------------------------
        this->create_folder(dataset_folder);
    }

    while (rclcpp::ok()) {
        if (this->fetch_data_flag_) {
            if (video_frame.empty())
                break;
            if (!is_paused)
                cv::imshow("FetchData", video_frame);

            //----------------------------------------
            // 按键处理
            char ch = cv::waitKey(static_cast<int>(delay));
            if (ch == ' ')
                  is_paused = !is_paused;

            else if (ch == 83 || ch == 100) {
              // 按下 '->', 前进 skip_frames 帧
                cap.set(cv::CAP_PROP_POS_FRAMES, std::clamp((static_cast<int>(cap.get(cv::CAP_PROP_POS_FRAMES)) + skip_frames), 0, static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT) - 1)));
                cap >> video_frame;
            } 

            else if (ch == 81 || ch == 97) {
              // 按下 '<-', 后退skip_frames 帧
                cap.set(cv::CAP_PROP_POS_FRAMES, std::clamp((static_cast<int>(cap.get(cv::CAP_PROP_POS_FRAMES)) - skip_frames), 0, static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT) - 1)));
                cap >> video_frame;
            } 

            else if (ch == 'f' || ch == 'F') {
                ch = cv::waitKey();
                if (ch == 'y' || ch == 'Y') {
                    cv::imwrite(dataset_folder + "/" + this->get_current_time_as_string()  + ".jpeg", video_frame);
                    std::cout << "write img success. name: " <<  dataset_folder + "/" + this->get_current_time_as_string()  + ".jpeg" << std::endl;
                }
            }

            //----------------------------------------
            // 图像获取处理
            if (!is_paused) {
                //----------------------------------------
                // 滑动条处理
                is_person_opt = false;
                int current_frame = static_cast<int>(cap.get(cv::CAP_PROP_POS_FRAMES));
                cv::setTrackbarPos("Position", "FetchData", current_frame);
                is_person_opt = true;
                cap >> video_frame;
            }
        } 
        else {
            rclcpp::spin_some(this->get_node_base_interface());
        }

        if (kbhit()) {
            key = getchar();
            if (key == 'q')
                break;
        }
    }

    // 垃圾处理
    tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
    cap.release();
}

/**
 * @brief 从键盘读取数据(在终端中)
 *
 * @return 终端中是否有数据的标志
 */
int FetchDataset::kbhit() {
    struct timeval tv;
    fd_set read_fd;

    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&read_fd);
    FD_SET(STDIN_FILENO, &read_fd);

    if (select(STDIN_FILENO + 1, &read_fd, nullptr, nullptr, &tv) == -1) {
        return 0;
    }

    return FD_ISSET(STDIN_FILENO, &read_fd);
}
