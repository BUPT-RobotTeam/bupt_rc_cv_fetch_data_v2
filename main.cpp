#include <iostream>
#include "include/fetch_dataset.h"
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    FetchDataset node_fetch_dataset(argc, argv);
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "|                    press [q] to exit                    |" << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    node_fetch_dataset.spin();
    rclcpp::shutdown();
    return 0;
}
