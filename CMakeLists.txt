cmake_minimum_required(VERSION 3.8)
project(bupt_rc_cv_fetch_dataset_v2)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
file(GLOB CPP_SRC ${PROJECT_SOURCE_DIR}/src/*.cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bupt_rc_cv_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

add_executable(fetch_dataset
    main.cpp
    ${CPP_SRC}
)

target_link_libraries(fetch_dataset
    ${OpenCV_LIBS}
    yaml-cpp::yaml-cpp
)
ament_target_dependencies(fetch_dataset rclcpp bupt_rc_cv_interfaces)

install(TARGETS fetch_dataset
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
