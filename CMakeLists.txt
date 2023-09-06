cmake_minimum_required(VERSION 3.5)
project(l3cam_fusion)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(fusion_lidar_img src/fusion_lidar_img.cpp)
ament_target_dependencies(fusion_lidar_img 
    rclcpp 
    sensor_msgs
    cv_bridge
    OpenCV)

install(TARGETS 
        fusion_lidar_img
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
        launch
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
