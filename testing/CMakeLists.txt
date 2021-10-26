cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2_MOD_TESTING)

find_package(OpenCV REQUIRED)

add_executable(opencv_test
        ./opencv_test.cpp)

target_link_libraries( opencv_test
${OpenCV_LIBS}
)