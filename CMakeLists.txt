cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2_MOD)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

list(APPEND CMAKE_INCLUDE_PATH "/usr/local/include")
find_package (Eigen3 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
# gridmapper/src/MapProcessor.cpp
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)

message(STATUS "Compile With map save/load function")
find_library(BOOST_SERIALIZATION boost_serialization)
if (NOT BOOST_SERIALIZATION)
    message(FATAL_ERROR "Can't find libboost_serialization")
endif()
target_link_libraries(${PROJECT_NAME} ${BOOST_SERIALIZATION})

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_kitti
Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})

add_executable(arducam_images 
Examples/Stereo/arducam_images.cpp 
)
target_link_libraries(arducam_images ${PROJECT_NAME})

add_executable(arducam_stereo Examples/Stereo/arducam_stereo.cpp)
include_directories(arducam_stereo
/home/pi/Libraries/MIPI_Camera/RPI
)
target_link_libraries(arducam_stereo 
/home/pi/Libraries/MIPI_Camera/RPI/lib/libarducam_mipicamera.so
${PROJECT_NAME}
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Vocabulary)

add_executable(bin_vocabulary Vocabulary/bin_vocabulary.cpp)
target_link_libraries(bin_vocabulary ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so ${OpenCV_LIBS})

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/gridmapper/src)
# 
# add_executable(gridmapper
#     gridmapper/src/gridmapper.cpp         
# )
# include_directories(gridmapper
#    ${PROJECT_SOURCE_DIR}/gridmapper/include 
# )
# target_link_libraries(gridmapper ${PROJECT_NAME} 
# )
