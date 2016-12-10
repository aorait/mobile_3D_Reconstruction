cmake_minimum_required(VERSION 2.8)
project(Reconstruction)

find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
find_package(Boost REQUIRED COMPONENTS system chrono filesystem unit_test_framework program_options)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

include_directories(${Boost_INCLUDE_DIR})

include_directories(
    include
    ${OPENCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(bundle_ios src/bundle_ios.cpp src/bundle_ios.cpp)
target_link_libraries(bundle_ios ${OpenCV_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})



