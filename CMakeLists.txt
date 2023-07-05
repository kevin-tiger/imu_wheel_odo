cmake_minimum_required(VERSION 2.8.3)
project(imu_wheel_odo)
# SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

find_package(Eigen3 REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/src
include_directories(${EIGEN3_INCLUDE_DIRS})
)

add_library(${PROJECT_NAME} SHARED
src/options.cc
)

add_executable(run_imu_wheel_odo app/run_imu_wheel_odo.cc)
target_link_libraries(run_imu_wheel_odo ${PROJECT_NAME})



