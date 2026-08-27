cmake_minimum_required(VERSION 3.10)
project(karpenko_gyro_calib CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# yaml-cpp：优先 CMake config，回退到手动查找
find_package(yaml-cpp QUIET)
if(NOT yaml-cpp_FOUND)
  find_path(YAMLCPP_INCLUDE_DIR yaml-cpp/yaml.h)
  find_library(YAMLCPP_LIBRARY yaml-cpp)
  if(NOT YAMLCPP_INCLUDE_DIR OR NOT YAMLCPP_LIBRARY)
    message(FATAL_ERROR "yaml-cpp not found")
  endif()
  add_library(yaml-cpp INTERFACE)
  target_include_directories(yaml-cpp INTERFACE ${YAMLCPP_INCLUDE_DIR})
  target_link_libraries(yaml-cpp INTERFACE ${YAMLCPP_LIBRARY})
endif()

# Ceres 可选（未安装则仅用坐标下降）
find_package(Ceres QUIET)
if(Ceres_FOUND)
  message(STATUS "Ceres found: LM refinement available (optional)")
  add_compile_definitions(KARPENKO_HAVE_CERES)
else()
  message(STATUS "Ceres not found: coordinate descent only")
endif()

# 核心库
add_library(karpenko_core
  src/io.cpp
  src/gyro_integrator.cpp
  src/objective.cpp
  src/optimizer.cpp
  src/calibrate.cpp
  src/feature_tracker.cpp)
target_include_directories(karpenko_core PUBLIC
  src ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(karpenko_core PUBLIC
  Eigen3::Eigen ${OpenCV_LIBS} yaml-cpp)

# 标定主程序
add_executable(karpenko_calib src/main.cpp)
target_link_libraries(karpenko_calib PRIVATE karpenko_core)

# 合成自检
enable_testing()
add_executable(karpenko_selfcheck test/synthetic_selfcheck.cpp)
target_link_libraries(karpenko_selfcheck PRIVATE karpenko_core)
add_test(NAME synthetic_selfcheck COMMAND karpenko_selfcheck)

# 连续 R_cam_gyro 微旋转精修自检
add_executable(karpenko_rot_selfcheck test/synthetic_rot_selfcheck.cpp)
target_link_libraries(karpenko_rot_selfcheck PRIVATE karpenko_core)
add_test(NAME synthetic_rot_selfcheck COMMAND karpenko_rot_selfcheck)
