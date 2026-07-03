cmake_minimum_required(VERSION 3.4...3.18)
project(patrol_car)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O3 -Wall -mcpu=native -flto -pthread")
#关闭不同类型变量比较警告与未使用变量警告
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-sign-compare")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-variable")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)                 #允许cmake将debug信息加入
set(INCLUDE_PATH "/usr/local/include")
set(LIB_PATH "/usr/local/lib")
include_directories(include)
#---------------------------------------------------------------------
#       [ Include、Lib  : Define ] ==> []
#---------------------------------------------------------------------
link_directories(${COMMON_LIB_DIR})
include_directories( ${COMMON_INCLUDE_DIR} )
#---------------------------------------------------------------------
#       [ Include、Lib  : Path ] ==> [ glib opencv ]
#---------------------------------------------------------------------
find_package(PkgConfig)
pkg_search_module(GLIB REQUIRED glib-2.0)
include_directories(${GLIB_INCLUDE_DIRS})

find_package( OpenCV REQUIRED )    
include_directories( ${OpenCV_INCLUDE_DIRS} )

pkg_search_module(SERIAL REQUIRED libserial)
include_directories(${SERIAL_INCLUDE_DIRS})
link_directories(${SERIAL_LIBRARY_DIRS})

# find ppnc
pkg_search_module(PPNC REQUIRED ppnc)
include_directories(${PPNC_INCLUDE_DIRS})
link_directories(${PPNC_LIBRARY_DIRS})

# find onnx
pkg_search_module(ONNX REQUIRED onnx)
include_directories(${ONNX_INCLUDE_DIRS})
link_directories(${ONNX_LIBRARY_DIRS})
#---------------------------------------------------------------------
#               [ bin ] ==> [ demo ]
#---------------------------------------------------------------------
# 开机自启程序
add_executable(boot src/tool/boot.cpp)
target_include_directories(boot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(boot pthread ${OpenCV_LIBS} ${SERIAL_LIBRARIES})

# 相机测试
add_executable(camera src/tool/camera.cpp src/ctrl/predeal.cpp)
target_include_directories(camera PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(camera pthread ${OpenCV_LIBS})

# 图像合成
add_executable(img2video src/tool/img2video.cpp)
target_include_directories(img2video PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(img2video pthread ${OpenCV_LIBS})

# 相机矫正
add_executable(calibration src/tool/calibration.cpp)
target_include_directories(calibration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(calibration pthread ${OpenCV_LIBS})

# 图像测试
add_executable(imging src/tool/imging.cpp src/ctrl/predeal.cpp)
target_include_directories(imging PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(imging pthread ${OpenCV_LIBS})

# 目标检测
add_executable(detection src/tool/detection.cpp src/ctrl/predeal.cpp)
target_include_directories(detection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(detection pthread ${PPNC_LIBRARIES} ${ONNX_LIBRARIES} ${OpenCV_LIBS})

# 图像采集
add_executable(collection src/tool/collection.cpp src/ctrl/predeal.cpp)
target_include_directories(collection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(collection pthread ${OpenCV_LIBS} ${SERIAL_LIBRARIES})
#---------------------------------------------------------------------
#               [ bin ] ==> [ main ]
#---------------------------------------------------------------------
file(GLOB_RECURSE SRC_LIST
    "src/ctrl/*.cpp"
    "src/fsm/*.cpp"
)
add_executable(icar src/icar.cpp ${SRC_LIST})
target_include_directories(icar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(icar pthread ${PPNC_LIBRARIES} ${ONNX_LIBRARIES} ${OpenCV_LIBS} ${SERIAL_LIBRARIES})

