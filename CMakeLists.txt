cmake_minimum_required(VERSION 3.16)
project(dm_main)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

set(DM_RUNTIME_LIBS "")
set(DM_RUNTIME_RPATH "")

set(DM_CONDA_LIBUSB "/home/ubuntu/anaconda3/envs/lerobot/lib/libusb-1.0.so")
if(EXISTS ${DM_CONDA_LIBUSB})
    list(APPEND DM_RUNTIME_LIBS ${DM_CONDA_LIBUSB})
    list(APPEND DM_RUNTIME_RPATH "/home/ubuntu/anaconda3/envs/lerobot/lib")
endif()

set(DM_CONDA_LIBSTDCXX "/home/ubuntu/anaconda3/lib/libstdc++.so.6")
if(EXISTS ${DM_CONDA_LIBSTDCXX})
    list(APPEND DM_RUNTIME_LIBS ${DM_CONDA_LIBSTDCXX})
    list(APPEND DM_RUNTIME_RPATH "/home/ubuntu/anaconda3/lib")
endif()


# 在 CMakeLists.txt 的任何位置
message("CMAKE_CURRENT_SOURCE_DIR = ${CMAKE_CURRENT_SOURCE_DIR}")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                        ${CMAKE_CURRENT_SOURCE_DIR}/lib
                        ${LIBUSB_INCLUDE_DIRS})
                        


add_executable(${PROJECT_NAME}
    src/main.cpp
    src/protocol/damiao.cpp 
    src/protocol/usb_class.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE 
                        ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/libdm_device.so
                        ${DM_RUNTIME_LIBS}
                        ${LIBUSB_LIBRARIES})

if(DM_RUNTIME_RPATH)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        BUILD_RPATH "${DM_RUNTIME_RPATH};${CMAKE_CURRENT_SOURCE_DIR}/lib/linux")
endif()


add_executable(dev_sn
    src/dev_sn.cpp
)

# 链接库到可执行文件
target_link_libraries(dev_sn PRIVATE
    ${LIBUSB_LIBRARIES}
)

add_executable(gripper_probe
    src/gripper_probe.cpp
)

target_link_libraries(gripper_probe PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/libdm_device.so
    ${DM_RUNTIME_LIBS}
    ${LIBUSB_LIBRARIES}
)

if(DM_RUNTIME_RPATH)
    set_target_properties(gripper_probe PROPERTIES
        BUILD_RPATH "${DM_RUNTIME_RPATH};${CMAKE_CURRENT_SOURCE_DIR}/lib/linux")
endif()

add_executable(gripper_daemon
    src/gripper_daemon.cpp
)

target_link_libraries(gripper_daemon PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/libdm_device.so
    ${DM_RUNTIME_LIBS}
    ${LIBUSB_LIBRARIES}
)

if(DM_RUNTIME_RPATH)
    set_target_properties(gripper_daemon PROPERTIES
        BUILD_RPATH "${DM_RUNTIME_RPATH};${CMAKE_CURRENT_SOURCE_DIR}/lib/linux")
endif()
