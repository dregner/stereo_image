cmake_minimum_required(VERSION 2.8)
project(stereo_image)

#Adicionando os plugins ROS
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

find_package(ignition-math4)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        sensor_msgs
        cv_bridge
        image_transport
        geometry_msgs
        dji_osdk_ros

        )
find_package(DJIOSDK REQUIRED)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 14)
find_package(darknet_ros QUIET)
if(darknet_ros_FOUND)
    message(STATUS "Found darknet_ros package, will use it for object depth perception demo.")
    add_definitions(-DUSE_DARKNET_ROS)
endif()


find_package(OpenCV 3.3 REQUIRED)
if (OpenCV_FOUND)
    message(STATUS "Found OpenCV ${OpenCV_VERSION} installed in the system")

    FILE(GLOB STEREO_UTILITY_SRC_FILE
            src/stereo_utility/camera_param.cpp
            src/stereo_utility/stereo_frame.cpp
            src/stereo_utility/config.cpp)

    message(STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
    include_directories( ${OpenCV_INCLUDE_DIRS})
    add_definitions(-DOPEN_CV_INSTALLED)
    add_definitions(-DUSE_OPEN_CV_CONTRIB)

endif()


find_package(CUDA QUIET)
if(CUDA_FOUND)
    message(STATUS "Found ${CUDA_VERSION} CUDA installed in the system, will use it for depth perception sample")
    message(STATUS " - Includes: ${CUDA_INCLUDE_DIRS}")
#    add_definitions(-DUSE_GPU)
elseif()
    message(STATUS "Did not find CUDA in the system")
endif()



catkin_package(
        CATKIN_DEPENDS
        roscpp
        rospy
        actionlib_msgs
        geometry_msgs
        sensor_msgs
        std_msgs
        dji_osdk_ros
        stereo_vant
)

include_directories(include ${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_TARGET_INCLUDE_PATH
        "/usr/include/ignition/math4")


foreach(dir launch models urdf worlds config)
    install(DIRECTORY ${dir}/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)




######### READ SENSORS DJI #################

add_executable(read_attitude src/read_attitude.cpp)
target_link_libraries(read_attitude ${catkin_LIBRARIES} ignition-math4::ignition-math4 )

add_executable(read_imu src/read_imu.cpp)
target_link_libraries(read_imu ${catkin_LIBRARIES} ignition-math4::ignition-math4 )


add_executable(read_imu_gimbal src/read_gimbal_imu.cpp)
target_link_libraries(read_imu_gimbal ${catkin_LIBRARIES} ignition-math4::ignition-math4 )

add_executable(show_image src/show_stereo.cpp)
target_link_libraries(show_image ${OpenCV_LIBS})

add_executable(vga_depth src/stereo_depth_vga.cpp)
target_link_libraries(vga_depth ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(save_image src/stereo_acquisition.cpp)
target_link_libraries(save_image ${catkin_LIBRARIES} )

add_executable(save_zed src/zed_stereo_acquisition.cpp)
target_link_libraries(save_zed ${catkin_LIBRARIES} ignition-math4::ignition-math4)

add_executable(calib_stereo src/calib_stereo.cpp)
target_link_libraries(calib_stereo  ${OpenCV_LIBS})

add_executable(save_gps_image src/stereo_gps_acquisition.cpp)
target_link_libraries(save_gps_image ${catkin_LIBRARIES} ignition-math4::ignition-math4 )

add_executable(save_gps_imu_image src/stereo_gps_imu_acquisition.cpp)
target_link_libraries(save_gps_imu_image ${catkin_LIBRARIES} ignition-math4::ignition-math4 )

add_executable(zed_stereo src/zed_gps_acquisition.cpp)
target_link_libraries(zed_stereo ${catkin_LIBRARIES} ignition-math4::ignition-math4)

add_executable(save_cameras src/zed_gimbal_image.cpp)
target_link_libraries(save_cameras ${catkin_LIBRARIES} ignition-math4::ignition-math4)

add_executable(m210_stereo_depth
        src/m210_stereo_vga.cpp
        ${STEREO_UTILITY_SRC_FILE} )
target_link_libraries(m210_stereo_depth
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        ${OpenCV_LIBS}
        )
#add_dependencies(m210_stereo_depth dji_sdk_generate_messages_cpp)