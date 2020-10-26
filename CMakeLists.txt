cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(stereo_image)

#Adicionando os plugins ROS

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})


find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        sensor_msgs
        cv_bridge
        image_transport
        geometry_msgs
        dji_sdk
        darknet_ros)

find_package(DJIOSDK REQUIRED)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(darknet_ros QUIET)
if(darknet_ros_FOUND)
    message(STATUS "Found darknet_ros package, will use it for object depth perception demo.")
    add_definitions(-DUSE_DARKNET_ROS)
endif()

find_package(OpenCV 3.3.0 QUIET)
if (OpenCV_FOUND)
    message(STATUS "Found OpenCV ${OpenCV_VERSION} installed in the system, will use it for depth perception sample")
    message(STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
    add_definitions(-DOPEN_CV_INSTALLED)
    set(OPEN_CV_3_3_0_INSTALLED TRUE)

    find_package(OpenCVContribImgProc QUIET)

    find_package(OpenCVViz QUIET)
else()
    find_package(OpenCV QUIET )
    if (OpenCV_FOUND)
        message(STATUS "Found OpenCV ${OpenCV_VERSION} installed in the system, depth percpetion demo requires 3.3.0+ for better experience")
        message(STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
        add_definitions(-DOPEN_CV_INSTALLED)
    else()
        message(STATUS "Did not find OpenCV in the system, image data is inside RecvContainer as raw data")
    endif()
endif()


find_package(CUDA QUIET)
if(CUDA_FOUND)
    message(STATUS "Found ${CUDA_VERSION} CUDA installed in the system, will use it for depth perception sample")
    message(STATUS " - Includes: ${CUDA_INCLUDE_DIRS}")
    add_definitions(-DUSE_GPU)
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
        darknet_ros
)

include_directories(include ${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_TARGET_INCLUDE_PATH
        "/usr/include/ignition/math2")


foreach(dir launch models urdf worlds config)
    install(DIRECTORY ${dir}/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)




######### READ SENSORS DJI #################

add_executable(read_attitude src/read_attitude.cpp)
target_link_libraries(read_attitude ${catkin_LIBRARIES})

add_executable(read_imu src/read_imu.cpp)
target_link_libraries(read_imu ${catkin_LIBRARIES})

add_executable(read_vopositon src/read_voposition.cpp)
target_link_libraries(read_vopositon ${catkin_LIBRARIES})

add_executable(read_imu_gimbal src/read_gimbal_imu.cpp)
target_link_libraries(read_imu_gimbal ${catkin_LIBRARIES})

add_executable(save_gps_image src/stereo_gps_acquisition.cpp)
target_link_libraries(save_gps_image ${catkin_LIBRARIES})