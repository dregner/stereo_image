cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(gimbal_controller)

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


add_message_files(
        FILES
        BoundingBox.msg
)

generate_messages(
        DEPENDENCIES
        actionlib_msgs
        geometry_msgs
        sensor_msgs
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        rospy
        actionlib_msgs
        geometry_msgs
        sensor_msgs
        std_msgs
        darknet_ros
        darknet_ros_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_TARGET_INCLUDE_PATH
        "/usr/include/ignition/math2")


foreach(dir launch models urdf worlds config)
    install(DIRECTORY ${dir}/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

#########  FEEDBACK CONTROL ##########

#add_executable(inv_kinematic src/gimbal/inverse_kinematic.cpp)
#target_link_libraries(inv_kinematic ${catkin_LIBRARIES})

add_executable(dji_ik_control src/gimbal/track_control/ik_track_control.cpp)
#add_dependencies(gimbal_dji dji_sdk_generate_messages_cpp)
target_link_libraries(dji_ik_control
                        ${catkin_LIBRARIES}
                        ${DJIOSDK_LIBRARIES})

add_executable(dji_pid_control src/gimbal/track_control/pid_track_control.cpp)
#add_dependencies(gimbal_dji dji_sdk_generate_messages_cpp)
target_link_libraries(dji_pid_control
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES})

add_executable(dji_pid_imu_control src/gimbal/track_control/pid_track_control_imu.cpp)
target_link_libraries(dji_pid_imu_control
                        ${catkin_LIBRARIES}
                        ${DJIOSDK_LIBRARIES})



add_executable(dji_ik_imu_control src/gimbal/track_control/ik_track_control_imu.cpp)
target_link_libraries(dji_ik_imu_control
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES})

add_executable(control_imu src/gimbal/position_control/gimbal_imu_control.cpp)
target_link_libraries(control_imu
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES})

######### READ BOUNDING BOXES DARKNET ##################

add_executable(read_bb src/darknet/darknet_bb.cpp)
target_link_libraries(read_bb ${catkin_LIBRARIES})

######### READ/WRITE DJI GIMBAL #################

add_executable(read_image src/gimbal/read_image.cpp)
target_link_libraries(read_image
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES})


add_executable(read_gimbal src/gimbal/read_gimbal.cpp)
target_link_libraries(read_gimbal
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES})

add_executable(write_gimbal src/gimbal/write_gimbal_angle.cpp)
target_link_libraries(write_gimbal
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES})

add_executable(read_attitude src/sensors/read_attitude.cpp)
target_link_libraries(read_attitude ${catkin_LIBRARIES})

add_executable(read_imu src/sensors/read_imu.cpp)
target_link_libraries(read_imu ${catkin_LIBRARIES})

add_executable(read_vopositon src/sensors/read_voposition.cpp)
target_link_libraries(read_vopositon ${catkin_LIBRARIES})

add_executable(read_imu_gimbal src/sensors/read_gimbal_imu.cpp)
target_link_libraries(read_imu_gimbal ${catkin_LIBRARIES})