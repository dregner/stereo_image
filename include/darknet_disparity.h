//
// Created by vant3d on 19/12/22.
//

#ifndef STEREO_IMAGE_DARKNET_DISPARITY_H
#define STEREO_IMAGE_DARKNET_DISPARITY_H

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <opencv2/opencv.hpp>

class DarknetDisparity {
private:
    ros::NodeHandle nh;

    ros::Subscriber darknet_bb_sub;
    ros::Subscriber m210_disparity_sub;
    ros::Subscriber darknet_obj_sub;

    cv_bridge::CvImagePtr cv_disp;
    cv::Mat disp_img;
    cv::Point top_left;
    cv::Point bot_right;
    std::string object_to_track;
    int n_detected_obj = 0;
    int found_obj_darknet = 0;

public:
    DarknetDisparity();

    ~DarknetDisparity();

    void subscribing(ros::NodeHandle &nh);

    void darknet_cb(const darknet_ros_msgs::BoundingBoxesConstPtr &bb_msg);

    void disparity_cb(const sensor_msgs::ImageConstPtr &disp_msgs);

    void darknet_obj_count_cb(const darknet_ros_msgs::ObjectCount::ConstPtr &obj_msg);

    float calculate_distance();

    void show_disp_image();
};

#endif //STEREO_IMAGE_DARKNET_DISPARITY_H
