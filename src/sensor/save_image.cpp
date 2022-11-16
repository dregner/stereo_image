#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

static int counter = 1;

std::string decimal(int r);

void callback(const sensor_msgs::Image::ConstPtr &msg){


    cv_bridge::CvImagePtr cv_ptr_FPV;

    std::string img_counter = "";
    if (counter < 10) {
        img_counter = "000";
    } else if (counter > 9 && counter < 100) {
        img_counter = "00";
    } else if (counter < 99 && counter > 1000){
        img_counter = "0";
    }

    try{
        cv_ptr_FPV = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    std::stringstream writeFPV;
    writeFPV << "disparity" << img_counter << counter << ".png";
    cv::imwrite(writeFPV.str(), cv_ptr_FPV->image);

    std::cout << "Aqcuired Image - " << counter << std::endl;

    ++counter;
    ros::Duration(0.5).sleep();

}

int main(int argc, char **argv) {


    ros::init(argc, argv, "camera_image_node");

    ros::NodeHandle nh;

    ros::Subscriber sub_imu = nh.subscribe("/stereo_depth_perception/disparity_front_left_image", 1, callback);

    ros::spin();
    return 0;
}
