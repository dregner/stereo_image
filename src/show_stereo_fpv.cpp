#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

using namespace sensor_msgs;

using namespace message_filters;


/// Variavel para leitura GPS RTK


double RadToDeg = 180 / M_PI;


static int counter = 1;

std::string decimal(int r);

static std::ofstream images_file;

void callback(const sensor_msgs::ImageConstPtr &image_R,
              const sensor_msgs::ImageConstPtr &image_L,
              const sensor_msgs::ImageConstPtr &image_fpv) {

    cv_bridge::CvImagePtr cv_ptr_R;
    cv_bridge::CvImagePtr cv_ptr_L;
    cv_bridge::CvImagePtr cv_ptr_FPV;


    namespace enc = sensor_msgs::image_encodings;

    try {
        cv_ptr_R = cv_bridge::toCvCopy(image_R, sensor_msgs::image_encodings::BGR8);
        cv_ptr_L = cv_bridge::toCvCopy(image_L, sensor_msgs::image_encodings::BGR8);
        cv_ptr_FPV = cv_bridge::toCvCopy(image_fpv, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
   cv::imshow("fpv",cv_ptr_FPV->image);
   cv::waitKey(1);

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "show_fpv");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> image_sub_R(nh, "/dji_osdk_ros/stereo_vga_front_right_images", 1);
    message_filters::Subscriber<sensor_msgs::Image> image_sub_L(nh, "/dji_osdk_ros/stereo_vga_front_left_images", 1);
    message_filters::Subscriber<sensor_msgs::Image> image_sub_FPV(nh, "/dji_osdk_ros/fpv_camera_images", 1);


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), image_sub_R, image_sub_L, image_sub_FPV);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));
/*
    while(ros::ok()) {
        ros::spinOnce();
    }*/
    ros::spin();
    return 0;
}
