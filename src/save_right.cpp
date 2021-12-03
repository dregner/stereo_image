#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <image_transport/image_transport.h>


using namespace sensor_msgs;

using namespace message_filters;


/// Variavel para leitura GPS RTK


double RadToDeg = 180 / M_PI;



static int counter = 1;

std::string decimal(int r);

static std::ofstream images_file;

void callback(const sensor_msgs::ImageConstPtr &image) {

    cv_bridge::CvImagePtr cv_ptr;


    try {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    std::stringstream writeR;
    std::stringstream writeL;
    writeR << "image_R" << counter << ".png";
    cv::imwrite(writeR.str(), cv_ptr->image);
    std::cout << "Aqcuired Image R -" << counter << std::endl;

    ++counter;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "save_right");
    ros::NodeHandle nh;

    ros::Subscriber image_sub_R = nh.subscribe("/stereo/right/image_raw", 1, callback);

/*
    while(ros::ok()) {
        ros::spinOnce();
    }*/
    ros::spin();
    return 0;
}
