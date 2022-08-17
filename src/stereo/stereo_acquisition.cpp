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

void callback(const sensor_msgs::ImageConstPtr &image_R,
              const sensor_msgs::ImageConstPtr &image_L) {

    cv_bridge::CvImagePtr cv_ptr_R;
    cv_bridge::CvImagePtr cv_ptr_L;

    // Renaming images to the correct format.
    std::string img_counter = "";
    if (counter < 10) {
        img_counter = "000";
    } else if (counter > 9 && counter < 100) {
        img_counter = "00";
    } else if (counter < 99 && counter > 1000){
        img_counter = "0";
    }


    namespace enc = sensor_msgs::image_encodings;

    try {
        cv_ptr_R = cv_bridge::toCvCopy(image_R, sensor_msgs::image_encodings::BGR8);
        cv_ptr_L = cv_bridge::toCvCopy(image_L, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    std::stringstream writeR;
    std::stringstream writeL;
    writeR << "R" << img_counter << counter << ".png";
    writeL << "L" << img_counter << counter << ".png";
    cv::imwrite(writeL.str(), cv_ptr_L->image);
    cv::imwrite(writeR.str(), cv_ptr_R->image);
    std::cout << "Aqcuired Image -" << counter << std::endl;

    std::cout << "Press Enter to Continue";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');

    ++counter;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "stereo_thread");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> image_sub_R(nh, "/dji_osdk_ros/stereo_vga_front_right_images", 1);
    message_filters::Subscriber<sensor_msgs::Image> image_sub_L(nh, "/dji_osdk_ros/stereo_vga_front_left_images", 1);


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), image_sub_R, image_sub_L);
    sync.registerCallback(boost::bind(&callback, _1, _2));
/*
    while(ros::ok()) {
        ros::spinOnce();
    }*/
    ros::spin();
    return 0;
}
