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



static int counter_R = 0;
static int counter_L = 0;


std::string decimal(int r);

static std::ofstream images_file;

void callback(const ImageConstPtr &image_R,
              const ImageConstPtr &image_L) {

    cv_bridge::CvImagePtr cv_ptr_R;
    cv_bridge::CvImagePtr cv_ptr_L;

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
    writeR << "image_R" << counter_R << ".png";
    writeL << "image_L" << counter_L << ".png";
    cv::imwrite(writeR.str(), cv_ptr_R->image);
    cv::imwrite(writeL.str(), cv_ptr_L->image);

    ++counter_R;
    ++counter_L;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "stereo_thread");
    ros::NodeHandle nh;

    message_filters::Subscriber<Image> image_sub_R(nh, "/dji_sdk/stereo_vga_front_right_images", 1);
    message_filters::Subscriber<Image> image_sub_L(nh, "/dji_sdk/stereo_vga_front_left_images", 1);


    typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), image_sub_R, image_sub_L);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    while(ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
