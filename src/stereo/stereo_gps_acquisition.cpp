#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <fstream>
#include <image_transport/image_transport.h>


using namespace sensor_msgs;

using namespace message_filters;


/// Variavel para leitura GPS RTK


double RadToDeg = 180 / M_PI;



static int counter_R = 0;
static int counter_L = 0;
float long_GPS;
float lat_GPS;
float alt_GPS;
float long_RTK;
float lat_RTK;
float alt_RTK;

std::string decimal(int r);

static std::ofstream images_file;

void callback(const ImageConstPtr &image_R,
              const ImageConstPtr &image_L,
              const NavSatFixConstPtr &pose_GPS,
              const NavSatFixConstPtr &pose_RTK) {

    long_GPS = pose_GPS->longitude;
    lat_GPS = pose_GPS->latitude;
    alt_GPS = pose_GPS->altitude;

    long_RTK = pose_RTK->longitude;
    lat_RTK = pose_RTK->latitude;
    alt_RTK = pose_RTK->altitude;

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
    if (images_file.is_open()) {
        images_file << writeR.str() << "\t" << writeL.str()
                    << "\t" << long_GPS << "\t" << lat_GPS << "\t" << alt_GPS
                    << "\t" << long_RTK << "\t" << lat_RTK << "\t" << alt_RTK <<  "\n";
    }
    ++counter_R;
    ++counter_L;

    std::cerr << "\n Saved Imgs:  [" <<  writeR.str() << "] [" << writeL.str() << "]\n";
    //std::cerr << "\n Saved Img:  [" <<  writeL.str() << "]\n";
    ROS_INFO("GPS: %f %f %f", long_GPS, lat_GPS, alt_GPS);
    ROS_INFO("RTK: %f %f %f", long_RTK, lat_RTK, alt_RTK);
    sleep(5);

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "stereo_thread");
    ros::NodeHandle nh;

    message_filters::Subscriber<Image> image_sub_R(nh, "/stereo/right/image_raw", 1);
    message_filters::Subscriber<Image> image_sub_L(nh, "/stereo/left/image_raw", 1);
    message_filters::Subscriber<NavSatFix> gps_pose(nh, "/dji_sdk/gps_position", 1);
    message_filters::Subscriber<NavSatFix> rtk_pose(nh, "/dji_sdk/rtk_position", 1);

    images_file.open("stereo_images.txt");


    typedef sync_policies::ApproximateTime<Image, Image, NavSatFix, NavSatFix> MySyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), image_sub_R, image_sub_L, gps_pose, rtk_pose);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

    while(ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
