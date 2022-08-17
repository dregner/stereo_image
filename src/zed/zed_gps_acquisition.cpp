#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <fstream>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/QuaternionStamped.h>

/// Variavel para leitura GPS RTK


double RadToDeg = 180 / M_PI;



static int counter = 0;

float long_GPS;
float lat_GPS;
float alt_GPS;
float long_RTK;
float lat_RTK;
float alt_RTK;

std::string decimal(int r);

static std::ofstream images_file;

void callback(const sensor_msgs::ImageConstPtr &image_R,
              const sensor_msgs::ImageConstPtr &image_L,
              const sensor_msgs::NavSatFixConstPtr &pose_GPS,
              const geometry_msgs::QuaternionStampedConstPtr &atti) {

    long_GPS = pose_GPS->longitude;
    lat_GPS = pose_GPS->latitude;
    alt_GPS = pose_GPS->altitude;



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

    writeR << "zed_R" << counter << ".png";
    writeL << "zed_L" << counter << ".png";
    cv::imwrite(writeR.str(), cv_ptr_R->image);
    cv::imwrite(writeL.str(), cv_ptr_L->image);
    if (images_file.is_open()) {
        images_file << writeR.str() << "\t" << writeL.str()
                    << "\t" << std::setprecision(10) << long_GPS << "\t" << std::setprecision(10) << lat_GPS << "\t" << std::setprecision(10) << alt_GPS
                    << "\t" << atti->quaternion.w << "\t" << atti->quaternion.x << "\t" << atti->quaternion.y << "\t" << atti->quaternion.z
                    <<"\n";
    }
    ++counter;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "zed_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> image_sub_R(nh, "/zed2/zed_node/right/image_rect_color", 10);
    message_filters::Subscriber<sensor_msgs::Image> image_sub_L(nh, "/zed2/zed_node/left/image_rect_color", 10);
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_pose(nh, "/dji_sdk/gps_position", 100);
    message_filters::Subscriber<geometry_msgs::QuaternionStamped> atti_sub(nh, "/dji_sdk/attitude", 100);

//    message_filters::Subscriber<sensor_msgs::NavSatFix> rtk_pose(nh, "/dji_sdk/rtk_position", 1);

    images_file.open("zed_stereo.txt");


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::NavSatFix, geometry_msgs::QuaternionStamped> MySyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000), image_sub_R, image_sub_L, gps_pose, atti_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3,_4));

    while(ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
