#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <image_transport/image_transport.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <ignition/math/Pose3.hh>



/// Variavel para leitura GPS RTK


double RadToDeg = 180 / M_PI;


static int counter = 1;

std::string decimal(int r);
ignition::math::Quaterniond dji_imu_eu;
ignition::math::Quaterniond zed_imu_eu;

static std::ofstream images_file;

void callback(const sensor_msgs::ImageConstPtr &image_R,
              const sensor_msgs::ImageConstPtr &image_L,
              const sensor_msgs::NavSatFixConstPtr &gps,
              const sensor_msgs::NavSatFixConstPtr &rtk,
              const sensor_msgs::ImuConstPtr &zed_imu,
              const sensor_msgs::ImuConstPtr &dji_imu) {

    dji_imu_eu.Set(dji_imu->orientation.w, dji_imu->orientation.x, dji_imu->orientation.y, dji_imu->orientation.z);
    zed_imu_eu.Set(zed_imu->orientation.w, zed_imu->orientation.x, zed_imu->orientation.y, zed_imu->orientation.z);

    cv_bridge::CvImagePtr cv_ptr_R;
    cv_bridge::CvImagePtr cv_ptr_L;
    // Renaming images to the correct format.
    std::string img_counter = "";
    if (counter < 10) {
        img_counter = "000";
    } else if (counter > 9 && counter < 100) {
        img_counter = "00";
    } else if (counter < 99 && counter > 1000) {
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
    if (images_file.is_open()) {
        images_file << counter
                    << "\t" << gps->longitude << "\t" << gps->latitude << "\t" << gps->altitude
                    << "\t" << rtk->longitude << "\t" << rtk->latitude << "\t" << rtk->altitude
                    << "\t" <<  dji_imu_eu.Roll() << "\t" << dji_imu_eu.Pitch()  << "\t" << dji_imu_eu.Yaw()
                    << "\t" <<  zed_imu_eu.Roll() << "\t" << zed_imu_eu.Pitch()  << "\t" << zed_imu_eu.Yaw() << "\n";
    }
    ROS_INFO("GPS: %f %f %f", gps->longitude, gps->latitude, gps->altitude);
        ROS_INFO("RTK: %f %f %f", rtk->longitude, rtk->latitude, rtk->altitude);
    ROS_INFO("DJI IMU: %f %f %f", dji_imu_eu.Roll(), dji_imu_eu.Pitch(), dji_imu_eu.Yaw());
    ROS_INFO("ZED IMU: %f %f %f", zed_imu_eu.Roll(), zed_imu_eu.Pitch(), zed_imu_eu.Yaw());
    sleep(1);
//    std::cout << "Press Enter to Continue";
//    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    ++counter;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "zed_gps_imu_thread");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> image_sub_R(nh, "/stereo/right/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::Image> image_sub_L(nh, "/stereo/left/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub(nh, "/dji_sdk/gps_position", 1);
    message_filters::Subscriber<sensor_msgs::NavSatFix> rtk_sub(nh, "/dji_sdk/rtk_position", 1);
    message_filters::Subscriber<sensor_msgs::Imu> zed_imu_sub(nh, "/zed2/imu", 1);
    message_filters::Subscriber<sensor_msgs::Imu> dji_imu_sub(nh, "/dji_sdk/imu", 1);


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
            sensor_msgs::NavSatFix, sensor_msgs::NavSatFix,
            sensor_msgs::Imu, sensor_msgs::Imu> MySyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub_R, image_sub_L,
                                                     gps_sub, rtk_sub,
                                                     zed_imu_sub, dji_imu_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5, _6));
    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
