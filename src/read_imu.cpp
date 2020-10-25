//
// Created by daniel.regner on 14/09/2020.
//

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <ignition/math2/ignition/math/Pose3.hh>

#define RAD2DEG(RAD) ((RAD) * 180 / M_PI)

using namespace std;

void callback(const sensor_msgs::Imu::ConstPtr &msg){
    msg->orientation.x;
    ignition::math::Quaterniond rpy;
    rpy.Set(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

    cout << "R: " << RAD2DEG(rpy.Roll()) << endl;
    cout << "P: " << RAD2DEG(rpy.Pitch()) << endl;
    cout << "Y: " << RAD2DEG(rpy.Yaw()) << endl;
    cout << "OFFSET YAW:" << 90-RAD2DEG(rpy.Yaw()) << endl;
    cout << "\033[2J\033[1;1H";     // clear terminal



}

int main(int argc, char **argv) {


    ros::init(argc, argv, "sdk_imu_listener");

    ros::NodeHandle nh;

    ros::Subscriber sub_imu = nh.subscribe("/dji_sdk/imu", 1000l, callback);

    ros::spin();
    return 0;
}