//
// Created by vant3d on 14/09/2020.
//
#include <geometry_msgs/QuaternionStamped.h>
#include <ros/ros.h>
#include <ignition/math2/ignition/math/Pose3.hh>


#define RAD2DEG(RAD) ((RAD) * 180 / M_PI)

using namespace std;

void callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg){
    ignition::math::Quaterniond rpy;
    rpy.Set(msg->quaternion.w, msg->quaternion.x, msg->quaternion.y, msg->quaternion.z);

    cout << "R: " << RAD2DEG(rpy.Roll()) << endl;
    cout << "P: " << RAD2DEG(rpy.Pitch()) << endl;
    cout << "Y: " << RAD2DEG(rpy.Yaw()) << endl;
    cout << "OFFSET YAW:" << 90-RAD2DEG(rpy.Yaw()) << endl;
    cout << "\033[2J\033[1;1H";     // clear terminal



}

int main(int argc, char **argv) {


    ros::init(argc, argv, "sdk_attitude_listener");

    ros::NodeHandle nh;

    ros::Subscriber sub_imu = nh.subscribe("/dji_sdk/attitude", 1000l, callback);

    ros::spin();
    return 0;
}

