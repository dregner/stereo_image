//
// Created by vant3d on 05/07/2020.
//

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <ignition/math2/ignition/math/Pose3.hh>


#define RAD2DEG(RAD) ((RAD) * (180.0) / (M_PI))
#define DEG2RAD(DEG) ((DEG) * (M_PI) / (180.0))

float g_roll, g_pitch, g_yaw;
float rpa_roll, rpa_pitch, rpa_yaw;
int i = 0;
bool first_time = false;

using namespace std;

class ReadAngles {
private:


    ros::NodeHandle nh;

    geometry_msgs::Vector3Stamped gimbal_angle;
    ros::Subscriber imu_angle_sub;
    ros::Subscriber gimbal_angle_sub;


public:
    ReadAngles() {

    gimbal_angle_sub = nh.subscribe<geometry_msgs::Vector3Stamped>
            ("dji_sdk/gimbal_angle", 10, &ReadAngles::gimbalAngleCallback, this);
    imu_angle_sub = nh.subscribe<sensor_msgs::Imu>
            ("dji_sdk/imu",10, &ReadAngles::read_imu, this);


    }


    ~ReadAngles() {}

    void read_imu(const sensor_msgs::Imu::ConstPtr &msg){
        msg->orientation.x;
        ignition::math::Quaterniond rpy;
        rpy.Set(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

        rpa_roll = RAD2DEG(rpy.Roll());
        rpa_pitch = RAD2DEG(rpy.Pitch());
        rpa_yaw = RAD2DEG(rpy.Yaw());

    }

    void gimbalAngleCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
        gimbal_angle = *msg;
        g_roll = gimbal_angle.vector.y;
        g_pitch = gimbal_angle.vector.x;
        g_yaw = gimbal_angle.vector.z;
        float offset_yaw = 90 - rpa_yaw;
        ROS_INFO("Gimbal angle: [ %f, %f, %f ] deg", g_roll, g_pitch, g_yaw);
        ROS_INFO("Gimbal offset yaw: %f", offset_yaw);
        ROS_INFO("Imu angle: [ %f, %f, %f ] deg", rpa_roll, rpa_pitch, rpa_yaw);
    }
};


int main(int argc, char **argv) {


    ros::init(argc, argv, "read_angles");
    ros::NodeHandle nh;

    ReadAngles gimbal_imu;


    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}



/*
int main(int argc, char **argv) {
    ros::init(argc, argv, "write_sdk_gimbal");
    ros::NodeHandle nh;

    // ROS stuff


    // Display interactive prompt
    cout << "| Available commands:" << endl;
    cout << "\t[a] Change moode" << endl;
    cout << "\t[b] Change roll position" << endl;
    cout << "\t[c] Change pitch position" << endl;
    cout << "\t[d] Change yaw position" << endl;
    char inputChar;
    cin >> inputChar;

    switch (inputChar) {
        case 'a':
            cout << "0 - Incremental or 1 - Absolute" << endl;
            cin >> mode;
            gimbal_angle_func(mode);
            break;
        case 'b':
            cout << "Roll value (deg): ";
            cin >> inputValue_r;
            gimbal_angle_func(inputValue_r);
            break;
        case 'c':
            cout << "Pitch value (deg): ";
            cin >> inputValue_p;
            gimbal_angle_func(inputValue_p);
            break;
        case 'd':
            cout << "Yaw value (deg): ";
            cin >> inputValue_y;
            gimbal_angle_func(inputValue_y);
            break;
        default:
            break;
    }
}*/
