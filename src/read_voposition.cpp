//
// Created by vant3d on 08/10/2020.
//
#include <ros/ros.h>
#include <dji_sdk/VOPosition.h>
#include <fstream>

#define RAD2DEG(RAD) ((RAD) * 180 / M_PI)

using namespace std;
static std::ofstream vo_position;
double start;
float x, y, z;

void save_txt() {
    if (vo_position.is_open()) {
        double time_step = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec - start;
        vo_position << time_step << "\t" << x << "\t" << y << "\t" << z  << "\n";
    }
}

void callback(const dji_sdk::VOPosition::ConstPtr &msg){
    x = msg->x;
    y = msg->y;
    z = msg->z;

    cout << "X: " << x << endl;
    cout << "Y: " << y << endl;
    cout << "Z: " << z << endl;
    cout << "\033[2J\033[1;1H";     // clear terminal
    save_txt();
}



int main(int argc, char **argv) {


    ros::init(argc, argv, "vo_position_reader");

    ros::NodeHandle nh;

    vo_position.open("vo_position.txt");
    start = ros::Time::now().nsec * 1e-9 + ros::Time::now().sec;
    ros::Subscriber sub_imu = nh.subscribe("/dji_sdk/vo_position", 10, callback);

    ros::spin();
    return 0;
}