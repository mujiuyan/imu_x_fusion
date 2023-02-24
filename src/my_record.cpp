#include <ros/ros.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <Eigen/Dense>
#include <sbg_driver/SbgEkfNav.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <iostream>

Eigen::Vector3d origin;
std::ofstream rawDataFile;
std::ofstream ekfDataFile;
std::ofstream eskfDataFile;
GeographicLib::LocalCartesian local_cartesian;

void rawDataCB(sensor_msgs::NavSatFixConstPtr rawDataPtr) {
    Eigen::Vector3d point;
    local_cartesian.Forward(rawDataPtr->latitude, rawDataPtr->longitude, rawDataPtr->altitude, point(0), point(1), point(2));
    rawDataFile << ros::Time::now() << "," << point(0) << "," << point(1) << "," << point(2) << std::endl;
}

void sbgDataCB(sbg_driver::SbgEkfNavConstPtr ekfDataPtr) {
    Eigen::Vector3d point;
    local_cartesian.Forward(ekfDataPtr->latitude, ekfDataPtr->longitude, ekfDataPtr->altitude, point(0), point(1), point(2));
    ekfDataFile << ros::Time::now() << "," << point(0) << "," << point(1) << "," << point(2) << std::endl;
}

void eskfDataCB(nav_msgs::OdometryConstPtr eskfDataPtr) {
    eskfDataFile << ros::Time::now() << "," << eskfDataPtr->pose.pose.position.x << "," << eskfDataPtr->pose.pose.position.y << "," << eskfDataPtr->pose.pose.position.z << std::endl;
}

int main(int argc, char** argv) {
    origin << 33.711130499999996, 110.24378, 544.65;
    local_cartesian.Reset(origin(0), origin(1), origin(2));
    rawDataFile.open("raw_data.csv");
    ekfDataFile.open("ekf_data.csv");
    eskfDataFile.open("eskf_data.csv");
    ros::init(argc, argv, "my_record");
    ros::NodeHandle nh;
    ros::Subscriber rawDataSub;
    ros::Subscriber ekfDataSub;
    ros::Subscriber eskfDataSub;
    rawDataSub = nh.subscribe<sensor_msgs::NavSatFix>("/imu/nav_sat_fix", 30, rawDataCB);
    ekfDataSub = nh.subscribe<sbg_driver::SbgEkfNav>("/sbg/ekf_nav", 30, sbgDataCB);
    eskfDataSub = nh.subscribe<nav_msgs::Odometry>("/odom_est", 30, eskfDataCB);
    ros::spin();
}