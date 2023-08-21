#pragma once

#include <cmath>

#include <pcl/point_types.h>

typedef pcl::PointXYZI PointType;

inline double rad2deg(double radians)
{
    return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees)
{
    return degrees * M_PI / 180.0;
}

Eigen::Vector3f tf2euler(Eigen::Matrix4f tf)
{
    double roll = atan2(tf(2, 1), tf(2, 2));
    double pitch = atan2(-tf(2, 0), sqrt(pow(tf(2, 1), 2) + pow(tf(2, 2), 2)));
    double yaw = atan2(tf(1, 0), tf(0, 0));
    Eigen::Vector3f theta;
    theta << roll, pitch, yaw;
    return theta;
}

Eigen::Matrix4f euler2tf(Eigen::Vector3f theta)
{
    // Calculate rotation about x axis
    Eigen::Matrix4f R_x;
    Eigen::Matrix4f R_y;
    Eigen::Matrix4f R_z;

    R_x << 1, 0, 0, 0,
        0, cos(theta(0)), -sin(theta(0)), 0,
        0, sin(theta(0)), cos(theta(0)), 0,
        0, 0, 0, 1;

    R_y << cos(theta(1)), 0, sin(theta(1)), 0,
        0, 1, 0, 0,
        -sin(theta(1)), 0, cos(theta(1)), 0,
        0, 0, 0, 1;

    R_z << cos(theta(2)), -sin(theta(2)), 0, 0,
        sin(theta(2)), cos(theta(2)), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    Eigen::Matrix4f tf = R_z * R_y * R_x;

    return tf;

}