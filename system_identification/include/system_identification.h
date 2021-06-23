#ifndef SYSTEM_IDENTIFICATION_H_
#define SYSTEM_IDENTIFICATION_H_

#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace Eigen;
//! @brief Common variables
const double PI = 3.141592653589793;
const double TAU = 6.283185307179587;

ros::Publisher Tau_pub;


#endif

