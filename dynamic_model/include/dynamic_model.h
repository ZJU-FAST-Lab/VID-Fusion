#ifndef DYNAMIC_MODEL_H_
#define DYNAMIC_MODEL_H_

#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>

//! @brief Common variables
const double PI = 3.141592653589793;
const double TAU = 6.283185307179587;

ros::Publisher f_ext_pub, f_ext_show_pub, f_ext_show_pub1, f_ext_show_pub2;
ros::Publisher f_thrust_pub, f_thrust_sameTau_pub;

void visualizeForce(Eigen::Vector3d position, Eigen::Vector3d Force, int id, Eigen::Vector3d color, ros::Publisher pub_force);

#endif

