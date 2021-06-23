#include "dynamic_model.h"
#include "conversion.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <queue>

#include <visualization_msgs/Marker.h>

#include <mav_msgs/Actuators.h>
#include <quadrotor_msgs/ControlCommand.h>

using namespace std;
using namespace Eigen;

Matrix3d Ri_b;     
Vector3d ti_b;  //  imu to body

Vector3d f_ext(0, 0, 0);
Vector3d acc_filtered, attitude_ekf;
Vector3d position;

ros::Time rotors_time;

//system parameters
// double mass = 1.1062;//vid-dataset: tarj8_with_gt 
double mass = 1.1381;  //vid-dataset: line_with_force_gt

// Vector4d Tau_ls(1.85153e-08, 2.01043e-08, 1.96869e-08, 2.22119e-08);
// double Tau_all_ls(2.00822e-08); //vid-dataset: tarj8_with_gt
Vector4d Tau_ls(1.59792e-08, 2.09762e-08, 1.78088e-08, 2.40334e-08);
double Tau_all_ls(1.91243e-08); //vid-dataset: line_with_force_gt

double gravity = 9.7936;  //Hzngzhou
double mg = mass * gravity;

void rotor_callback(const mav_msgs::Actuators::ConstPtr &msg)
{
    Vector4d w_2(msg->angular_velocities.at(0) * msg->angular_velocities.at(0),
                 msg->angular_velocities.at(1) * msg->angular_velocities.at(1),
                 msg->angular_velocities.at(2) * msg->angular_velocities.at(2),
                 msg->angular_velocities.at(3) * msg->angular_velocities.at(3));

    double Thrust = Tau_ls.transpose() * w_2;

    quadrotor_msgs::ControlCommand f_thrust_b;
    // f_thrust_b.collective_thrust = (Thrust-mg) / mass;
    f_thrust_b.collective_thrust = Thrust / mass;
    f_thrust_b.header = msg->header;
    f_thrust_pub.publish(f_thrust_b);

    double Thrust_sameTau = Tau_all_ls * w_2.sum();
    f_thrust_b.collective_thrust = Thrust_sameTau / mass;
    f_thrust_sameTau_pub.publish(f_thrust_b);

    rotors_time = msg->header.stamp;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamic_model");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("/dynamic_model/motor_speed", 100, rotor_callback, ros::TransportHints().tcpNoDelay()); //identification
    f_thrust_pub = n.advertise<quadrotor_msgs::ControlCommand>("/dynamic_model/thrust", 1000, true);
    f_thrust_sameTau_pub = n.advertise<quadrotor_msgs::ControlCommand>("/dynamic_model/thrust_sameTau", 1000, true);

    // Ri_b = Quaterniond(0.9987503, 0, 0.0499792, 0).toRotationMatrix(); 
    // ti_b << 0.1, 0.0, -0.03;

    // n.getParam("mass", mass);

    std::vector<double> Tau_ls_temp;
    n.getParam("mass", mass);
    n.getParam("thrust_coefficient", Tau_ls_temp);
    Tau_ls = Vector4d(Tau_ls_temp[0], Tau_ls_temp[1], Tau_ls_temp[2], Tau_ls_temp[3]);
    cout << "mass: " << mass << " thrust_coefficient: " << Tau_ls << endl;

    ROS_INFO("dynamic_model;");
    ros::spin();
}
