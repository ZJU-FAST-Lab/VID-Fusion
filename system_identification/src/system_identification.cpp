#include "system_identification.h"
#include "conversion.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <queue>

#include <mav_msgs/Actuators.h>

using namespace std;
using namespace Eigen;

double mass = 1.1618;   
double gravity = 9.7936;  //Hangzhou
double mg;

//system parameters
Vector4d Tau, Tau_ls, Tau_ls_k;
double Tau_all, Tau_all_ls, sum_xi_all, sum_xi2_all, sum_xiyi_all, sum_yi_all;
Vector4d X_Tau(Vector4d(1.0e-7,1.0e-7,1.0e-7,1.0e-7));  //RLS
Matrix4d P(1.0 * Matrix4d::Identity()), Q(1.0e-19 * Matrix4d::Identity()), R(1.6e-18 * Matrix4d::Identity());
bool init = false;

//Least squares  cT = sum(mg*xi)/sum(xi^2)    xi = sum(wi^2)
Vector4d sum_xi(0,0,0,0), sum_xi2(0,0,0,0), sum_xiyi(0,0,0,0), sum_yi(0,0,0,0);
int counter = 0;
int counter_all = 0;
#define N 1000

void rotor_callback(const mav_msgs::Actuators::ConstPtr &msg)
{
    nav_msgs::Odometry Taupub;
    Taupub.header = msg->header;

    std::vector<double> RotorSpeed = msg->angular_velocities;

    //LS
    Vector4d w_2(msg->angular_velocities.at(0) * msg->angular_velocities.at(0),
                 msg->angular_velocities.at(1) * msg->angular_velocities.at(1),
                 msg->angular_velocities.at(2) * msg->angular_velocities.at(2),
                 msg->angular_velocities.at(3) * msg->angular_velocities.at(3));
    if(w_2(0) > 0 && w_2(1) > 0 && w_2(1) > 0 && w_2(3) > 0)
    {
        Tau(0) = 0.25*mg/w_2(0);
        Tau(1) = 0.25*mg/w_2(1);
        Tau(2) = 0.25*mg/w_2(2);
        Tau(3) = 0.25*mg/w_2(3);

        counter++;

        sum_xi += Vector4d(counter, counter, counter, counter);
        sum_xi2 += Vector4d(counter*counter, counter*counter, counter*counter, counter*counter);
        sum_xiyi += Vector4d(counter*Tau(0), counter*Tau(1), counter*Tau(2), counter*Tau(3));
        sum_yi += Tau;
    }

    #if 1 // using same Tau
    double W_2_sum = w_2.sum();
    if(W_2_sum > 0)
    {
        Tau_all = mg / W_2_sum;
        counter_all++;

        sum_xi_all += counter_all;
        sum_xi2_all += counter_all*counter_all;
        sum_xiyi_all += counter_all*Tau_all;
        sum_yi_all += Tau_all;
    }
    #endif
    
    // sum_xi += w_2;
    // sum_xi2 += Vector4d(w_2(0)*w_2(0), w_2(1)*w_2(1), w_2(2)*w_2(2), w_2(3)*w_2(3));
    
    //LS by N points
    if(counter >= N)
    {
        for(int i = 0; i < 4; i++)
        {
            double den = sum_xi(i)*sum_xi(i) - N*sum_xi2(i);
            Tau_ls_k(i) = (sum_xi(i)*sum_yi(i) - N*sum_xiyi(i)) / den;
            Tau_ls(i) = (sum_xi(i)*sum_xiyi(i) - sum_xi2(i)*sum_yi(i)) / den;
        }

        counter = 0;
        sum_xi = Vector4d(0,0,0,0);
        sum_xi2 = Vector4d(0,0,0,0);
        sum_xiyi = Vector4d(0,0,0,0);
        sum_yi = Vector4d(0,0,0,0);
        
        double den_all = sum_xi_all*sum_xi_all - N*sum_xi2_all;
        Tau_all_ls = (sum_xi_all*sum_xiyi_all - sum_xi2_all*sum_yi_all) / den_all;
        counter_all = 0;
        sum_xi_all = 0;
        sum_xi2_all = 0;
        sum_xiyi_all = 0;
        sum_yi_all = 0;

        // cout << "Tau_k: " << Tau_ls_k(0) << " " << Tau_ls_k(1) << " " << Tau_ls_k(2) << " " << Tau_ls_k(3) << endl;
        // cout << "Tau: " << Tau_ls(0) << ", " << Tau_ls(1) << ", " << Tau_ls(2) << ", " << Tau_ls(3) << endl;
        // cout << "Tau_all: " << Tau_all_ls << endl;

        if(!init)
        {
            // X_Tau = Vector4d(Tau_all_ls, Tau_all_ls,Tau_all_ls,Tau_all_ls);
            init = true;
        }
    }

    //recursive Least Square
    // if(w_2(0) > 0 && w_2(1) > 0 && w_2(1) > 0 && w_2(3) > 0 && init)
    if(w_2(0) > 0 && w_2(1) > 0 && w_2(1) > 0 && w_2(3) > 0)
    {
        // NOTE: Hovering or moving slowly while identifying 

        //RLS with forgetting factor
        double numda = 0.9;  // forgetting factor

        Matrix4d Kt = P * ((numda*Matrix4d::Identity() + P).inverse());
        // cout << "Kt 0: "  << Kt(0,0) << endl;
        P = P - Kt * P;
        // cout << "    P0: " << P(0,0) << endl;
        P = P / numda;
        // cout << "Kt 0: "  << Kt(0,0) << "    P0: " << P(0,0) << endl;
        Vector4d measurement(0.25*mg/w_2(0), 0.25*mg/w_2(1), 0.25*mg/w_2(2), 0.25*mg/w_2(3));
        X_Tau = X_Tau + Kt * (measurement - X_Tau);

        // cout << "norm_E: " << P.norm() << endl;
        if(counter > N-2)
            cout << "Tau_RLS: " << X_Tau(0) << ", " << X_Tau(1) << ", " << X_Tau(2) << ", " << X_Tau(3) << endl;
         
    }

    Taupub.pose.pose.orientation.w = X_Tau(0);  //Thrust coefficient
    Taupub.pose.pose.orientation.x = X_Tau(1);
    Taupub.pose.pose.orientation.y = X_Tau(2);
    Taupub.pose.pose.orientation.z = X_Tau(3);

    Taupub.pose.covariance.at(0) = Tau_ls(0);
    Taupub.pose.covariance.at(1) = Tau_ls(1);
    Taupub.pose.covariance.at(2) = Tau_ls(2);
    Taupub.pose.covariance.at(3) = Tau_ls(3);

    Taupub.pose.pose.position.x = Tau_all_ls;

    Tau_pub.publish(Taupub);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "system_identification");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("/motor_speed", 100, rotor_callback, ros::TransportHints().tcpNoDelay());
    Tau_pub = n.advertise<nav_msgs::Odometry>("/Tau", 100); 

    n.getParam("mass", mass);
    mg = mass * gravity;

    ros::spin();
}
    