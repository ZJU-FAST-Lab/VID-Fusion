#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"

#include <mav_msgs/Actuators.h>


Estimator estimator;

std::condition_variable con;
double current_time = -1;

using controlMsgType = quadrotor_msgs::ControlCommand::ConstPtr;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<controlMsgType> control_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::PointCloudConstPtr> relo_buf;
bool start_recording = true;

int sum_of_wait = 0;

std::mutex m_buf;
std::mutex m_state;
std::mutex m_estimator;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
Eigen::Vector3d tmp_force;
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0;

bool beginning = true;
double last_Fz = 0;
Eigen::Vector3d last_accel(0.0,0.0,0.0);
Eigen::Vector3d last_angvel(0.0,0.0,0.0);
double last_control_t = 0;

Quaterniond q_gt,q_gt0;
Vector3d p_gt, p_gt0;
bool first_pose_in_world = false;

double Thrust_n = 0;
double Thrust_n_0 = 0;

void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    double un_thrust = 0.5 * (Thrust_n + Thrust_n_0);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
    Thrust_n_0 = Thrust_n;

    //raw data sub to show noise limiting
    // tmp_force = Eigen::Vector3d(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z) - tmp_Ba - Thrust_n * Eigen::Vector3d(0,0,1);
    // tmp_force = tmp_Q.toRotationMatrix() * tmp_force;

    //for propagate a higher rate of external force for SO3 controller
    tmp_force = un_acc + estimator.g - tmp_Q.toRotationMatrix() * un_thrust * Eigen::Vector3d(0,0,1);
}

void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());
}


std::vector<std::pair<std::pair<std::vector<sensor_msgs::ImuConstPtr>,std::vector<controlMsgType>>, sensor_msgs::PointCloudConstPtr>>
getMeasurements()
{
    std::vector<std::pair<std::pair<std::vector<sensor_msgs::ImuConstPtr>,std::vector<controlMsgType>>, sensor_msgs::PointCloudConstPtr>> measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            //ROS_WARN("wait for imu or control, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }

        if (USE_VID)
        {
            if (control_buf.empty() || !(control_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td))
                return measurements;
            
            // Ensure we have messages from slower frequency sensor filled first in their buffers
            if (beginning)
            {
                if (PROCESS_AT_CONTROL_INPUT_RATE)
                {
                    if (control_buf.front()->header.stamp.toSec() < imu_buf.front()->header.stamp.toSec())
                    {
                        // For control rate higher than imu rate, let IMU come first and then control msgs
                        ROS_WARN_STREAM("throw CONTROLS with ts, only should happen at the beginning " << control_buf.front()->header.stamp.toSec());
                        control_buf.pop();
                        continue;
                    }
                }
                else
                {
                    if (imu_buf.front()->header.stamp.toSec() < control_buf.front()->header.stamp.toSec())
                    {
                        ROS_WARN_STREAM("throw IMUs with ts, only should happen at the beginning " << imu_buf.front()->header.stamp.toSec());
                        imu_buf.pop();
                        continue;
                    }
                }
            }
            
            beginning =  false;

        }
        
        if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
 
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());

        std::vector<controlMsgType> controls;
        if (USE_VID)
        {
            while (control_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
            {
                controls.emplace_back(control_buf.front());
                control_buf.pop();
            }
            controls.emplace_back(control_buf.front());

            if (controls.empty())
                ROS_WARN("no control input between two images");
        }
        
        if (IMUs.empty())
            ROS_WARN("no imu between two images");
        measurements.emplace_back(std::make_pair(IMUs, controls), img_msg);
    }
    return measurements;
}

Matrix3d quaternion2mat(Quaterniond q)
{
  Matrix3d m;
  double a = q.w(), b = q.x(), c = q.y(), d = q.z();
  m << a*a + b*b - c*c - d*d, 2*(b*c - a*d), 2*(b*d+a*c),
       2*(b*c+a*d), a*a - b*b + c*c - d*d, 2*(c*d - a*b),
       2*(b*d - a*c), 2*(c*d+a*b), a*a-b*b - c*c + d*d;
  return m;
}

Vector3d mat2euler(Matrix3d m)
{ 
  double r = atan2(m(2, 1), m(2, 2));
  double p = asin(-m(2, 0));
  double y = atan2(m(1, 0), m(0, 0));
  Vector3d rpy(r, p, y);
  return rpy;
}

Vector3d quaternion2euler(Quaterniond q)
{
  return mat2euler(quaternion2mat(q));
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    last_imu_t = imu_msg->header.stamp.toSec();

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
        {
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, tmp_force, header);
            // cout << "s" << endl;
            if(!first_pose_in_world)
            {
                first_pose_in_world = true;
                q_gt0 = q_gt;
                p_gt0 = p_gt;
                // cout << "q_gt0: " << endl << quaternion2euler(q_gt0) << endl << "p_gt0: " << endl << p_gt0 << endl;
                // G = q_gt0.inverse()*Vector3d(0,0,9.8) ;
                // cout << "g0: " << G << endl;
            }
        }
    }
}

void control_inputs_callback(const controlMsgType& torque_thrust_msg)
{
    if (torque_thrust_msg->header.stamp.toSec() <= last_control_t)
    {
        // ROS_WARN("torque_thrust message in disorder!");
        // if (torque_thrust_msg->header.stamp.toSec() == last_control_t)
        //     ROS_WARN("skipping duplicate timestamped torque_thrust message!");
        return;
    }
    last_control_t = torque_thrust_msg->header.stamp.toSec();
    // cout << "control: " << torque_thrust_msg->collective_thrust << endl;
    Thrust_n = torque_thrust_msg->collective_thrust;
    
    m_buf.lock();
    control_buf.push(torque_thrust_msg);
    m_buf.unlock();
    con.notify_one();

}

void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }

    m_buf.lock();
    feature_buf.push(feature_msg); //one frame msg
    m_buf.unlock();
    con.notify_one();
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        while(!control_buf.empty())
            control_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        //beginning = true;
        last_imu_t = 0;
        last_control_t = 0;
    }
    return;
}

void relocalization_callback(const sensor_msgs::PointCloudConstPtr &points_msg)
{
    m_buf.lock();
    relo_buf.push(points_msg);
    m_buf.unlock();
}

void process_measurements_at_imu_rate(std::pair<std::pair<std::vector<sensor_msgs::ImuConstPtr>,std::vector<controlMsgType>>, sensor_msgs::PointCloudConstPtr> & measurement)
{
    auto img_msg = measurement.second;
    double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0, Fz = last_Fz;
    std::vector<sensor_msgs::ImuConstPtr>::iterator imu_it = measurement.first.first.begin();
    std::vector<controlMsgType>::iterator control_it = measurement.first.second.begin();

    double begin_t = (*imu_it)->header.stamp.toSec();
    double end_t;
    for (; imu_it!=measurement.first.first.end(); ++imu_it)
    {

        double t = (*imu_it)->header.stamp.toSec();
        end_t = t;
        double img_t = img_msg->header.stamp.toSec() + estimator.td;

        if (t <= img_t)
        { 
            if (current_time < 0)
                current_time = t;


            double dt = t - current_time;
            ROS_ASSERT(dt >= 0);
            current_time = t;
            

            dx = (*imu_it)->linear_acceleration.x;
            dy = (*imu_it)->linear_acceleration.y;
            dz = (*imu_it)->linear_acceleration.z;
            rx = (*imu_it)->angular_velocity.x;
            ry = (*imu_it)->angular_velocity.y;
            rz = (*imu_it)->angular_velocity.z;

            if (USE_VID)
            {
                if (control_it!=measurement.first.second.end())
                {
                    while ((control_it+1)!=measurement.first.second.end() && (*(control_it+1))->header.stamp.toSec() < current_time)
                    {
                        ++control_it;
                    }
 
                    Fz = SCALE_THRUST_INPUT * (*control_it)->collective_thrust;
                    ++control_it;
                }
            }
            

            estimator.processIMUandThrust(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz), Fz);
        }
        else
        {
            double dt_1 = img_t - current_time;
            double dt_2 = t - img_t;
            current_time = img_t;
            ROS_ASSERT(dt_1 >= 0);
            ROS_ASSERT(dt_2 >= 0);
            ROS_ASSERT(dt_1 + dt_2 > 0);
            double w1 = dt_2 / (dt_1 + dt_2);
            double w2 = dt_1 / (dt_1 + dt_2);
            dx = w1 * dx + w2 * (*imu_it)->linear_acceleration.x;
            dy = w1 * dy + w2 * (*imu_it)->linear_acceleration.y;
            dz = w1 * dz + w2 * (*imu_it)->linear_acceleration.z;
            rx = w1 * rx + w2 * (*imu_it)->angular_velocity.x;
            ry = w1 * ry + w2 * (*imu_it)->angular_velocity.y;
            rz = w1 * rz + w2 * (*imu_it)->angular_velocity.z;

            if (USE_VID)
            {
                if (control_it!=measurement.first.second.end())
                {
                    while ((control_it+1)!=measurement.first.second.end() && (*(control_it+1))->header.stamp.toSec() < img_t)
                    {
                        ++control_it;
                    }

                    Fz = SCALE_THRUST_INPUT * (*control_it)->collective_thrust;
                    ++control_it;
                }
            }

            estimator.processIMUandThrust(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz), Fz);
        }

        last_Fz = Fz;
    }
    if (USE_VID)
    {
        //After processing imu data between 2 adjacent keyframes 
        estimator.sum_dt_preintegration_force += end_t - begin_t;  //accumulate time between 2 adjacent keyframes
        // cout << "sum_dt_: " << estimator.sum_dt_preintegration_force << endl;
        //NOTE Ps Vs... update in processIMUandThrust propagation;  Fexts update in process_measurements_at_imu_rate as an average force
        estimator.Fexts[estimator.frame_count] = estimator.Fexts_temp/ estimator.sum_dt_preintegration_force;  
        //NOTE optimization init value
        // cout << "estimator.Fexts[" << estimator.frame_count << "]:  "  << estimator.Fexts[estimator.frame_count] << endl;
        // cout << "sliding_window: " << endl;
        //Fz_avg for average Fz between Windows
        estimator.Fz_avg_vector.at(estimator.frame_count) = estimator.Fz_avg / estimator.sum_dt_preintegration_force;
        
        // for(int i=0; i < estimator.frame_count; i++)
        // {
        //     cout << i << ": " << estimator.Fexts[i] << endl;
        // }
    }
}

#if 0  // processing in imu rate
void process_measurements_at_control_input_rate(std::pair<std::pair<std::vector<sensor_msgs::ImuConstPtr>,std::vector<controlMsgType>>, sensor_msgs::PointCloudConstPtr> & measurement)
{
    auto img_msg = measurement.second;
    double dx = last_accel.x(), dy = last_accel.y(), dz = last_accel.z(), rx = last_angvel.x(), ry = last_angvel.y(), rz = last_angvel.z(), Fz = 0.0;
    std::vector<sensor_msgs::ImuConstPtr>::iterator imu_it = measurement.first.first.begin();
    std::vector<controlMsgType>::iterator control_it = measurement.first.second.begin();

    for (; control_it!=measurement.first.second.end(); ++control_it)
    {

        double t = (*control_it)->header.stamp.toSec();
        double img_t = img_msg->header.stamp.toSec() + estimator.td;

        if (t <= img_t)
        { 
            if (current_time < 0)
                current_time = t;

            double dt = t - current_time;
            ROS_ASSERT(dt >= 0);
            current_time = t;

            Fz = SCALE_THRUST_INPUT * (*control_it)->collective_thrust;

            if (imu_it!=measurement.first.first.end())
            {
                while ((imu_it+1)!=measurement.first.first.end() && (*(imu_it+1))->header.stamp.toSec() < current_time) //while
                {
                    ++imu_it; // Skip imu messages to get to IMU message just before current time
                }
                
                // If there exists imu msg after current time, interpolate
                if ((imu_it+1)!=measurement.first.first.end())
                {
                    double dt_1 = current_time - (*imu_it)->header.stamp.toSec();
                    double dt_2 = (*(imu_it+1))->header.stamp.toSec() - current_time;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                                                   
                    dx = w1 * (*imu_it)->linear_acceleration.x + w2 * (*(imu_it+1))->linear_acceleration.x;
                    dy = w1 * (*imu_it)->linear_acceleration.y + w2 * (*(imu_it+1))->linear_acceleration.y;
                    dz = w1 * (*imu_it)->linear_acceleration.z + w2 * (*(imu_it+1))->linear_acceleration.z;
                    rx = w1 * (*imu_it)->angular_velocity.x + w2 * (*(imu_it+1))->angular_velocity.x;
                    ry = w1 * (*imu_it)->angular_velocity.y + w2 * (*(imu_it+1))->angular_velocity.y;
                    rz = w1 * (*imu_it)->angular_velocity.z + w2 * (*(imu_it+1))->angular_velocity.z;
                }
                else
                {
                    dx = (*imu_it)->linear_acceleration.x;
                    dy = (*imu_it)->linear_acceleration.y;
                    dz = (*imu_it)->linear_acceleration.z;
                    rx = (*imu_it)->angular_velocity.x;
                    ry = (*imu_it)->angular_velocity.y;
                    rz = (*imu_it)->angular_velocity.z;
                }  
                ++imu_it;
            } 

            estimator.processIMUandThrust(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz), Fz);
                
        }
        else
        {
            double dt_1 = img_t - current_time;
            double dt_2 = t - img_t;
            current_time = img_t;
            ROS_ASSERT(dt_1 >= 0);
            ROS_ASSERT(dt_2 >= 0);
            ROS_ASSERT(dt_1 + dt_2 > 0);
            double w1 = dt_2 / (dt_1 + dt_2);
            double w2 = dt_1 / (dt_1 + dt_2);

            Fz = w1 * Fz + w2 * SCALE_THRUST_INPUT * (*control_it)->collective_thrust;

            if (imu_it!=measurement.first.first.end())
            {
                while ((imu_it+1)!=measurement.first.first.end() && (*(imu_it+1))->header.stamp.toSec() < img_t) //while
                {
                    ++imu_it; // Skip imu messages to get to IMU message just before current time
                }

                // if ((imu_it+1)!=measurement.first.first.end())
                // {
                //     // TODO: Interpolate between imu and imu1 to get imu msg at img_t
                // }
                // else
                // {
                dx = (*imu_it)->linear_acceleration.x;
                dy = (*imu_it)->linear_acceleration.y;
                dz = (*imu_it)->linear_acceleration.z;
                rx = (*imu_it)->angular_velocity.x;
                ry = (*imu_it)->angular_velocity.y;
                rz = (*imu_it)->angular_velocity.z;
                //}

                ++imu_it;
            }



            estimator.processIMUandThrust(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz), Fz);
        }

        last_accel = Vector3d(dx,dy,dz);
        last_angvel = Vector3d(rx,ry,rz);
    } 
}
#endif

// thread: visual-inertial (model-based) odometry
void VID_process()
{
    while (true)
    {
        std::vector<std::pair<std::pair<std::vector<sensor_msgs::ImuConstPtr>,std::vector<controlMsgType>>, sensor_msgs::PointCloudConstPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;
                 });
        lk.unlock();
        m_estimator.lock();
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second;

            if (PROCESS_AT_CONTROL_INPUT_RATE)
            {
                // process_measurements_at_control_input_rate(measurement);
            }
            else
            {
                process_measurements_at_imu_rate(measurement);
            }
            
            // set relocalization frame
            sensor_msgs::PointCloudConstPtr relo_msg = NULL;
            while (!relo_buf.empty())
            {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }
            if (relo_msg != NULL)
            {
                vector<Vector3d> match_points;
                double frame_stamp = relo_msg->header.stamp.toSec();
                for (unsigned int i = 0; i < relo_msg->points.size(); i++)
                {
                    Vector3d u_v_id;
                    u_v_id.x() = relo_msg->points[i].x;
                    u_v_id.y() = relo_msg->points[i].y;
                    u_v_id.z() = relo_msg->points[i].z;
                    match_points.push_back(u_v_id);
                }
                Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
            }

            ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());

            TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];
                ROS_ASSERT(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;

                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            }
            estimator.processImage(image, img_msg->header);

            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);
            std_msgs::Header header = img_msg->header;
            header.frame_id = "world";

            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
            if (relo_msg != NULL)
                pubRelocalization(estimator);
            //ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}


void gt_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    q_gt.w() = msg->pose.pose.orientation.w;
    q_gt.x() = msg->pose.pose.orientation.x;
    q_gt.y() = msg->pose.pose.orientation.y;
    q_gt.z() = msg->pose.pose.orientation.z;
    p_gt[0] = msg->pose.pose.position.x;
    p_gt[1] = msg->pose.pose.position.y;
    p_gt[2] = msg->pose.pose.position.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vid_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info); //Debug
    readParameters(n);
    estimator.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_control_inputs = n.subscribe(CONTROL_TOPIC, 2000, control_inputs_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_image = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_restart = n.subscribe("/feature_tracker/restart", 2000, restart_callback);
    ros::Subscriber sub_relo_points = n.subscribe("/pose_graph/match_points", 2000, relocalization_callback);
    // ros::Subscriber s4 = n.subscribe("gt_", 40, gt_callback, ros::TransportHints().tcpNoDelay());


    std::thread measurement_process{VID_process};
    ros::spin();

    return 0;
}