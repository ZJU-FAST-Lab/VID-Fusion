#include <cstdio>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/WrenchStamped.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace Eigen;

void visualizeForce(Eigen::Vector3d position, Eigen::Vector3d Force, int id, Eigen::Vector3d color, ros::Publisher pub_force);

int cur_kind = 0; //0 only odom; 1 only force; //2 both

const int SKIP = 50;
string benchmark_output_path;
string estimate_output_path;
template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

struct Data
{
    Data(FILE *f)
    {
        if (fscanf(f, " %lf %f %f %f %f %f %f %f", &t,
               &px, &py, &pz,
               &qx, &qy, &qz, &qw) != EOF)
        {
            // t /= 1e9;
        }
    }
    double t;
    float px, py, pz;
    float qw, qx, qy, qz;
};

struct Data_force
{
    Data_force(FILE *f)
    {
        if (fscanf(f, " %lf %f %f %f", &t,
               &px, &py, &pz) != EOF)
        {
            // t /= 1e9;
        }
    }
    double t;
    float px, py, pz;
};

int idx = 1;
vector<Data> benchmark;
vector<Data_force> benchmark_force;

ros::Publisher pub_odom;
ros::Publisher pub_path;
nav_msgs::Path path;

ros::Publisher pub_force;
ros::Publisher pub_force_gt_show;
Matrix3d R_force;
Vector3d f0_sensor;
bool first_force = true;
ros::Publisher pub_path_drone_force_sensor;
nav_msgs::Path path_drone_force_sensor;

int init = 0;
Quaterniond baseRgt;
Vector3d baseTgt;
vector<Quaterniond> baseRgt_set;
vector<Vector3d> baseTgt_set;
tf::Transform trans;

double mass(1.138);

void odom_callback(const nav_msgs::OdometryConstPtr &odom_msg)
{
    // cout <<"t: o g: " << odom_msg->header.stamp.toSec() << endl << benchmark.back().t << endl;
    if (odom_msg->header.stamp.toSec() > benchmark.back().t)
      return;

    // ROS_INFO("odom callback!");

    for (; idx < static_cast<int>(benchmark.size()) && benchmark[idx].t <= odom_msg->header.stamp.toSec(); idx++)
        ;


    if (init++ < SKIP)
    {
        // cout <<  "init++ < SKIP" << endl;
        baseRgt = Quaterniond(odom_msg->pose.pose.orientation.w,
                              odom_msg->pose.pose.orientation.x,
                              odom_msg->pose.pose.orientation.y,
                              odom_msg->pose.pose.orientation.z) *
                  Quaterniond(benchmark[idx - 1].qw,
                              benchmark[idx - 1].qx,
                              benchmark[idx - 1].qy,
                              benchmark[idx - 1].qz).inverse();
        baseTgt = Vector3d{odom_msg->pose.pose.position.x,
                           odom_msg->pose.pose.position.y,
                           odom_msg->pose.pose.position.z} -
                  baseRgt * Vector3d{benchmark[idx - 1].px, benchmark[idx - 1].py, benchmark[idx - 1].pz};
        return;
    }

    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(benchmark[idx - 1].t);
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";

    Vector3d tmp_T = baseTgt + baseRgt * Vector3d{benchmark[idx - 1].px, benchmark[idx - 1].py, benchmark[idx - 1].pz};
    odometry.pose.pose.position.x = tmp_T.x();
    odometry.pose.pose.position.y = tmp_T.y();
    odometry.pose.pose.position.z = tmp_T.z();

    Quaterniond tmp_R = baseRgt * Quaterniond{benchmark[idx - 1].qw,
                                              benchmark[idx - 1].qx,
                                              benchmark[idx - 1].qy,
                                              benchmark[idx - 1].qz};
    odometry.pose.pose.orientation.w = tmp_R.w();
    odometry.pose.pose.orientation.x = tmp_R.x();
    odometry.pose.pose.orientation.y = tmp_R.y();
    odometry.pose.pose.orientation.z = tmp_R.z();

    pub_odom.publish(odometry);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = odometry.header;
    pose_stamped.pose = odometry.pose.pose;
    path.header = odometry.header;
    path.poses.push_back(pose_stamped);
    pub_path.publish(path);
}


void odom_callback_force(const nav_msgs::OdometryConstPtr &odom_msg)
{
    // cout << "time: " << odom_msg->header.stamp.toSec() << endl << benchmark_force.back().t << endl;
    // cout << "px: " << benchmark_force.size() << endl;
    if (odom_msg->header.stamp.toSec() > benchmark_force.back().t)
      return;

    double time_diff = 2.8;
    for (; idx < static_cast<int>(benchmark_force.size()) && (benchmark_force[idx].t-time_diff) <= odom_msg->header.stamp.toSec(); idx++)
        ;

    // printf("time: %.9lf \n %.9lf \n", odom_msg->header.stamp.toSec(), benchmark_force[idx].t);

    Vector3d force_gt(benchmark_force[idx - 1].px, benchmark_force[idx - 1].py, benchmark_force[idx - 1].pz);


    if(first_force)
    {
        // f0_sensor = force_gt;
        first_force = false;
    }

    Vector3d force_gt_tmp = force_gt - f0_sensor;
    force_gt = R_force * force_gt_tmp;

    force_gt = force_gt/mass;
    force_gt(2) = -force_gt(2);
    
    geometry_msgs::WrenchStamped extF_gt;
    extF_gt.header.stamp = ros::Time(benchmark_force[idx - 1].t);
    extF_gt.header.frame_id = "world";

    extF_gt.wrench.force.x = force_gt[0];
    extF_gt.wrench.force.y = force_gt[1];
    extF_gt.wrench.force.z = force_gt[2];

    pub_force.publish(extF_gt);
 
    //show ext force
    visualizeForce(Vector3d(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z), force_gt, 0, Eigen::Vector3d(0, 0, 1), pub_force_gt_show); //blue

}

Quaterniond yaw2quaternion(double yaw)
{
  double cr = 0;
  double sr = 0;
  double cp = 0;
  double sp = 0;
  double cy = cos(yaw/2);
  double sy = sin(yaw/2);
  Quaterniond q;
  q.w() = cr*cp*cy + sr*sp*sy;
  q.x() = sr*cp*cy - cr*sp*sy;
  q.y() = cr*sp*cy + sr*cp*sy;
  q.z() = cr*cp*sy - sr*sp*cy;
  return q; 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "benchmark_publisher");
    ros::NodeHandle n("~");

    n.getParam("cur_kind", cur_kind);
    n.getParam("mass", mass);
    cout << "mass: " << mass << endl;

    if(cur_kind == 0){  //gt odom

        string csv_file = readParam<string>(n, "data_name");
        std::cout << "load ground truth " << csv_file << std::endl;
        FILE *f = fopen(csv_file.c_str(), "r");
        if (f==NULL)
        {
        ROS_WARN("can't load ground truth; wrong path");
        //std::cerr << "can't load ground truth; wrong path " << csv_file << std::endl;
        return 0;
        }
        char tmp[10000];
        if (fgets(tmp, 10000, f) == NULL)
        {
            ROS_WARN("can't load ground truth; no data available");
        }
        while (!feof(f))
            benchmark.emplace_back(f);
        fclose(f);
        benchmark.pop_back();
        ROS_INFO("Data loaded: %d", (int)benchmark.size());

        pub_odom = n.advertise<nav_msgs::Odometry>("odometry", 1000);
        pub_path = n.advertise<nav_msgs::Path>("path", 1000);

        ros::Subscriber sub_odom = n.subscribe("estimated_odometry", 1000, odom_callback);

        ros::Rate r(20);
        ros::spin();
    }

    if(cur_kind == 1)   //gt force
    {
        std::cout << "load force_gt "  << std::endl;
        Matrix3d R_force_tmp; 
        R_force_tmp << 0.342020143325669, 0.939692620785908, 0,
                      -0.939692620785908, 0.342020143325669, 0,
                                       0,                 0, 1;  //matlab
        R_force = R_force_tmp.transpose();
        
        f0_sensor = Vector3d(0.119999997318, 1.379999995232, 3.019999980927);
        std::cout << "load force_gtsd "  << std::endl;

        string csv_file = readParam<string>(n, "data_name");
        std::cout << "load force_gt " << csv_file << std::endl;
        FILE *f = fopen(csv_file.c_str(), "r");
        if (f==NULL)
        {
        ROS_WARN("can't load force_gt; wrong path");
        //std::cerr << "can't load ground truth; wrong path " << csv_file << std::endl;
        return 0;
        }
        char tmp[10000];
        if (fgets(tmp, 10000, f) == NULL)
        {
            ROS_WARN("can't load force_gt; no data available");
        }
        while (!feof(f))
            benchmark_force.emplace_back(f);
        fclose(f);
        benchmark_force.pop_back();
        ROS_INFO("force_gt Data loaded: %d", (int)benchmark_force.size());

        pub_force = n.advertise<geometry_msgs::WrenchStamped>("extForces_gt", 1000);
        pub_force_gt_show = n.advertise<visualization_msgs::Marker>("pub_extForces_gt_show", 1000, true);

        ros::Subscriber sub_odom = n.subscribe("estimated_odometry", 1000, odom_callback_force);

        ros::Rate r(20);
        ros::spin();

    }
}


void visualizeForce(Eigen::Vector3d position, Eigen::Vector3d Force, int id, Eigen::Vector3d color, ros::Publisher pub_force)
{
    double scale = 1;
    visualization_msgs::Marker m;
    m.header.frame_id = "world";
    m.id = id;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::MODIFY;
    m.scale.x = 0.05;
    m.scale.y = 0.2;
    m.scale.z = 0;
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.pose.position.z = 0;
    m.pose.orientation.w = 1;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.color.a = 1.0;
    m.color.r = color.x();
    m.color.g = color.y();
    m.color.b = color.z();
    m.points.clear();
    geometry_msgs::Point point;
    point.x = position.x();
    point.y = position.y();
    point.z = position.z();
    m.points.push_back(point);
    point.x = position.x() + Force.x() * scale;
    point.y = position.y() + Force.y() * scale;
    point.z = position.z() + Force.z() * scale;
    m.points.push_back(point);
    pub_force.publish(m);
}
