#include "visualization.h"

using namespace ros;
using namespace Eigen;

ros::Publisher pub_extForces_norm;
ros::Publisher pub_extForces;
ros::Publisher pub_extForces_show;
ros::Publisher pub_bas_norm; //show the bias of acc from imu
ros::Publisher pub_odometry, pub_latest_odometry;
ros::Publisher pub_path, pub_relo_path;
ros::Publisher pub_point_cloud, pub_margin_cloud;
ros::Publisher pub_key_poses;
ros::Publisher pub_relo_relative_pose;
ros::Publisher pub_camera_pose;
ros::Publisher pub_camera_pose_visual;
nav_msgs::Path path, relo_path;

ros::Publisher pub_keyframe_pose;
ros::Publisher pub_keyframe_point;
ros::Publisher pub_extrinsic;

ros::Publisher             meshPub;
ros::Publisher pub_mesh_camera_pose_visual;
visualization_msgs::Marker meshROS;
static string              mesh_resource;
static double              color_r=1.0, color_g=0.0, color_b=0.0, color_a=1.0;
CameraPoseVisualization mesh_cameraposevisual(0, 0, 1, 1);

CameraPoseVisualization cameraposevisual(0, 1, 0, 1);
CameraPoseVisualization keyframebasevisual(0.0, 0.0, 1.0, 1.0);
static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);
static bool start_recording_vis = true;

bool first_pub = false;
extern double Thrust_n;

void registerPub(ros::NodeHandle &n)
{
    pub_latest_odometry = n.advertise<nav_msgs::Odometry>("imu_propagate", 1000);
    pub_path = n.advertise<nav_msgs::Path>("path", 1000);
    pub_relo_path = n.advertise<nav_msgs::Path>("relocalization_path", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1000);
    pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("history_cloud", 1000);
    pub_key_poses = n.advertise<visualization_msgs::Marker>("key_poses", 1000);
    pub_camera_pose = n.advertise<nav_msgs::Odometry>("camera_pose", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    pub_keyframe_pose = n.advertise<nav_msgs::Odometry>("keyframe_pose", 1000);
    pub_keyframe_point = n.advertise<sensor_msgs::PointCloud>("keyframe_point", 1000);
    pub_extrinsic = n.advertise<nav_msgs::Odometry>("extrinsic", 1000);
    pub_relo_relative_pose = n.advertise<nav_msgs::Odometry>("relo_relative_pose", 1000);
    pub_extForces = n.advertise<geometry_msgs::WrenchStamped>("extForces", 1000);
    pub_extForces_norm = n.advertise<std_msgs::Float32>("extForces_n", 1000);
    pub_bas_norm = n.advertise<std_msgs::Float32>("bas_norm", 1000);
    pub_extForces_show = n.advertise<visualization_msgs::Marker>("pub_extForces_show", 1000, true);
    meshPub   = n.advertise<visualization_msgs::Marker>("robot", 5, true);
    pub_mesh_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("mesh_camera_pose_visual", 1000);
    n.param("mesh_resource", mesh_resource,
          std::string("package://vid_estimator/../meshes/hummingbird.mesh"));

    cameraposevisual.setScale(1);
    cameraposevisual.setLineWidth(0.05);
    // cameraposevisual.setImageBoundaryColor(0,0,1);
    // cameraposevisual.setOpticalCenterConnectorColor(0,0,1); // set camera color for VIO
    keyframebasevisual.setScale(0.1);
    keyframebasevisual.setLineWidth(0.01);
    //mesh camera 
    mesh_cameraposevisual.setScale(0.3);
    mesh_cameraposevisual.setLineWidth(0.03);
}

// Pub at imu rate.
void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, const Eigen::Vector3d &force, const std_msgs::Header &header)
{ //estimated pose and velocity from last frame in the window + integrated imu meas until we receive next frame
    Eigen::Quaterniond quadrotor_Q = Q;

    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = quadrotor_Q.x();
    odometry.pose.pose.orientation.y = quadrotor_Q.y();
    odometry.pose.pose.orientation.z = quadrotor_Q.z();
    odometry.pose.pose.orientation.w = quadrotor_Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    odometry.twist.twist.angular.x = force.x();
    odometry.twist.twist.angular.y = force.y();
    odometry.twist.twist.angular.z = force.z();
    pub_latest_odometry.publish(odometry);
}

void printStatistics(const Estimator &estimator, double t)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    //printf("position: %f, %f, %f\r", estimator.Ps[WINDOW_SIZE].x(), estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z());
    ROS_DEBUG_STREAM("position: " << estimator.Ps[WINDOW_SIZE].transpose());
    ROS_DEBUG_STREAM("orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        //ROS_DEBUG("calibration result for camera %d", i);
        ROS_DEBUG_STREAM("extirnsic tic: " << estimator.tic[i].transpose());
        ROS_DEBUG_STREAM("extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose());
        if (ESTIMATE_EXTRINSIC)
        {
            cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
            Eigen::Matrix3d eigen_R;
            Eigen::Vector3d eigen_T;
            eigen_R = estimator.ric[i];
            eigen_T = estimator.tic[i];
            cv::Mat cv_R, cv_T;
            cv::eigen2cv(eigen_R, cv_R);
            cv::eigen2cv(eigen_T, cv_T);
            fs << "extrinsicRotation" << cv_R << "extrinsicTranslation" << cv_T;
            fs.release();
        }
    }

    static double sum_of_time = 0;
    static int sum_of_calculation = 0;
    sum_of_time += t;
    sum_of_calculation++;
    ROS_DEBUG("vo solver costs: %f ms", t);
    ROS_DEBUG("average of time %f ms", sum_of_time / sum_of_calculation);

    sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
    last_path = estimator.Ps[WINDOW_SIZE];
    ROS_DEBUG("sum of path %f", sum_of_path);
    // if (ESTIMATE_TD)
    //     ROS_INFO("td %f", estimator.td);
}

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header)
{
    if (estimator.solver_flag == Estimator::SolverFlag::INITIAL)
    {
        ofstream fout(VINS_RESULT_PATH, ios::out);
        //cout << "VINS_RESULT_PATH is opened? " << !fout << " "<< fout.bad() << " "<< fout.fail()<< endl;
        fout << "time_ns"
             << ","
             << "px"
             << ","
             << "py"
             << ","
             << "pz"
             << ","
             << "qw"
             << ","
             << "qx"
             << ","
             << "qy"
             << ","
             << "qz"
             << ","
             << "vx"
             << ","
             << "vy"
             << ","
             << "vz"
             << ","
             << "ba_x"
             << ","
             << "ba_y"
             << ","
             << "ba_z"
             << ","
             << "bg_x"
             << ","
             << "bg_y"
             << ","
             << "bg_z"
             << ","
             << "Fx"
             << ","
             << "Fy"
             << ","
             << "Fz"
             << ","
             << "solve_time_ms" << endl;
        fout.close();
    }

    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {

        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.child_frame_id = "world";
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
        odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
        odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
        odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
        odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
        odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
        pub_odometry.publish(odometry); //publish last estimated pose

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose = odometry.pose.pose;
        path.header = header;
        path.header.frame_id = "world";
        path.poses.push_back(pose_stamped);
        pub_path.publish(path); // publish updated path of estimated poses

        geometry_msgs::WrenchStamped extF_stamped;
        extF_stamped.header = header;
        Vector3d Fext;
        Fext[0] = estimator.Fexts[WINDOW_SIZE].x();   
        Fext[1] = estimator.Fexts[WINDOW_SIZE].y();
        Fext[2] = estimator.Fexts[WINDOW_SIZE].z();
        Fext = tmp_Q.toRotationMatrix() * Fext;   // pub fext in world frame   

        if(force_wo_rotor_drag)
        {
            // estimate rotor drag
            Eigen::Vector3d velocity;
            Eigen::Matrix3d R, Kdrag;
            R = tmp_Q.toRotationMatrix();
            Kdrag << k_d_x, 0.0, 0.0,
                    0.0, k_d_y, 0.0,
                    0.0, 0.0, 0.0;
            velocity << estimator.Vs[WINDOW_SIZE].x(),
                        estimator.Vs[WINDOW_SIZE].y(),
                        estimator.Vs[WINDOW_SIZE].z();
            Eigen::Vector3d drag_acc = R*Kdrag*R.transpose()*velocity; 

            Vector3d Fext_wo_drag = Fext + drag_acc;

            extF_stamped.wrench.force.x = Fext_wo_drag[0];
            extF_stamped.wrench.force.y = Fext_wo_drag[1];
            extF_stamped.wrench.force.z = Fext_wo_drag[2];
        }
        else
        {
            extF_stamped.wrench.force.x = Fext[0];
            extF_stamped.wrench.force.y = Fext[1];
            extF_stamped.wrench.force.z = Fext[2];
        }
        extF_stamped.wrench.torque.x = estimator.Bas[WINDOW_SIZE][0]; //imu acc bias
        extF_stamped.wrench.torque.y = estimator.Bas[WINDOW_SIZE][1];
        extF_stamped.wrench.torque.z = estimator.Bas[WINDOW_SIZE][2];
        // ROS_INFO_STREAM("Fext " << Fext);
        // ROS_INFO_STREAM("velocity " << velocity);
        // ROS_INFO_STREAM("drag_acc " << drag_acc);
        // ROS_INFO_STREAM("extF_stamped.wrench.force.x " << extF_stamped.wrench.force.x);
        // ROS_INFO_STREAM("extF_stamped.wrench.force.y " << extF_stamped.wrench.force.y);
        // ROS_INFO_STREAM("extF_stamped.wrench.force.z " << extF_stamped.wrench.force.z);
        pub_extForces.publish(extF_stamped);

        std_msgs::Float32 F_n;
        // F_n.data = sqrt(pow(extF_stamped.wrench.force.x, 2) + pow(extF_stamped.wrench.force.y, 2) + pow(extF_stamped.wrench.force.z, 2) + Thrust_n*Thrust_n) - 9.7936;
        // F_n.data = Thrust_n - 9.7936;
        F_n.data = sqrt(pow(extF_stamped.wrench.force.x, 2) + pow(extF_stamped.wrench.force.y, 2) + pow(extF_stamped.wrench.force.z, 2));
        pub_extForces_norm.publish(F_n);
        std_msgs::Float32 bas_n;
        bas_n.data = sqrt(pow(extF_stamped.wrench.torque.x, 2) + pow(extF_stamped.wrench.torque.y, 2) + pow(extF_stamped.wrench.torque.z, 2));
        pub_bas_norm.publish(bas_n);

        //show ext force
        visualizeForce(estimator.Ps[WINDOW_SIZE], Vector3d(extF_stamped.wrench.force.x,extF_stamped.wrench.force.y,extF_stamped.wrench.force.z), 0, Eigen::Vector3d(0, 1, 0), pub_extForces_show); //blue

        Vector3d correct_t;
        //Vector3d correct_v;
        Quaterniond correct_q;
        correct_t = estimator.drift_correct_r * estimator.Ps[WINDOW_SIZE] + estimator.drift_correct_t;
        correct_q = estimator.drift_correct_r * estimator.Rs[WINDOW_SIZE];
        odometry.pose.pose.position.x = correct_t.x();
        odometry.pose.pose.position.y = correct_t.y();
        odometry.pose.pose.position.z = correct_t.z();
        odometry.pose.pose.orientation.x = correct_q.x();
        odometry.pose.pose.orientation.y = correct_q.y();
        odometry.pose.pose.orientation.z = correct_q.z();
        odometry.pose.pose.orientation.w = correct_q.w();

        pose_stamped.pose = odometry.pose.pose;
        relo_path.header = header;
        relo_path.header.frame_id = "world";
        relo_path.poses.push_back(pose_stamped);
        pub_relo_path.publish(relo_path); // publish loop closure corrected pose path

        //mesh pub
        // Mesh model
        meshROS.header = header;
        meshROS.header.frame_id = "world";
        meshROS.ns           = "drone";
        meshROS.id           = 0; //::count; //! @todo implement 3/12/18 1 ~ retrackable id
        meshROS.type         = visualization_msgs::Marker::MESH_RESOURCE;
        meshROS.action       = visualization_msgs::Marker::ADD;

        meshROS.pose = odometry.pose.pose;

        meshROS.scale.x       = 1.5;
        meshROS.scale.y       = 1.5;
        meshROS.scale.z       = 1.5;
        meshROS.color.a       = color_a;
        meshROS.color.r       = color_r;
        meshROS.color.g       = color_g;
        meshROS.color.b       = color_b;
        meshROS.mesh_resource = mesh_resource;
        meshPub.publish(meshROS);

        // mesh camera
        Vector3d mesh_P = estimator.Ps[WINDOW_SIZE] + estimator.Rs[WINDOW_SIZE] * estimator.tic[0];
        Quaterniond mesh_R = Quaterniond(estimator.Rs[WINDOW_SIZE] * estimator.ric[0]);
        nav_msgs::Odometry mesh_cam_odometry;
        mesh_cam_odometry.header = header;
        mesh_cam_odometry.header.frame_id = "world";
        mesh_cam_odometry.pose.pose.position.x = mesh_P.x();
        mesh_cam_odometry.pose.pose.position.y = mesh_P.y();
        mesh_cam_odometry.pose.pose.position.z = mesh_P.z();
        mesh_cam_odometry.pose.pose.orientation.x = mesh_R.x();
        mesh_cam_odometry.pose.pose.orientation.y = mesh_R.y();
        mesh_cam_odometry.pose.pose.orientation.z = mesh_R.z();
        mesh_cam_odometry.pose.pose.orientation.w = mesh_R.w();
        mesh_cameraposevisual.reset();
        mesh_cameraposevisual.add_pose(mesh_P, mesh_R);
        mesh_cameraposevisual.publish_by(pub_mesh_camera_pose_visual, mesh_cam_odometry.header);


        // write result to file
        ofstream foutC(VINS_RESULT_PATH, ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(0);
        foutC << header.stamp.toSec() * 1e9 << ",";
        foutC.precision(5);
        foutC << estimator.Ps[WINDOW_SIZE].x() << ","
              << estimator.Ps[WINDOW_SIZE].y() << ","
              << estimator.Ps[WINDOW_SIZE].z() << ","
              << tmp_Q.w() << ","
              << tmp_Q.x() << ","
              << tmp_Q.y() << ","
              << tmp_Q.z() << ","
              << estimator.Vs[WINDOW_SIZE].x() << ","
              << estimator.Vs[WINDOW_SIZE].y() << ","
              << estimator.Vs[WINDOW_SIZE].z() << ","
              << estimator.Bas[WINDOW_SIZE].x() << ","
              << estimator.Bas[WINDOW_SIZE].y() << ","
              << estimator.Bas[WINDOW_SIZE].z() << ","
              << estimator.Bgs[WINDOW_SIZE].x() << ","
              << estimator.Bgs[WINDOW_SIZE].y() << ","
              << estimator.Bgs[WINDOW_SIZE].z() << ","
              << estimator.Fexts[WINDOW_SIZE].x() << ","
              << estimator.Fexts[WINDOW_SIZE].y() << ","
              << estimator.Fexts[WINDOW_SIZE].z() << ","
              << estimator.solve_time_ms << endl;
        foutC.close();
    }
}

void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header)
{
    if (estimator.key_poses.size() == 0)
        return;
    visualization_msgs::Marker key_poses;
    key_poses.header = header;
    key_poses.header.frame_id = "world";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    key_poses.id = 0; //key_poses_id++;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        geometry_msgs::Point pose_marker;
        Vector3d correct_pose;
        correct_pose = estimator.key_poses[i];
        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = correct_pose.z();
        key_poses.points.push_back(pose_marker); //publish estimated pose Ps[i]
    }
    pub_key_poses.publish(key_poses);
}

void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header)
{
    int idx2 = WINDOW_SIZE - 1;

    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int i = idx2;
        Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[0]);

        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        pub_camera_pose.publish(odometry); // publish pose of camera wrt world frame

        cameraposevisual.reset();
        cameraposevisual.add_pose(P, R);
        cameraposevisual.publish_by(pub_camera_pose_visual, odometry.header);
    }
}

void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header)
{
    sensor_msgs::PointCloud point_cloud, loop_point_cloud;
    point_cloud.header = header;
    loop_point_cloud.header = header;

    for (auto &it_per_id : estimator.f_manager.feature)
    {
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
            continue;
        int imu_i = it_per_id.start_frame;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

        geometry_msgs::Point32 p;
        p.x = w_pts_i(0);
        p.y = w_pts_i(1);
        p.z = w_pts_i(2);
        point_cloud.points.push_back(p);
    }
    pub_point_cloud.publish(point_cloud);

    // pub margined potin
    sensor_msgs::PointCloud margin_cloud;
    margin_cloud.header = header;

    for (auto &it_per_id : estimator.f_manager.feature)
    {
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        //if (it_per_id->start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id->solve_flag != 1)
        //        continue;

        if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2 && it_per_id.solve_flag == 1)
        {
            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

            geometry_msgs::Point32 p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);
            margin_cloud.points.push_back(p);
        }
    }
    pub_margin_cloud.publish(margin_cloud);
}

void pubTF(const Estimator &estimator, const std_msgs::Header &header)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    // body frame
    Vector3d correct_t;
    Quaterniond correct_q;
    correct_t = estimator.Ps[WINDOW_SIZE];
    correct_q = estimator.Rs[WINDOW_SIZE];

    transform.setOrigin(tf::Vector3(correct_t(0),
                                    correct_t(1),
                                    correct_t(2)));
    q.setW(correct_q.w());
    q.setX(correct_q.x());
    q.setY(correct_q.y());
    q.setZ(correct_q.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "world", "body"));

    // camera frame
    transform.setOrigin(tf::Vector3(estimator.tic[0].x(),
                                    estimator.tic[0].y(),
                                    estimator.tic[0].z()));
    q.setW(Quaterniond(estimator.ric[0]).w());
    q.setX(Quaterniond(estimator.ric[0]).x());
    q.setY(Quaterniond(estimator.ric[0]).y());
    q.setZ(Quaterniond(estimator.ric[0]).z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "body", "camera"));

    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = estimator.tic[0].x();
    odometry.pose.pose.position.y = estimator.tic[0].y();
    odometry.pose.pose.position.z = estimator.tic[0].z();
    Quaterniond tmp_q{estimator.ric[0]};
    odometry.pose.pose.orientation.x = tmp_q.x();
    odometry.pose.pose.orientation.y = tmp_q.y();
    odometry.pose.pose.orientation.z = tmp_q.z();
    odometry.pose.pose.orientation.w = tmp_q.w();
    pub_extrinsic.publish(odometry);
}

void pubKeyframe(const Estimator &estimator)
{
    // pub camera pose, 2D-3D points of keyframe
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR && estimator.marginalization_flag == 0)
    {
        int i = WINDOW_SIZE - 2;
        //Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Vector3d P = estimator.Ps[i];
        Quaterniond R = Quaterniond(estimator.Rs[i]);

        nav_msgs::Odometry odometry;
        odometry.header = estimator.Headers[WINDOW_SIZE - 2];
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();
        //printf("time: %f t: %f %f %f r: %f %f %f %f\n", odometry.header.stamp.toSec(), P.x(), P.y(), P.z(), R.w(), R.x(), R.y(), R.z());

        pub_keyframe_pose.publish(odometry);

        sensor_msgs::PointCloud point_cloud;
        point_cloud.header = estimator.Headers[WINDOW_SIZE - 2];
        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int frame_size = it_per_id.feature_per_frame.size();
            if (it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.start_frame + frame_size - 1 >= WINDOW_SIZE - 2 && it_per_id.solve_flag == 1)
            {

                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];
                geometry_msgs::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                point_cloud.points.push_back(p);

                int imu_j = WINDOW_SIZE - 2 - it_per_id.start_frame;
                sensor_msgs::ChannelFloat32 p_2d;
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.y());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.y());
                p_2d.values.push_back(it_per_id.feature_id);
                point_cloud.channels.push_back(p_2d);
            }
        }
        pub_keyframe_point.publish(point_cloud);
    }
}

void pubRelocalization(const Estimator &estimator)
{
    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(estimator.relo_frame_stamp);
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = estimator.relo_relative_t.x();
    odometry.pose.pose.position.y = estimator.relo_relative_t.y();
    odometry.pose.pose.position.z = estimator.relo_relative_t.z();
    odometry.pose.pose.orientation.x = estimator.relo_relative_q.x();
    odometry.pose.pose.orientation.y = estimator.relo_relative_q.y();
    odometry.pose.pose.orientation.z = estimator.relo_relative_q.z();
    odometry.pose.pose.orientation.w = estimator.relo_relative_q.w();
    odometry.twist.twist.linear.x = estimator.relo_relative_yaw;
    odometry.twist.twist.linear.y = estimator.relo_frame_index;

    pub_relo_relative_pose.publish(odometry);
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
