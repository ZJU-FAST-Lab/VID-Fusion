#pragma once

#include "../utility/utility.h"
#include "../parameters.h"

#include <ceres/ceres.h>
using namespace Eigen;

class IntegrationBase
{
public:
    IntegrationBase() = delete;
    IntegrationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0, const double &_Fz_0,
                    const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg, const Eigen::Quaterniond &q_gt0)
        : acc_0{_acc_0}, gyr_0{_gyr_0}, linearized_acc{_acc_0}, linearized_gyr{_gyr_0},
          linearized_ba{_linearized_ba}, linearized_bg{_linearized_bg},
          jacobian{Eigen::Matrix<double, 15, 15>::Identity()}, covariance{Eigen::Matrix<double, 15, 15>::Zero()},
          sum_dt{0.0}, delta_p{Eigen::Vector3d::Zero()}, delta_q{Eigen::Quaterniond::Identity()}, delta_v{Eigen::Vector3d::Zero()},
          Fz_0{_Fz_0}, linearized_Fz{_Fz_0}, jacobian_model{Eigen::Matrix<double, 18, 18>::Identity()}, covariance_model{Eigen::Matrix<double, 18, 18>::Zero()},
          delta_p_model{Eigen::Vector3d::Zero()}, delta_v_model{Eigen::Vector3d::Zero()}, delta_f_model{Eigen::Vector3d::Zero()}, dbg_model{Eigen::Vector3d::Zero()}

    {
        q_gt0_ = q_gt0;

        noise = Eigen::Matrix<double, 18, 18>::Zero();
        noise(0, 0) = ACC_N(0) * ACC_N(0);
        noise(1, 1) = ACC_N(1) * ACC_N(1);
        noise(2, 2) = ACC_N(2) * ACC_N(2);
        noise.block<3, 3>(3, 3) = (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise(6, 6) = ACC_N(0) * ACC_N(0);
        noise(7, 7) = ACC_N(1) * ACC_N(1);
        noise(8, 8) = ACC_N(2) * ACC_N(2);
        noise.block<3, 3>(9, 9) = (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(12, 12) = (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(15, 15) = (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();

        // for midpoint integration - makes sense when we use motor speeds instead of control commands
        noise_model = Eigen::Matrix<double, 24, 24>::Zero();                             // Q_model
        noise_model(0, 0) = THRUST_X_Y_N * THRUST_X_Y_N;                                 // sigma_Fx_0 sq
        noise_model(1, 1) = THRUST_X_Y_N * THRUST_X_Y_N;                                 // sigma_Fy_0 sq
        noise_model(2, 2) = THRUST_Z_N * THRUST_Z_N;                                     // sigma_Fz_0 sq
        noise_model.block<3, 3>(3, 3) = (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();   // sigma_gyr_0 sq
        noise_model(6, 6) = THRUST_X_Y_N * THRUST_X_Y_N;                                 // sigma_Fx_1 sq
        noise_model(7, 7) = THRUST_X_Y_N * THRUST_X_Y_N;                                 // sigma_Fy_1 sq
        noise_model(8, 8) = THRUST_Z_N * THRUST_Z_N;                                     // sigma_Fz_1 sq
        noise_model.block<3, 3>(9, 9) = (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();   // sigma_gyr_1 sq
        noise_model.block<3, 3>(12, 12) = (GYR_W * GYR_W) * Eigen::Matrix3d::Identity(); // sigma_bw_ sq
        noise_model.block<3, 3>(15, 15) = (ACC_W * ACC_W) * Eigen::Matrix3d::Identity(); // sigma_ba_ sq
        noise_model(18, 18) = ACC_N(0) * ACC_N(0);                                       // sigma_acc_0 sq
        noise_model(19, 19) = ACC_N(1) * ACC_N(1);
        noise_model(20, 20) = ACC_N(2) * ACC_N(2);
        noise_model(21, 21) = ACC_N(0) * ACC_N(0);                                       // sigma_acc_1 sq
        noise_model(22, 22) = ACC_N(1) * ACC_N(1);
        noise_model(23, 23) = ACC_N(2) * ACC_N(2);

    }

    void push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr, const double &Fz)
    {
        dt_buf.push_back(dt);
        acc_buf.push_back(acc);
        gyr_buf.push_back(gyr);
        Fz_buf.push_back(Fz);
        propagate(dt, acc, gyr, Fz);
    }

    void repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
    {
        //printf("repropagate imu and model measurements bcz bias estimate has changed alot. \n");
        sum_dt = 0.0;
        acc_0 = linearized_acc;
        gyr_0 = linearized_gyr;
        Fz_0 = linearized_Fz;
        delta_p.setZero();
        delta_q.setIdentity();
        delta_v.setZero();
        delta_p_model.setZero();
        delta_v_model.setZero();
        
        linearized_ba = _linearized_ba;
        linearized_bg = _linearized_bg;
        jacobian.setIdentity();
        jacobian_model.setIdentity();
        dbg_model.setZero();
        covariance.setZero();
        covariance_model.setZero();
        for (int i = 0; i < static_cast<int>(dt_buf.size()); i++)
            propagate(dt_buf[i], acc_buf[i], gyr_buf[i], Fz_buf[i]);
    }

    void midPointIntegration(double _dt,
                             const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                             const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                             const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                             const double &_Fz_0, const double &_Fz_1,
                             const Eigen::Vector3d &delta_p_model, const Eigen::Vector3d &delta_v_model,
                             const Eigen::Vector3d &delta_f_model,
                             const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                             Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                             Eigen::Vector3d &result_delta_p_model, Eigen::Vector3d &result_delta_v_model,
                             Eigen::Vector3d &result_delta_f_model,
                             Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian)
    {

        /**IMU preintegration**/
        Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);
        Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;

        result_delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);

        Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

        result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
        result_delta_v = delta_v + un_acc * _dt;
        result_linearized_ba = linearized_ba;
        result_linearized_bg = linearized_bg;

        /**Model preintegration**/   
        Vector3d body_thrust_0(0.0, 0.0, _Fz_0); // body z axis is pointing upwards parallel to propellers axis of rotation. This is mass-normalized thrust (m/s^2).
        Vector3d body_thrust_1(0.0, 0.0, _Fz_1);

        Vector3d control_acc_0 = delta_q * body_thrust_0;
        Vector3d control_acc_1 = result_delta_q * body_thrust_1;
        // Vector3d control_acc_0 = delta_q * (body_thrust_0 + delta_f_model/sum_dt);
        // Vector3d control_acc_1 = result_delta_q * (body_thrust_1 + delta_f_model/sum_dt);
        Vector3d control_acc = 0.5 * (control_acc_0 + control_acc_1);

        Vector3d fext_acc = 0.5 * ((un_acc_0 - control_acc_0) + (un_acc_1 - control_acc_1));
        // Vector3d fext_acc = 0.5 * (((_acc_0 - linearized_ba) - body_thrust_0) + ((_acc_1 - linearized_ba) - body_thrust_1));
        // Vector3d fext_acc = 0.5 * ((un_acc_0 - q_gt0_.toRotationMatrix() * body_thrust_0) + (un_acc_1 - q_gt0_.toRotationMatrix() * body_thrust_1));
        // Vector3d fext_acc = 0.5 * ((delta_q * _acc_0 - control_acc_0) + (result_delta_q * _acc_1 - control_acc_1));

        result_delta_p_model = delta_p_model + delta_v_model * _dt + 0.5 * control_acc * _dt * _dt;
        result_delta_v_model = delta_v_model + control_acc * _dt;
        result_delta_f_model = delta_f_model + fext_acc * _dt;

        if (update_jacobian)
        {
            Vector3d w_x = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
            Vector3d a_0_x = _acc_0 - linearized_ba;
            Vector3d a_1_x = _acc_1 - linearized_ba;
            Matrix3d R_w_x, R_a_0_x, R_a_1_x, R_F_0_x, R_F_1_x;
            Vector3d a_F_0_x = a_0_x - body_thrust_0;
            Vector3d a_F_1_x = a_1_x - body_thrust_1;
            // Vector3d F_0_x = body_thrust_0 + delta_f_model/sum_dt;
            // Vector3d F_1_x = body_thrust_1 + delta_f_model/sum_dt;
            Matrix3d R_a_F_0, R_a_F_1;

            R_w_x << 0, -w_x(2), w_x(1),
                w_x(2), 0, -w_x(0),
                -w_x(1), w_x(0), 0;
            R_a_0_x << 0, -a_0_x(2), a_0_x(1),
                a_0_x(2), 0, -a_0_x(0),
                -a_0_x(1), a_0_x(0), 0;
            R_a_1_x << 0, -a_1_x(2), a_1_x(1),
                a_1_x(2), 0, -a_1_x(0),
                -a_1_x(1), a_1_x(0), 0;

            R_F_0_x << 0, -body_thrust_0(2), body_thrust_0(1),
                body_thrust_0(2), 0, -body_thrust_0(0),
                -body_thrust_0(1), body_thrust_0(0), 0;

            R_F_1_x << 0, -body_thrust_1(2), body_thrust_1(1),
                body_thrust_1(2), 0, -body_thrust_1(0),
                -body_thrust_1(1), body_thrust_1(0), 0;
            
            // R_F_0_x << 0, -F_0_x(2), F_0_x(1),
            //     F_0_x(2), 0, -F_0_x(0),
            //     -F_0_x(1), F_0_x(0), 0;

            // R_F_1_x << 0, -F_1_x(2), F_1_x(1),
            //     F_1_x(2), 0, -F_1_x(0),
            //     -F_1_x(1), F_1_x(0), 0;

            R_a_F_0 << 0,           -a_F_0_x(2),  a_F_0_x(1),
                       a_F_0_x(2),   0,           -a_F_0_x(0),
                      -a_F_0_x(1),   a_F_0_x(0),  0; 
            R_a_F_1 << 0,           -a_F_1_x(2),  a_F_1_x(1),
                       a_F_1_x(2),   0,           -a_F_1_x(0),
                      -a_F_1_x(1),   a_F_1_x(0),  0; 

            MatrixXd F_model = MatrixXd::Zero(18, 18); // delta p,q,v,bw,ba,f
            F_model.block<3, 3>(0, 0) = Matrix3d::Identity();
            F_model.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_F_0_x * _dt * _dt +
                                        -0.25 * result_delta_q.toRotationMatrix() * R_F_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;  //f01
            F_model.block<3, 3>(0, 6) = MatrixXd::Identity(3, 3) * _dt;
            F_model.block<3, 3>(0, 9) = -0.25 * result_delta_q.toRotationMatrix() * R_F_1_x * _dt * _dt * -_dt;  //f04
            F_model.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * _dt;                 // delta_q wrt delta q
            F_model.block<3, 3>(3, 9) = -1.0 * MatrixXd::Identity(3, 3) * _dt;              // delta_q wrt bw
            F_model.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_F_0_x * _dt + // delta v wrt delta q
                                        -0.5 * result_delta_q.toRotationMatrix() * R_F_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt;  //f21
            F_model.block<3, 3>(6, 6) = Matrix3d::Identity(); // delta v wrt delta v
            F_model.block<3, 3>(6, 9) = -0.5 * result_delta_q.toRotationMatrix() * R_F_1_x * _dt * -_dt;  //f24
            F_model.block<3, 3>(9, 9) = Matrix3d::Identity();
            F_model.block<3, 3>(12, 12) = Matrix3d::Identity();
            F_model.block<3, 3>(15, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_F_0 * _dt + // delta f_v wrt delta q
                                        -0.5 * result_delta_q.toRotationMatrix() * R_a_F_1 * (Matrix3d::Identity() - R_w_x * _dt) * _dt;   //f'21
            F_model.block<3, 3>(15, 9) = -0.5 * result_delta_q.toRotationMatrix() * R_a_F_1 * _dt * -_dt;  //f'24
            F_model.block<3, 3>(15, 12) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
            F_model.block<3, 3>(15, 15) = Matrix3d::Identity();

            MatrixXd V_model = MatrixXd::Zero(18, 24); //fz0,gyr0,fz1,gyr1,bw,ba,a0,a1
            Matrix3d delta_q_2rotmat = delta_q.toRotationMatrix();
            Matrix3d result_delta_q_2rotmat = result_delta_q.toRotationMatrix();

            V_model.block<3, 3>(0, 0) = 0.25 * delta_q_2rotmat * _dt * _dt;                                          //delta p  wrt Fz_0_noise
            V_model.block<3, 3>(0, 3) = 0.25 * -result_delta_q.toRotationMatrix() * R_F_1_x * _dt * _dt * 0.5 * _dt; //delta p  wrt gyr_0_noise  //v01
            V_model.block<3, 3>(0, 6) = 0.25 * result_delta_q_2rotmat * _dt * _dt;                                   //delta p  wrt Fz_1_noise
            V_model.block<3, 3>(0, 9) = V_model.block<3, 3>(0, 3);                                                   //delta p  wrt gyr_1_noise  //v03
            V_model.block<3, 3>(3, 3) = 0.5 * MatrixXd::Identity(3, 3) * _dt;                                        //delta q  wrt gyr_0_noise
            V_model.block<3, 3>(3, 9) = 0.5 * MatrixXd::Identity(3, 3) * _dt;                                        //delta q  wrt gyr_1_noise
            V_model.block<3, 3>(6, 0) = 0.5 * delta_q_2rotmat * _dt;                                                 //delta v  wrt fz0_noise
            V_model.block<3, 3>(6, 3) = 0.5 * -result_delta_q.toRotationMatrix() * R_F_1_x * _dt * 0.5 * _dt;        //delta v  wrt gyr0_noise   //v21
            V_model.block<3, 3>(6, 6) = 0.5 * result_delta_q_2rotmat * _dt;                                          //delta v  wrt fz1_noise
            V_model.block<3, 3>(6, 9) = V_model.block<3, 3>(6, 3);                                                   //delta v  wrt gyr1_noise   //v23
            V_model.block<3, 3>(9, 12) = MatrixXd::Identity(3, 3) * _dt;                                             //delta bw wrt bw_noise 
            V_model.block<3, 3>(12, 15) = MatrixXd::Identity(3, 3) * _dt;                                            //delta ba wrt ba_noise 
            V_model.block<3, 3>(15, 0) = -0.5 * delta_q_2rotmat * _dt;                                               //delta f  wrt Fz_0_noise 
            V_model.block<3, 3>(15, 3) = 0.5 * -result_delta_q.toRotationMatrix() * R_a_F_1 * _dt * 0.5 * _dt;       //delta f  wrt gyr_0_noise   //v'21
            V_model.block<3, 3>(15, 6) = -0.5 * result_delta_q_2rotmat * _dt;                                        //delta f  wrt Fz_1_noise
            V_model.block<3, 3>(15, 9) = V_model.block<3, 3>(15, 3);                                                 //delta f  wrt gyr_1_noise    //v'23
            V_model.block<3, 3>(15, 18) = 0.5 * delta_q_2rotmat * _dt;                                               //delta f  wrt a_0_noise
            V_model.block<3, 3>(15, 21) = 0.5 * result_delta_q_2rotmat * _dt;                                        //delta f  wrt a_1_noise

            MatrixXd F = MatrixXd::Zero(15, 15); // delta p,r,v, ba, bw
            F.block<3, 3>(0, 0) = Matrix3d::Identity();
            F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt +
                                  -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
            F.block<3, 3>(0, 6) = MatrixXd::Identity(3, 3) * _dt;
            F.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt * _dt;
            F.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
            F.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * _dt;
            F.block<3, 3>(3, 12) = -1.0 * MatrixXd::Identity(3, 3) * _dt;
            F.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt +
                                  -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt;
            F.block<3, 3>(6, 6) = Matrix3d::Identity();
            F.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
            F.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;
            F.block<3, 3>(9, 9) = Matrix3d::Identity();
            F.block<3, 3>(12, 12) = Matrix3d::Identity();

            MatrixXd V = MatrixXd::Zero(15, 18); //acc0,gyr0,acc1,gyr1,ba,bw
            V.block<3, 3>(0, 0) = 0.25 * delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 3) = 0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * 0.5 * _dt;
            V.block<3, 3>(0, 6) = 0.25 * result_delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 9) = V.block<3, 3>(0, 3);
            V.block<3, 3>(3, 3) = 0.5 * MatrixXd::Identity(3, 3) * _dt;
            V.block<3, 3>(3, 9) = 0.5 * MatrixXd::Identity(3, 3) * _dt;
            V.block<3, 3>(6, 0) = 0.5 * delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 3) = 0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x * _dt * 0.5 * _dt;
            V.block<3, 3>(6, 6) = 0.5 * result_delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 9) = V.block<3, 3>(6, 3);
            V.block<3, 3>(9, 12) = MatrixXd::Identity(3, 3) * _dt;
            V.block<3, 3>(12, 15) = MatrixXd::Identity(3, 3) * _dt;


            jacobian = F * jacobian;
            jacobian_model = F_model * jacobian_model;

            covariance = F * covariance * F.transpose() + V * noise * V.transpose();
            covariance_model = F_model * covariance_model * F_model.transpose() + V_model * noise_model * V_model.transpose();
        }
    }

    void propagate(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1, const double &_Fz_1)
    {
        dt = _dt;
        acc_1 = _acc_1;
        gyr_1 = _gyr_1;
        Fz_1 = _Fz_1;

        Vector3d result_delta_p;
        Quaterniond result_delta_q;
        Vector3d result_delta_v;
        Vector3d result_linearized_ba;
        Vector3d result_linearized_bg;
        Vector3d result_delta_p_model;
        Vector3d result_delta_v_model;
        Vector3d result_delta_f_model;

        midPointIntegration(_dt, acc_0, gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
                            Fz_0, _Fz_1, delta_p_model, delta_v_model,
                            delta_f_model,
                            linearized_ba, linearized_bg,
                            result_delta_p, result_delta_q, result_delta_v,
                            result_delta_p_model, result_delta_v_model,
                            result_delta_f_model,
                            result_linearized_ba, result_linearized_bg, 1);

        delta_p = result_delta_p;
        delta_q = result_delta_q;
        delta_v = result_delta_v;
        linearized_ba = result_linearized_ba;
        linearized_bg = result_linearized_bg;
        delta_p_model = result_delta_p_model;
        delta_v_model = result_delta_v_model;
        delta_f_model = result_delta_f_model;

        delta_q.normalize();
        sum_dt += dt;
        acc_0 = acc_1;
        gyr_0 = gyr_1;
        Fz_0 = Fz_1;
    }

    Eigen::Matrix<double, 15, 1> evaluate(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
                                          const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj)
    {
        Eigen::Matrix<double, 15, 1> residuals;

        Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
        Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);

        Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);

        Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
        Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);

        Eigen::Vector3d dba = Bai - linearized_ba;
        Eigen::Vector3d dbg = Bgi - linearized_bg;
        dba_model = Bai - linearized_ba;// linearized_ba is optimized result lasttime, and Bai is the result this time
        dbg_model = Bgi - linearized_bg;

        Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);
        Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
        Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;

        residuals.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
        residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
        residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_delta_v;
        residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
        residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;

        // std::cout << "ri3: " << residuals.block<3, 1>(6, 0) << std::endl;

        return residuals;
    }

    Eigen::Matrix<double, 15, 1> evaluate_model(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Fexti, const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
                                               const Eigen::Vector3d &Pj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Fextj, const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj) const
    {
        Eigen::Matrix<double, 15, 1> residuals_model;

        Eigen::Vector3d corrected_delta_p_model = delta_p_model + jacobian_model.block<3, 3>(0, 9) * dbg_model;
        Eigen::Vector3d corrected_delta_v_model = delta_v_model + jacobian_model.block<3, 3>(6, 9) * dbg_model;
        Eigen::Vector3d corrected_delta_f_model = 1.0 / sum_dt * (delta_f_model + jacobian_model.block<3, 3>(15, 9) * dbg_model + jacobian_model.block<3, 3>(15, 12) * dba_model);   //bias correction，and force doesnot need to correct

        // std::cout << "dbg_model: " << dbg_model[0] << " " << dbg_model[1] << " " << dbg_model[2] << std::endl;
        // std::cout << "delta_f_model: " << delta_f_model << std::endl;
        // std::cout << "delta_p_model: " << delta_p_model << std::endl;
        // std::cout << "corrected_delta_f_model: " << corrected_delta_f_model << " " << corrected_delta_f_model - delta_f_model << std::endl;

        residuals_model.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - 0.5 * Fextj * sum_dt * sum_dt - corrected_delta_p_model;

        residuals_model.block<3, 1>(O_V - 3, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - Fextj * sum_dt - corrected_delta_v_model;
        residuals_model.block<3, 1>(6, 0) = Fextj - corrected_delta_f_model;  //NOTE: Fexti is in body_k frame  representing the average force between bk and bk+1, in bk frame
        // residuals_model.block<3, 1>(6, 0) = Fexti;  //NOTE: Fexti is in body_k frame  representing the average force between bk and bk+1, in bk frame
        // std::cout << "rd1: " << residuals_model.block<3, 1>(O_P, 0) << std::endl;
        // std::cout << "rd2: " << residuals_model.block<3, 1>(O_V-3, 0) << std::endl;
        // std::cout << "rd3: " << residuals_model.block<3, 1>(6, 0) << std::endl;
        // residuals_model.block<3, 1>(6, 0) = Eigen::Vector3d(0, 0, 0);  // set force in ed residual 0

        residuals_model.block<3, 1>(9, 0) = Baj - Bai;  //Eigen::Vector3d(0, 0, 0); //Baj - Bai;  //do not update ba in model residual
        // std::cout << "norm_ba: " << residuals_model.block<3, 1>(9, 0).norm() << std::endl;
        residuals_model.block<3, 1>(12, 0) = Eigen::Vector3d(0, 0, 0);  //TODO: do not update bw in model residual

        // std::cout << "sum_dt: " << sum_dt << std::endl;

        return residuals_model*1.0;   
    }


    double dt;
    Eigen::Vector3d acc_0, gyr_0;
    Eigen::Vector3d acc_1, gyr_1;
    double Fz_0, Fz_1;

    const Eigen::Vector3d linearized_acc, linearized_gyr;
    const double linearized_Fz;
    Eigen::Vector3d linearized_ba, linearized_bg;

    Eigen::Matrix<double, 15, 15> jacobian, covariance;
    Eigen::Matrix<double, 18, 18> noise;

    Eigen::Vector3d dba_model;
    Eigen::Vector3d dbg_model;
    Eigen::Vector3d df_model; 
    Eigen::Matrix<double, 18, 18> jacobian_model;
    Eigen::Matrix<double, 18, 18> covariance_model;
    Eigen::MatrixXd noise_model;

    double sum_dt;
    Eigen::Vector3d delta_p;
    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_v;

    Eigen::Vector3d delta_p_model;
    Eigen::Vector3d delta_v_model;
    Eigen::Vector3d delta_f_model;

    std::vector<double> dt_buf;
    std::vector<Eigen::Vector3d> acc_buf;
    std::vector<Eigen::Vector3d> gyr_buf;
    std::vector<double> Fz_buf;
    
    Quaterniond q_gt0_;
    Vector3d p_gt0_;
};

