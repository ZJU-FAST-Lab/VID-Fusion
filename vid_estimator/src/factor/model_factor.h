
#pragma once
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../parameters.h"
#include "integration_base.h"

#include <ceres/ceres.h>
class ModelFactor : public ceres::SizedCostFunction<15, 3, 4, 3, 3, 6, 3, 3, 3, 6> //15 d residual; jacobian size: position i, attitude i,  speed i, fext i, bai，bgi(don't change), position j, speed j, fext i, baj, bgi(don't change)
{
  public:
    ModelFactor() = delete;
    ModelFactor(IntegrationBase* _pre_integration):pre_integration(_pre_integration)
    {
    }
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {

        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[1][3], parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Vector3d Vi(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Vector3d Fexti(parameters[3][0], parameters[3][1], parameters[3][2]);
        Eigen::Vector3d Bai(parameters[4][0], parameters[4][1], parameters[4][2]);
        Eigen::Vector3d Bgi(parameters[4][3], parameters[4][4], parameters[4][5]);

        Eigen::Vector3d Pj(parameters[5][0], parameters[5][1], parameters[5][2]);
        Eigen::Vector3d Vj(parameters[6][0], parameters[6][1], parameters[6][2]);
        Eigen::Vector3d Fextj(parameters[7][0], parameters[7][1], parameters[7][2]);
        Eigen::Vector3d Baj(parameters[8][0], parameters[8][1], parameters[8][2]);
        Eigen::Vector3d Bgj(parameters[8][3], parameters[8][4], parameters[8][5]);


        Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
        residual = pre_integration->evaluate_model(Pi, Qi, Vi, Fexti, Bai, Bgi,
                                                   Pj, Vj, Fextj, Baj, Bgj);
        // std::cout << "evaluate_model" << std::endl;

        Eigen::Matrix<double, 15, 15> covariance_model_pvf;
        covariance_model_pvf.setIdentity();
        covariance_model_pvf.block<3, 3>(0, 0) = pre_integration->covariance_model.block<3, 3>(O_P, O_P);   //dp / dp   0
        covariance_model_pvf.block<3, 3>(0, 3) = pre_integration->covariance_model.block<3, 3>(O_P, O_V);   //dp / dv   6
        covariance_model_pvf.block<3, 3>(0, 6) = pre_integration->covariance_model.block<3, 3>(O_P, 15);    //dp / df   15
        covariance_model_pvf.block<3, 3>(0, 9) = pre_integration->covariance_model.block<3, 3>(O_P, 12);    //dp / dba  12
        covariance_model_pvf.block<3, 3>(0, 12) = pre_integration->covariance_model.block<3, 3>(O_P, 9);    //dp / dbw  9
        covariance_model_pvf.block<3, 3>(3, 0) = pre_integration->covariance_model.block<3, 3>(O_V, O_P);   
        covariance_model_pvf.block<3, 3>(3, 3) = pre_integration->covariance_model.block<3, 3>(O_V, O_V);
        covariance_model_pvf.block<3, 3>(3, 6) = pre_integration->covariance_model.block<3, 3>(O_V, 15);
        covariance_model_pvf.block<3, 3>(3, 9) = pre_integration->covariance_model.block<3, 3>(O_V, 12);
        covariance_model_pvf.block<3, 3>(3, 12) = pre_integration->covariance_model.block<3, 3>(O_V, 9);
        covariance_model_pvf.block<3, 3>(6, 0) = pre_integration->covariance_model.block<3, 3>(15, O_P);
        covariance_model_pvf.block<3, 3>(6, 3) = pre_integration->covariance_model.block<3, 3>(15, O_V);
        covariance_model_pvf.block<3, 3>(6, 6) = pre_integration->covariance_model.block<3, 3>(15, 15);
        covariance_model_pvf.block<3, 3>(6, 9) = pre_integration->covariance_model.block<3, 3>(15, 12);
        covariance_model_pvf.block<3, 3>(6, 12) = pre_integration->covariance_model.block<3, 3>(15, 9);
        covariance_model_pvf.block<3, 3>(9, 0) = pre_integration->covariance_model.block<3, 3>(12, O_P);
        covariance_model_pvf.block<3, 3>(9, 3) = pre_integration->covariance_model.block<3, 3>(12, O_V);
        covariance_model_pvf.block<3, 3>(9, 6) = pre_integration->covariance_model.block<3, 3>(12, 15);
        covariance_model_pvf.block<3, 3>(9, 9) = pre_integration->covariance_model.block<3, 3>(12, 12);
        covariance_model_pvf.block<3, 3>(9, 12) = pre_integration->covariance_model.block<3, 3>(12, 9);
        covariance_model_pvf.block<3, 3>(12, 0) = pre_integration->covariance_model.block<3, 3>(9, O_P);
        covariance_model_pvf.block<3, 3>(12, 3) = pre_integration->covariance_model.block<3, 3>(9, O_V);
        covariance_model_pvf.block<3, 3>(12, 6) = pre_integration->covariance_model.block<3, 3>(9, 15);
        covariance_model_pvf.block<3, 3>(12, 9) = pre_integration->covariance_model.block<3, 3>(9, 12);
        covariance_model_pvf.block<3, 3>(12, 12) = pre_integration->covariance_model.block<3, 3>(9, 9);
        
        // std::cout << "covariance_model_pvf_norm: " << std::endl << covariance_model_pvf.norm() << std::endl;
        Eigen::Matrix<double, 15, 15> info_matrix(covariance_model_pvf.inverse());
        // info_matrix.setZero();
        // info_matrix.block<15, 15>(0,0) = covariance_model_pvf.inverse();
        // info_matrix.block<15, 15>(0,0) = covariance_model_pvf.inverse();
        // info_matrix.block<15, 15>(0,0) = pre_integration->covariance_model.inverse();
        // std::cout << "info_matrix_norm: " << std::endl << info_matrix.norm() << std::endl;

        Eigen::Matrix<double, 15, 15> sqrt_info;
        sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(info_matrix).matrixL().transpose();
        // cout << "ed_norm: " << residual.norm() << endl;
        residual = sqrt_info * residual;
        // if(residual.norm() > 30)
        // {
        //     cout << "sqrt_info: " << sqrt_info << endl;
        //     // cout << "sqrt_info: " << sqrt_info << endl;
        // }
        // cout << "ed_norm_info: " << residual.norm() << endl;

        if (jacobians)
        {
            double sum_dt = pre_integration->sum_dt;
            //  dx / dbias_k
            Eigen::Matrix3d dp_dba = pre_integration->jacobian_model.template block<3, 3>(O_P, 12);
            Eigen::Matrix3d dv_dba = pre_integration->jacobian_model.template block<3, 3>(O_V, 12);
            Eigen::Matrix3d df_dba = pre_integration->jacobian_model.template block<3, 3>(O_F, 12);

            if (jacobians[0]) // derivative of residual wrt the first parameter block i.e. 3D position_i
            {
                Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobian_position_i(jacobians[0]);
                jacobian_position_i.setZero();

                jacobian_position_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();

                jacobian_position_i = sqrt_info * jacobian_position_i;

                if (jacobian_position_i.maxCoeff() > 1e8 || jacobian_position_i.minCoeff() < -1e8)
                {
                    ROS_WARN("numerical unstable in jacobian of model residual wrt position_i");
                    //std::cout << sqrt_info << std::endl;
                    //ROS_BREAK();
                }
                ROS_DEBUG_STREAM_ONCE("Model jacobian_position_i after:" << jacobian_position_i);
            }
            if (jacobians[1]) // derivative of residual wrt parameter block i.e. 4D attitude_i
            {
                Eigen::Map<Eigen::Matrix<double, 15, 4, Eigen::RowMajor>> jacobian_attitude_i(jacobians[1]);  //TODO: 9,3 ????
                jacobian_attitude_i.setZero();

                jacobian_attitude_i.block<3, 3>(O_P, O_R-O_R) = Utility::skewSymmetric(Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

                jacobian_attitude_i.block<3, 3>(O_V-3, O_R-O_R) = Utility::skewSymmetric(Qi.inverse() * (G  * sum_dt + Vj - Vi));
              
                jacobian_attitude_i = sqrt_info * jacobian_attitude_i;

                if (jacobian_attitude_i.maxCoeff() > 1e8 || jacobian_attitude_i.minCoeff() < -1e8)
                {
                    ROS_WARN("numerical unstable in jacobian of model residual wrt attitude_i");
                    //std::cout << sqrt_info << std::endl;
                    //ROS_BREAK();
                }
            }
            if (jacobians[2]) // derivative of residual wrt parameter block i.e. 3D speed i
            {
                Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobian_speed_i(jacobians[2]);
                jacobian_speed_i.setZero();
                
                jacobian_speed_i.block<3, 3>(O_P, O_V - O_V) = -Qi.inverse().toRotationMatrix() * sum_dt;

                jacobian_speed_i.block<3, 3>(O_V-3, O_V - O_V) = -Qi.inverse().toRotationMatrix();

                jacobian_speed_i = sqrt_info * jacobian_speed_i;

                if (jacobian_speed_i.maxCoeff() > 1e8 || jacobian_speed_i.minCoeff() < -1e8)
                {
                    ROS_WARN("numerical unstable in jacobian of model residual wrt speed_i");
                    //std::cout << sqrt_info << std::endl;
                    //ROS_BREAK();
                }

                //ROS_ASSERT(fabs(jacobian_speed_i.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_speed_i.minCoeff()) < 1e8);
            }
            if (jacobians[3]) // derivative of residual wrt  3D external force i
            {
                Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobian_fext_i(jacobians[3]);
                jacobian_fext_i.setZero();

                // jacobian_fext_i.block<3, 3>(O_P, 0) = - 0.5 * sum_dt * sum_dt * Eigen::Matrix3d::Identity();
                // jacobian_fext_i.block<3, 3>(O_V-3, 0) = - sum_dt * Eigen::Matrix3d::Identity();
                // jacobian_fext_i.block<3, 3>(6, 0) = Eigen::Matrix3d::Identity();
                // jacobian_fext_i.block<3, 3>(6, 0) = Eigen::Matrix3d::Zero(); // set force in ed residual 0

                jacobian_fext_i = sqrt_info * jacobian_fext_i;

                if (jacobian_fext_i.maxCoeff() > 1e8 || jacobian_fext_i.minCoeff() < -1e8)
                {
                    ROS_WARN("numerical unstable in jacobian of model residual wrt fext_i");
                    //std::cout << sqrt_info << std::endl;
                    //ROS_BREAK();
                }

                //ROS_ASSERT(fabs(jacobian_fext_i.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_fext_i.minCoeff()) < 1e8);
            }
            if (jacobians[4]) // derivative of residual wrt the parameter block bias i
            {
                Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor>> jacobian_bias_i(jacobians[4]);
                jacobian_bias_i.setZero();
                jacobian_bias_i.block<3, 3>(O_P, 0) = -dp_dba;
                jacobian_bias_i.block<3, 3>(3, 0) = -dv_dba;
                // std::cout << "dp_dba: " << dp_dba << std::endl << " dv_dba: " << dv_dba <<std::endl;
                jacobian_bias_i.block<3, 3>(6, 0) = -1.0 / sum_dt * df_dba;  //~= I
                // std::cout << "df_dba: " << df_dba << std::endl << jacobian_bias_i.block<3, 3>(6, 0) << std::endl;  
                jacobian_bias_i.block<3, 3>(9, 0) = -Eigen::Matrix3d::Identity();

                jacobian_bias_i = sqrt_info * jacobian_bias_i;

                //ROS_ASSERT(fabs(jacobian_bias_i.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_bias_i.minCoeff()) < 1e8);
            }
            if (jacobians[5]) // derivative of residual wrt  3D position j
            {
                Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobian_position_j(jacobians[5]);
                jacobian_position_j.setZero();

                jacobian_position_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

                jacobian_position_j = sqrt_info * jacobian_position_j;
                ROS_DEBUG_STREAM_ONCE("Model jacobian_position_j after:" << jacobian_position_j);

                if (jacobian_position_j.maxCoeff() > 1e8 || jacobian_position_j.minCoeff() < -1e8)
                {
                    ROS_WARN("numerical unstable in jacobian of model residual wrt position_j");
                    //std::cout << sqrt_info << std::endl;
                    //ROS_BREAK();
                }

                //ROS_ASSERT(fabs(jacobian_position_j.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_position_j.minCoeff()) < 1e8);
            }
            if (jacobians[6]) // derivative of residual wrt parameter block i.e. 3D speed j
            {
                Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobian_speed_j(jacobians[6]);
                jacobian_speed_j.setZero();

                jacobian_speed_j.block<3, 3>(O_V-3, O_V - O_V) = Qi.inverse().toRotationMatrix();

                jacobian_speed_j = sqrt_info * jacobian_speed_j;

                if (jacobian_speed_j.maxCoeff() > 1e8 || jacobian_speed_j.minCoeff() < -1e8)
                {
                    ROS_WARN("numerical unstable in jacobian of model residual wrt speed_j");
                    //std::cout << sqrt_info << std::endl;
                    //ROS_BREAK();
                }

                //ROS_ASSERT(fabs(jacobian_speed_j.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_speed_j.minCoeff()) < 1e8);
            }
            if (jacobians[7])  // derivative of residual wrt  3D external force j
            {
                Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobian_fext_j(jacobians[7]);
                jacobian_fext_j.setZero();

                jacobian_fext_j.block<3, 3>(O_P, 0) = - 0.5 * sum_dt * sum_dt * Eigen::Matrix3d::Identity();
                jacobian_fext_j.block<3, 3>(O_V-3, 0) = - sum_dt * Eigen::Matrix3d::Identity();
                jacobian_fext_j.block<3, 3>(6, 0) = Eigen::Matrix3d::Identity();


                // jacobian_bias_j.block<3, 3>(O_BG, O_BG - O_BA) = Eigen::Matrix3d::Identity();

                jacobian_fext_j = sqrt_info * jacobian_fext_j;

                //ROS_ASSERT(fabs(jacobian_bias_j.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_bias_j.minCoeff()) < 1e8);
            }
            if (jacobians[8])  // derivative of residual wrt the parameter block bias j
            {
                Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor>> jacobian_bias_j(jacobians[8]);
                jacobian_bias_j.setZero();

                jacobian_bias_j.block<3, 3>(9, 0) = Eigen::Matrix3d::Identity();

                // jacobian_bias_j.block<3, 3>(O_BG, O_BG - O_BA) = Eigen::Matrix3d::Identity();

                jacobian_bias_j = sqrt_info * jacobian_bias_j;

                //ROS_ASSERT(fabs(jacobian_bias_j.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_bias_j.minCoeff()) < 1e8);
            }
        }

        return true;
    }

    IntegrationBase* pre_integration;

};
