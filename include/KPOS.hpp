#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>  

#include <stdexcept>

class KPOS
{
public:
  void initialize(const Eigen::VectorXd& KUKA_l_q_init,
    const Eigen::VectorXd& KUKA_r_q_init,
     const Eigen::VectorXd& Indy12_q_init,
     const Eigen::VectorXd& SIM_Indy12_q_init);

  void setInput(const Eigen::VectorXd &next_q_l,
                         const Eigen::VectorXd &next_q_r,
                         const Eigen::VectorXd &indy12_tau,
                         int gripper_state);
  void setSimInput(const Eigen::VectorXd& indy12_tau,
                          const Eigen::VectorXd& CRS_F0,
                          const Eigen::VectorXd& TGT_F0,
                          const Eigen::VectorXd& CRS_RW_tau,
                          const Eigen::VectorXd& TGT_RW_tau, 
                          int gripper_state);
  // 결과 저장
  Eigen::VectorXd q_l = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd q_r = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd r_XYZ = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd r_ABC = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd l_XYZ = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd l_ABC = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd indy12_XYZ = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd indy12_ABC = Eigen::VectorXd::Zero(3);
  Eigen::Matrix3d indy12_R = Eigen::Matrix3d::Identity();
  Eigen::VectorXd gripper_q   = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd indy12_q    = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd indy12_qdot = Eigen::VectorXd::Zero(7);
  Eigen::Vector3d indy12_grav = Eigen::Vector3d::Zero();

  Eigen::VectorXd sim_indy12_q    = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd sim_indy12_qdot = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd sim_gripper_q   = Eigen::VectorXd::Zero(2);
  Eigen::Matrix4d sim_T_tcp_CRSTGT   = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d sim_T_com_CRSTGT   = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d sim_T_indy7_tcp_ring = Eigen::Matrix4d::Identity();
  Eigen::VectorXd sim_CRS_SatPos  = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd sim_TGT_SatPos  = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd sim_CRS_SatVel  = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd sim_TGT_SatVel  = Eigen::VectorXd::Zero(6);

};
