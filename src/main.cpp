// kpos_node/src/main.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <Eigen/Geometry>
#include "KPOS.hpp"

using std::placeholders::_1;

class KPOSNode : public rclcpp::Node {
private:
  KPOS kpos_;

  Eigen::VectorXd kuka_l_init_ = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd kuka_r_init_ = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd indy12_q_init_ = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd sim_indy12_q_init_ = Eigen::VectorXd::Zero(7);

  Eigen::VectorXd next_q_l_ = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd next_q_r_ = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd indy12_tau_ = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd CRS_F0_ = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd TGT_F0_ = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd CRS_RW_tau_ = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd TGT_RW_tau_ = Eigen::VectorXd::Zero(6);
  int gripper_state_ = 0;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_kl_js_, pub_kr_js_, pub_i12_js_, pub_sim_js_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_kl_tcp_, pub_kr_tcp_, pub_i12_pose_, pub_sim_crs_pose_, pub_sim_tgt_pose_, pub_sim_tcp_, pub_sim_com_, pub_sim_ring_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_sim_crs_vel_, pub_sim_tgt_vel_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_next_q_l_, sub_next_q_r_, sub_i12_tau_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_gripper_, sub_sim_gripper_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_sim_tau_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_CRS_F0_, sub_TGT_F0_, sub_CRS_RW_tau_, sub_TGT_RW_tau_;

  std::vector<std::string> joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
public:
  KPOSNode() : Node("kpos_node") {
    indy12_q_init_ << 0, -M_PI_2, 0, 0, 0, 0, 0;
    sim_indy12_q_init_ = indy12_q_init_;
    kuka_l_init_ << 0, -M_PI_2, -M_PI_2, M_PI / 1.8, 0, -M_PI / 18, 0;
    kuka_r_init_ << 0, -M_PI_2, M_PI / 1.8, 0, -M_PI / 18, 0;

    kpos_.initialize(kuka_l_init_, kuka_r_init_, indy12_q_init_, sim_indy12_q_init_);

    // Publishers
    pub_kl_js_ = create_publisher<sensor_msgs::msg::JointState>("/KUKA/Left/joint_states", 10);
    pub_kr_js_ = create_publisher<sensor_msgs::msg::JointState>("/KUKA/Right/joint_states", 10);
    pub_kl_tcp_ = create_publisher<geometry_msgs::msg::PoseStamped>("/KUKA/Left/tcp", 10);
    pub_kr_tcp_ = create_publisher<geometry_msgs::msg::PoseStamped>("/KUKA/Right/tcp", 10);
    pub_i12_js_ = create_publisher<sensor_msgs::msg::JointState>("/Indy12/joint_states", 10);
    pub_i12_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("/Indy12/base", 10);
    pub_sim_js_ = create_publisher<sensor_msgs::msg::JointState>("/SIM/CRS/joint_states", 10);
    pub_sim_crs_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("/SIM/CRS/pose", 10);
    pub_sim_tgt_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("/SIM/TGT/pose", 10);
    pub_sim_tcp_ = create_publisher<geometry_msgs::msg::PoseStamped>("/SIM/T_tcp_CRSTGT", 10);
    pub_sim_com_ = create_publisher<geometry_msgs::msg::PoseStamped>("/SIM/T_com_CRSTGT", 10);
    pub_sim_ring_ = create_publisher<geometry_msgs::msg::PoseStamped>("/SIM/T_indytcp_ring", 10);
    pub_sim_crs_vel_ = create_publisher<geometry_msgs::msg::Twist>("/SIM/CRS/vel", 10);
    pub_sim_tgt_vel_ = create_publisher<geometry_msgs::msg::Twist>("/SIM/TGT/vel", 10);

    // Subscribers
    sub_next_q_l_ = create_subscription<sensor_msgs::msg::JointState>(
      "/KUKA/Left/command/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (int i = 0; i < 7; ++i) next_q_l_(i) = msg->position[i];
      });

    sub_next_q_r_ = create_subscription<sensor_msgs::msg::JointState>(
      "/KUKA/Right/command/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (int i = 0; i < 6; ++i) next_q_r_(i) = msg->position[i];
      });

    sub_i12_tau_ = create_subscription<sensor_msgs::msg::JointState>(
      "/Indy12/command/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (int i = 0; i < 7; ++i) indy12_tau_(i) = msg->effort[i];
      });

    sub_gripper_ = create_subscription<std_msgs::msg::Int32>(
      "/Indy12/command/GripperState", 10,
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        gripper_state_ = msg->data;
      });

    timer_ = create_wall_timer(std::chrono::milliseconds(1), std::bind(&KPOSNode::update, this));

    // SIM 입력 관련 Subscriber 추가
    sub_sim_tau_ = create_subscription<sensor_msgs::msg::JointState>(
      "/SIM/CRS/command/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (int i = 0; i < 7; ++i) indy12_tau_(i) = msg->effort[i];
      });

    sub_CRS_F0_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/SIM/CRS/command/wrench", 10,
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        CRS_F0_ << msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z,
                   msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;
      });

    sub_TGT_F0_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/SIM/TGT/command/wrench", 10,
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        TGT_F0_ << msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z,
                   msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;
      });

    sub_CRS_RW_tau_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/SIM/CRS/command/rw_joint_states", 10,
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        CRS_RW_tau_ << msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z,
                      msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;
      });

    sub_TGT_RW_tau_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/SIM/TGT/command/rw_joint_states", 10,
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        TGT_RW_tau_ << msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z,
                      msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;
      });

    sub_sim_gripper_ = create_subscription<std_msgs::msg::Int32>(
      "/SIM/CRS/command/GripperState", 10,
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        gripper_state_ = msg->data;
      });
  }

  void update() {
    kpos_.setInput(next_q_l_, next_q_r_, indy12_tau_, gripper_state_);
    kpos_.setSimInput(indy12_tau_, CRS_F0_, TGT_F0_, CRS_RW_tau_, TGT_RW_tau_, gripper_state_);

    auto pub_pose = [&](const Eigen::Vector3d &p, const Eigen::Vector3d &r, auto pub) {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.stamp = now();
      msg.pose.position.x = p(0);
      msg.pose.position.y = p(1);
      msg.pose.position.z = p(2);
      Eigen::Quaterniond q(Eigen::AngleAxisd(r(2), Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(r(1), Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(r(0), Eigen::Vector3d::UnitZ()));
      msg.pose.orientation.x = q.x(); msg.pose.orientation.y = q.y();
      msg.pose.orientation.z = q.z(); msg.pose.orientation.w = q.w();
      pub->publish(msg);
    };

    pub_pose(kpos_.l_XYZ, kpos_.l_ABC, pub_kl_tcp_);
    pub_pose(kpos_.r_XYZ, kpos_.r_ABC, pub_kr_tcp_);
    pub_pose(kpos_.indy12_XYZ, kpos_.indy12_ABC, pub_i12_pose_);

    auto pub_js = [&](auto pub, const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& grip, const std::vector<std::string>& names) {
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = now();
      msg.name = names;
      msg.position.assign(q.data(), q.data() + q.size());
      msg.velocity.assign(qd.data(), qd.data() + qd.size());
      msg.effort.resize(msg.position.size(), 0.0);
      if (grip.size() > 0) msg.position.push_back(grip(0));
      if (grip.size() > 1) msg.position.push_back(grip(1));
      pub->publish(msg);
    };

    pub_js(pub_kl_js_, kpos_.q_l, Eigen::VectorXd::Zero(7), Eigen::VectorXd(), joint_names_);
    pub_js(pub_kr_js_, kpos_.q_r, Eigen::VectorXd::Zero(6), Eigen::VectorXd(), joint_names_);
    pub_js(pub_i12_js_, kpos_.indy12_q, kpos_.indy12_qdot, kpos_.gripper_q, joint_names_);
    pub_js(pub_sim_js_, kpos_.sim_indy12_q, kpos_.sim_indy12_qdot, kpos_.sim_gripper_q, joint_names_);

    auto pub_sat = [&](const Eigen::VectorXd& sat, auto pub) {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.stamp = now();
      msg.pose.position.x = sat(4);
      msg.pose.position.y = sat(5);
      msg.pose.position.z = sat(6);
      msg.pose.orientation.w = sat(0);
      msg.pose.orientation.x = sat(1);
      msg.pose.orientation.y = sat(2);
      msg.pose.orientation.z = sat(3);
      pub->publish(msg);
    };
    pub_sat(kpos_.sim_CRS_SatPos, pub_sim_crs_pose_);
    pub_sat(kpos_.sim_TGT_SatPos, pub_sim_tgt_pose_);

    auto pub_mat = [&](const Eigen::Matrix4d& T, auto pub) {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.stamp = now();
      msg.pose.position.x = T(0, 3);
      msg.pose.position.y = T(1, 3);
      msg.pose.position.z = T(2, 3);
      Eigen::Quaterniond q(T.block<3,3>(0,0));
      msg.pose.orientation.x = q.x(); msg.pose.orientation.y = q.y();
      msg.pose.orientation.z = q.z(); msg.pose.orientation.w = q.w();
      pub->publish(msg);
    };
    pub_mat(kpos_.sim_T_tcp_CRSTGT, pub_sim_tcp_);
    pub_mat(kpos_.sim_T_com_CRSTGT, pub_sim_com_);
    pub_mat(kpos_.sim_T_indy7_tcp_ring, pub_sim_ring_);

    auto pub_twist = [&](const Eigen::VectorXd& v, auto pub) {
      geometry_msgs::msg::Twist msg;
      msg.angular.x = v(0);
      msg.angular.y = v(1);
      msg.angular.z = v(2);
      msg.linear.x = v(3);
      msg.linear.y = v(4);
      msg.linear.z = v(5);
      pub->publish(msg);
    };
    pub_twist(kpos_.sim_CRS_SatVel, pub_sim_crs_vel_);
    pub_twist(kpos_.sim_TGT_SatVel, pub_sim_tgt_vel_);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KPOSNode>());
  rclcpp::shutdown();
  return 0;
}
