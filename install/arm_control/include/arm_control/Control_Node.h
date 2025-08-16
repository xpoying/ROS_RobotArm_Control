#ifndef ARM_CONTROL_CONTROL_NODE_H
#define ARM_CONTROL_CONTROL_NODE_H

#include "arm_control/Inverse_Resolse.h"
#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <eigen3/Eigen/Dense>
#include <atomic>
#include <mutex>
#include <vector>
#include <memory>



namespace motionplaning::resolve
{



class ArmNode : public rclcpp::Node
{
public:
  explicit ArmNode(const rclcpp::NodeOptions &options);
  
private:
  void control_loop();
  void publish_joint(const Eigen::Matrix<double, 6, 1> &q);
  void publish_current_joints();

  // 逆运动学求解器
  std::unique_ptr<UR_IK> ik_;
  
  // 关节限位
  std::vector<double> q_min_;
  std::vector<double> q_max_;
  std::vector<double> q_cur_; // 当前关节角度
  
  // 位姿参数（原子变量保证线程安全）
  std::atomic<double> default_x_;
  std::atomic<double> default_y_;
  std::atomic<double> default_z_;
  std::atomic<double> default_roll_;
  std::atomic<double> default_pitch_;
  std::atomic<double> default_yaw_;
  
  // 目标位姿处理
  geometry_msgs::msg::Pose target_pose_;
  bool has_new_target_;
  std::mutex target_pose_mutex_;
  
  // ROS接口
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // 参数处理
  std::shared_ptr<rclcpp::ParameterEventHandler> param_event_handler_;
  std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> cb_handles_;
  
  // 关节状态消息缓存
  sensor_msgs::msg::JointState joint_cmd_;
};

} // namespace motionplaning::resolve

#endif // ARM_CONTROL_CONTROL_NODE_H