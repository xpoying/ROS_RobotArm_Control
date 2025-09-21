// arm_control/src/Control_Node.cpp
#include "arm_control/Control_Node.h"
#include "rclcpp/parameter_event_handler.hpp"

#include <atomic>
#include <eigen3/Eigen/Geometry>
#include <chrono>

namespace motionplaning::resolve
{
ArmNode::ArmNode(const rclcpp::NodeOptions &options)
    : Node("arm_control_node", options),
      default_x_(0.35),
      default_y_(0.15),
      default_z_(0.45),
      default_roll_(0.0),
      default_pitch_(-0.2),
      default_yaw_(0.5),
      has_new_target_(false)
{
  // DH 参数
  this->declare_parameter<std::vector<double>>("dh_a", {0.0, -0.425, -0.3922, 0.0, 0.0, 0.0});
  this->declare_parameter<std::vector<double>>("dh_d", {0.1625, 0.0, 0.0, 0.1333, -0.0997, 0.0996});
  this->declare_parameter<std::vector<double>>("dh_alpha", {M_PI / 2, 0.0, 0.0, M_PI / 2, -M_PI / 2, 0.0});
  this->declare_parameter<std::vector<double>>("dh_offset", {0.0, 0.0, 0.0, 0.0, 0.0, M_PI});
  this->declare_parameter<std::vector<double>>("q_min", {-2 * M_PI, -2 * M_PI, -2 * M_PI, -2 * M_PI, -2 * M_PI, -2 * M_PI});
  this->declare_parameter<std::vector<double>>("q_max", {2 * M_PI, 2 * M_PI, 2 * M_PI, 2 * M_PI, 2 * M_PI, 2 * M_PI});
  this->declare_parameter<std::vector<double>>("q_cur", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  // 默认位姿
  this->declare_parameter("default_x", 0.35);
  this->declare_parameter("default_y", 0.15);
  this->declare_parameter("default_z", 0.45);
  this->declare_parameter("default_roll", 0.0);
  this->declare_parameter("default_pitch", -0.2);
  this->declare_parameter("default_yaw", 0.5);
  this->declare_parameter<double>("move_time", 2.0); // 秒

  auto validate = [&](const std::vector<double> &v, const char *name)
  {
    if (v.size() != 6)
    {
      RCLCPP_FATAL(get_logger(), "%s must have 6 elements, got %zu", name, v.size());
      rclcpp::shutdown();
    }
  };

  const auto dh_a     = get_parameter("dh_a").as_double_array();
  const auto dh_d     = get_parameter("dh_d").as_double_array();
  const auto dh_alpha = get_parameter("dh_alpha").as_double_array();
  const auto dh_off   = get_parameter("dh_offset").as_double_array();
  const auto q_min    = get_parameter("q_min").as_double_array();
  const auto q_max    = get_parameter("q_max").as_double_array();
  q_cur_              = get_parameter("q_cur").as_double_array();
  move_time_          = get_parameter("move_time").as_double();

  validate(dh_a, "dh_a");
  validate(dh_d, "dh_d");
  validate(dh_alpha, "dh_alpha");
  validate(dh_off, "dh_offset");
  validate(q_min, "q_min");
  validate(q_max, "q_max");
  validate(q_cur_, "q_cur");

  std::array<DH, 6> dh;
  for (int i = 0; i < 6; ++i)
  {
    dh[i].a            = dh_a[i];
    dh[i].d            = dh_d[i];
    dh[i].alpha        = dh_alpha[i];
    dh[i].theta_offset = dh_off[i];
  }

  ik_ = std::make_unique<UR_IK>(dh);
  if (!ik_)
  {
    RCLCPP_FATAL(get_logger(), "IK solver init failed!");
    rclcpp::shutdown();
  }

  q_min_.assign(q_min.begin(), q_min.end());
  q_max_.assign(q_max.begin(), q_max.end());

  default_x_.store(get_parameter("default_x").as_double());
  default_y_.store(get_parameter("default_y").as_double());
  default_z_.store(get_parameter("default_z").as_double());
  default_roll_.store(get_parameter("default_roll").as_double());
  default_pitch_.store(get_parameter("default_pitch").as_double());
  default_yaw_.store(get_parameter("default_yaw").as_double());

  // 参数回调
  param_cb_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  auto cb = [this](const rclcpp::Parameter &p)
  {
    if (p.get_name() == "default_x")       default_x_.store(p.as_double());
    else if (p.get_name() == "default_y")  default_y_.store(p.as_double());
    else if (p.get_name() == "default_z")  default_z_.store(p.as_double());
    else if (p.get_name() == "default_roll")  default_roll_.store(p.as_double());
    else if (p.get_name() == "default_pitch") default_pitch_.store(p.as_double());
    else if (p.get_name() == "default_yaw")   default_yaw_.store(p.as_double());
  };
  param_cb_handles_.push_back(param_cb_handler_->add_parameter_callback("default_x", cb));
  param_cb_handles_.push_back(param_cb_handler_->add_parameter_callback("default_y", cb));
  param_cb_handles_.push_back(param_cb_handler_->add_parameter_callback("default_z", cb));
  param_cb_handles_.push_back(param_cb_handler_->add_parameter_callback("default_roll", cb));
  param_cb_handles_.push_back(param_cb_handler_->add_parameter_callback("default_pitch", cb));
  param_cb_handles_.push_back(param_cb_handler_->add_parameter_callback("default_yaw", cb));

  // 发布 / 订阅
  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  pose_sub_  = create_subscription<geometry_msgs::msg::Pose>(
      "target_pose", 10,
      [this](geometry_msgs::msg::Pose::UniquePtr msg)
      {
        std::lock_guard<std::mutex> lock(target_pose_mutex_);
        target_pose_  = *msg;
        has_new_target_ = true;
        RCLCPP_INFO(get_logger(), "New target pose received");
      });

  // 50 Hz 控制循环
  timer_ = create_wall_timer(std::chrono::milliseconds(20),
                             std::bind(&ArmNode::control_loop, this));

  // 初始化 JointState
  joint_cmd_.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                     "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  joint_cmd_.position.resize(6, 0.0);

  RCLCPP_INFO(get_logger(), "Arm control node started");
}

void ArmNode::control_loop()
{
  geometry_msgs::msg::Pose target;
  bool use_default = true;
  double roll = 0.0, pitch = 0.0, yaw = 0.0;

  // 1. 读取目标
  {
    std::lock_guard<std::mutex> lock(target_pose_mutex_);
    if (has_new_target_)
    {
      target        = target_pose_;
      use_default   = false;
      has_new_target_ = false;

      Eigen::Quaterniond q(target.orientation.w,
                           target.orientation.x,
                           target.orientation.y,
                           target.orientation.z);
      q.normalize();
      Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0); // yaw-pitch-roll
      roll  = euler.z();
      pitch = euler.y();
      yaw   = euler.x();

      RCLCPP_DEBUG(get_logger(), "Target pose: [%.3f %.3f %.3f] RPY[%.3f %.3f %.3f]",
                   target.position.x, target.position.y, target.position.z,
                   roll, pitch, yaw);
    }
  }

  // 2. 若无目标，使用默认
  if (use_default)
  {
    target.position.x = default_x_.load();
    target.position.y = default_y_.load();
    target.position.z = default_z_.load();
    roll  = default_roll_.load();
    pitch = default_pitch_.load();
    yaw   = default_yaw_.load();
    RCLCPP_DEBUG(get_logger(), "Using default pose");
  }

  try
  {
    Eigen::VectorXd q_cur     = Eigen::Map<const Eigen::VectorXd>(q_cur_.data(), 6);
    Eigen::VectorXd q_min_vec = Eigen::Map<const Eigen::VectorXd>(q_min_.data(), 6);
    Eigen::VectorXd q_max_vec = Eigen::Map<const Eigen::VectorXd>(q_max_.data(), 6);

    Eigen::Matrix<double, 6, 1> solution =
        ik_->inverse_rpy(target.position.x, target.position.y, target.position.z,
                         roll, pitch, yaw, q_cur, q_min_vec, q_max_vec);

    // 快速关节限位检查（向量化）
    Eigen::ArrayXd sol_arr = solution.array();
    Eigen::ArrayXd min_arr = q_min_vec.array();
    Eigen::ArrayXd max_arr = q_max_vec.array();
    bool ok = (sol_arr >= min_arr && sol_arr <= max_arr).all();
    if (!ok)
    {
      for (int i = 0; i < 6; ++i)
        if (solution(i) < q_min_[i] || solution(i) > q_max_[i])
          RCLCPP_WARN(get_logger(), "Joint %d out of bounds: %.3f (%.3f, %.3f)",
                      i, solution(i), q_min_[i], q_max_[i]);
      throw std::runtime_error("Solution violates joint limits");
    }

    // 3. 轨迹机
    if (traj_step_.load() == 0)
    {
      traj_start_ = q_cur;
      const double dt = 0.02; // 50 Hz
      int total_steps = static_cast<int>(std::floor(move_time_.load() / dt));
      if (total_steps <= 0) total_steps = 1;
      traj_total_.store(total_steps);
      traj_delta_ = (solution - traj_start_) / total_steps;
      traj_step_.store(1);
    }

    Eigen::Matrix<double, 6, 1> traj =
        traj_start_ + traj_delta_ * traj_step_.load();
    publish_joint(traj);

    // 更新当前角
    Eigen::VectorXd::Map(q_cur_.data(), 6) = traj;

    int step = traj_step_.fetch_add(1);
    if (step >= traj_total_.load())
      traj_step_.store(0);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(get_logger(), "IK failed: %s", e.what());
    traj_step_.store(0);
    publish_current_joints();
  }
  catch (...)
  {
    RCLCPP_ERROR(get_logger(), "IK failed with unknown error");
    traj_step_.store(0);
    publish_current_joints();
  }
}

void ArmNode::publish_joint(const Eigen::Matrix<double, 6, 1> &q)
{
  sensor_msgs::msg::JointState js;
  js.header.stamp = now();
  js.header.frame_id = "base_link";
  js.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  js.position = {q(0), q(1), q(2), q(3), q(4), q(5)};
  joint_pub_->publish(js);
}

void ArmNode::publish_current_joints()
{
  sensor_msgs::msg::JointState js;
  js.header.stamp = now();
  js.header.frame_id = "base_link";
  js.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  js.position = q_cur_;
  joint_pub_->publish(js);
}
} // namespace motionplaning::resolve

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(motionplaning::resolve::ArmNode)