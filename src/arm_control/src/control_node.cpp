//Control_Node

#include "arm_control/Control_Node.h"
#include "rclcpp/parameter_event_handler.hpp"


#include <atomic>
#include <eigen3/Eigen/Geometry>


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
    // dh 关节限制
    this->declare_parameter<std::vector<double>>("dh_a", {0.0, -0.425, -0.3922, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("dh_d", {0.1625, 0.0, 0.0, 0.1333, -0.0997, 0.0996});
    this->declare_parameter<std::vector<double>>("dh_alpha", {M_PI / 2, 0.0, 0.0, M_PI / 2, -M_PI / 2, 0.0});
    this->declare_parameter<std::vector<double>>("dh_offset", {0.0, 0.0, 0.0, 0.0, 0.0, M_PI});
    this->declare_parameter<std::vector<double>>("q_min", {-2 * M_PI, -2 * M_PI, -2 * M_PI, -2 * M_PI, -2 * M_PI, -2 * M_PI});
    this->declare_parameter<std::vector<double>>("q_max", {2 * M_PI, 2 * M_PI, 2 * M_PI, 2 * M_PI, 2 * M_PI, 2 * M_PI});
    this->declare_parameter<std::vector<double>>("q_cur", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    // 位姿参数
    this->declare_parameter("default_x", 0.35);
    this->declare_parameter("default_y", 0.15);
    this->declare_parameter("default_z", 0.45);
    this->declare_parameter("default_roll", 0.0);
    this->declare_parameter("default_pitch", -0.2);
    this->declare_parameter("default_yaw", 0.5);

    // 参数验证
    auto validate_params = [&](const auto &param, const char *name)
    {
      if (param.size() != 6)
      {
        RCLCPP_FATAL(
            this->get_logger(),
            "Invalid %s size: %zu. Must be 6 elements.",
            name, param.size());
        rclcpp::shutdown();
      }
    };

    const auto dh_a = this->get_parameter("dh_a").as_double_array();
    const auto dh_d = this->get_parameter("dh_d").as_double_array();
    const auto dh_alpha = this->get_parameter("dh_alpha").as_double_array();
    const auto dh_offset = this->get_parameter("dh_offset").as_double_array();
    const auto q_min = this->get_parameter("q_min").as_double_array();
    const auto q_max = this->get_parameter("q_max").as_double_array();
    q_cur_ = this->get_parameter("q_cur").as_double_array();

    validate_params(dh_a, "dh_a");
    validate_params(dh_d, "dh_d");
    validate_params(dh_alpha, "dh_alpha");
    validate_params(dh_offset, "dh_offset");
    validate_params(q_min, "q_min");
    validate_params(q_max, "q_max");
    validate_params(q_cur_, "q_cur");

    // 构造DH参数
    std::array<DH, 6> dh;
    for (int i = 0; i < 6; ++i)
    {
      dh[i].a = dh_a[i];
      dh[i].d = dh_d[i];
      dh[i].alpha = dh_alpha[i];
      dh[i].theta_offset = dh_offset[i];
    }

    // 初始化逆运动学求解器
    ik_ = std::make_unique<UR_IK>(dh);
    if (!ik_)
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize IK solver!");
      rclcpp::shutdown();
    }

    // 存储关节限位
    q_min_.assign(q_min.begin(), q_min.end());
    q_max_.assign(q_max.begin(), q_max.end());

    // 初始化默认位姿
    default_x_.store(this->get_parameter("default_x").as_double());
    default_y_.store(this->get_parameter("default_y").as_double());
    default_z_.store(this->get_parameter("default_z").as_double());
    default_roll_.store(this->get_parameter("default_roll").as_double());
    default_pitch_.store(this->get_parameter("default_pitch").as_double());
    default_yaw_.store(this->get_parameter("default_yaw").as_double());

    // 设置参数回调
    param_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    auto callback = [this](const rclcpp::Parameter &param) {
      if (param.get_name() == "default_x") {
        default_x_.store(param.as_double());
        RCLCPP_INFO(this->get_logger(), "Updated default_x: %.3f", param.as_double());
      } else if (param.get_name() == "default_y") {
        default_y_.store(param.as_double());
        RCLCPP_INFO(this->get_logger(), "Updated default_y: %.3f", param.as_double());
      } else if (param.get_name() == "default_z") {
        default_z_.store(param.as_double());
        RCLCPP_INFO(this->get_logger(), "Updated default_z: %.3f", param.as_double());
      } else if (param.get_name() == "default_roll") {
        default_roll_.store(param.as_double());
        RCLCPP_INFO(this->get_logger(), "Updated default_roll: %.3f", param.as_double());
      } else if (param.get_name() == "default_pitch") {
        default_pitch_.store(param.as_double());
        RCLCPP_INFO(this->get_logger(), "Updated default_pitch: %.3f", param.as_double());
      } else if (param.get_name() == "default_yaw") {
        default_yaw_.store(param.as_double());
        RCLCPP_INFO(this->get_logger(), "Updated default_yaw: %.3f", param.as_double());
      }
    };

    // 为每个参数注册回调
    cb_handles_.push_back(param_event_handler_->add_parameter_callback("default_x", callback));
    cb_handles_.push_back(param_event_handler_->add_parameter_callback("default_y", callback));
    cb_handles_.push_back(param_event_handler_->add_parameter_callback("default_z", callback));
    cb_handles_.push_back(param_event_handler_->add_parameter_callback("default_roll", callback));
    cb_handles_.push_back(param_event_handler_->add_parameter_callback("default_pitch", callback));
    cb_handles_.push_back(param_event_handler_->add_parameter_callback("default_yaw", callback));

    // 创建发布器和订阅器
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "target_pose", 10,
        [this](geometry_msgs::msg::Pose::UniquePtr msg)
        {
          std::lock_guard<std::mutex> lock(target_pose_mutex_);
          target_pose_ = *msg;
          has_new_target_ = true;
          RCLCPP_INFO(this->get_logger(), "New target pose received");
        });

    // 定时器 - 50Hz控制循环
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&ArmNode::control_loop, this));

    // 初始化关节命令消息
    joint_cmd_.name = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    joint_cmd_.position.resize(6, 0.0);

    RCLCPP_INFO(this->get_logger(), "Arm control node initialized");
    RCLCPP_INFO(this->get_logger(), "Default position: (%.2f, %.2f, %.2f)", 
                default_x_.load(), default_y_.load(), default_z_.load());
    RCLCPP_INFO(this->get_logger(), "Default orientation: (%.2f, %.2f, %.2f)",
                default_roll_.load(), default_pitch_.load(), default_yaw_.load());
  }

  void ArmNode::control_loop()
  {
    geometry_msgs::msg::Pose target;
    bool use_default = true;
    double roll = 0.0, pitch = 0.0, yaw = 0.0;

    // 检查是否有新目标
    {
      std::lock_guard<std::mutex> lock(target_pose_mutex_);
      if (has_new_target_)
      {
        target = target_pose_;
        use_default = false;
        has_new_target_ = false;

        // 转换四元数到RPY 
        Eigen::Quaterniond q(
            target.orientation.w,
            target.orientation.x,
            target.orientation.y,
            target.orientation.z);

        //四元数
        q.normalize();

        // 转换为欧拉角 (XYZ顺序对应RPY)
        Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
        roll = euler.x();
        pitch = euler.y();
        yaw = euler.z();
        
        RCLCPP_DEBUG(this->get_logger(), "Target pose: Position(%.3f, %.3f, %.3f), RPY(%.3f, %.3f, %.3f)",
                     target.position.x, target.position.y, target.position.z,
                     roll, pitch, yaw);
      }
    }

    if (use_default)
    {
      // 使用默认位姿
      target.position.x = default_x_.load();
      target.position.y = default_y_.load();
      target.position.z = default_z_.load();
      roll = default_roll_.load();
      pitch = default_pitch_.load();
      yaw = default_yaw_.load();
      
      RCLCPP_DEBUG(this->get_logger(), "Using default pose: Position(%.3f, %.3f, %.3f), RPY(%.3f, %.3f, %.3f)",
                   target.position.x, target.position.y, target.position.z,
                   roll, pitch, yaw);
    }

    try
    {
      // 准备当前关节状态
      Eigen::VectorXd q_cur(6);
      for (int i = 0; i < 6; ++i)
        q_cur(i) = q_cur_[i];

      Eigen::VectorXd q_min_vec(6);
      for (int i = 0; i < 6; ++i)
        q_min_vec(i) = q_min_[i];

      Eigen::VectorXd q_max_vec(6);
      for (int i = 0; i < 6; ++i)
        q_max_vec(i) = q_max_[i];

      // 逆运动学求解
      Eigen::Matrix<double, 6, 1> solution = ik_->inverse_rpy(
          target.position.x, target.position.y, target.position.z,
          roll, pitch, yaw,
          q_cur, q_min_vec, q_max_vec);

      // 检查关节限位
      bool within_limits = true;
      for (int i = 0; i < 6; i++)
      {
        if (solution(i) < q_min_[i] || solution(i) > q_max_[i])
        {
          within_limits = false;
          RCLCPP_WARN(this->get_logger(), "Joint %d out of bounds: %.3f (min: %.3f, max: %.3f)",
                      i, solution(i), q_min_[i], q_max_[i]);
        }
      }

      if (!within_limits) {
        throw std::runtime_error("Solution violates joint limits");
      }

      // 发布并更新关节状态
      publish_joint(solution);

      // 更新当前关节角度
      for (int i = 0; i < 6; i++)
      {
        q_cur_[i] = solution(i);
      }

      RCLCPP_DEBUG(this->get_logger(),
                   "IK Solution: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                   solution(0), solution(1), solution(2),
                   solution(3), solution(4), solution(5));
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "IK failed: %s", e.what());
      // 发布当前关节状态保持位置
      publish_current_joints();
    }
    catch (...)
    {
      RCLCPP_ERROR(this->get_logger(), "IK failed with unknown error");
      publish_current_joints();
    }
  }

  void ArmNode::publish_joint(const Eigen::Matrix<double, 6, 1> &q)
  {
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.header.frame_id = "base_link";
    js.name = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    js.position = {
        q(0), q(1), q(2), q(3), q(4), q(5)};

    joint_pub_->publish(js);
  }

  void ArmNode::publish_current_joints()
  {
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.header.frame_id = "base_link";
    js.name = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    js.position = q_cur_;
    joint_pub_->publish(js);
  }
} // namespace motionplaning::resolve

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(motionplaning::resolve::ArmNode)