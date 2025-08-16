# Robot Arm Control 
  By IMCA xwp
# 1.1介绍
  ## 1.2环境
    环境:ROS 2 Humble 
            C++ 
            PY3 
            Rviz  
            Eigen3

  ## 1.2内容
     机械臂逆运动解算(满足Pieper条件)
     不考虑速度 关节限制 障碍物 
     使用rviz可视化机械臂验证
     机械臂模型: https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git

  ## 1.3 算法原理  
  
关节i	aᵢ（连杆长）	dᵢ（偏距）	αᵢ（扭角）	θᵢ（变量角）
 1	    0	            d₁	        +90°	   θ₁
 2	    a₂	          0	           0°	     θ₂
 3	    a₃	          0	           0°	     θ₃
 4	    0	            d₄	        +90°	   θ₄
 5	    0	            d₅	        –90°	   θ₅
 6	    0	            d₆	         0°	     θ₆

给定末端位姿
T₆⁰ = [ R   p ]
      [ 0   1 ]
求 θ₁ … θ₆。

满足Pieper条件
(1) 位置逆解（3-DOF）：由 W 的坐标反求 θ₁,θ₂,θ₃；
(2) 姿态逆解（3-DOF）：由 R 反求 θ₄,θ₅,θ₆。

腕心坐标
设末端坐标系原点为 P，则腕心
W = P – d₆ · R · ẑ₆
（ẑ₆ 是末端 z 轴单位向量，在 0 系下为 R 的第 3 列。）
把 W 记为 (x, y, z)。

求 θ₁,θ₂,θ₃（位置）
────────────────
Step 1 – θ₁
把腕心投影到 x-y 平面，令 r = √(x²+y²)。
有
θ₁ = atan2(y, x)    (主值)
以及
θ₁′ = θ₁ + π    (对称解，共 2 个)

Step 2 – θ₃
在 2-3 连杆平面内，用余弦定理：
r² + (z – d₁)² = a₂² + a₃² + 2 a₂ a₃ cosθ₃
⇒ cosθ₃ = (r² + (z – d₁)² – a₂² – a₃²) / (2 a₂ a₃)
因此
θ₃ = ± arccos( … )    (共 2 个)
Step 3 – θ₂
把 θ₃ 代回三角形，得
θ₂ = atan2(z – d₁, r) – atan2(a₃ sinθ₃, a₂ + a₃ cosθ₃)
至此 θ₁/θ₂/θ₃ 的组合共 2×2×1 = 4 组（再乘 2 种 θ₁ 共 8 组）。

求 θ₄,θ₅,θ₆（姿态）
────────────────
把前三轴的旋转矩阵
R₀₃ = Rotz(θ₁) · Rotx(90°) · Rotz(θ₂) · Rotz(θ₃)
则
R₃₆ = R₀₃ᵀ · R_des
把 R₃₆ 写成 Z-Y-Z 欧拉角形式：
R₃₆ = Rotz(θ₄) · Roty(θ₅) · Rotz(θ₆)
对比矩阵元素得：
θ₅ = atan2(√(R₃₆(0,2)² + R₃₆(1,2)²), R₃₆(2,2))  (主值)
θ₅′ = -θ₅                 (腕翻转)
θ₄ = atan2(R₃₆(1,2)/sinθ₅, R₃₆(0,2)/sinθ₅)
θ₆ = atan2(R₃₆(2,1)/sinθ₅, -R₃₆(2,0)/sinθ₅)
当 sinθ₅ ≈ 0（奇异）时，取 θ₄ = 0，θ₆ = atan2(-R₃₆(1,0), R₃₆(0,0))。
每种 (θ₁,θ₂,θ₃) 组合再对应 2 种腕翻转，故总解数 8 组。

R₃₆ =
⎡ c₄c₅c₆ − s₄s₆   −c₄c₅s₆ − s₄c₆   c₄s₅ ⎤
⎢ s₄c₅c₆ + c₄s₆   −s₄c₅s₆ + c₄c₆   s₄s₅ ⎥
⎣ −s₅c₆            s₅s₆            c₅ ⎦

## 2.1使用
   可能需要改一下 src/arm_control/include/arm_control/Inverse_Resolse.h
                                                    Control_Node.h      
                                                    这两个的头文件这一段： #include <eigen3/Eigen/Dense>
                                                    改成                #include <Eigen/Dense>
### 2.2运行
    colcon build
    source install/setup.bash 
    ros2 launch ur_description view_ur.launch.py ur_type:=ur5e
    新终端
    source /opt/ros/humble/setup.bash
    ros2 run rqt_gui rqt_gui  //动态调整参数

### 2.3 参数修改
    src/arm_control/src/control_node.cpp  

默认参数
```cpp    
    // dh 关节限制
        this->declare_parameter<std::vector<double>>("dh_a", {0.0, -0.425, -0.3922, 0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("dh_d", {0.1625, 0.0, 0.0, 0.1333, -0.0997, 0.0996});
        this->declare_parameter<std::vector<double>>("dh_alpha", {M_PI/2, 0.0, 0.0, M_PI/2, -M_PI/2, 0.0});
        this->declare_parameter<std::vector<double>>("dh_offset", {0.0, 0.0, 0.0, 0.0, 0.0, M_PI});
        this->declare_parameter<std::vector<double>>("q_min", {-2*M_PI, -2*M_PI, -2*M_PI, -2*M_PI, -2*M_PI, -2*M_PI});
        this->declare_parameter<std::vector<double>>("q_max", {2*M_PI, 2*M_PI, 2*M_PI, 2*M_PI, 2*M_PI, 2*M_PI});
        this->declare_parameter<std::vector<double>>("q_cur", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});


    // 位姿参数
        this->declare_parameter("default_x", 0.35);
        this->declare_parameter("default_y", 0.15);
        this->declare_parameter("default_z", 0.45);
        this->declare_parameter("default_roll", 0.0);
        this->declare_parameter("default_pitch", -0.2);
        this->declare_parameter("default_yaw", 0.5);


算法
```cpp
    const double EPS = 1e-8;
        std::vector<Eigen::Matrix<double, 6, 1>> sols;

        Eigen::Vector3d P = T_des.translation();
        const double d1 = dh_[0].d, a2 = dh_[1].a, a3 = dh_[2].a,
                     d4 = dh_[3].d, d5 = dh_[4].d, d6 = dh_[5].d;
        (void)d4;
        (void)d5;

        // 腕心 WC
        Eigen::Vector3d WC = P - d6 * T_des.linear().col(2); // 用实际姿态
        double R = hypot(WC.x(), WC.y());
        double Z = WC.z() - d1;

        double r_sq = R * R + Z * Z;
        double c3_nom = r_sq - a2 * a2 - a3 * a3;
        double c3_den = 2 * a2 * a3;
        if (std::fabs(c3_nom / c3_den) > 1.0)
            return sols;

        double th3_a = std::acos(c3_nom / c3_den);
        double th3_b = -th3_a;

        for (double th3 : {th3_a, th3_b})
        {
            double s3 = std::sin(th3), c3 = std::cos(th3);
            double th2 = std::atan2(Z, R) - std::atan2(a3 * s3, a2 + a3 * c3);

            for (double th1 : {std::atan2(WC.y(), WC.x()),
                               std::atan2(-WC.y(), -WC.x())})
            {
                Eigen::Matrix<double, 3, 1> q123(th1, th2, th3);

                // 计算 R03
                Eigen::Isometry3d T03 = Eigen::Isometry3d::Identity();
                for (int i = 0; i < 3; ++i)
                {
                    const auto &h = dh_[i];
                    double theta = (i == 0 ? th1 : (i == 1 ? th2 : th3)) + h.theta_offset;
                    double ct = std::cos(theta), st = std::sin(theta);
                    double ca = std::cos(h.alpha), sa = std::sin(h.alpha);
                    Eigen::Matrix4d Ti;
                    Ti << ct, -st, 0, h.a,
                        st * ca, ct * ca, -sa, -sa * h.d,
                        st * sa, ct * sa, ca, ca * h.d,
                        0, 0, 0, 1;
                    T03 = T03 * Ti;
                }

                Eigen::Matrix3d R03 = T03.linear();
                Eigen::Matrix3d R36 = R03.transpose() * T_des.linear();

                // Z-Y-Z 分解
                double th5 = std::atan2(std::hypot(R36(0, 2), R36(1, 2)), R36(2, 2));
                if (std::fabs(std::sin(th5)) < EPS)
                    th5 = 0;

                double th4, th6;
                if (std::fabs(std::sin(th5)) < EPS)
                { // 奇异
                    th4 = 0;
                    th6 = std::atan2(-R36(1, 0), R36(0, 0));
                }
                else
                {
                    th4 = std::atan2(R36(1, 2), R36(0, 2));
                    th6 = std::atan2(R36(2, 1), -R36(2, 0));
                }

                Eigen::Matrix<double, 6, 1> q;
                q << th1, th2, th3, th4, th5, th6;
                sols.push_back(q);

                // 腕翻转
                q[3] += M_PI;
                q[4] = -q[4];
                q[5] += M_PI;
                sols.push_back(q);
            }
        }