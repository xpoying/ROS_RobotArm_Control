#ifndef INVERSE_RESOLSE_H
#define INVERSE_RESOLSE_H

#include <eigen3/Eigen/Dense>
#include <array>
#include <vector>
#include <cmath>

namespace motionplaning::resolve
{
    struct DH
    {
        double a, d, alpha, theta_offset;
    };

    struct Cost
    {
        double joint_dist = 0.0;
        double limit_pen = 0.0;
        bool valid = true;
    };

    class UR_IK
    {
    public:
        explicit UR_IK(const std::array<DH, 6> &dh);

        // 1) 接口
        std::vector<Eigen::Matrix<double, 6, 1>> inverse(const Eigen::Isometry3d &T_des) const;
        Eigen::Matrix<double, 6, 1> pickBest(const std::vector<Eigen::Matrix<double, 6, 1>> &sols,
                                             const Eigen::Matrix<double, 6, 1> &q_cur,
                                             const Eigen::Matrix<double, 6, 1> &q_min,
                                             const Eigen::Matrix<double, 6, 1> &q_max) const;
        // 2) XYZ + RPY 欧拉角
        Eigen::Matrix<double, 6, 1> inverse_rpy(double x, double y, double z,
                                                double roll, double pitch, double yaw,
                                                const Eigen::Matrix<double, 6, 1> &q_cur = {},
                                                const Eigen::Matrix<double, 6, 1> &q_min = {},
                                                const Eigen::Matrix<double, 6, 1> &q_max = {}) const;

        // 3) 正运动学
        Eigen::Isometry3d forward(const Eigen::Matrix<double, 6, 1> &q) const;

    private:
        Cost evaluate(const Eigen::Matrix<double, 6, 1> &q,
                      const Eigen::Matrix<double, 6, 1> &q_cur,
                      const Eigen::Matrix<double, 6, 1> &q_min,
                      const Eigen::Matrix<double, 6, 1> &q_max) const;

        std::array<DH, 6> dh_;
    };
} // namespace motionplaning::resolve

#endif