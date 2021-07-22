#ifndef PR_KALMAN_FILTER_HPP_
#define PR_KALMAN_FILTER_HPP_

// This Kalman filter is used for smoothing position and velocity
// It assumes a constant-velocity model

#include "rclcpp/rclcpp.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"

#include <array>
#include <vector>

#include "pr_lib/pr_utils.hpp"
#include "eigen3/Eigen/Dense"

namespace pr_aux
{
    class KalmanFilter : public rclcpp::Node
    {
        public:
            //PR_AUX_PUBLIC
            explicit KalmanFilter(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr pos_msg);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_pos;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_vel;

            double ts, q, r;
            std::vector<double> init_val;
            Eigen::Vector4d pos, pos_filt, vel_filt;
            Eigen::Matrix<double, 8, 8> Q;
            Eigen::Matrix<double, 4, 4> R;
            Eigen::Matrix<double, 8, 8> Ad;
            Eigen::Matrix<double, 4, 8> Cd;
            Eigen::Matrix<double, 4, 4> Dd;

            Eigen::Matrix<double, 8, 4> K;

            Eigen::Matrix<double, 8, 1> x;
            Eigen::Matrix<double, 8, 8> P;
    };
}

#endif // PR_KALMAN_FILTER_HPP_