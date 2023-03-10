#ifndef PR_CONTROLLERS__PID_CONTROLLER_HPP_
#define PR_CONTROLLERS__PID_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"

#include "eigen3/Eigen/Dense"

#include "pr_msgs/msg/pr_array_h.hpp"


namespace pr_controllers
{
    class PIDController : public rclcpp::Node
    {
        public:
            //PR_COONTROLLERS_PUBLIC
            explicit PIDController(const rclcpp::NodeOptions & options);
            ~PIDController();

        protected:

            void controller_callback(const pr_msgs::msg::PRArrayH::SharedPtr pos_msg);
            void ref_callback(const pr_msgs::msg::PRArrayH::SharedPtr ref_msg);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr sub_ref;
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr sub_pos;

            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_proportional_;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_derivative_;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_integral_;

            std::vector<double> Kp, Kv, Ki;

            Eigen::Vector4d pos_ant;
            Eigen::Vector4d ref, pos, vel;
            Eigen::Vector4d integ; // Integrator term
            Eigen::Vector4d integ_ant;
            Eigen::Vector4d e, e_ant;
            Eigen::Vector4d control_action;
            Eigen::Matrix4d Kp_mat, Kv_mat, Ki_mat;

            Eigen::Vector4d proportional_action;
            Eigen::Vector4d derivative_action;
            Eigen::Vector4d integral_action;

            double ts;

            bool init_ref = false;
            bool first_iter = true;

            // For calculation of performance indices
            int iter = 0;
            Eigen::Vector4d control_action_ant;
            Eigen::Vector4d index_error = Eigen::Vector4d::Zero();
            Eigen::Vector4d index_action = Eigen::Vector4d::Zero();
            double Je, Ju;

            // For saturation of the control action
            double max_v;
            std::vector<double> vp_conversion;
            Eigen::Vector4d max_force;
    };
}

#endif // PR_CONTROLLERS__PID_CONTROLLER_HPP_