#ifndef PR_CONTROLLERS2__P_CONTROLLER_HPP_
#define PR_CONTROLLERS2__P_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "eigen3/Eigen/Dense"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>
#include <functional>

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_lib/pr_utils.hpp"


namespace pr_controllers2
{
    class PController : public rclcpp::Node
    {
        public:
            //PR_COONTROLLERS_PUBLIC
            explicit PController(const rclcpp::NodeOptions & options);
            ~PController();

        protected:

            void controller_callback(const pr_msgs::msg::PRArrayH::SharedPtr pos_msg);
            void ref_callback(const pr_msgs::msg::PRArrayH::SharedPtr ref_msg);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr sub_ref;
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr sub_pos;

            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;

            std::vector<double> Kp;

            Eigen::Vector4d ref, pos;
            Eigen::Vector4d e;
            Eigen::Vector4d control_action;
            Eigen::Matrix4d Kp_mat;

            double ts;

            bool init_ref = false;
            //bool first_iter = true;

            // For calculation of performance indices
            int iter = 0;
            
    };
}

#endif