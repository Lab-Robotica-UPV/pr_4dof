#ifndef PR_DMP__FORCE_ADAPTED_HPP_
#define PR_DMP__FORCE_ADAPTED__HPP_

#include "rclcpp/rclcpp.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_force_state.hpp"

#include <vector>
#include <chrono>
#include <memory>
#include <utility>

#include "eigen3/Eigen/Dense"

namespace pr_dmp
{
    class ForceAdapted : public rclcpp::Node
    {
        public:
            //PR_DMP_PUBLIC
            explicit ForceAdapted(const rclcpp::NodeOptions & options);

        protected:
            void ref_force_callback(const pr_msgs::msg::PRArrayH::SharedPtr ref_force_msg);
            void force_callback(const pr_msgs::msg::PRForceState::SharedPtr force_msg);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_ref_force_;
            rclcpp::Subscription<pr_msgs::msg::PRForceState>::SharedPtr subscription_force_;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;
            // Ultima fuerza medida
            Eigen::Vector4d current_force;
            // Medida comenzada
            bool init_f = false;
    };
}

#endif // PR_DMP__FORCE_ADAPTED_HPP_