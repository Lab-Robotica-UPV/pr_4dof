#ifndef PR_MOCAP__PR_X_MOCAP_SYNCHRONIZER_HPP_
#define PR_MOCAP__PR_X_MOCAP_SYNCHRONIZER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_mocap.hpp"

#include <array>
#include <vector>

namespace pr_mocap
{
    class PRXMocapSynchronizer : public rclcpp::Node
    {
        public:
            //PR_AUX_PUBLIC
            explicit PRXMocapSynchronizer(const rclcpp::NodeOptions & options);

        protected:
            void sampling_callback(const pr_msgs::msg::PRArrayH::SharedPtr x_sampling_msg);
            void mocap_callback(const pr_msgs::msg::PRMocap::SharedPtr x_mocap_msg);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_sampling_;
            rclcpp::Subscription<pr_msgs::msg::PRMocap>::SharedPtr subscription_mocap_;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_mocap_;

            double tol;
            bool is_connected;
            pr_msgs::msg::PRMocap x_mocap;
    };
}

#endif // PR_MOCAP__PR_X_MOCAP_SYNCHRONIZER_HPP_