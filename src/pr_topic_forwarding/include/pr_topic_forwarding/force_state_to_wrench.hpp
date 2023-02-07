#ifndef PR_TOPIC_FORWARDING__FORCE_STATE_TO_WRENCH_HPP_
#define PR_TOPIC_FORWARDING__FORCE_STATE_TO_WRENCH_HPP_

#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <memory>
#include <chrono>

#include "pr_msgs/msg/pr_force_state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"


namespace pr_topic_forwarding
{
    class ForceStateToWrench : public rclcpp::Node
    {
        public:
            //PR_COONTROLLERS_PUBLIC
            explicit ForceStateToWrench(const rclcpp::NodeOptions & options);

        protected:

            void topic_callback_force(const pr_msgs::msg::PRForceState::SharedPtr msg);


        private:

            // Publisher
            rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_wrench;

            // Subscriber
            rclcpp::Subscription<pr_msgs::msg::PRForceState>::SharedPtr sub_force;
        



    };
}

#endif // PR_TOPIC_FORWARDING__TOPIC_FORWARDING_HPP_