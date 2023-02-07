#ifndef PR_TOPIC_FORWARDING__ARRAY_TO_WRENCH_HPP_
#define PR_TOPIC_FORWARDING__ARRAY_TO_WRENCH_HPP_

#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <memory>
#include <chrono>

#include "pr_msgs/msg/pr_array_h.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"


namespace pr_topic_forwarding
{
    class ArrayToWrench : public rclcpp::Node
    {
        public:
            //PR_COONTROLLERS_PUBLIC
            explicit ArrayToWrench(const rclcpp::NodeOptions & options);

        protected:

            void topic_callback_array(const pr_msgs::msg::PRArrayH::SharedPtr msg);


        private:

            // Publisher
            rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_wrench;

            // Subscriber
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr sub_array;
        



    };
}

#endif // PR_TOPIC_FORWARDING__TOPIC_FORWARDING_HPP_