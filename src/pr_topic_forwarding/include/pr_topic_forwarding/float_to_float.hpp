#ifndef PR_TOPIC_FORWARDING__FLOAT_TO_FLOAT_HPP_
#define PR_TOPIC_FORWARDING__FLOAT_TO_FLOAT_HPP_

#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <memory>
#include <chrono>

#include "pr_msgs/msg/pr_float_h.hpp"
#include "std_msgs/msg/float64.hpp"


namespace pr_topic_forwarding
{
    class FloatToFloat : public rclcpp::Node
    {
        public:
            //PR_COONTROLLERS_PUBLIC
            explicit FloatToFloat(const rclcpp::NodeOptions & options);

        protected:

            void topic_callback(const pr_msgs::msg::PRFloatH::SharedPtr msg);


        private:

            // Publisher
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_float;

            // Subscriber
            rclcpp::Subscription<pr_msgs::msg::PRFloatH>::SharedPtr sub_float;
        



    };
}

#endif // PR_TOPIC_FORWARDING__TOPIC_FORWARDING_HPP_