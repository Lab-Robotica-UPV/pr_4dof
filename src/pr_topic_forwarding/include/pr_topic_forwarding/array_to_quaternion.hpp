#ifndef PR_TOPIC_FORWARDING__ARRAY_TO_QUATERNION_HPP_
#define PR_TOPIC_FORWARDING__ARRAY_TO_QUATERNION_HPP_

#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <memory>
#include <chrono>

#include "pr_msgs/msg/pr_array_h.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"


namespace pr_topic_forwarding
{
    class ArrayToQuaternion : public rclcpp::Node
    {
        public:
            //PR_COONTROLLERS_PUBLIC
            explicit ArrayToQuaternion(const rclcpp::NodeOptions & options);

        protected:

            void topic_callback_array(const pr_msgs::msg::PRArrayH::SharedPtr msg);


        private:

            // Publisher
            rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr pub_quaternion;

            // Subscriber
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr sub_array;
        



    };
}

#endif // PR_TOPIC_FORWARDING__TOPIC_FORWARDING_HPP_