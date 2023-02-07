#ifndef PR_JOY__JOY_STOP_HPP_
#define PR_JOY__JOY_STOP_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"


namespace pr_joy
{
    class JoyStop : public rclcpp::Node
    {
        public:
            //PR_JOY_PUBLIC
            explicit JoyStop(const rclcpp::NodeOptions & options);

        protected:

            void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);


        private:
            
           
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy;

            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_stop;

            bool joy_stop = false;


    };
}

#endif // PR_CONTROLLERS__GUS_CONTROLLER_HPP_