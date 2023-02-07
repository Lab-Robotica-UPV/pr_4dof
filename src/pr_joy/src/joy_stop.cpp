#include "pr_joy/joy_stop.hpp"

#include <memory>
#include <array>
#include <vector>



namespace pr_joy
{
    /**** JOY STOP COMPONENT ****/
    JoyStop::JoyStop(const rclcpp::NodeOptions & options)
    : Node("joy_stop", options)
    {


        sub_joy = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy",
            1,
            std::bind(&JoyStop::joy_callback, this, std::placeholders::_1)
        );



        publisher_stop = this->create_publisher<std_msgs::msg::Bool>("joy_stop", 1
        );


    }

    void JoyStop::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        auto joy_stop_msg = std_msgs::msg::Bool();
        joy_stop_msg.data = std::any_of(joy_msg->buttons.begin(), joy_msg->buttons.end(), [](int i){return i==1;});
        publisher_stop->publish(joy_stop_msg);
    }   

    
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_joy::JoyStop)