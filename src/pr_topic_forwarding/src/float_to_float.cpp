#include "pr_topic_forwarding/float_to_float.hpp"


namespace pr_topic_forwarding
{
    /**** TOPIC FORWARDING COMPONENT ****/
    FloatToFloat::FloatToFloat(const rclcpp::NodeOptions & options)
    : Node("float_to_float", options)
    {
    
        sub_float = this->create_subscription<pr_msgs::msg::PRFloatH>(
            "float_in_topic",
            1,//rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_sensor_data),
            std::bind(&FloatToFloat::topic_callback, this, std::placeholders::_1)
        );

        pub_float = this->create_publisher<std_msgs::msg::Float64>("float_out_topic", 1);


    }

    void FloatToFloat::topic_callback(const pr_msgs::msg::PRFloatH::SharedPtr msg)
    {
        auto float_msg = std_msgs::msg::Float64();
		float_msg.data = msg->data;
		pub_float->publish(float_msg);
    }


}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_topic_forwarding::FloatToFloat)