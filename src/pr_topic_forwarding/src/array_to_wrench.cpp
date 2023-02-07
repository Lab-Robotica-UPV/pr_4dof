#include "pr_topic_forwarding/array_to_wrench.hpp"


namespace pr_topic_forwarding
{
    /**** TOPIC FORWARDING COMPONENT ****/
    ArrayToWrench::ArrayToWrench(const rclcpp::NodeOptions & options)
    : Node("array_to_wrench", options)
    {
    
        sub_array = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "array_topic",
            1,//rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_sensor_data),
            std::bind(&ArrayToWrench::topic_callback_array, this, std::placeholders::_1)
        );

        pub_wrench = this->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench_topic", 1);


    }

    // void GusController::ref_callback(const pr_msgs::msg::PRArrayH::SharedPtr ref_msg)
    // {
    //     PRUtils::ArRMsg2Eigen(ref_msg, ref);
    //     init_ref = true;
    // }   

    void ArrayToWrench::topic_callback_array(const pr_msgs::msg::PRArrayH::SharedPtr msg)
    {
        auto wrench_msg = geometry_msgs::msg::WrenchStamped();
		wrench_msg.wrench.force.x = msg->data[0];
		wrench_msg.wrench.force.y = 0.0;
		wrench_msg.wrench.force.z = msg->data[1];
		wrench_msg.wrench.torque.x = 0.0;
        wrench_msg.wrench.torque.y = msg->data[2];
        wrench_msg.wrench.torque.z = msg->data[3];
		wrench_msg.header.frame_id = msg->header.frame_id;
		wrench_msg.header.stamp = msg->header.stamp;
		pub_wrench->publish(wrench_msg);
    }


}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_topic_forwarding::ArrayToWrench)