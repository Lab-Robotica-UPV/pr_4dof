#include "pr_topic_forwarding/array_to_quaternion.hpp"


namespace pr_topic_forwarding
{
    /**** TOPIC FORWARDING COMPONENT ****/
    ArrayToQuaternion::ArrayToQuaternion(const rclcpp::NodeOptions & options)
    : Node("array_to_quaternion", options)
    {
    
        sub_array = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "array_topic",
            1,//rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_sensor_data),
            std::bind(&ArrayToQuaternion::topic_callback_array, this, std::placeholders::_1)
        );

        pub_quaternion = this->create_publisher<geometry_msgs::msg::QuaternionStamped>("quaternion_topic", 1);


    }

    // void GusController::ref_callback(const pr_msgs::msg::PRArrayH::SharedPtr ref_msg)
    // {
    //     PRUtils::ArRMsg2Eigen(ref_msg, ref);
    //     init_ref = true;
    // }   

    void ArrayToQuaternion::topic_callback_array(const pr_msgs::msg::PRArrayH::SharedPtr msg)
    {
        auto quaternion_msg = geometry_msgs::msg::QuaternionStamped();
		quaternion_msg.quaternion.x = msg->data[0];
		quaternion_msg.quaternion.y = msg->data[1];
		quaternion_msg.quaternion.z = msg->data[2];
		quaternion_msg.quaternion.w = msg->data[3];
		quaternion_msg.header.frame_id = msg->header.frame_id;
		quaternion_msg.header.stamp = msg->header.stamp;
		pub_quaternion->publish(quaternion_msg);
    }


}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_topic_forwarding::ArrayToQuaternion)