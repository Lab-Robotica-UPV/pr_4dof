#include "pr_topic_forwarding/force_state_to_wrench.hpp"


namespace pr_topic_forwarding
{
    /**** TOPIC FORWARDING COMPONENT ****/
    ForceStateToWrench::ForceStateToWrench(const rclcpp::NodeOptions & options)
    : Node("force_state_to_wrench", options)
    {
    
        sub_force = this->create_subscription<pr_msgs::msg::PRForceState>(
            "force_topic",
            1,//rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_sensor_data),
            std::bind(&ForceStateToWrench::topic_callback_force, this, std::placeholders::_1)
        );

        pub_wrench = this->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench_topic", 1);


    }
  

    void ForceStateToWrench::topic_callback_force(const pr_msgs::msg::PRForceState::SharedPtr msg)
    {
        auto wrench_msg = geometry_msgs::msg::WrenchStamped();
		wrench_msg.wrench.force.x = msg->force[0];
		wrench_msg.wrench.force.y = msg->force[1];
		wrench_msg.wrench.force.z = msg->force[2];
		wrench_msg.wrench.torque.x = msg->momentum[0];
        wrench_msg.wrench.torque.y = msg->momentum[1];
        wrench_msg.wrench.torque.z = msg->momentum[2];
		wrench_msg.header.frame_id = msg->header.frame_id;
		wrench_msg.header.stamp = msg->header.stamp;
		pub_wrench->publish(wrench_msg);
    }


}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_topic_forwarding::ForceStateToWrench)