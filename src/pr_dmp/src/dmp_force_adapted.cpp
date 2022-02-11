#include "pr_dmp/dmp_force_adapted.hpp"


using std::placeholders::_1;

namespace pr_dmp
{
    /**** ForceAdapted COMPONENT ****/
    ForceAdapted::ForceAdapted(const rclcpp::NodeOptions & options)
    : Node("dmp_force_adapted", options)
    {
;
        
        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
			"force_adapted", 
			1);
        subscription_ref_force_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "ref_force",
            1,
            std::bind(&ForceAdapted::ref_force_callback, this, _1));

        subscription_force_ = this->create_subscription<pr_msgs::msg::PRForceState>(
            "force_state",
            1,
            std::bind(&ForceAdapted::force_callback, this, _1)
        );
    }

    void ForceAdapted::ref_force_callback(const pr_msgs::msg::PRArrayH::SharedPtr ref_force_msg)
    {
        if (init_f){

            // force message and init time
            auto f_adapted_msg = pr_msgs::msg::PRArrayH();
            f_adapted_msg.init_time = this->get_clock()->now();

            for (int i=0;i<4; i++){
                if (abs(current_force(i)) < abs(ref_force_msg->data[i])){
                    f_adapted_msg.data[i] = current_force(i)-ref_force_msg->data[i];
                    // if (ref_force_msg->data[i]>0){
                    //     f_adapted_msg.data[i] = current_force(i)-ref_force_msg->data[i];
                    // }
                    // else{
                    //     f_adapted_msg.data[i] = ref_force_msg->data[i]-current_force(i);
                    // }
                }
            }

            f_adapted_msg.header.stamp = ref_force_msg->header.stamp;
            f_adapted_msg.header.frame_id = ref_force_msg->header.frame_id;
            f_adapted_msg.current_time = this->get_clock()->now();
            publisher_->publish(f_adapted_msg);

        }
    }

    void ForceAdapted::force_callback(const pr_msgs::msg::PRForceState::SharedPtr force_msg)
    {
        // Update the msg with the new force msg
        current_force(0) = force_msg->force[0];
        current_force(1) = force_msg->force[2];
        current_force(2) = force_msg->momentum[1];
        current_force(3) = force_msg->momentum[2];
        
        init_f = true;
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_dmp::ForceAdapted)