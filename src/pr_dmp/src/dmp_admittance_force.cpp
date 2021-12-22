#include "pr_dmp/dmp_admittance_force.hpp"


using std::placeholders::_1;

namespace pr_dmp
{
    /**** AdmittanceForce COMPONENT ****/
    AdmittanceForce::AdmittanceForce(const rclcpp::NodeOptions & options)
    : Node("dmp_admittance_force", options)
    {
        //Parameter declaration
        this->declare_parameter<std::vector<double>>("K_adm", {1.0, 1.0, 1.0, 1.0});

        this->get_parameter("K_adm", K_adm);
        
        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
			"adm_force", 
			1);
        subscription_ref_force_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "ref_force",
            1,
            std::bind(&AdmittanceForce::ref_force_callback, this, _1));

        subscription_force_ = this->create_subscription<pr_msgs::msg::PRForceState>(
            "force_state",
            1,
            std::bind(&AdmittanceForce::force_callback, this, _1)
        );
    }

    void AdmittanceForce::ref_force_callback(const pr_msgs::msg::PRArrayH::SharedPtr ref_force_msg)
    {
        if (init_f){

            // force admittance message and init time
            auto f_admittance_msg = pr_msgs::msg::PRArrayH();
            f_admittance_msg.init_time = this->get_clock()->now();

            for (int i=0;i<4; i++)
                f_admittance_msg.data[i] = -K_adm[i]*(ref_force_msg->data[i]-current_force(i));


            f_admittance_msg.header.stamp = ref_force_msg->header.stamp;
            f_admittance_msg.header.frame_id = ref_force_msg->header.frame_id;
            f_admittance_msg.current_time = this->get_clock()->now();
            publisher_->publish(f_admittance_msg);

        }
    }

    void AdmittanceForce::force_callback(const pr_msgs::msg::PRForceState::SharedPtr force_msg)
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
RCLCPP_COMPONENTS_REGISTER_NODE(pr_dmp::AdmittanceForce)