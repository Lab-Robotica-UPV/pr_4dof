#include "pr_biomech/admittance_euler_biomech.hpp"

using std::placeholders::_1;

namespace pr_biomech
{
    /**** ADMITTANCE COMPONENT ****/
    AdmittanceEulerBiomech::AdmittanceEulerBiomech(const rclcpp::NodeOptions & options)
    : Node("admittance_biomech", options)
    {
        //Parameter declaration
        this->declare_parameter<std::vector<double>>("mass", {1.0, 1.0, 1.0, 1.0});
        this->declare_parameter<std::vector<double>>("damping", {100.0, 100.0, 100.0, 100.0});
        this->declare_parameter<double>("ts", 0.01);
        

        this->get_parameter("mass", mass);
        this->get_parameter("damping", damping);
        this->get_parameter("ts", ts);

        f_ass_sub = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "ass_force",
            1,
            std::bind(&AdmittanceEulerBiomech::ass_force_callback, this, std::placeholders::_1)
        );

        f_sub = this->create_subscription<pr_msgs::msg::PRForceState>(
            "force_state",
            1,
            std::bind(&AdmittanceEulerBiomech::force_callback, this, std::placeholders::_1)
        );

        saturation_pin_sub = this->create_subscription<pr_msgs::msg::PRBoolH>(
            "saturation_pin",
            1,
            std::bind(&AdmittanceEulerBiomech::saturation_pin_callback, this, std::placeholders::_1)
        );

        publisher_vel_ = this->create_publisher<pr_msgs::msg::PRArrayH>("vel_admittance", 1);
        publisher_pos_ = this->create_publisher<pr_msgs::msg::PRArrayH>("pos_admittance", 1);

        // Initialize velocity and position vectors
        vel.resize(damping.size());
        aux_pos.resize(damping.size());
        aux_vel.resize(damping.size());
        pos.resize(damping.size());  
        dvel.resize(damping.size()); 
        dpos.resize(damping.size());

        std::fill(vel.begin(), vel.end(), 0.0);
        std::fill(pos.begin(), pos.end(), 0.0);
        std::fill(aux_vel.begin(), aux_vel.end(), 0.0);
        std::fill(aux_pos.begin(), aux_pos.end(), 0.0);
        std::fill(dvel.begin(), dvel.end(), 0.0);
        std::fill(dpos.begin(), dpos.end(), 0.0);

        force_ass = Eigen::Vector4d::Zero();
    }

    void AdmittanceEulerBiomech::force_callback(const pr_msgs::msg::PRForceState::SharedPtr f_msg)
    {

            // x admittance message and init time
            auto x_admittance_msg = pr_msgs::msg::PRArrayH();
            x_admittance_msg.init_time = this->get_clock()->now();

            // v admittance message and init time
            auto v_admittance_msg = pr_msgs::msg::PRArrayH();
            v_admittance_msg.init_time = this->get_clock()->now();

            // If the pin is not activated, the force errors will be zero
            // x coordinate
            force_total(0) = force_ass(0) + f_msg->force[0];
            // z coordinate
            force_total(1) = force_ass(1) + f_msg->force[2];
            // theta coordinate
            force_total(2) = force_ass(2) + f_msg->momentum[1];
            // psi coordinate
            force_total(3) = force_ass(3) + f_msg->momentum[2];

            // State update
            for (int i=0; i<4; i++){
                dvel[i] = 1/mass[i]*(- damping[i]*vel[i] + force_total(i));
                dpos[i] = vel[i];

                aux_vel[i]=vel[i]+dvel[i]*ts;
                aux_pos[i]=pos[i]+dpos[i]*ts;

                if (saturation_pin && (abs(aux_pos[i])>abs(pos[i]))){
                    vel[i] = 0;
                }
                else{
                    vel[i] = aux_vel[i];
                    pos[i] = aux_pos[i];
                }
            }

            // Output msgs
            x_admittance_msg.data[0] = pos[0];
            x_admittance_msg.data[1] = pos[1];
            x_admittance_msg.data[2] = pos[2];
            x_admittance_msg.data[3] = pos[3];
            v_admittance_msg.data[0] = vel[0];
            v_admittance_msg.data[1] = vel[1];
            v_admittance_msg.data[2] = vel[2];
            v_admittance_msg.data[3] = vel[3];
           
            // Send data
            x_admittance_msg.header.stamp = f_msg->header.stamp;
            x_admittance_msg.header.frame_id = f_msg->header.frame_id;

            v_admittance_msg.header.stamp = f_msg->header.stamp;
            v_admittance_msg.header.frame_id = f_msg->header.frame_id;

            x_admittance_msg.current_time = this->get_clock()->now();
            publisher_pos_->publish(x_admittance_msg);

            v_admittance_msg.current_time = this->get_clock()->now();
            publisher_vel_->publish(v_admittance_msg);

    }

    void AdmittanceEulerBiomech::ass_force_callback(const pr_msgs::msg::PRArrayH::SharedPtr f_ass_msg)
    {
        // Update the msg with the new force msg
        force_ass[0] = f_ass_msg->data[0];
        force_ass[1] = f_ass_msg->data[1];
        force_ass[2] = f_ass_msg->data[2];
        force_ass[3] = f_ass_msg->data[3];
        init_f = true;
    }

    void AdmittanceEulerBiomech::saturation_pin_callback(const pr_msgs::msg::PRBoolH::SharedPtr saturation_pin_msg)
    {
        // Update the saturation pin
        saturation_pin = saturation_pin_msg->data;
    }

    
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_biomech::AdmittanceEulerBiomech)