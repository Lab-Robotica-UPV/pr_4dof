#include "pr_biomech/model_pau_sim.hpp"

using std::placeholders::_1;

namespace pr_biomech
{
    /**** ADMITTANCE COMPONENT ****/
    ModelPauSim::ModelPauSim(const rclcpp::NodeOptions & options)
    : Node("admittance_biomech", options)
    {
        //Parameter declaration
        this->declare_parameter<double>("ref_muscle_force", 20.0);
        this->declare_parameter<std::vector<double>>("dir_force", {1.0, 0.0, 0.0, 0.0});
        this->declare_parameter<double>("length_tibia", 0.5);
        this->declare_parameter<double>("length_foot", 0.25);

        this->get_parameter("ref_muscle_force", ref_muscle_force);
        this->get_parameter("dir_force", dir_force);
        this->get_parameter("length_tibia", length_tibia);
        this->get_parameter("length_foot", length_foot);

        sub_force_state = this->create_subscription<pr_msgs::msg::PRForceState>(
            "force_state",
            1,
            std::bind(&ModelPauSim::force_callback, this, std::placeholders::_1)
        );

        pub_ref_muscle_force = this->create_publisher<pr_msgs::msg::PRFloatH>("ref_muscle_force", 1);
        pub_muscle_force = this->create_publisher<pr_msgs::msg::PRFloatH>("muscle_force", 1);
        pub_dir_force = this->create_publisher<pr_msgs::msg::PRArrayH>("dir_force", 1);
    }

    void ModelPauSim::force_callback(const pr_msgs::msg::PRForceState::SharedPtr f_msg)
    {

            // Initialize msgs
            ref_muscle_force_msg.init_time = this->get_clock()->now();
            muscle_force_msg.init_time = this->get_clock()->now();
            dir_force_msg.init_time = this->get_clock()->now();

            // Messages to variables
            force_state[0] =  f_msg->force[0];
            force_state[1] =  f_msg->force[2];
            force_state[2] =  f_msg->momentum[1]/length_tibia;
            force_state[3] =  0*f_msg->momentum[2]/length_foot;

            muscle_force = force_state.norm();


            // Variables to messages
            for (int i=0; i<3; i++) dir_force_msg.data[i] = dir_force[i];
            ref_muscle_force_msg.data = ref_muscle_force;
            muscle_force_msg.data = muscle_force;


            // Send data
            ref_muscle_force_msg.header.stamp = f_msg->header.stamp;
            ref_muscle_force_msg.header.frame_id = f_msg->header.frame_id;

            muscle_force_msg.header.stamp = f_msg->header.stamp;
            muscle_force_msg.header.frame_id = f_msg->header.frame_id;

            dir_force_msg.header.stamp = f_msg->header.stamp;
            dir_force_msg.header.frame_id = f_msg->header.frame_id;

            ref_muscle_force_msg.current_time = this->get_clock()->now();
            pub_ref_muscle_force->publish(ref_muscle_force_msg);

            muscle_force_msg.current_time = this->get_clock()->now();
            pub_muscle_force->publish(muscle_force_msg);

            dir_force_msg.current_time = this->get_clock()->now();
            pub_dir_force->publish(dir_force_msg);

    }
    
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_biomech::ModelPauSim)