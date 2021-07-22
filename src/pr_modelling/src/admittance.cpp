#include "pr_modelling/admittance.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_mat_h.hpp"

#include "pr_lib/pr_model.hpp"
#include "pr_lib/pr_utils.hpp"

using std::placeholders::_1;

namespace pr_modelling
{
    /**** ADMITTANCE COMPONENT ****/
    Admittance::Admittance(const rclcpp::NodeOptions & options)
    : Node("admittance", options)
    {
        //Parameter declaration
        this->declare_parameter<std::vector<double>>("mass", {1.0, 1.0, 1.0, 1.0});
        this->declare_parameter<std::vector<double>>("damping", {100.0, 100.0, 100.0, 100.0});
        this->declare_parameter<std::vector<double>>("stiffness", {10.0, 10.0, 10.0, 10.0});
        this->declare_parameter<double>("ts", 0.01);
        

        this->get_parameter("mass", mass);
        this->get_parameter("damping", damping);
        this->get_parameter("stiffness", stiffness);
        this->get_parameter("ts", ts);

        f_ref_sub = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "ref_force",
            1,
            std::bind(&Admittance::ref_force_callback, this, std::placeholders::_1)
        );

        f_sub = this->create_subscription<pr_msgs::msg::PRForceState>(
            "force_state",
            1,
            std::bind(&Admittance::force_callback, this, std::placeholders::_1)
        );

        // sync_.reset(new Synchronizer(SyncPolicy(1), sub_Fref, sub_F));
        // sync_->registerCallback(std::bind(&Admittance::topic_callback, this, std::placeholders::_1, std::placeholders::_2));
        // sync_->setMaxIntervalDuration(rclcpp::Duration(0, 1000000));

        publisher_vel_ = this->create_publisher<pr_msgs::msg::PRArrayH>("vel_admittance", 1);
        publisher_pos_ = this->create_publisher<pr_msgs::msg::PRArrayH>("pos_admittance", 1);

        // Admittance state space model
        // Create continuous space matrix

        Ac << 0, 0, 0, 0, 1, 0, 0, 0,
              0, 0, 0, 0, 0, 1, 0, 0,
              0, 0, 0, 0, 0, 0, 1, 0,
              0, 0, 0, 0, 0, 0, 0, 1,
              -stiffness[0]/mass[0], 0, 0, 0, -damping[0]/mass[0], 0, 0, 0,
              0, -stiffness[1]/mass[1], 0, 0, 0, -damping[1]/mass[1], 0, 0,
              0, 0, -stiffness[2]/mass[2], 0, 0, 0, -damping[2]/mass[2], 0,
              0, 0, 0, -stiffness[3]/mass[3], 0, 0, 0, -damping[3]/mass[3];

        Bc << 0, 0, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0,
              1/mass[0], 0, 0, 0,
              0, 1/mass[1], 0, 0,
              0, 0, 1/mass[2], 0,
              0, 0, 0, 1/mass[3];

        Cc = Eigen::Matrix<double, 8, 8>::Identity();
        Dc = Eigen::Matrix<double, 8, 4>::Zero();

        // Discretized model
        Ad = (Ac*ts).exp();
        Bd = Ac.inverse()*(Ad-Eigen::Matrix<double,8,8>::Identity())*Bc;
        Cd = Cc;
        Dd = Dd;

        state = Eigen::Matrix<double,8,1>::Zero();
        state_ = Eigen::Matrix<double,8,1>::Zero();
        output = Eigen::Matrix<double,8,1>::Zero();


    }

    void Admittance::ref_force_callback(const pr_msgs::msg::PRArrayH::SharedPtr fref_msg)
    {
        if (init_f){

            // x admittance message and init time
            auto x_admittance_msg = pr_msgs::msg::PRArrayH();
            x_admittance_msg.init_time = this->get_clock()->now();

            // v admittance message and init time
            auto v_admittance_msg = pr_msgs::msg::PRArrayH();
            v_admittance_msg.init_time = this->get_clock()->now();

            // x coordinate
            force_error(0) = fref_msg->data[0] - msgForce.force[0];
            // z coordinate
            force_error(1) = fref_msg->data[1] - msgForce.force[2];
            // theta coordinate
            force_error(2) = fref_msg->data[2] - msgForce.momentum[1];
            // psi coordinate
            force_error(3) = fref_msg->data[3] - msgForce.momentum[2];

            // State update
            state_ = Ad*state + Bd*force_error;
            output = -(Cd*state + Dd*force_error);

            state = state_;

            // Output msgs
            x_admittance_msg.data[0] = output(0);
            x_admittance_msg.data[1] = output(1);
            x_admittance_msg.data[2] = output(2);
            x_admittance_msg.data[3] = output(3);
            v_admittance_msg.data[0] = output(4);
            v_admittance_msg.data[1] = output(5);
            v_admittance_msg.data[2] = output(6);
            v_admittance_msg.data[3] = output(7);
            // for (int i=0; i<4; i++){
            //     if (abs(v_admittance_msg.data[i]) < 2e-4){
            //         v_admittance_msg.data[i] = 0;
            //     }
            // }

            x_admittance_msg.header.stamp = fref_msg->header.stamp;
            x_admittance_msg.header.frame_id = fref_msg->header.frame_id;

            v_admittance_msg.header.stamp = fref_msg->header.stamp;
            v_admittance_msg.header.frame_id = fref_msg->header.frame_id;

            x_admittance_msg.current_time = this->get_clock()->now();
            publisher_pos_->publish(x_admittance_msg);

            v_admittance_msg.current_time = this->get_clock()->now();
            publisher_vel_->publish(v_admittance_msg);

        }
    }

    void Admittance::force_callback(const pr_msgs::msg::PRForceState::SharedPtr f_msg)
    {
        // Update the msg with the new force msg
        msgForce.force[0] = f_msg->force[0];
        msgForce.force[1] = f_msg->force[1];
        msgForce.force[2] = f_msg->force[2];
        msgForce.momentum[0] = f_msg->momentum[0];
        msgForce.momentum[1] = f_msg->momentum[1];
        msgForce.momentum[2] = f_msg->momentum[2];
        init_f = true;
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_modelling::Admittance)