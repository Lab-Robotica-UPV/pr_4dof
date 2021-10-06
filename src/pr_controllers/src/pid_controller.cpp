#include "pr_controllers/pid_controller.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_lib/pr_utils.hpp"

namespace pr_controllers
{
    /**** PID CONTROLLER COMPONENT ****/
    PIDController::PIDController(const rclcpp::NodeOptions & options)
    : Node("pid_controller", options)
    {
        //Parameter declaration
        this->declare_parameter<std::vector<double>>("kp_gain", {27190.0, 27190.0, 27190.0, 361023.0});
        this->declare_parameter<std::vector<double>>("kv_gain", {114.27, 114.27, 114.27, 491.32});
        this->declare_parameter<std::vector<double>>("ki_gain", {0.0, 0.0, 0.0, 0.0});

        this->declare_parameter<double>("ts", 0.01);

        this->get_parameter("kp_gain", Kp);
        this->get_parameter("kv_gain", Kv);
        this->get_parameter("ki_gain", Ki);

        this->get_parameter("ts", ts);

        //Conversión, hacer función
        Kp_mat = Eigen::Matrix4d::Zero(4,4);
        Kv_mat = Eigen::Matrix4d::Zero(4,4);
        Ki_mat = Eigen::Matrix4d::Zero(4,4);
        for(int i=0; i<4; i++){
            Kp_mat(i,i) = Kp[i];
            Kv_mat(i,i) = Kv[i];
            Ki_mat(i,i) = Ki[i];
        }
        integ = Eigen::Vector4d::Zero(4);
        e_ant = Eigen::Vector4d::Zero(4);

        RCLCPP_INFO(this->get_logger(), "Creating communication");

        sub_ref = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "ref_pos",
            1,
            std::bind(&PIDController::ref_callback, this, std::placeholders::_1)
        );

        sub_pos = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "pos",
            1,
            std::bind(&PIDController::controller_callback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>("control_action", 1);
        publisher_proportional_ = this->create_publisher<pr_msgs::msg::PRArrayH>("proportional_action", 1);
        publisher_derivative_ = this->create_publisher<pr_msgs::msg::PRArrayH>("derivative_action", 1);
        publisher_integral_ = this->create_publisher<pr_msgs::msg::PRArrayH>("integral_action", 1);

    }

    void PIDController::ref_callback(const pr_msgs::msg::PRArrayH::SharedPtr ref_msg)
    {
        PRUtils::ArRMsg2Eigen(ref_msg, ref);
        init_ref = true;
    }

    void PIDController::controller_callback(const pr_msgs::msg::PRArrayH::SharedPtr pos_msg)
    {
        if (init_ref){
            // Control action message and init time
            auto control_action_msg = pr_msgs::msg::PRArrayH();
            control_action_msg.init_time = this->get_clock()->now();

            auto proportional_action_msg = pr_msgs::msg::PRArrayH();
            proportional_action_msg.init_time = this->get_clock()->now();
            auto derivative_action_msg = pr_msgs::msg::PRArrayH();
            derivative_action_msg.init_time = this->get_clock()->now();
            auto integral_action_msg = pr_msgs::msg::PRArrayH();
            integral_action_msg.init_time = this->get_clock()->now();

            PRUtils::ArRMsg2Eigen(pos_msg, pos);

            e = pos-ref;
            if (first_iter){
                pos_ant = pos;
                first_iter = false;
            }
            vel = PRUtils::derivation(pos, pos_ant, ts);
            integ = PRUtils::integration_forward_euler(e, e_ant, integ, ts);
            pos_ant = pos;
            e_ant = e;

            //Calculate control action
            proportional_action = -Kp_mat*e;
            derivative_action = -Kv_mat*vel;
            integral_action = -Ki_mat*integ;
            control_action = proportional_action + derivative_action + integral_action;

            PRUtils::Eigen2ArMsg(control_action, control_action_msg);
            PRUtils::Eigen2ArMsg(proportional_action, proportional_action_msg);
            PRUtils::Eigen2ArMsg(derivative_action, derivative_action_msg);
            PRUtils::Eigen2ArMsg(integral_action, integral_action_msg);

            control_action_msg.header.stamp = pos_msg->header.stamp;
            control_action_msg.header.frame_id = pos_msg->header.frame_id;

            proportional_action_msg.header.stamp = pos_msg->header.stamp;
            proportional_action_msg.header.frame_id = pos_msg->header.frame_id;

            derivative_action_msg.header.stamp = pos_msg->header.stamp;
            derivative_action_msg.header.frame_id = pos_msg->header.frame_id;

            integral_action_msg.header.stamp = pos_msg->header.stamp;
            integral_action_msg.header.frame_id = pos_msg->header.frame_id;

            control_action_msg.current_time = this->get_clock()->now();
            publisher_->publish(control_action_msg);

            proportional_action_msg.current_time = this->get_clock()->now();
            publisher_proportional_->publish(proportional_action_msg);

            derivative_action_msg.current_time = this->get_clock()->now();
            publisher_derivative_->publish(derivative_action_msg);

            integral_action_msg.current_time = this->get_clock()->now();
            publisher_integral_->publish(integral_action_msg);

            /*RCLCPP_INFO(this->get_logger(), "I heard: %f, %f, %f, %f", control_action_msg.data[0],
                                                                    control_action_msg.data[1],
                                                                    control_action_msg.data[2],
                                                                    control_action_msg.data[3]);
            */
        }
        
    }


}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_controllers::PIDController)