#include "pr_mocap/pr_x_mocap_synchronizer.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace pr_mocap
{
    /**** MODEL MOCAP ERROR COMPONENT ****/
    PRXMocapSynchronizer::PRXMocapSynchronizer(const rclcpp::NodeOptions & options)
    : Node("pr_x_mocap_synchronizer", options)
    {
        //Parameter declaration
        this->declare_parameter<double>("tol", 0.01);

        this->get_parameter("tol", tol);

        publisher_mocap_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
            "x_mocap_sync",
            1
        );

        publisher_end_ = this->create_publisher<std_msgs::msg::Bool>(
            "end_flag",
            1//rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_default)
        );

        subscription_mocap_ = this->create_subscription<pr_msgs::msg::PRMocap>(
            "x_coord_mocap",
            1,
            std::bind(&PRXMocapSynchronizer::mocap_callback, this, _1));
        
        subscription_sampling_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "joint_position",
            1,
            std::bind(&PRXMocapSynchronizer::sampling_callback, this, _1));

        is_connected = false;

        x_ant = Eigen::Vector4d::Zero();
        x_current = Eigen::Vector4d::Zero();
    }

    void PRXMocapSynchronizer::mocap_callback(const pr_msgs::msg::PRMocap::SharedPtr x_mocap_msg)
    {
        is_connected = true;
        iter_disconnected = 0;
        x_mocap.x_coord.data = x_mocap_msg->x_coord.data;
        x_mocap.latency = x_mocap_msg->latency;
        //Test connection with Mocap node
        //if (is_connected) std::cout << "MC2" << std::endl;
    }

    void PRXMocapSynchronizer::sampling_callback(const pr_msgs::msg::PRArrayH::SharedPtr x_sampling_msg)
    {
        if(is_connected) {
            //Publish mocap coordinates syncronized with sample time
            auto x_mocap_msg = pr_msgs::msg::PRArrayH();
            x_mocap_msg.init_time = this->get_clock()->now();

            x_mocap_msg.data = x_mocap.x_coord.data;
            x_mocap_msg.header.stamp = x_sampling_msg->header.stamp;
            x_mocap_msg.current_time = this->get_clock()->now();

            if (first_iter){
                x_ant[0] = x_mocap_msg.data[0];
                x_ant[1] = x_mocap_msg.data[1];
                x_ant[2] = x_mocap_msg.data[2];
                x_ant[3] = x_mocap_msg.data[3];
                first_iter = false;
            }
            x_current[0] = x_mocap_msg.data[0];
            x_current[1] = x_mocap_msg.data[1];
            x_current[2] = x_mocap_msg.data[2];
            x_current[3] = x_mocap_msg.data[3];
            distance = (x_current-x_ant).norm();

            iter_disconnected++;
            x_ant = x_current;
            publisher_mocap_->publish(x_mocap_msg);
        }
        if (!is_connected || iter_disconnected >= 20 || distance > 0.1 || std::isnan(distance)){
            auto end_msg = std_msgs::msg::Bool();
            end_msg.data = true;
            publisher_end_->publish(end_msg);
            if (!is_connected) std::cout << "Cameras initially disconnected!" << std::endl;
            if (iter_disconnected >= 20) std::cout << "Cameras disconnected for very long!" << std::endl;
            if (distance > 0.1) std::cout << "No consistent measurements from cameras!" << std::endl;
            if (std::isnan(distance)) std::cout << "NaN values for cameras!" << std::endl;
        }  

    }
    
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_mocap::PRXMocapSynchronizer)