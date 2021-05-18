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

        subscription_mocap_ = this->create_subscription<pr_msgs::msg::PRMocap>(
            "x_coord_mocap",
            1,
            std::bind(&PRXMocapSynchronizer::mocap_callback, this, _1));
        
        subscription_sampling_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "joint_position",
            1,
            std::bind(&PRXMocapSynchronizer::sampling_callback, this, _1));

        is_connected = false;
    }

    void PRXMocapSynchronizer::mocap_callback(const pr_msgs::msg::PRMocap::SharedPtr x_mocap_msg)
    {
        is_connected = true;
        x_mocap.x_coord.data = x_mocap_msg->x_coord.data;
        x_mocap.latency = x_mocap_msg->latency;
    }

    void PRXMocapSynchronizer::sampling_callback(const pr_msgs::msg::PRArrayH::SharedPtr x_sampling_msg)
    {
        if(is_connected) {
            //Publish mocap coordinates syncronized with sample time
            auto x_mocap_msg = pr_msgs::msg::PRArrayH();

            x_mocap_msg.data = x_mocap.x_coord.data;
            x_mocap_msg.header.stamp = x_sampling_msg->header.stamp;
            x_mocap_msg.current_time = this->get_clock()->now();

            publisher_mocap_->publish(x_mocap_msg);

        }

    }
    
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_mocap::PRXMocapSynchronizer)