#include "pr_modelling/for_jac.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>

#include "pr_lib/pr_model.hpp"

using std::placeholders::_1;

namespace pr_modelling
{
    /**** FORWARD JACOBIAN COMPONENT ****/
    ForwardJacobian::ForwardJacobian(const rclcpp::NodeOptions & options)
    : Node("for_jac", options)
    {
        //Parameter declaration
        this->declare_parameter<std::vector<double>>(
            "robot_config_params", 
            {0.4, 0.4, 0.4, 0.15, 90*(M_PI/180), 45*(M_PI/180), 0.3, 0.3, 0.3, 50*(M_PI/180), 90*(M_PI/180)});
        
        this->get_parameter("robot_config_params", robot_params);

        subscription_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "x_coord", 
            10, 
            std::bind(&ForwardJacobian::topic_callback, this, _1));

        publisher_det_ = this->create_publisher<pr_msgs::msg::PRFloatH>(
            "for_jac_det", 
            1);

        publisher_jac_ = this->create_publisher<pr_msgs::msg::PRMatH>(
            "for_jac", 
            1);
    }

    void ForwardJacobian::topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr x_msg)
    {
        // Forward Jacobian and determinant message and Init time
        auto for_jac_det_msg = pr_msgs::msg::PRFloatH();
        for_jac_det_msg.init_time = this->get_clock()->now();

        // Forward Jacobian message and Init time
        auto for_jac_msg = pr_msgs::msg::PRMatH();
        for_jac_msg.init_time = this->get_clock()->now();

        PRModel::ForwardJacobian(ForJac, x_msg->data, robot_params);

        double for_jac_det = ForJac.determinant();

        for(int i=0; i<ForJac.rows(); i++)
        {
            for(int j=0; j<ForJac.cols(); j++)
                for_jac_msg.data.push_back(ForJac(i,j));
        }
        for_jac_msg.rows = ForJac.rows();
        for_jac_msg.cols = ForJac.cols();

        for_jac_msg.header.stamp = x_msg->header.stamp;
        for_jac_msg.header.frame_id = x_msg->header.frame_id;


        for_jac_det_msg.data = for_jac_det;
        for_jac_det_msg.header.stamp = x_msg->header.stamp;
        for_jac_det_msg.header.frame_id = x_msg->header.frame_id;

        for_jac_det_msg.current_time = this->get_clock()->now();
        for_jac_msg.current_time = this->get_clock()->now();

        publisher_det_->publish(for_jac_det_msg);
        publisher_jac_->publish(for_jac_msg);
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_modelling::ForwardJacobian)