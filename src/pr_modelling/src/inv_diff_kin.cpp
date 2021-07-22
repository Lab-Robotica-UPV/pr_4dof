#include "pr_modelling/inv_diff_kin.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_lib/pr_model.hpp"
#include "pr_lib/pr_utils.hpp"

using std::placeholders::_1;

namespace pr_modelling
{
    /**** FORWARD KINEMATICS COMPONENT ****/
    InvDiffKinematics::InvDiffKinematics(const rclcpp::NodeOptions & options)
    : Node("inv_diff_kin", options)
    {
        //Parameter declaration
        this->declare_parameter<std::vector<double>>(
            "robot_config_params", 
            {0.4, 0.4, 0.4, 0.15, 90*(M_PI/180), 45*(M_PI/180), 0.3, 0.3, 0.3, 50*(M_PI/180), 90*(M_PI/180)});
        this->declare_parameter<double>("ts", 0.01);

        this->get_parameter("robot_config_params", robot_params);
        this->get_parameter("ts", ts);

        publisher_qp = this->create_publisher<pr_msgs::msg::PRArrayH>(
            "joint_velocity", 
            10);

        publisher_q = this->create_publisher<pr_msgs::msg::PRArrayH>(
            "joint_position", 
            10);

        publisher_detFwdJac = this->create_publisher<pr_msgs::msg::PRFloatH>(
            "det_fwd_jac", 
            10);

        integ = Eigen::Vector4d::Zero(4);
        qp_ant = Eigen::Vector4d::Zero(4);

        sub_vel_cart.subscribe(this, "vel_cart");
        sub_pos_cart.subscribe(this, "x_cart");

        sync_.reset(new Synchronizer(SyncPolicy(1), sub_vel_cart, sub_pos_cart));
        sync_->registerCallback(std::bind(&InvDiffKinematics::topic_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

    void InvDiffKinematics::topic_callback(const pr_msgs::msg::PRArrayH::ConstPtr& vel_cart_msg,
                                           const pr_msgs::msg::PRArrayH::ConstPtr& pos_cart_msg)
    {
        // Joint velocity message and init time
        auto qp_msg = pr_msgs::msg::PRArrayH();
        qp_msg.init_time = this->get_clock()->now();

        // Joint position message and init time
        auto q_msg = pr_msgs::msg::PRArrayH();
        q_msg.init_time = this->get_clock()->now();

        // Forward Jacobian Determinant message and init time
        auto det_fwd_jac_msg = pr_msgs::msg::PRFloatH();
        det_fwd_jac_msg.init_time = this->get_clock()->now();

        // FwdJacobian calculation
        PRModel::ForwardJacobian(ForJ, pos_cart_msg->data, robot_params);

        PRUtils::ArRMsg2Eigen(vel_cart_msg, v);

        qp = ForJ*v;
        det_fwd_jac_msg.data = ForJ.determinant();

        integ = PRUtils::integration_trapezoidal(qp, qp_ant, integ, ts);
        qp_ant = qp;

        PRUtils::Eigen2ArMsg(qp, qp_msg);
        PRUtils::Eigen2ArMsg(integ, q_msg);

        qp_msg.header.stamp = vel_cart_msg->header.stamp;
        qp_msg.header.frame_id = vel_cart_msg->header.frame_id;

        q_msg.header.stamp = vel_cart_msg->header.stamp;
        q_msg.header.frame_id = vel_cart_msg->header.frame_id;

        det_fwd_jac_msg.header.stamp = vel_cart_msg->header.stamp;
        det_fwd_jac_msg.header.frame_id = vel_cart_msg->header.frame_id;

        qp_msg.current_time = this->get_clock()->now();
        publisher_qp->publish(qp_msg);

        q_msg.current_time = this->get_clock()->now();
        publisher_q->publish(q_msg);

        det_fwd_jac_msg.current_time = this->get_clock()->now();
        publisher_detFwdJac->publish(det_fwd_jac_msg);
    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_modelling::InvDiffKinematics)