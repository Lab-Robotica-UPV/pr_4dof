#include "pr_aux/kalman_filter.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"

using std::placeholders::_1;

namespace pr_aux
{
    /**** KALMAN FILTER COMPONENT ****/
    KalmanFilter::KalmanFilter(const rclcpp::NodeOptions & options)
    : Node("kalman_filter", options)
    {
        //Parameter declaration
        this->declare_parameter<std::vector<double>>("initial_value", {0.679005, 0.708169, 0.684298, 0.637145});
        this->declare_parameter<double>("ts", 0.01);
        this->declare_parameter<double>("q", 100);
        this->declare_parameter<double>("r", 1e-4);

        this->get_parameter("initial_value", init_val);
        this->get_parameter("ts", ts);
        this->get_parameter("q", q);
        this->get_parameter("r", r);
        
        publisher_pos = this->create_publisher<pr_msgs::msg::PRArrayH>(
			"joint_position_filt", 
			1);
        publisher_vel = this->create_publisher<pr_msgs::msg::PRArrayH>(
			"joint_velocity_filt", 
			1);
        subscription_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "joint_position",
            1,
            std::bind(&KalmanFilter::topic_callback, this, _1));

        // Process noise covariance
        Q << q*pow(ts,4)/4*Eigen::Matrix<double, 4, 4>::Identity(), q*pow(ts,3)/2*Eigen::Matrix<double, 4, 4>::Identity(),
             q*pow(ts,3)/2*Eigen::Matrix<double, 4, 4>::Identity(), q*pow(ts,2)/2*Eigen::Matrix<double, 4, 4>::Identity();
        
        // Sensor noise covariance
        R = r*Eigen::Matrix<double, 4, 4>::Identity();

        // Process model (constant velocity)
        Ad = Eigen::Matrix<double, 8, 8>::Identity();
        Ad(0,4) = ts; Ad(1,5) = ts; Ad(2,6) = ts; Ad(3,7) = ts;
        Cd << Eigen::Matrix<double, 4, 4>::Identity(), Eigen::Matrix<double,4,4>::Zero();
        Dd = Eigen::Matrix<double,4,4>::Zero();

        // State
        x = Eigen::Matrix<double,8,1>::Zero();
        x(0) = init_val[0];
        x(1) = init_val[1];
        x(2) = init_val[2];
        x(3) = init_val[3];

        // Covariance matrix
        P = 0.01*Eigen::Matrix<double, 8,8>::Identity();

    }

    void KalmanFilter::topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr pos_msg)
    {
        //Pos filt message and init time
        auto pos_filt_msg = pr_msgs::msg::PRArrayH();
        pos_filt_msg.init_time = this->get_clock()->now();

        //Vel filt message and init time
        auto vel_filt_msg = pr_msgs::msg::PRArrayH();
        vel_filt_msg.init_time = this->get_clock()->now();

        PRUtils::ArRMsg2Eigen(pos_msg, pos);

        // Kalman Filter Algorithm
        x = Ad*x;
        P = Ad*P*Ad.transpose() + Q;

        K = P*Cd.transpose()*(Cd*P*Cd.transpose() + R).inverse();
        
        x = x + K*(pos-Cd*x);

        P = (Eigen::Matrix<double, 8, 8>::Identity() - K*Cd)*P;

        pos_filt = x.head(4);
        vel_filt = x.tail(4);

        PRUtils::Eigen2ArMsg(pos_filt, pos_filt_msg);
        PRUtils::Eigen2ArMsg(vel_filt, vel_filt_msg);

        pos_filt_msg.header.stamp = pos_msg->header.stamp;
        pos_filt_msg.header.frame_id = pos_msg->header.frame_id;

        vel_filt_msg.header.stamp = pos_msg->header.stamp;
        vel_filt_msg.header.frame_id = pos_msg->header.frame_id;
        
        pos_filt_msg.current_time = this->get_clock()->now();
        publisher_pos->publish(pos_filt_msg);

        vel_filt_msg.current_time = this->get_clock()->now();
        publisher_vel->publish(vel_filt_msg);
    }

    
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_aux::KalmanFilter)