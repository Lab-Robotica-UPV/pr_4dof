#ifndef PR_MODELLING_ADMITTANCE_HPP_
#define PR_MODELLING_ADMITTANCE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_force_state.hpp"

#include "eigen3/Eigen/Dense"
#include "eigen3/unsupported/Eigen/MatrixFunctions"

namespace pr_modelling
{
    class Admittance : public rclcpp::Node
    {
        public:
            explicit Admittance(const rclcpp::NodeOptions & options);

        protected:
            void ref_force_callback(const pr_msgs::msg::PRArrayH::SharedPtr f_ref_msg);
            void force_callback(const pr_msgs::msg::PRForceState::SharedPtr f_msg);

        private:

            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr f_ref_sub;
            rclcpp::Subscription<pr_msgs::msg::PRForceState>::SharedPtr f_sub;
            
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_pos_;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_vel_;

            std::vector<double> mass, damping, stiffness;
            double ts;

            Eigen::Matrix<double,8,8> Ac;
            Eigen::Matrix<double,8,4> Bc;
            Eigen::Matrix<double,8,8> Cc;
            Eigen::Matrix<double,8,4> Dc;

            Eigen::Matrix<double,8,8> Ad;
            Eigen::Matrix<double,8,4> Bd;
            Eigen::Matrix<double,8,8> Cd;
            Eigen::Matrix<double,8,4> Dd;

            Eigen::Matrix<double,8,1> state, state_, output;

            Eigen::Vector4d force_error;

            pr_msgs::msg::PRForceState msgForce;
            bool init_f = false;
    };

}

#endif // PR_MODELLING_ADMITTANCE_HPP_