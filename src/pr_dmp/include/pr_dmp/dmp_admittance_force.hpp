#ifndef PR_DMP__ADMITTANCE_FORCE_HPP_
#define PR_DMP__ADMITTANCE_FORCE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_bool_h.hpp"
#include "pr_msgs/msg/pr_force_state.hpp"

#include <vector>
#include <chrono>
#include <memory>
#include <utility>

#include "eigen3/Eigen/Dense"

namespace pr_dmp
{
    class AdmittanceForce : public rclcpp::Node
    {
        public:
            //PR_DMP_PUBLIC
            explicit AdmittanceForce(const rclcpp::NodeOptions & options);

        protected:
            void ref_force_callback(const pr_msgs::msg::PRArrayH::SharedPtr ref_force_msg);
            void force_callback(const pr_msgs::msg::PRForceState::SharedPtr force_msg);
            void activation_pin_callback(const pr_msgs::msg::PRBoolH::SharedPtr activation_pin_msg);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_ref_force_;
            rclcpp::Subscription<pr_msgs::msg::PRForceState>::SharedPtr subscription_force_;
            rclcpp::Subscription<pr_msgs::msg::PRBoolH>::SharedPtr subscription_activation_pin_;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;
            // Ganancia de la admitancia (parametro)
            std::vector<double> K_adm;
            // activation_pin
            bool activation_pin = true;
            // Ultima fuerza medida
            Eigen::Vector4d current_force;
            // Medida comenzada
            bool init_f = false;
    };
}

#endif // PR_DMP__ADMITTANCE_FORCE_HPP_