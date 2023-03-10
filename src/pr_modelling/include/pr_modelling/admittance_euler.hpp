#ifndef PR_MODELLING_ADMITTANCE_EULER_HPP_
#define PR_MODELLING_ADMITTANCE_EULER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_force_state.hpp"

#include "pr_msgs/msg/pr_bool_h.hpp"

#include "eigen3/Eigen/Dense"
#include "eigen3/unsupported/Eigen/MatrixFunctions"

namespace pr_modelling
{
    class AdmittanceEuler : public rclcpp::Node
    {
        public:
            explicit AdmittanceEuler(const rclcpp::NodeOptions & options);

        protected:
            void ref_force_callback(const pr_msgs::msg::PRArrayH::SharedPtr f_ref_msg);
            void force_callback(const pr_msgs::msg::PRForceState::SharedPtr f_msg);
            void activation_pin_callback(const pr_msgs::msg::PRBoolH::SharedPtr activation_pin_msg);
            void saturation_pin_callback(const pr_msgs::msg::PRBoolH::SharedPtr saturation_pin_msg);
        private:

            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr f_ref_sub;
            rclcpp::Subscription<pr_msgs::msg::PRForceState>::SharedPtr f_sub;
            rclcpp::Subscription<pr_msgs::msg::PRBoolH>::SharedPtr activation_pin_sub;
            rclcpp::Subscription<pr_msgs::msg::PRBoolH>::SharedPtr saturation_pin_sub;
            
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_pos_;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_vel_;

            std::vector<double> mass, damping, stiffness;
            double ts;

            std::vector<double> vel, pos, dvel, dpos, aux_pos, aux_vel;

            Eigen::Vector4d force_error;

            pr_msgs::msg::PRForceState msgForce;
            bool init_f = false;
            bool activation_pin = true;
            // By default, activation_pin is in true, which means that if no program changes it, the admittance
            // will be computed as expected
            bool saturation_pin = false;
            // By default, saturation_pin is in false, which means that if no program changes it, the admittance
            // will be computed as expected
    };

}

#endif // PR_MODELLING_ADMITTANCE_EULER_HPP_