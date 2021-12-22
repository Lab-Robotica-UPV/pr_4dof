#ifndef PR_MODELLING_FORCE_FIXED_FRAME_HPP_
#define PR_MODELLING_FORCE_FIXED_FRAME_HPP_

#include "rclcpp/rclcpp.hpp"

#include "pr_msgs/msg/pr_force_state.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_lib/pr_utils.hpp"



#include "eigen3/Eigen/Dense"

// This node calculates the force of the sensor in the fixed frame, with the possibility
// of compensating the boot force to avoid unexpected movements.
// Inputs: force_state_sync and x_coord_mocap_sync
// Output: compensated force
// Parameters:: boot_compensation, boot_mass, boot_cdg

namespace pr_modelling
{
    class ForceFixedFrame : public rclcpp::Node
    {
        public:
            explicit ForceFixedFrame(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback_force(const pr_msgs::msg::PRForceState::SharedPtr f_msg);
            void topic_callback_x_mocap(const pr_msgs::msg::PRArrayH::SharedPtr x_msg);
            void apply_threshold(Eigen::Vector3d &force, Eigen::Vector3d &torque);

        private:

            // Subscriptor
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr x_sub;
            rclcpp::Subscription<pr_msgs::msg::PRForceState>::SharedPtr f_sub;

            
            rclcpp::Publisher<pr_msgs::msg::PRForceState>::SharedPtr publisher_f_fixed;

            // Boolean for first iteration and message for x_msg
            bool init_x = false, first_iter = true;

            // Boot mass parameter
            double boot_mass;

            // Centre of gravity of the boot
            std::vector<double> boot_cdg;
            Eigen::Vector3d boot_cdg_vector;

            // Boot compensation activation
            bool boot_compensation;

            // Noise threshold activation
            bool fixed_frame_noise_threshold;

            // Init forces
            Eigen::Vector3d Fb_ini_m, Tb_ini_m;

            // Init theta and psi
            double theta_ini, psi_ini;

            // Current theta and psi
            double theta, psi;

            // Noise threshold
            std::vector<double> std_noise;
            
            // Gravity vector
            const Eigen::Vector3d g = {0, 0, -9.81};
            

    };

}

#endif // PR_MODELLING_FORCE_FIXED_FRAME_HPP_