#ifndef PR_MODELLING_FORCE_FIXED_FRAME_HPP_
#define PR_MODELLING_FORCE_FIXED_FRAME_HPP_

#include "rclcpp/rclcpp.hpp"

#include "pr_msgs/msg/pr_force_state.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_lib/pr_utils.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"


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
            void topic_callback(const pr_msgs::msg::PRArrayH::ConstPtr& x_msg,
                                const pr_msgs::msg::PRForceState::ConstPtr& f_msg);
            void apply_threshold(Eigen::Vector3d &force, Eigen::Vector3d &torque);

        private:

            // Message filters to coordinate force and cameras
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_x;
            message_filters::Subscriber<pr_msgs::msg::PRForceState> sub_f;

            typedef message_filters::sync_policies::ExactTime
                    <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRForceState> SyncPolicy;

            typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
            std::shared_ptr<Synchronizer> sync_;

            
            rclcpp::Publisher<pr_msgs::msg::PRForceState>::SharedPtr publisher_f_fixed;

            // Boolean for first iteration
            bool first_iter = true;

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

            // Noise threshold
            std::vector<double> std_noise;
            
            // Gravity vector
            const Eigen::Vector3d g = {0, 0, -9.81};
            

    };

}

#endif // PR_MODELLING_FORCE_FIXED_FRAME_HPP_