#ifndef PR_ALGORITHM__ASSISTANCE_HPP_
#define PR_ALGORITHM__ASSISTANCE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pr_msgs/msg/pr_force_state.hpp"
#include "pr_msgs/msg/pr_float_h.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"


#include "eigen3/Eigen/Dense"

// JSON writer from rapidjson library
#include <chrono>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <iostream>
#include <memory>
#include <math.h>

#include "pr_lib/pr_utils.hpp"


namespace pr_biomech
{
    class AlgorithmAssistance : public rclcpp::Node
    {
        public:
            // PUBLIC_CONSTRUCTOR
            explicit AlgorithmAssistance(const rclcpp::NodeOptions & options);
            // DESTRUCTOR
            ~AlgorithmAssistance();
            

        protected:
            void force_callback(const pr_msgs::msg::PRForceState::SharedPtr force_state_msg);
            void gen_force_knee_callback(const pr_msgs::msg::PRFloatH::SharedPtr gen_force_knee_msg);
            void muscle_force_callback(const pr_msgs::msg::PRFloatH::SharedPtr muscle_force_msg);
            void ref_muscle_force_callback(const pr_msgs::msg::PRFloatH::SharedPtr ref_muscle_force_msg);
            void muscle_dir_callback(const pr_msgs::msg::PRArrayH::SharedPtr muscle_dir_msg);


        private:

            rclcpp::Subscription<pr_msgs::msg::PRForceState>::SharedPtr sub_force_state;
            rclcpp::Subscription<pr_msgs::msg::PRFloatH>::SharedPtr sub_gen_force_knee;
            rclcpp::Subscription<pr_msgs::msg::PRFloatH>::SharedPtr sub_muscle_force;
            rclcpp::Subscription<pr_msgs::msg::PRFloatH>::SharedPtr sub_ref_muscle_force;
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr sub_muscle_dir;

            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr pub_ass_force;
            rclcpp::Publisher<pr_msgs::msg::PRFloatH>::SharedPtr pub_ass_force_prop;
            rclcpp::Publisher<pr_msgs::msg::PRFloatH>::SharedPtr pub_ass_force_der;
            rclcpp::Publisher<pr_msgs::msg::PRFloatH>::SharedPtr pub_ass_force_filt;
            rclcpp::Publisher<pr_msgs::msg::PRFloatH>::SharedPtr pub_ass_alpha;
            //rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_F_opt_ref;

            // PARAMETERS
            // Sample time
            double ts;
            // Proportional constant of PID
            double kp, ki, kd;
            // Geometry limb
            double length_tibia, length_foot;


            // Booleans of first iteration
            bool read_force_state, read_gen_force_knee, read_muscle_force, read_ref_muscle_force, read_muscle_dir;

            // Variables from the callback
            Eigen::Vector4d force_state, muscle_dir;
            double gen_force_knee, muscle_force, ref_muscle_force;

            // Assistive force (vector and norm)
            Eigen::Vector4d ass_force;
            double ass_force_scalar; 
            
            // Reduction factor of the error
            double alpha = 0, alpha_desired, alpha_inc = 0.01; // alpha is the reduction factor of the assistive force

            // Angle between force_state_dir and muscle_dir
            Eigen::Vector4d force_state_dir;
            double force_state_norm;
            double force_angle, force_angle_lim = M_PI/4;

            // Variables for controller
            double e, e_ant=0, ud, ud_ant=0, c_filt;
            // Auxiliar variables for visualization
            double ass_force_prop, ass_force_der, ass_force_filt;


    };

}

#endif // PR_ALGORITHM__ASSISTANCE_HPP_