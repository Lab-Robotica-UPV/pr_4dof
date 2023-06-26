#ifndef PR_BIOMECH_ADMITTANCE_EULER_BIOMECH_HPP_
#define PR_BIOMECH_ADMITTANCE_EULER_BIOMECH_HPP_

#include "rclcpp/rclcpp.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_force_state.hpp"
#include "pr_msgs/msg/pr_bool_h.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "pr_lib/pr_model.hpp"
#include "pr_lib/pr_utils.hpp"

#include "eigen3/Eigen/Dense"
#include "eigen3/unsupported/Eigen/MatrixFunctions"

namespace pr_biomech
{
    class AdmittanceEulerBiomech : public rclcpp::Node
    {
        public:
            explicit AdmittanceEulerBiomech(const rclcpp::NodeOptions & options);

        protected:
            void ass_force_callback(const pr_msgs::msg::PRArrayH::SharedPtr f_ass_msg);
            void force_callback(const pr_msgs::msg::PRForceState::SharedPtr f_msg);
            void saturation_pin_callback(const pr_msgs::msg::PRBoolH::SharedPtr saturation_pin_msg);
        private:

            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr f_ass_sub;
            rclcpp::Subscription<pr_msgs::msg::PRForceState>::SharedPtr f_sub;
            rclcpp::Subscription<pr_msgs::msg::PRBoolH>::SharedPtr saturation_pin_sub;
            
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_pos_;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_vel_;

            std::vector<double> mass, damping;
            double ts;

            std::vector<double> vel, pos, dvel, dpos, aux_pos, aux_vel;

            Eigen::Vector4d force_total, force_ass;

            bool init_f = false;

            bool saturation_pin = false;
            // By default, saturation_pin is in false, which means that if no program changes it, the admittance
            // will be computed as expected
    };

}

#endif // PR_BIOMECH_ADMITTANCE_EULER_BIOMECH_HPP_