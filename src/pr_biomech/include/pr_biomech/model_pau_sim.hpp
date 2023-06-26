#ifndef PR_BIOMECH_MODEL_PAU_SIM_HPP_
#define PR_BIOMECH_MODEL_PAU_SIM_HPP_

#include "rclcpp/rclcpp.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_float_h.hpp"
#include "pr_msgs/msg/pr_force_state.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "pr_lib/pr_model.hpp"
#include "pr_lib/pr_utils.hpp"

#include "eigen3/Eigen/Dense"

namespace pr_biomech
{
    class ModelPauSim : public rclcpp::Node
    {
        public:
            explicit ModelPauSim(const rclcpp::NodeOptions & options);

        protected:
            void force_callback(const pr_msgs::msg::PRForceState::SharedPtr f_msg);

        private:

            rclcpp::Subscription<pr_msgs::msg::PRForceState>::SharedPtr sub_force_state;
            
            rclcpp::Publisher<pr_msgs::msg::PRFloatH>::SharedPtr pub_ref_muscle_force;
            rclcpp::Publisher<pr_msgs::msg::PRFloatH>::SharedPtr pub_muscle_force;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr pub_dir_force;

            // Geometry limb
            double length_tibia, length_foot;

            // Variables associated with pub/sub
            double ref_muscle_force, muscle_force;
            std::vector<double> dir_force;
            Eigen::Vector4d force_state;

            // Messages for publishers
            pr_msgs::msg::PRFloatH ref_muscle_force_msg, muscle_force_msg;
            pr_msgs::msg::PRArrayH dir_force_msg;
    };

}

#endif // PR_BIOMECH_MODEL_PAU_SIM_HPP_