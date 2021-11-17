#ifndef PR_BIOMECH__StreamingGDLF_HPP_
#define PR_BIOMECH__StreamingGDLF_HPP_

#include "rclcpp/rclcpp.hpp"

#include "eigen3/Eigen/Dense"

#include "pr_lib_biomech/pr_json_calibration.hpp"
#include "pr_lib_biomech/pr_json_gdlf.hpp"
#include "pr_lib_biomech/pr_mocap.hpp"
#include "pr_lib_biomech/pr_data_structures.hpp"

#include <chrono>
#include <iostream>
#include <memory>

#include "pr_msgs/msg/pr_force_state.hpp"


namespace pr_biomech
{
    class StreamingGDLF : public rclcpp::Node
    {
        public:
            //PR_COONTROLLERS_PUBLIC
            explicit StreamingGDLF(const rclcpp::NodeOptions & options);

            rclcpp::Subscription<pr_msgs::msg::PRForceState>::SharedPtr subscription_;

            std::unique_ptr<PRJsonData::PRJsonCal::Calibration_data_struct> cal_data;
            std::unique_ptr<PRJsonData::PRJsonGdlf::Gdlf_data_struct> gdlf_data;
            std::unique_ptr<PRMocap::Mocap> mocap_object;
            //PRDataStructures::G_r_G_struct G_r_G;

        protected:
            void force_callback(const pr_msgs::msg::PRForceState::SharedPtr force_state_msg);

    };
}

#endif // PR_BIOMECH__StreamingGDLF_HPP_