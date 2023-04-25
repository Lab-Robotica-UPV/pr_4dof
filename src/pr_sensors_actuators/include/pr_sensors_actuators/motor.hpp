#ifndef PR_SENSORS_ACTUATORS__MOTOR_HPP_
#define PR_SENSORS_ACTUATORS__MOTOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include <chrono>

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_float_h.hpp"
#include "std_msgs/msg/bool.hpp"



#include "bdaqctrl.h"
#define deviceDescription L"PCI-1720,BID#15"
using namespace Automation::BDaq;


namespace pr_sensors_actuators
{
    class Motor : public rclcpp::Node
    {
        public:
            explicit Motor(const rclcpp::NodeOptions & options);
            ~Motor();

        protected:
            void topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr control_action_msg);
            void end_callback(const std_msgs::msg::Bool::SharedPtr end_msg);
            void init_ao_pci(void);
            void sat_ca(double &control_action, const double &sat);
        
        private:
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_end_;
            rclcpp::Publisher<pr_msgs::msg::PRFloatH>::SharedPtr publisher_mean_;
            rclcpp::Publisher<pr_msgs::msg::PRFloatH>::SharedPtr publisher_var_;
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_end_;
            int n_motor;
            InstantAoCtrl * pci1720;
            ErrorCode ret;
            double volts = 0.0;
            double max_v;
            bool is_finished = false;
            double vp_conversion;

            // This variables are to study the oscillations in the control action
            double running_mean;
            double running_var = 0;
            double alpha = 0.9;
            bool init_sample = true;
            double threshold_var = 5;
    };

}   // Namespace pr_sensors_actuators


#endif // PR_SENSORS_ACTUATORS__MOTOR_HPP_