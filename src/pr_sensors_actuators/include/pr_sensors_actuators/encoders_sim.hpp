#ifndef PR_SENSORS_ACTUATORS__ENCODERS_SIM_HPP_
#define PR_SENSORS_ACTUATORS__ENCODERS_SIM_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "std_msgs/msg/bool.hpp"

#include "eigen3/Eigen/Dense"


namespace pr_sensors_actuators
{

    class EncodersSim : public rclcpp::Node
    {
        public:
            //PR_SENSORS_ACTUATORS_PUBLIC
            explicit EncodersSim(const rclcpp::NodeOptions & options);

        protected:

            void on_timer();

            void end_callback(const std_msgs::msg::Bool::SharedPtr end_msg);

        private:

            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_end_;
            rclcpp::TimerBase::SharedPtr timer_;
            
            
            // VAriable for detecting the end of the reference path
            bool is_finished = false;
            
            //Encoder reference path
            std::string enc_path;

            //Variable for iteration
            int n_ref, iter = 0;

            //Matrix for actuators reference
            Eigen::MatrixXd ref_matrix_q;

    };

}   // Namespace pr_sensors_actuators


#endif // PR_SENSORS_ACTUATORS__ENCODERS_SIM_HPP_