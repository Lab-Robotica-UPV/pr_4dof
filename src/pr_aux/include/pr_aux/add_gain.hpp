#ifndef PR_MODELLING_ADD_GAIN_HPP_
#define PR_MODELLING_ADD_GAIN_HPP_

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_lib/pr_utils.hpp"

#include "eigen3/Eigen/Dense"

namespace pr_aux
{
    class AddGain : public rclcpp::Node
    {
        public:
            //PR_AUX_PUBLIC
            explicit AddGain(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback1(const pr_msgs::msg::PRArrayH::SharedPtr input1_msg);
            void topic_callback2(const pr_msgs::msg::PRArrayH::SharedPtr input2_msg);
            void topic_callback3(const pr_msgs::msg::PRArrayH::SharedPtr input3_msg);
            void topic_callback4(const pr_msgs::msg::PRArrayH::SharedPtr input4_msg);
            void topic_callback5(const pr_msgs::msg::PRArrayH::SharedPtr input5_msg);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr sub_1;
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr sub_2;
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr sub_3;
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr sub_4;
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr sub_5;

            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;

            std::vector<double> signs, gains;
            int num_inputs;
            Eigen::VectorXd gains_eigen;
            Eigen::Vector4d in1, in2, in3, in4, in5, out;

            bool init_1 = false, init_2 = false, init_3 = false, init_4 = false, init_5 = false;


    };
}

#endif // PR_MODELLING_ADD_GAIN_HPP_