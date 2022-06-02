#ifndef PR_CONTROLLERS__GUS_DISCRETIZED_CONTROLLER_HPP_
#define PR_CONTROLLERS__GUS_DISCRETIZED_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"

#include "eigen3/Eigen/Dense"

#include "pr_msgs/msg/pr_array_h.hpp"


namespace pr_controllers
{
    class GusDiscretizedController : public rclcpp::Node
    {
        public:
            //PR_COONTROLLERS_PUBLIC
            explicit GusDiscretizedController(const rclcpp::NodeOptions & options);

        protected:

            void controller_callback(const pr_msgs::msg::PRArrayH::SharedPtr pos_msg);

            void ref_callback(const pr_msgs::msg::PRArrayH::SharedPtr ref_msg);

        private:
            
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr sub_ref;
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr sub_pos;

            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;

            double ts, kxc, k2c;
            Eigen::Vector4d ref_ant, q_ant, x_ez, x_ez_ant;
            Eigen::Vector4d ref, pos, vel;
            Eigen::Vector4d ca;

            // Variables auxiliares
            Eigen::Vector4d e_x1, derivada_qref, e_x2, derivada_x_ez;

            bool init_ref = false;

    };
}

#endif // PR_CONTROLLERS__GUS_DISCRETIZED_CONTROLLER_HPP_