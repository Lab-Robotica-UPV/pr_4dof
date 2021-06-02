#ifndef PR_CONTROLLERS__PID_CONTROLLER_HPP_
#define PR_CONTROLLERS__PID_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"

#include "pr_msgs/msg/pr_array_h.hpp"


namespace pr_controllers
{
    class PIDController : public rclcpp::Node
    {
        public:
            //PR_COONTROLLERS_PUBLIC
            explicit PIDController(const rclcpp::NodeOptions & options);

        protected:

            void controller_callback(const pr_msgs::msg::PRArrayH::ConstPtr& pos_msg);
            void ref_callback(const pr_msgs::msg::PRArrayH::SharedPtr ref_msg);

        private:
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;

            std::vector<double> Kp, Kv, Ki;

            Eigen::Vector4d pos_ant;
            Eigen::Vector4d ref, pos, vel;

            bool init_ref = false;
    };
}

#endif // PR_CONTROLLERS__PID_CONTROLLER_HPP_