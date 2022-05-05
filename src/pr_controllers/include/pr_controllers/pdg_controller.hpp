#ifndef PR_CONTROLLERS__PDG_CONTROLLER_HPP_
#define PR_CONTROLLERS__PDG_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"

#include "pr_msgs/msg/pr_array_h.hpp"

#include "eigen3/Eigen/Dense"


namespace pr_controllers
{
    class PDGController : public rclcpp::Node
    {
        public:
            //PR_COONTROLLERS_PUBLIC
            explicit PDGController(const rclcpp::NodeOptions & options);
            ~PDGController();

        protected:

            void controller_callback(const pr_msgs::msg::PRArrayH::ConstPtr& pos_msg,
                                     const pr_msgs::msg::PRArrayH::ConstPtr& vel_msg);
            void ref_callback(const pr_msgs::msg::PRArrayH::SharedPtr ref_msg);
            void grav_callback(const pr_msgs::msg::PRArrayH::SharedPtr grav_msg);

        private:

            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_pos;
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_vel;

            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr sub_ref;
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr sub_grav;

            typedef message_filters::sync_policies::ApproximateTime
                    <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH> SyncPolicy;
            /*
            typedef message_filters::sync_policies::ExactTime
                    <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH> SyncPolicy;
            */

            typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
            std::shared_ptr<Synchronizer> sync_;

            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;

            std::vector<double> Kp, Kv;

            pr_msgs::msg::PRArrayH ref, grav;

            bool init_ref = false;
            bool init_grav = false;

            // For calculation of performance indices
            int iter = 0;
            Eigen::Vector4d control_action_ant, e, control_action;
            Eigen::Vector4d index_error = Eigen::Vector4d::Zero();
            Eigen::Vector4d index_action = Eigen::Vector4d::Zero();
            double Je, Ju;
    };
}

#endif // PR_CONTROLLERS__PDG_CONTROLLER_HPP_