#ifndef PR_MODELLING_INV_DIFF_KIN_HPP_
#define PR_MODELLING_INV_DIFF_KIN_HPP_

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_mat_h.hpp"
#include "pr_lib/pr_model.hpp"
#include "pr_lib/pr_utils.hpp"

#include "eigen3/Eigen/Dense"

namespace pr_modelling
{
    class InvDiffKin : public rclcpp::Node
    {
        public:
            //PR_MODELLING_PUBLIC
            explicit InvDiffKin(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback(const pr_msgs::msg::PRArrayH::ConstPtr& vel_cart_msg,
                                const pr_msgs::msg::PRArrayH::ConstPtr& pos_cart_msg);

        private:
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_vel_cart;
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_vel_cart;

            typedef message_filters::sync_policies::ApproximateTime
                    <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH> SyncPolicy;
            /*
            typedef message_filters::sync_policies::ExactTime
                    <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH> SyncPolicy;
            */
            typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
            std::shared_ptr<Synchronizer> sync_;

            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_qp;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_q;
            rclcpp::Publisher<pr_msgs::msg::PRFloatH>::SharedPtr publisher_detFwdJac;

            std::vector<double> robot_params;
            
            Eigen::Vector4d integ, qp_ant, qp, v;

    };
}

#endif // PR_MODELLING_INV_DIFF_KIN_HPP_