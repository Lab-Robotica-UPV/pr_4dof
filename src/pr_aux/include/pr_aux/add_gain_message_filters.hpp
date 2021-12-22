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
            void topic_callback2(const pr_msgs::msg::PRArrayH::ConstPtr& input1_msg,
                                const pr_msgs::msg::PRArrayH::ConstPtr& input2_msg);
            void topic_callback3(const pr_msgs::msg::PRArrayH::ConstPtr& input1_msg,
                                const pr_msgs::msg::PRArrayH::ConstPtr& input2_msg,
                                const pr_msgs::msg::PRArrayH::ConstPtr& input3_msg);
            void topic_callback4(const pr_msgs::msg::PRArrayH::ConstPtr& input1_msg,
                                const pr_msgs::msg::PRArrayH::ConstPtr& input2_msg,
                                const pr_msgs::msg::PRArrayH::ConstPtr& input3_msg,
                                const pr_msgs::msg::PRArrayH::ConstPtr& input4_msg);
            void topic_callback5(const pr_msgs::msg::PRArrayH::ConstPtr& input1_msg,
                                const pr_msgs::msg::PRArrayH::ConstPtr& input2_msg,
                                const pr_msgs::msg::PRArrayH::ConstPtr& input3_msg,
                                const pr_msgs::msg::PRArrayH::ConstPtr& input4_msg,
                                const pr_msgs::msg::PRArrayH::ConstPtr& input5_msg);

        private:
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_in1;
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_in2;
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_in3;
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_in4;
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_in5;
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr sub_gain;

            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;

            std::vector<double> signs, gains;
            int num_inputs;
            Eigen::VectorXd gains_eigen;
            Eigen::Vector4d in1, in2, in3, in4, in5, out;

            typedef message_filters::sync_policies::ApproximateTime
                <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH> SyncPolicy2;
            typedef message_filters::Synchronizer<SyncPolicy2> Synchronizer2;
            std::shared_ptr<Synchronizer2> sync2_;

            typedef message_filters::sync_policies::ApproximateTime
                <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH> SyncPolicy3;
            typedef message_filters::Synchronizer<SyncPolicy3> Synchronizer3;
            std::shared_ptr<Synchronizer3> sync3_;

            typedef message_filters::sync_policies::ApproximateTime
                <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH> SyncPolicy4;
            typedef message_filters::Synchronizer<SyncPolicy4> Synchronizer4;
            std::shared_ptr<Synchronizer4> sync4_;

            typedef message_filters::sync_policies::ApproximateTime
                <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH> SyncPolicy5;
            typedef message_filters::Synchronizer<SyncPolicy5> Synchronizer5;
            std::shared_ptr<Synchronizer5> sync5_;


    };
}

#endif // PR_MODELLING_ADD_GAIN_HPP_