#ifndef PR_SING__SING_EVADER_HPP_
#define PR_SING__SING_EVADER_HPP_

#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/prots.hpp"
#include "pr_msgs/msg/pr_float_h.hpp"

#include "pr_lib/pr_singularity.hpp"


namespace pr_sing
{
    class SingEvader : public rclcpp::Node
    {
        public:
            //PR_SING_PUBLIC
            explicit SingEvader(const rclcpp::NodeOptions & options);

        protected:

            void topic_callback(const pr_msgs::msg::PRArrayH::ConstPtr& ref_msg,
                                     const pr_msgs::msg::PRArrayH::ConstPtr& x_msg,
                                     const pr_msgs::msg::PROTS::ConstPtr& ots_ref_msg,
                                     const pr_msgs::msg::PROTS::ConstPtr& ots_med_msg,
                                     const pr_msgs::msg::PRFloatH::ConstPtr& for_jac_det_ref,
                                     const pr_msgs::msg::PRFloatH::ConstPtr& for_jac_det_med);

        private:
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_ref;
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_x;
            message_filters::Subscriber<pr_msgs::msg::PROTS> sub_ots_ref;
            message_filters::Subscriber<pr_msgs::msg::PROTS> sub_ots_med;
            message_filters::Subscriber<pr_msgs::msg::PRFloatH> sub_det_ref;
            message_filters::Subscriber<pr_msgs::msg::PRFloatH> sub_det_med;

            typedef message_filters::sync_policies::ExactTime
                    <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH, 
                     pr_msgs::msg::PROTS, pr_msgs::msg::PROTS,
                     pr_msgs::msg::PRFloatH, pr_msgs::msg::PRFloatH> SyncPolicy;

            typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
            std::shared_ptr<Synchronizer> sync_;

            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_vc_;

            Eigen::Matrix<double,6,4> OTS_med = Eigen::Matrix<double,6,4>::Zero();
            Eigen::Matrix<double,6,1> angOTS_ref = Eigen::Matrix<double,6,1>::Zero();
            Eigen::Matrix<double,6,1> angOTS_med = Eigen::Matrix<double,6,1>::Zero();
            Eigen::MatrixXi minc_des;
            Eigen::Vector4i vc_des = Eigen::Vector4i::Zero();
            Eigen::Vector4d q_ind_mod = Eigen::Vector4d::Zero();
            Eigen::Vector4d x_coord = Eigen::Vector4d::Zero();
            Eigen::Vector4d q_ref = Eigen::Vector4d::Zero();
            Eigen::Matrix<double,4,2> Mlim_q_ind = Eigen::Matrix<double,4,2>::Zero();
            Eigen::Vector4d Vlim_angp = Eigen::Vector4d::Zero();
            
            std::vector<double> robot_params;
            int iterations=0, iter_max=30, ncomb, iter_OTS;
            double tol, tol_OTS, ts, des_qind, lmin_Ang_OTS, lmin_FJac;

    };
}

#endif // PR_SING__SING_EVADER_HPP_