#ifndef PR_LWPR_FWD_HPP_
#define PR_LWPR_FWD_HPP_

#include "rclcpp/rclcpp.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"

#include "eigen3/Eigen/Dense"

#include <lwpr.hh>
#include <vector>
#include <array>
#include <string>

namespace pr_lwpr
{
    class LWPRFwd : public rclcpp::Node
    {
        public:
            explicit LWPRFwd(const rclcpp::NodeOptions & options);
            ~LWPRFwd();

        protected:
            void topic_callback(const pr_msgs::msg::PRArrayH::ConstPtr& q_msg,
                                const pr_msgs::msg::PRArrayH::ConstPtr& qp_msg,
                                const pr_msgs::msg::PRArrayH::ConstPtr& u_msg);

        private:

            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_q;
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_qp;
            message_filters::Subscriber<pr_msgs::msg::PRArrayH> sub_u;

            typedef message_filters::sync_policies::ApproximateTime
                    <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH> SyncPolicy;

            /*typedef message_filters::sync_policies::ExactTime
                    <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH, 
                     pr_msgs::msg::PROTS, pr_msgs::msg::PROTS,
                     pr_msgs::msg::PRFloatH, pr_msgs::msg::PRFloatH> SyncPolicy;*/

            typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
            std::shared_ptr<Synchronizer> sync_;
            
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;

            std::vector<double> initD;
            std::vector<double> initAlpha, penalty, initLambda, finalLambda;
            bool activateLearning;
            bool activatePrediction;
            std::string loadModel, saveModel;
            
            Eigen::Vector4d q, qp, q_ant, qp_ant, u, u_ant;

            Eigen::VectorXd state;
            Eigen::VectorXd y;
            Eigen::VectorXd output;

            //std::array<std::unique_ptr<LWPR_Object>,4> models;
            std::vector<LWPR_Object *> models;
            //LWPR_Object models[4];
            bool first_iter = true;
            
    };

}

#endif // PR_LWPR_FWD_HPP_