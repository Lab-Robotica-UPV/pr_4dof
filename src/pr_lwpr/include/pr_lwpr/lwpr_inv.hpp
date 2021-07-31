#ifndef PR_LWPR_INV_HPP_
#define PR_LWPR_INV_HPP_

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
    class LWPRInv : public rclcpp::Node
    {
        public:
            explicit LWPRInv(const rclcpp::NodeOptions & options);
            ~LWPRInv();

        protected:
            void ref_callback(const pr_msgs::msg::PRArrayH::SharedPtr q_msg);
            void u_callback(const pr_msgs::msg::PRArrayH::SharedPtr u_msg);

        private:

            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_ref;
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_u;

            std::vector<double> initD;
            std::vector<double> initAlpha, penalty, initLambda, finalLambda;
            bool activateLearning;
            bool activatePrediction;
            std::string loadModel, saveModel;
            
            Eigen::Vector4d q, q_ant, qp, qp_ant, qpp, u;

            Eigen::VectorXd state;
            Eigen::VectorXd y;
            Eigen::VectorXd output;
            Eigen::Vector4d scale_output;

            //std::array<std::unique_ptr<LWPR_Object>,4> models;
            std::vector<LWPR_Object *> models;
            //LWPR_Object models[4];
            bool first_iter = true;
            bool read_pos = false;
            bool read_u = false;
            double ts;
            
    };

}

#endif // PR_LWPR_FWD_HPP_