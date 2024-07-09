#ifndef PR_REF_GEN__REF_POSE_HPP_
#define PR_REF_GEN__REF_POSE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "std_msgs/msg/bool.hpp"

#include "eigen3/Eigen/Dense"

namespace pr_ref_gen
{
    class RefPose : public rclcpp::Node
    {
        public:
            //PR_REF_GEN_PUBLIC
            explicit RefPose(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr q_msg);
            void external_stop_callback(const std_msgs::msg::Bool::SharedPtr external_stop_msg);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_external_stop_;

            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_x_;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_end_;
            std::vector<double> robot_params;
            std::string ref_path;
            bool is_cart;
            Eigen::MatrixXd ref_matrix_x;
            Eigen::MatrixXd ref_matrix_q;
            int n_ref, aux_n;
            int idx=0;
            // Samples to execute in the trajectory
            // If Nptos_set=0 the number of sample is equal to the rows in the reference file
            // Else the number of samples are defined by the user
            int Nptos_set;

            bool external_stop = false;
    };
}

#endif // PR_REF_GEN__REF_POSE_HPP_