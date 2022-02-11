#ifndef PR_AUX__MATRIX_MULT_HPP_
#define PR_AUX__MATRIX_MULT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_mat_h.hpp"
#include "std_msgs/msg/bool.hpp"

#include <chrono>
#include <memory>
#include <utility>

#include "eigen3/Eigen/Dense"

#include <array>
#include <vector>

// This node performs a matrix multiplication between a matrix of 4x4 and a vector 4x1 given in topics,
// and outputs the resulting vector in another topic

namespace pr_aux
{
    class MatrixMult : public rclcpp::Node
    {
        public:
            //PR_AUX_PUBLIC
            explicit MatrixMult(const rclcpp::NodeOptions & options);

        protected:
            void topic_vector_callback(const pr_msgs::msg::PRArrayH::SharedPtr matrix_msg);
            void topic_matrix_callback(const pr_msgs::msg::PRMatH::SharedPtr vector_msg);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_vector;
            rclcpp::Subscription<pr_msgs::msg::PRMatH>::SharedPtr subscription_matrix;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;

            Eigen::Matrix<double, 4, 4> input_matrix;
            Eigen::Vector4d input_vector;
            Eigen::Vector4d output_vector;
            bool init_matrix = false;
    };
}

#endif // PR_AUX__MatrixMult_HPP_