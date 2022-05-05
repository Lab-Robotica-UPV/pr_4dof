#ifndef PR_AUX__ELEMENT_WISE_MULT_HPP_
#define PR_AUX__ELEMENT_WISE_MULT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"
#include "std_msgs/msg/bool.hpp"
#include "pr_lib/pr_utils.hpp"

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
    class ElementWiseMult : public rclcpp::Node
    {
        public:
            //PR_AUX_PUBLIC
            explicit ElementWiseMult(const rclcpp::NodeOptions & options);

        protected:
            void topic_vector1_callback(const pr_msgs::msg::PRArrayH::SharedPtr vector1_msg);
            void topic_vector2_callback(const pr_msgs::msg::PRArrayH::SharedPtr vector2_msg);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_vector1;
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_vector2;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;

            Eigen::Vector4d input_vector2;
            bool init_vector2 = false;
    };
}

#endif // PR_AUX__ELEMENT_WISE_HPP_