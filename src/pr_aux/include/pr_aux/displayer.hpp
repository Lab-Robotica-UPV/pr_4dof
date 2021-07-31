#ifndef PR_AUX__DISPLAYER_HPP_
#define PR_AUX__DISPLAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"
#include "std_msgs/msg/bool.hpp"

#include <array>
#include <vector>

#include "eigen3/Eigen/Dense"

namespace pr_aux
{
    class Displayer : public rclcpp::Node
    {
        public:
            //PR_AUX_PUBLIC
            explicit Displayer(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr q_msg);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;
            Eigen::MatrixXd data_matrix;
            std::string data_path;
            int idx = 0;
    };
}

#endif // PR_AUX__DISPLAYER_HPP_