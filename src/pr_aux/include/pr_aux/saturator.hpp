#ifndef PR_AUX__SATURATOR_HPP_
#define PR_AUX__SATURATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_bool_h.hpp"

#include <array>
#include <vector>

namespace pr_aux
{
    class Saturator : public rclcpp::Node
    {
        public:
            //PR_AUX_PUBLIC
            explicit Saturator(const rclcpp::NodeOptions & options);

        protected:
            void topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr var_msg);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_;
            rclcpp::Publisher<pr_msgs::msg::PRBoolH>::SharedPtr publisher_pin_sat_;
            std::vector<double> min_val;
            std::vector<double> max_val;
    };
}

#endif // PR_AUX__SATURATOR_HPP_