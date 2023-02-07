#ifndef PR_TOPIC_FORWARDING__TOPIC_FORWARDING_HPP_
#define PR_TOPIC_FORWARDING__TOPIC_FORWARDING_HPP_

#include "rclcpp/rclcpp.hpp"

#include "eigen3/Eigen/Dense"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_bool_h.hpp"
#include "pr_msgs/msg/pr_float_h.hpp"
#include "pr_msgs/msg/pr_force_state.hpp"
#include "pr_msgs/msg/pr_mat_h.hpp"
#include "pr_msgs/msg/prots.hpp"


namespace pr_topic_forwarding
{
    class TopicForwarding : public rclcpp::Node
    {
        public:
            //PR_COONTROLLERS_PUBLIC
            explicit TopicForwarding(const rclcpp::NodeOptions & options);

        protected:

            void topic_callback_arrayh(const pr_msgs::msg::PRArrayH::SharedPtr msg, int index);


        private:

            // Suffix for publishers
            std::string suffix_publisher;

            // Topics to exclude
            std::vector<std::string> topics_exclude = {
                "/end_flag",
                "/parameter_events",
                "/rosout"
            };

            // Topics include and their message
            std::map<std::string,std::string> topics_include;

            // Topics include and their number (1-6 for the pub/sub and location on vector)
            std::map<std::string, std::pair<int,int>> topics_pubsub;

            // Publishers
            //1
            std::vector<rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr> publishers_arrayh;
            //2
            std::vector<rclcpp::Publisher<pr_msgs::msg::PRBoolH>::SharedPtr> publishers_boolh;
            //3
            std::vector<rclcpp::Publisher<pr_msgs::msg::PRFloatH>::SharedPtr> publishers_floath;
            //4
            std::vector<rclcpp::Publisher<pr_msgs::msg::PRForceState>::SharedPtr> publishers_forcestate;
            //5
            std::vector<rclcpp::Publisher<pr_msgs::msg::PRMatH>::SharedPtr> publishers_math;
            //6
            std::vector<rclcpp::Publisher<pr_msgs::msg::PROTS>::SharedPtr> publishers_ots;

            // Subscribers
            //1
            std::vector<rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr> subscribers_arrayh;
            //2
            std::vector<rclcpp::Subscription<pr_msgs::msg::PRBoolH>::SharedPtr> subscribers_boolh;
            //3
            std::vector<rclcpp::Subscription<pr_msgs::msg::PRFloatH>::SharedPtr> subscribers_floath;
            //4
            std::vector<rclcpp::Subscription<pr_msgs::msg::PRForceState>::SharedPtr> subscribers_forcestate;
            //5
            std::vector<rclcpp::Subscription<pr_msgs::msg::PRMatH>::SharedPtr> subscribers_math;
            //6
            std::vector<rclcpp::Subscription<pr_msgs::msg::PROTS>::SharedPtr> subscribers_ots;


            double ts, k1, k2;
            Eigen::Vector4d ref_ant, q_ant, up_1_ant;
            Eigen::Vector4d ref, pos, vel;
            Eigen::Vector4d up_1, up_2, ca;

            bool init_ref = false;

    };
}

#endif // PR_TOPIC_FORWARDING__TOPIC_FORWARDING_HPP_