#ifndef PR_MOCAP__PR_X_MOCAP_RECORDER_HPP_
#define PR_MOCAP__PR_X_MOCAP_RECORDER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "std_msgs/msg/bool.hpp"

#include <iostream>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <iostream>
#include <netdb.h> //gethostbyname


namespace pr_mocap
{
    class PRXMocapRecorder : public rclcpp::Node
    {
        public:
            //PR_AUX_PUBLIC
            explicit PRXMocapRecorder(const rclcpp::NodeOptions & options);
            ~PRXMocapRecorder();

        protected:
            void start_callback(const pr_msgs::msg::PRArrayH::SharedPtr joint_position_msg);
            void stop_callback(const std_msgs::msg::Bool::SharedPtr end_msg);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRArrayH>::SharedPtr subscription_start_;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_stop_;

            // bool to indicate the recording has started
            bool recording_started = false;
            bool recording_stopped = false;

            // Socket variables
            const int port_xml = 1512;
            int receiverSocket;
            struct sockaddr_in ToAddr;

            std::string filename;

            std::string start_message_str; 
            std::string stop_message_str;
            char start_message_char[512], stop_message_char[512];
            const int value = 1;

            
    };
}

#endif // PR_MOCAP__PR_X_MOCAP_RECORDER_HPP_