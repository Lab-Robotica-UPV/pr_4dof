#include "pr_topic_forwarding/topic_forwarding.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"



namespace pr_topic_forwarding
{
    /**** TOPIC FORWARDING COMPONENT ****/
    TopicForwarding::TopicForwarding(const rclcpp::NodeOptions & options)
    : Node("topic_forwarding", options)
    {
        //Parameter declaration
        this->declare_parameter<std::string>("suffix_publisher", "_p");
        

        this->get_parameter("suffix_publisher", suffix_publisher);
        

        // Pause before start
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        
        // get_node_names
        auto node_names = rclcpp::Node::get_node_names();
        for (auto name:node_names){
            std::cout << name << std::endl;
        }

        // get_topic_names_and_types
        auto topic_names_and_types = rclcpp::Node::get_topic_names_and_types();
        for (const auto& [key, value] : topic_names_and_types){
            if (std::find(topics_exclude.begin(), topics_exclude.end(), key) == topics_exclude.end()){
                topics_include.insert(std::pair<std::string,std::string>(key,value[0]));
            }  
        }

        for (const auto& [key, value] : topics_include){
            if (value == "pr_msgs/msg/PRArrayH"){
                // Update subscribers
                std::function<void(const pr_msgs::msg::PRArrayH::SharedPtr msg)> callback_func =
                std::bind(&TopicForwarding::topic_callback_arrayh, this, std::placeholders::_1, 
                static_cast<int>(subscribers_arrayh.size()));

                subscribers_arrayh.push_back(this->create_subscription<pr_msgs::msg::PRArrayH>(
                key,
                1,
                callback_func
                ));
                // // Update publishers
                publishers_arrayh.push_back(this->create_publisher<pr_msgs::msg::PRArrayH>(key+suffix_publisher, 1//rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_default)
                ));
            }
        }

        // Final topic names and messages
        for (const auto& [key, value] : topics_include){
            std::cout << '[' << key << "] = " << value << std::endl;
        }

        // Calculation of used topics



        // sub_pos = this->create_subscription<pr_msgs::msg::PRArrayH>(
        //     "joint_position",
        //     1,//rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_sensor_data),
        //     std::bind(&GusController::controller_callback, this, std::placeholders::_1)
        // );


        // sub_ref = this->create_subscription<pr_msgs::msg::PRArrayH>(
        //     "ref_pose",
        //     1,//rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_sensor_data),
        //     std::bind(&GusController::ref_callback, this, std::placeholders::_1)
        // );

        // publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>("control_action", 1//rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_default)
        // );


    }

    // void GusController::ref_callback(const pr_msgs::msg::PRArrayH::SharedPtr ref_msg)
    // {
    //     PRUtils::ArRMsg2Eigen(ref_msg, ref);
    //     init_ref = true;
    // }   

    void TopicForwarding::topic_callback_arrayh(const pr_msgs::msg::PRArrayH::SharedPtr msg, int index)
    {
             publishers_arrayh.at(index)->publish(*msg);
    }


}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_topic_forwarding::TopicForwarding)