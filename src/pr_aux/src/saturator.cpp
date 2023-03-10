#include "pr_aux/saturator.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"

using std::placeholders::_1;


namespace pr_aux
{
    /**** SATURATOR COMPONENT ****/
    Saturator::Saturator(const rclcpp::NodeOptions & options)
    : Node("saturator", options)
    {
        //Parameter declaration
        this->declare_parameter<std::vector<double>>("min_val", {0.6537, 0.6474, 0.6502, 0.549});
        this->declare_parameter<std::vector<double>>("max_val", {0.9337, 0.9274, 0.9302, 0.8290});

        this->get_parameter("min_val", min_val);
        this->get_parameter("max_val", max_val);
        
        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
			"signal_saturated", 
			1);

        subscription_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "signal_init",
            1,
            std::bind(&Saturator::topic_callback, this, _1));

        publisher_pin_sat_ = this->create_publisher<pr_msgs::msg::PRBoolH>(
			"saturation_pin", 
		1);
    }

    void Saturator::topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr var_msg)
    {
        //Saturator message and init time
        auto var_sat_msg = pr_msgs::msg::PRArrayH();
        var_sat_msg.init_time = this->get_clock()->now();
        auto pin_sat_msg = pr_msgs::msg::PRBoolH();
        pin_sat_msg.init_time = this->get_clock()->now();
        pin_sat_msg.data = false;
            

        for(int i=0; i<4; i++)
        {
            if (var_msg->data[i] > max_val[i]){
                var_sat_msg.data[i] = max_val[i];
                pin_sat_msg.data = true;
                std::cout << "Pata " << i+1 << " saturada en el máximo!" << std::endl;
            }
            else if (var_msg->data[i] < min_val[i]){
                var_sat_msg.data[i] = min_val[i];
                pin_sat_msg.data = true;
                std::cout << "Pata " << i+1 << " saturada en el mínimo!" << std::endl;
            }
            else
                var_sat_msg.data[i] = var_msg->data[i];
        }

        var_sat_msg.header.stamp = var_msg->header.stamp;
        var_sat_msg.header.frame_id = var_msg->header.frame_id;

        pin_sat_msg.header.stamp = var_msg->header.stamp;
        pin_sat_msg.header.frame_id = var_msg->header.frame_id;
        
        var_sat_msg.current_time = this->get_clock()->now();
        publisher_->publish(var_sat_msg);

        pin_sat_msg.current_time = this->get_clock()->now();
        publisher_pin_sat_->publish(pin_sat_msg);
    }

    
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_aux::Saturator)