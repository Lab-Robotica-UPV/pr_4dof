#include "pr_aux/displayer.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_lib/pr_utils.hpp"

using std::placeholders::_1;


namespace pr_aux
{
    /**** DISPLAYER COMPONENT ****/
    Displayer::Displayer(const rclcpp::NodeOptions & options)
    : Node("displayer", options)
    {
        //Parameter declaration
        this->declare_parameter<std::string>("data_path", 
            "/home/paralelo4dofnew/ros2_eloquent_ws/pr_4dof/pauses/ind_pause.txt");

        this->get_parameter("data_path", data_path);

        //Read file
        if(PRUtils::read_file(data_matrix, data_path)==-1){
            RCLCPP_ERROR(this->get_logger(), "Could not open file");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Pose references file opened");

        subscription_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "callback",
            1,
            std::bind(&Displayer::topic_callback, this, _1));
    }

    void Displayer::topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr callback_msg)
    {
        std::cout << "Display: " << data_matrix.row(idx) << std::endl;
        idx ++;
    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_aux::Displayer)