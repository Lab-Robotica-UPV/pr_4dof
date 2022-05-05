#include "pr_dmp/dmp_evader_force_cart.hpp"

using std::placeholders::_1;

namespace pr_dmp
{
    /**** EvaderForceCart COMPONENT ****/
    EvaderForceCart::EvaderForceCart(const rclcpp::NodeOptions & options)
    : Node("dmp_evader_force_pid", options)
    {
        //Parameter declaration
        this->declare_parameter<std::vector<double>>(
            "gain", 
            {1.0,1.0,1.0,1.0});

        this->get_parameter("gain", gain);
        
        jacobian_dir_sub = this->create_subscription<pr_msgs::msg::PRMatH>(
            "for_jac",
            1,
            std::bind(&EvaderForceCart::topic_callback_jac, this, std::placeholders::_1)
        );

        force_evader_prism_sub = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "evader_force_prism",
            1,
            std::bind(&EvaderForceCart::topic_callback_force_evader_prism, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>("evader_force_cart", 1);

    }

    void EvaderForceCart::topic_callback_force_evader_prism(const pr_msgs::msg::PRArrayH::SharedPtr force_evader_prism_msg)
    {
        if (init_jac_dir){

            // Ref mod message and init time
            auto force_evader_cart_msg = pr_msgs::msg::PRArrayH();
            force_evader_cart_msg.init_time = this->get_clock()->now();

            PRUtils::ArRMsg2Eigen(force_evader_prism_msg, force_evader_prism);

            force_evader_cart = JDir.inverse()*force_evader_prism;

            for(int i=0;i<4;i++)
                force_evader_cart_msg.data[i] = force_evader_cart(i)*gain[i];


            force_evader_cart_msg.header.stamp = force_evader_prism_msg->header.stamp;
            force_evader_cart_msg.header.frame_id = force_evader_prism_msg->header.frame_id;


            force_evader_cart_msg.current_time = this->get_clock()->now();
            publisher_->publish(force_evader_cart_msg);

        }
    }

    void EvaderForceCart::topic_callback_jac(const pr_msgs::msg::PRMatH::SharedPtr jacobian_dir_msg)
    {
        init_jac_dir = true;
        for(int i=0; i<(int)jacobian_dir_msg->data.size(); i++) {
            int row = i/JDir.cols();
            int col = i%JDir.cols();
            JDir(row,col) = jacobian_dir_msg->data[i];
        }
    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_dmp::EvaderForceCart)
