#include "pr_aux/element_wise_mult.hpp"


#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"


using std::placeholders::_1;


namespace pr_aux
{
    /**** ELEMENT_WISE_MULT COMPONENT ****/
    ElementWiseMult::ElementWiseMult(const rclcpp::NodeOptions & options)
    : Node("element_wise_mult", options)
    {
        
        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
			"output_vector", 
			1);

        subscription_vector1 = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "input_vector1",
            1,
            std::bind(&ElementWiseMult::topic_vector1_callback, this, _1));

        subscription_vector2 = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "input_vector2",
            1,
            std::bind(&ElementWiseMult::topic_vector2_callback, this, _1)
        );
    }

    void ElementWiseMult::topic_vector1_callback(const pr_msgs::msg::PRArrayH::SharedPtr input_vector1_msg)
    {
        if (init_vector2){
            //Derivator message and init time
            auto output_vector_msg = pr_msgs::msg::PRArrayH();
            output_vector_msg.init_time = this->get_clock()->now();
            
            for(int i=0; i<4; i++)
                output_vector_msg.data[i] = input_vector1_msg->data[i]*input_vector2(i);

            output_vector_msg.header.stamp = input_vector1_msg->header.stamp;
            output_vector_msg.header.frame_id = input_vector1_msg->header.frame_id;
            
            output_vector_msg.current_time = this->get_clock()->now();
            publisher_->publish(output_vector_msg);

        }
    }

    void ElementWiseMult::topic_vector2_callback(const pr_msgs::msg::PRArrayH::SharedPtr input_vector2_msg)
    {
        PRUtils::ArRMsg2Eigen(input_vector2_msg, input_vector2);
        init_vector2 = true;
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_aux::ElementWiseMult)