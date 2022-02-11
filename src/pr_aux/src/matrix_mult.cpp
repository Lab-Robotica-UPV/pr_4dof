#include "pr_aux/matrix_mult.hpp"


#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"


using std::placeholders::_1;


namespace pr_aux
{
    /**** MATRIX_MULT COMPONENT ****/
    MatrixMult::MatrixMult(const rclcpp::NodeOptions & options)
    : Node("matrix_mult", options)
    {
        
        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
			"output_vector", 
			1);

        subscription_vector = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "input_vector",
            1,
            std::bind(&MatrixMult::topic_vector_callback, this, _1));

        subscription_matrix = this->create_subscription<pr_msgs::msg::PRMatH>(
            "input_matrix",
            1,
            std::bind(&MatrixMult::topic_matrix_callback, this, _1)
        );
    }

    void MatrixMult::topic_vector_callback(const pr_msgs::msg::PRArrayH::SharedPtr input_vector_msg)
    {
        if (init_matrix){
            //Derivator message and init time
            auto output_vector_msg = pr_msgs::msg::PRArrayH();
            output_vector_msg.init_time = this->get_clock()->now();

            for(int i=0; i<4; i++)
                input_vector(i) = input_vector_msg->data[i];

            output_vector = input_matrix*input_vector;
            
            for(int i=0; i<4; i++)
                output_vector_msg.data[i] = output_vector(i);

            output_vector_msg.header.stamp = input_vector_msg->header.stamp;
            output_vector_msg.header.frame_id = input_vector_msg->header.frame_id;
            
            output_vector_msg.current_time = this->get_clock()->now();
            publisher_->publish(output_vector_msg);

        }
    }

    void MatrixMult::topic_matrix_callback(const pr_msgs::msg::PRMatH::SharedPtr input_matrix_msg)
    {
        for(int i=0; i<(int)input_matrix_msg->data.size(); i++) {
            int row = i/input_matrix.cols();
            int col = i%input_matrix.cols();
            input_matrix(row,col) = input_matrix_msg->data[i];
        }
        init_matrix = true;
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_aux::MatrixMult)