#include "pr_aux/add_gain.hpp"

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
    /**** ADD_GAIN COMPONENT ****/
    AddGain::AddGain(const rclcpp::NodeOptions & options)
    : Node("add_gain", options)
    {
        //Parameter declaration
        // Signs 1.0 for sum and -1.0 for subtraction. The number of inputs is the number of entries in this vector
        this->declare_parameter<std::vector<double>>("signs", {1.0, 1.0});
        // Vector of 4 components, gain for the resulting operation
        this->declare_parameter<std::vector<double>>("gains", {1.0, 1.0, 1.0, 1.0});

        this->get_parameter("signs", signs);
        this->get_parameter("gains", gains);
        
        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
			"output", 
			1);
        
        num_inputs = signs.size();
        PRUtils::vector2EigenVector(gains, gains_eigen);

       if (num_inputs>=1)
        {            
            sub_1 = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "input1", 
            10, 
            std::bind(&AddGain::topic_callback1, this, _1));

            if (num_inputs>=2)
            {
                sub_2 = this->create_subscription<pr_msgs::msg::PRArrayH>(
                "input2", 
                10, 
                std::bind(&AddGain::topic_callback2, this, _1));

                if (num_inputs>=3)
                {
                    sub_3 = this->create_subscription<pr_msgs::msg::PRArrayH>(
                    "input3", 
                    10, 
                    std::bind(&AddGain::topic_callback3, this, _1));

                    if (num_inputs>=4)
                    {
                        sub_4 = this->create_subscription<pr_msgs::msg::PRArrayH>(
                        "input4", 
                        10, 
                        std::bind(&AddGain::topic_callback4, this, _1));

                        if (num_inputs>=5)
                        {
                            sub_5 = this->create_subscription<pr_msgs::msg::PRArrayH>(
                            "input5", 
                            10, 
                            std::bind(&AddGain::topic_callback5, this, _1));

                        }
                    }
                }
            }
        }
    }

    void AddGain::topic_callback1(const pr_msgs::msg::PRArrayH::SharedPtr input1_msg){

        if (num_inputs==1 || 
            (num_inputs==2 && init_2) ||
            (num_inputs==3 && init_2 && init_3) ||
            (num_inputs==4 && init_2 && init_3 && init_4) ||
            (num_inputs==5 && init_2 && init_3 && init_4 && init_5)
            )
        {

            // Output message and init time
            auto out_msg = pr_msgs::msg::PRArrayH();
            out_msg.init_time = this->get_clock()->now();
            
            PRUtils::ArRMsg2Eigen(input1_msg, in1);

            // Output calculation
            if (num_inputs==1)
            out = gains_eigen.cwiseProduct(in1*signs[0]);

            else if (num_inputs==2)
            out = gains_eigen.cwiseProduct(in1*signs[0] + in2*signs[1]);

            else if (num_inputs==3)
            out = gains_eigen.cwiseProduct(in1*signs[0] + in2*signs[1] + in3*signs[2]);

            else if (num_inputs==4)
            out = gains_eigen.cwiseProduct(in1*signs[0] + in2*signs[1] + in3*signs[2] + in4*signs[3]);

            else if (num_inputs==5)
            out = gains_eigen.cwiseProduct(in1*signs[0] + in2*signs[1] + in3*signs[2] + in4*signs[3] + in5*signs[4]);


            // Output msg
            PRUtils::Eigen2ArMsg(out, out_msg);

            out_msg.header.stamp = input1_msg->header.stamp;
            out_msg.header.frame_id = input1_msg->header.frame_id;
            
            out_msg.current_time = this->get_clock()->now();
            publisher_->publish(out_msg);
        }
    }

    void AddGain::topic_callback2(const pr_msgs::msg::PRArrayH::SharedPtr input2_msg){
        init_2 = true;
        PRUtils::ArRMsg2Eigen(input2_msg, in2);
    }

    void AddGain::topic_callback3(const pr_msgs::msg::PRArrayH::SharedPtr input3_msg){
        init_3 = true;
        PRUtils::ArRMsg2Eigen(input3_msg, in3);
    }

    void AddGain::topic_callback4(const pr_msgs::msg::PRArrayH::SharedPtr input4_msg){
        init_4 = true;
        PRUtils::ArRMsg2Eigen(input4_msg, in4);
    }

    void AddGain::topic_callback5(const pr_msgs::msg::PRArrayH::SharedPtr input5_msg){
        init_5 = true;
        PRUtils::ArRMsg2Eigen(input5_msg, in5);
    }


    
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_aux::AddGain)