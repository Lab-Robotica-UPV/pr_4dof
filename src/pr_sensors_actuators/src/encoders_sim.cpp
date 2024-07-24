#include "pr_sensors_actuators/encoders_sim.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"

#include "pr_lib/pr_utils.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace pr_sensors_actuators
{
    /**** ENCODERS COMPONENT ****/
    
    EncodersSim::EncodersSim(const rclcpp::NodeOptions & options)
    : Node("position_sensors", options)
    {
      //Parameter declaration
      this->declare_parameter<float>("ts_ms", 10.0);
      this->declare_parameter<std::vector<double>>("initial_position", {0.665559, 0.654974, 0.691026, 0.631511});
      
      this->declare_parameter<std::string>("enc_path", 
            "/home/paralelo4dofnew/parallel_robot_ws/references/ref_cart_TRR0_CF1_IdV1.txt");

      this->get_parameter("enc_path", enc_path);


      //Read file
      if(PRUtils::read_file(ref_matrix_q, enc_path)==-1){
        RCLCPP_ERROR(this->get_logger(), "Could not open file");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Encoders references file opened");

      // Defining the number of samples for execution
      n_ref= ref_matrix_q.rows();

      //Position publisher
      publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
			"joint_position", 
			1);

      subscription_end_ = this->create_subscription<std_msgs::msg::Bool>(
            "end_flag",
            1,//rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_sensor_data),
            std::bind(&EncodersSim::end_callback, this, _1));
        
      timer_ = this->create_wall_timer(
        10ms, 
        std::bind(&EncodersSim::on_timer, this));

		RCLCPP_INFO(this->get_logger(), "Configuration completed, brake disabled");
    }

    void EncodersSim::on_timer()
    {
                 
      auto position_msg = pr_msgs::msg::PRArrayH();

      if(!is_finished){
        if (iter<n_ref){
          for(int i=0; i<4; i++){
                position_msg.data[i] = ref_matrix_q(iter, i);
            }
        }
        else{
          for(int i=0; i<4; i++){
                position_msg.data[i] = ref_matrix_q((n_ref-1), i);
            }
        }

        //Time clock
        position_msg.header.frame_id = std::to_string(iter);
            
        position_msg.current_time = this->get_clock()->now();
        position_msg.header.stamp = position_msg.current_time;

        //position_msg.data[0] = 3.0;
        //position_msg.data[1] = 2.0;
        //position_msg.data[2] = 1.0;
        //position_msg.data[3] = 4.0;

        publisher_->publish(position_msg);
        std::cout << "Ecoder sim data:" << ref_matrix_q(iter, 0) << ref_matrix_q(iter, 1) << ref_matrix_q(iter, 2) << ref_matrix_q(iter, 3) << std::endl;
        iter++;
	  	}
      else 
      {
			std::cout << "Ecoder sim end" << std::endl;
			}
	  }

    void EncodersSim::end_callback(const std_msgs::msg::Bool::SharedPtr end_msg)
    {
        //End callback looking for the final event
        if(end_msg->data)
			    is_finished = true;
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_sensors_actuators::EncodersSim)