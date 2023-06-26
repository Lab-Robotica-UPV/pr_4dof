#include "pr_sensors_actuators/motor.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"
#include "std_msgs/msg/bool.hpp"

#include <cmath>

using namespace Automation::BDaq;
using std::placeholders::_1;


namespace pr_sensors_actuators
{
    /**** ENCODER COMPONENT ****/
    Motor::Motor(const rclcpp::NodeOptions & options)
    : Node("motor_3", options)
    {
        //Parameter declaration
        this->declare_parameter<double>("max_v", 3.0);
        this->declare_parameter<double>("vp_conversion", 1.0);
        this->get_parameter("vp_conversion", vp_conversion);
        this->get_parameter("max_v", max_v);
        
        //Get motor number from node name
        const char *pn;
        pn = this->get_name();
        pn = pn + strlen(this->get_name())-1;
        n_motor = atoi(pn);

        if(n_motor>4 || n_motor<0){
            RCLCPP_ERROR(this->get_logger(), "Wrong motor number");
            //Find a better way to exit
            exit(1);
        }

        RCLCPP_INFO(this->get_logger(), "This is actuator number %d", n_motor);

        //Create communication
        publisher_end_ = this->create_publisher<std_msgs::msg::Bool>(
            "end_flag",
            1//rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_default)
        );

        publisher_mean_ = this->create_publisher<pr_msgs::msg::PRFloatH>(
            "mean_u_"+std::to_string(n_motor+1),
            1//rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_default)
        );

         publisher_var_ = this->create_publisher<pr_msgs::msg::PRFloatH>(
            "var_u_"+std::to_string(n_motor+1),
            1//rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST,5), rmw_qos_profile_default)
        );

        subscription_=this->create_subscription<pr_msgs::msg::PRArrayH>(
            "control_action",
                        1,
            std::bind(&Motor::topic_callback,this,_1));

        subscription_end_= this->create_subscription<std_msgs::msg::Bool>(
            "end_flag", 
                     1, 
            std::bind(&Motor::end_callback, this, _1));

        //Initialize actuator
        init_ao_pci();
        is_finished = false;
    
    }

    Motor::~Motor()
    {
        //Writing 0
        volts=0.0;
        pci1720->Write(n_motor, 1 ,&volts);
        pci1720->Dispose();
        RCLCPP_INFO(this->get_logger(), "Shutting down actuator number %d", n_motor);			
	}

    void Motor::topic_callback(const pr_msgs::msg::PRArrayH::SharedPtr control_action_msg){
        if(is_finished == false){
               
            volts = control_action_msg->data[n_motor]/vp_conversion;

            //RCLCPP_INFO(this->get_logger(), "I heard these and sat: '%f' %f", volts, max_v);

            //Control action saturation
            sat_ca(volts, max_v);
            if (abs(volts) > 3.33) std::cout << "Actuador: " << n_motor+1 << " " << volts << std::endl;
            //std::cout << n_motor << " " << volts << " ";

            // The following code allows to track the mean and variance of the control action
            // Incremental calculation of weighted mean and variance (Tony Finch)
            if (init_sample){
                running_mean = volts;
                init_sample = false;
            }
            running_var = alpha*(running_var+(1-alpha)*pow(volts-running_mean,2));
            running_mean = running_mean + (1-alpha)*(volts - running_mean);
            auto running_mean_msg = pr_msgs::msg::PRFloatH();
            auto running_var_msg = pr_msgs::msg::PRFloatH();

            running_mean_msg.init_time = this->get_clock()->now();
            running_var_msg.init_time = this->get_clock()->now();

            running_mean_msg.data = running_mean;
            running_var_msg.data = running_var;

            running_mean_msg.header.stamp = control_action_msg->header.stamp;
            running_mean_msg.header.frame_id = control_action_msg->header.frame_id;
            
            running_var_msg.header.stamp = control_action_msg->header.stamp;
            running_var_msg.header.frame_id = control_action_msg->header.frame_id;

            running_mean_msg.current_time = this->get_clock()->now();
            publisher_mean_->publish(running_mean_msg);
            running_var_msg.current_time = this->get_clock()->now();
            publisher_var_->publish(running_var_msg);


            // Time control
            // This message is auxiliar to get the time
            auto end_time_msg = pr_msgs::msg::PRArrayH();
            end_time_msg.current_time = this->get_clock()->now();
            auto end_time = end_time_msg.current_time;
            auto init_time = control_action_msg->header.stamp;
            auto duration_ms = (end_time.sec + end_time.nanosec*1e-9 - init_time.sec - init_time.nanosec*1e-9 )*1000;
            //std::cout << "Duration (ms): " << (a.sec+a.nanosec*1e-9 -b.sec-b.nanosec*1e-9)*1000.0 << std::endl;
            if (n_motor==1){
                std::cout << "Duration: " << duration_ms << std::endl;  
            }
            // std::cout << "Init time sec: " << init_time.sec << " Final time sec: " << end_time.sec << std::endl;


            //RCLCPP_INFO(this->get_logger(), "Control action %f", volts);
            // Check nan
            if (std::isnan(volts) || running_var>threshold_var)
            {
                auto end_msg = std_msgs::msg::Bool();
                end_msg.data = true;
                publisher_end_->publish(end_msg);
                if (std::isnan(volts))
                    std::cout << "Control action " << n_motor+1 << " with NaN!" << std::endl;
                if (running_var>threshold_var)
                    std::cout << "Control action " << n_motor+1 << "oscillatory!" << std::endl;
            } 
            else pci1720->Write(n_motor, 1, &volts);
            // pci1720->Write(n_motor, 1, &volts);
        }
    }

    void Motor::end_callback(const std_msgs::msg::Bool::SharedPtr end_msg){
          if(end_msg->data == true){
               is_finished = true;
               volts = 0;
               pci1720->Write(n_motor, 1, &volts);
               RCLCPP_INFO(this->get_logger(), "Finished, written: %f", volts);
          }
    }

    void Motor::init_ao_pci(void){
          ret = Success;

          pci1720=AdxInstantAoCtrlCreate();
          DeviceInformation devInfo(deviceDescription);
          ret = pci1720->setSelectedDevice(devInfo);
          
          RCLCPP_INFO(this->get_logger(), "Actuator %d initialised: %d", n_motor, ret);
          volts = 0.0;
          ret = pci1720->Write(n_motor, 1, &volts);
    }

    void Motor::sat_ca(double &control_action, const double &sat){
	     if(control_action > sat) control_action = sat;
	     if(control_action < -sat) control_action = -sat;
    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_sensors_actuators::Motor)