#include "pr_sensors_actuators/encoders.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using namespace Automation::BDaq;

using std::placeholders::_1;

namespace pr_sensors_actuators
{
    /**** ENCODERS COMPONENT ****/
    
    Encoders::Encoders(const rclcpp::NodeOptions & options)
    : Node("position_sensors", options)
    {
        //Parameter declaration
        this->declare_parameter<float>("ts_ms", 10.0);
        this->declare_parameter<std::vector<double>>("initial_position", {0.665559, 0.654974, 0.691026, 0.631511});
		this->declare_parameter<std::vector<double>>("gearbox_mult", {1.0, 1.0, 1.0, 1.0});

        //Read parameters
        this->get_parameter("ts_ms", ts);
        this->get_parameter("initial_position", initial_position);
		this->get_parameter("gearbox_mult", gearbox_mult);

        //Position publisher
        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
			"joint_position", 
			1);
        //End signal subscription
        subscription_end_ = this->create_subscription<std_msgs::msg::Bool>(
            "end_flag",
            1,
            std::bind(&Encoders::end_callback, this, _1));

        //Encoder channels initialization
		DeviceInformation devInfo(deviceDescription);
			
		udCounterCtrl0=AdxUdCounterCtrlCreate();
		udCounterCtrl1=AdxUdCounterCtrlCreate();
		udCounterCtrl2=AdxUdCounterCtrlCreate();
		udCounterCtrl3=AdxUdCounterCtrlCreate();
		instantDoCtrl = AdxInstantDoCtrlCreate();

		ret=udCounterCtrl0->setSelectedDevice(devInfo);
		ret=udCounterCtrl1->setSelectedDevice(devInfo);
		ret=udCounterCtrl2->setSelectedDevice(devInfo);
		ret=udCounterCtrl3->setSelectedDevice(devInfo);
		ret=instantDoCtrl->setSelectedDevice(devInfo);


		ret = udCounterCtrl0->setChannel(0);
		RCLCPP_INFO(this->get_logger(), "Channel 0 configuration completed: %d", ret);
		ret = udCounterCtrl1->setChannel(1);
		RCLCPP_INFO(this->get_logger(), "Channel 1 configuration completed: %d", ret);
		ret = udCounterCtrl2->setChannel(2);
		RCLCPP_INFO(this->get_logger(), "Channel 2 configuration completed: %d", ret);
		ret = udCounterCtrl3->setChannel(3);
		RCLCPP_INFO(this->get_logger(), "Channel 3 configuration completed: %d", ret);

		ret=udCounterCtrl0->setCountingType(AbPhaseX1);
		ret=udCounterCtrl1->setCountingType(AbPhaseX1);
		ret=udCounterCtrl2->setCountingType(AbPhaseX1);
		ret=udCounterCtrl3->setCountingType(AbPhaseX1);
		//ret=udCounterCtrl3->setCountingType(PulseDirection);

		ret=udCounterCtrl0->setEnabled(true);
		ret=udCounterCtrl1->setEnabled(true);
		ret=udCounterCtrl2->setEnabled(true);
		ret=udCounterCtrl3->setEnabled(true);

        RCLCPP_INFO(this->get_logger(), "Brake disabled");

			ret = instantDoCtrl->Read(0,1,DOut);

			for (int i=0; i<4; i++){
				DOut[0] = DOut[0]|(0x01<<i);
			}

			//std::cout << DOut[0] << std::endl;


		// Pause before start
		//std::this_thread::sleep_for(std::chrono::milliseconds(4000));

		// Brake deactivation
		//ret = instantDoCtrl->Write(0,1,DOut);
		//RCLCPP_INFO(this->get_logger(), "Configuration completed, brake disabled");

		//Create timer
        timer_ = this->create_wall_timer(
            std::chrono::duration<float, std::milli>(ts), 
            std::bind(&Encoders::on_timer, this));
    }

    Encoders::~Encoders()
    {
			
		DOut[0]=0;
	    ret = instantDoCtrl->Write(0,1,DOut);

		udCounterCtrl0->Dispose();
		udCounterCtrl1->Dispose();
		udCounterCtrl2->Dispose();
		udCounterCtrl3->Dispose();

	    instantDoCtrl->Dispose();
		RCLCPP_INFO(this->get_logger(), "Encoders node finished");
	}

    void Encoders::on_timer()
    {
		if(!is_finished)
        {
			// Position message and init time
			auto position_msg = pr_msgs::msg::PRArrayH();
			position_msg.init_time = this->get_clock()->now();

			//First joint
			pulsos[0] = udCounterCtrl0->getValue();
			position_msg.data[0] = pulsos[0]*0.00002*gearbox_mult[0] + initial_position[0]; //pulsos[0]*0.00002*3.0/13.0 + initial_position[0];
			// La constante 0.00002 sale de aplicar 1/(pulsosporvueltaencoder/pasoactuador*1000)

			//Second joint
			pulsos[1] = udCounterCtrl1->getValue();
			position_msg.data[1] = pulsos[1]*0.00002*gearbox_mult[1] + initial_position[1];

			//Third joint
			pulsos[2] = udCounterCtrl2->getValue();
			position_msg.data[2] = pulsos[2]*0.00002*gearbox_mult[2] + initial_position[2];

			//Fourth joint
			pulsos[3] = udCounterCtrl3->getValue();
			//position_msg.data[3] = pulsos[3]*0.000002325*gearbox_mult[3] + initial_position[3];
			//position_msg.data[3] = pulsos[3]*0.00000230571*gearbox_mult[3] + initial_position[3];
			position_msg.data[3] = pulsos[3]*0.00001*gearbox_mult[3] + initial_position[3];

			std::cout << pulsos[0] << " " << pulsos[1] << " " << pulsos[2] << " " << pulsos[3] << std::endl;
			//std::cout << position_msg.data[0] << " " << position_msg.data[1] << " " << position_msg.data[2] << " " << position_msg.data[3] << std::endl; 

			if (iter*ts/1000 == init_delay_sec){
				// Brake deactivation
				ret = instantDoCtrl->Write(0,1,DOut);
				RCLCPP_INFO(this->get_logger(), "Configuration completed, brake disabled");
			}
			if (iter*ts/1000 >= init_delay_sec){

				//Time clock
				position_msg.header.frame_id = std::to_string(iter);
				
				position_msg.current_time = this->get_clock()->now();
				position_msg.header.stamp = position_msg.current_time;
				publisher_->publish(position_msg);
            
				/*RCLCPP_INFO(this->get_logger(), "Pulishing: %f %f %f %f", 
				position_msg.data[0], 
				position_msg.data[1], 
				position_msg.data[2], 
				position_msg.data[3]);*/
			}
            iter++;
		}

        else 
        {
            //End flag activted
			DOut[0]=0;
	    	ret = instantDoCtrl->Write(0,1,DOut);
			//RCLCPP_INFO(this->get_logger(), "Brake disabled");
		}

		
	}

    void Encoders::end_callback(const std_msgs::msg::Bool::SharedPtr end_msg)
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
RCLCPP_COMPONENTS_REGISTER_NODE(pr_sensors_actuators::Encoders)