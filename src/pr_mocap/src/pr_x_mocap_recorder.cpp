#include "pr_mocap/pr_x_mocap_recorder.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace pr_mocap
{
    /**** MODEL MOCAP ERROR COMPONENT ****/
    PRXMocapRecorder::PRXMocapRecorder(const rclcpp::NodeOptions & options)
    : Node("pr_x_mocap_recorder", options)
    {
        //Parameter declaration
        this->declare_parameter<std::string>("filename", "TriggerTest");

        this->get_parameter("filename", filename);

        subscription_start_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "joint_position",
            1,
            std::bind(&PRXMocapRecorder::start_callback, this, _1));

        subscription_stop_ = this->create_subscription<std_msgs::msg::Bool>(
            "end_flag",
            1,
            std::bind(&PRXMocapRecorder::stop_callback, this, _1));

        start_message_str = "<CaptureStart><TimeCode VALUE=\"\"/><Name VALUE=\"" + filename + "\"/><Notes VALUE=\"\"/><Description VALUE=\"\"/><DatabasePath VALUE=\"S:/shared/testfolder/\"/><PacketID VALUE=\"0\"/><ProcessID VALUE=\"%d\"/></CaptureStart>";

        stop_message_str = "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?><CaptureStop><TimeCode VALUE=\"12 13 14 15 0 0 1 1\"/><Name VALUE=\"" + filename + "\"/><Notes VALUE=\"\"/><Description VALUE=\"\"/><DatabasePath VALUE=\"S:/shared/testfolder/\"/><PacketID VALUE=\"0\"/><ProcessID VALUE=\"%d\"/></CaptureStop>";
        
        if ((receiverSocket=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1){
            std::cout << "Error in socket creation" << std::endl;
        }

        int retvalbroadcast = setsockopt(receiverSocket, SOL_SOCKET, SO_BROADCAST, (char *)&value, sizeof(value));
        if (retvalbroadcast == -1){
            std:: cout << "Broadcast Mode failed!" << std::endl;
        }

        // destination address
        memset(&ToAddr, 0, sizeof(ToAddr));
        ToAddr.sin_family = AF_INET;        
        ToAddr.sin_port = htons(port_xml); 
        ToAddr.sin_addr.s_addr = htonl(INADDR_BROADCAST);

        // Message to send to start
        sprintf(start_message_char, start_message_str.c_str());

        // Message to send to stop
        sprintf(stop_message_char, stop_message_str.c_str());

    }

    PRXMocapRecorder::~PRXMocapRecorder(){

        // Si lo paro con Ctrl+C no se activara la end_flag, por lo que lo paro desde aqui
        if (recording_started && !recording_stopped){

            int retval = sendto(receiverSocket, (char *)stop_message_char, 4 + strlen(stop_message_char), 0, (sockaddr *)&ToAddr, sizeof(ToAddr));
            if (retval == -1){
                std:: cout << "Send stop failed!" << std::endl;
            }

        }  

    }

    void PRXMocapRecorder::start_callback(const pr_msgs::msg::PRArrayH::SharedPtr joint_position_msg)
    {
        if (!recording_started){

            int retval = sendto(receiverSocket, (char *)start_message_char, 4 + strlen(start_message_char), 0, (sockaddr *)&ToAddr, sizeof(ToAddr));
            if (retval == -1){
                std:: cout << "Send start failed!" << std::endl;
            }

            recording_started = true;

        }
    }

    void PRXMocapRecorder::stop_callback(const std_msgs::msg::Bool::SharedPtr end_msg)
    {
        if(end_msg->data == true){
            int retval = sendto(receiverSocket, (char *)stop_message_char, 4 + strlen(stop_message_char), 0, (sockaddr *)&ToAddr, sizeof(ToAddr));
            if (retval == -1){
                std:: cout << "Send stop failed!" << std::endl;
            }

            recording_stopped = true;
        }
    }
    
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_mocap::PRXMocapRecorder)