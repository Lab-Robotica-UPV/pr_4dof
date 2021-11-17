#include "pr_biomech/streaming_gdlf.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace pr_biomech
{
    /**** TEST COMPONENT ****/
    StreamingGDLF::StreamingGDLF(const rclcpp::NodeOptions & options)
    : Node("test", options)
    {
        
        try{
            // mocap_object = std::make_unique<PRMocap::Mocap>(1, 1, false);
            cal_data = std::make_unique<PRJsonData::PRJsonCal::Calibration_data_struct>("/home/paralelo4dofnew/ros2_eloquent_ws/pr_4dof/patient_data/calibration.txt");
            cal_data->print_data();
            gdlf_data = std::make_unique<PRJsonData::PRJsonGdlf::Gdlf_data_struct>("/home/paralelo4dofnew/ros2_eloquent_ws/pr_4dof/patient_data/CoefOffline_Data.txt");
            gdlf_data->print_data();
        }
        catch(const std::exception &e){
            std::cerr << e.what() << std::endl;
            throw;
        }

        subscription_ = this->create_subscription<pr_msgs::msg::PRForceState>(
            "force_state_sync",
            1,
            std::bind(&StreamingGDLF::force_callback, this, _1)
        );

    }

    void StreamingGDLF::force_callback(const pr_msgs::msg::PRForceState::SharedPtr force_state_msg){

        //mocap_object->print_data();

    }    
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_biomech::StreamingGDLF)