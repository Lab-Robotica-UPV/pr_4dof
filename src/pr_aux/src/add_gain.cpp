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
        sub_in1.subscribe(this, "input1");
        sub_in2.subscribe(this, "input2");
        sub_in3.subscribe(this, "input3");
        sub_in4.subscribe(this, "input4");
        sub_in5.subscribe(this, "input5");

        num_inputs = signs.size();
        PRUtils:vector2EigenVector(gains, gains_eigen);

        switch (num_inputs){
            case 1:
                typedef message_filters::sync_policies::ApproximateTime
                    <pr_msgs::msg::PRArrayH> SyncPolicy;
            
                typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
                std::shared_ptr<Synchronizer> sync_;

                sync_.reset(new Synchronizer(SyncPolicy(1), sub_in1));
                sync_->registerCallback(std::bind(&InvDiffKinematics::topic_callback1, this, std::placeholders::_1));
            break;

            case 2:
                typedef message_filters::sync_policies::ApproximateTime
                    <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH> SyncPolicy;
            
                typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
                std::shared_ptr<Synchronizer> sync_;

                sync_.reset(new Synchronizer(SyncPolicy(1), sub_in1, sub_in2));
                sync_->registerCallback(std::bind(&InvDiffKinematics::topic_callback2, this, std::placeholders::_1, std::placeholders::_2));
            break;

            case 3:
                typedef message_filters::sync_policies::ApproximateTime
                    <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH> SyncPolicy;
            
                typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
                std::shared_ptr<Synchronizer> sync_;

                sync_.reset(new Synchronizer(SyncPolicy(1), sub_in1, sub_in2, sub_in3));
                sync_->registerCallback(std::bind(&InvDiffKinematics::topic_callback3, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
            break;

            case 4:
                typedef message_filters::sync_policies::ApproximateTime
                    <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH> SyncPolicy;
            
                typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
                std::shared_ptr<Synchronizer> sync_;

                sync_.reset(new Synchronizer(SyncPolicy(1), sub_in1, sub_in2, sub_in3, sub_in4));
                sync_->registerCallback(std::bind(&InvDiffKinematics::topic_callback4, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
            break;

            case 5:
                typedef message_filters::sync_policies::ApproximateTime
                    <pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH, pr_msgs::msg::PRArrayH> SyncPolicy;
            
                typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
                std::shared_ptr<Synchronizer> sync_;

                sync_.reset(new Synchronizer(SyncPolicy(1), sub_in1, sub_in2, sub_in3, sub_in4, sub_in5));
                sync_->registerCallback(std::bind(&InvDiffKinematics::topic_callback5, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));
            break;
        }

    }

    void AddGain::topic_callback1(const pr_msgs::msg::PRArrayH::ConstPtr& input1_msg){

        // Output message and init time
        auto out_msg = pr_msgs::msg::PRArrayH();
        out_msg.init_time = this->get_clock()->now();
        
        PRUtils::ArRMsg2Eigen(input1_msg, in1);

        // Output calculation
        out = gains_vector.cwiseProduct(in1*signs[0]);

        // Output msg
        PRUtils::Eigen2ArMsg(out, out_msg);

        out_msg.header.stamp = out_msg->header.stamp;
        out_msg.header.frame_id = out_msg->header.frame_id;
        
        out_msg.current_time = this->get_clock()->now();
        publisher_->publish(out_msg);
    }

    void AddGain::topic_callback2(const pr_msgs::msg::PRArrayH::ConstPtr& input1_msg,
                                 const pr_msgs::msg::PRArrayH::ConstPtr& input2_msg)
        {

        // Output message and init time
        auto out_msg = pr_msgs::msg::PRArrayH();
        out_msg.init_time = this->get_clock()->now();

        PRUtils::ArRMsg2Eigen(input1_msg, in1);
        PRUtils::ArRMsg2Eigen(input2_msg, in2);

        // Output calculation
        out = gains_vector.cwiseProduct(in1*signs[0] + in2*signs[1]);

        // Output msg
        PRUtils::Eigen2ArMsg(out, out_msg);

        out_msg.header.stamp = out_msg->header.stamp;
        out_msg.header.frame_id = out_msg->header.frame_id;
        
        out_msg.current_time = this->get_clock()->now();
        publisher_->publish(out_msg);
    }

    void AddGain::topic_callback3(const pr_msgs::msg::PRArrayH::ConstPtr& input1_msg,
                                 const pr_msgs::msg::PRArrayH::ConstPtr& input2_msg,
                                 const pr_msgs::msg::PRArrayH::ConstPtr& input3_msg)
        {

        // Output message and init time
        auto out_msg = pr_msgs::msg::PRArrayH();
        out_msg.init_time = this->get_clock()->now();

        PRUtils::ArRMsg2Eigen(input1_msg, in1);
        PRUtils::ArRMsg2Eigen(input2_msg, in2);
        PRUtils::ArRMsg2Eigen(input3_msg, in3);

        // Output calculation
        out = gains_vector.cwiseProduct(in1*signs[0] + in2*signs[1] + in3*signs[2]);

        // Output msg
        PRUtils::Eigen2ArMsg(out, out_msg);

        out_msg.header.stamp = out_msg->header.stamp;
        out_msg.header.frame_id = out_msg->header.frame_id;
        
        out_msg.current_time = this->get_clock()->now();
        publisher_->publish(out_msg);
    }

    void AddGain::topic_callback4(const pr_msgs::msg::PRArrayH::ConstPtr& input1_msg,
                                 const pr_msgs::msg::PRArrayH::ConstPtr& input2_msg,
                                 const pr_msgs::msg::PRArrayH::ConstPtr& input3_msg,
                                 const pr_msgs::msg::PRArrayH::ConstPtr& input4_msg)
        {

        // Output message and init time
        auto out_msg = pr_msgs::msg::PRArrayH();
        out_msg.init_time = this->get_clock()->now();

        PRUtils::ArRMsg2Eigen(input1_msg, in1);
        PRUtils::ArRMsg2Eigen(input2_msg, in2);
        PRUtils::ArRMsg2Eigen(input3_msg, in3);
        PRUtils::ArRMsg2Eigen(input4_msg, in4);

        // Output calculation
        out = gains_vector.cwiseProduct(in1*signs[0] + in2*signs[1] + in3*signs[2] + in4*signs[3]);

        // Output msg
        PRUtils::Eigen2ArMsg(out, out_msg);

        out_msg.header.stamp = out_msg->header.stamp;
        out_msg.header.frame_id = out_msg->header.frame_id;
        
        out_msg.current_time = this->get_clock()->now();
        publisher_->publish(out_msg);
    }

    void AddGain::topic_callback5(const pr_msgs::msg::PRArrayH::ConstPtr& input1_msg,
                                 const pr_msgs::msg::PRArrayH::ConstPtr& input2_msg,
                                 const pr_msgs::msg::PRArrayH::ConstPtr& input3_msg,
                                 const pr_msgs::msg::PRArrayH::ConstPtr& input4_msg,
                                 const pr_msgs::msg::PRArrayH::ConstPtr& input5_msg)
        {

        // Output message and init time
        auto out_msg = pr_msgs::msg::PRArrayH();
        out_msg.init_time = this->get_clock()->now();

        PRUtils::ArRMsg2Eigen(input1_msg, in1);
        PRUtils::ArRMsg2Eigen(input2_msg, in2);
        PRUtils::ArRMsg2Eigen(input3_msg, in3);
        PRUtils::ArRMsg2Eigen(input4_msg, in4);
        PRUtils::ArRMsg2Eigen(input5_msg, in5);

        // Output calculation
        out = gains_vector.cwiseProduct(in1*signs[0] + in2*signs[1] + in3*signs[2] + in4*signs[3] + in5*sings[4]);

        // Output msg
        PRUtils::Eigen2ArMsg(out, out_msg);

        out_msg.header.stamp = out_msg->header.stamp;
        out_msg.header.frame_id = out_msg->header.frame_id;
        
        out_msg.current_time = this->get_clock()->now();
        publisher_->publish(out_msg);
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_aux::AddGain)