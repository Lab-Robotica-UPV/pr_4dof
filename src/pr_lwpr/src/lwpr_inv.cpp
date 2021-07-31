#include "pr_lwpr/lwpr_inv.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "pr_lib/pr_utils.hpp"

namespace pr_lwpr
{
    /**** LWPR INV COMPONENT ****/
    LWPRInv::LWPRInv(const rclcpp::NodeOptions & options)
    : Node("lwpr_fwd", options)
    {
        //Parameter declaration
        this->declare_parameter<std::vector<double>>("initD", {20.0, 20.0, 20.0, 20.0});
        this->declare_parameter<std::vector<double>>("initAlpha", {0.1, 0.1, 0.1, 0.1});
        this->declare_parameter<std::vector<double>>("penalty", {0.001, 0.001, 0.001, 0.001});
        this->declare_parameter<std::vector<double>>("initLambda", {0.999, 0.999, 0.999, 0.999});
        this->declare_parameter<std::vector<double>>("finalLambda", {0.9999, 0.9999, 0.9999, 0.9999});
        this->declare_parameter<bool>("activateLearning", true);
        this->declare_parameter<bool>("activatePrediction", false);
        this->declare_parameter<std::string>("loadModel", "");
        this->declare_parameter<std::string>("saveModel", "");
        this->declare_parameter<double>("ts", 0.01);
        

        this->get_parameter("initD", initD);
        this->get_parameter("initAlpha", initAlpha);
        this->get_parameter("penalty", penalty);
        this->get_parameter("initLambda", initLambda);
        this->get_parameter("finalLambda", finalLambda);
        this->get_parameter("activateLearning", activateLearning);
        this->get_parameter("activatePrediction", activatePrediction);
        this->get_parameter("loadModel", loadModel);
        this->get_parameter("saveModel", saveModel);
        this->get_parameter("ts", ts);

        subscription_ref = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "ref_pose",
            1,
            std::bind(&LWPRInv::ref_callback, this, std::placeholders::_1)
        );

        subscription_u = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "control_action",
            1,
            std::bind(&LWPRInv::u_callback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>("out_lwpr_inv", 1);

        state.resize(12);
        y.resize(1);
        output.resize(4);
        qp_ant = Eigen::Vector4d::Zero();
        scale_output << 60, 60, 60, 400;

        // Open previous model or create new model
        if (loadModel != ""){
          for (int i=0; i<4; i++)
          {
            std::string name_model = loadModel+std::to_string(i+1)+".bin";
            models.push_back(new LWPR_Object(name_model.c_str()));
          }
        }
        
        else{
          for (int i=0; i<4; i++)
          {
            models.push_back(new LWPR_Object(12,1));
            models[i]->setInitD(static_cast<int>(initD[i]));
            models[i]->setInitAlpha(initAlpha[i]);
            models[i]->updateD(true);
            models[i]->penalty(penalty[i]);
            models[i]->initLambda(initLambda[i]);
            models[i]->finalLambda(finalLambda[i]);
            
          }
        }

    }

    LWPRInv::~LWPRInv(){
      if (saveModel!= ""){
        for (int i=0; i<4; i++)
          {
            std::string name_model = saveModel+std::to_string(i+1)+".bin";
            models[i]->writeBinary(name_model.c_str());
            delete models[i];
          }
      }
      models.clear();
    }

    void LWPRInv::ref_callback(const pr_msgs::msg::PRArrayH::SharedPtr q_msg)
    {

        //Convert to Eigen
        PRUtils::ArRMsg2Eigen(q_msg, q);

        qp = PRUtils::derivation(q, q_ant, ts);
        qpp = PRUtils::derivation(qp, qp_ant, ts);

        if (!first_iter){

          if (activatePrediction){
            // Output message and init time
            auto output_msg = pr_msgs::msg::PRArrayH();
            output_msg.init_time = this->get_clock()->now();

            // Jorge divides the control actions by 60 or 400 to normalize
            // Is there a better strategy?
            state << q, qp/0.02, qpp;
            for (int i=0; i<q_msg->data.size(); i++){
              y = models[i]->predict(state);
              output(i) = y(0);
              output(i) *= scale_output(i);
            }

            for(int i=0; i<output.size(); i++)
            output_msg.data[i] = output(i);

            output_msg.header.frame_id = q_msg->header.frame_id;
            output_msg.header.stamp = q_msg->header.stamp;
            output_msg.current_time = this->get_clock()->now();
            publisher_->publish(output_msg);
          }

          if (activateLearning){
            for (int i=0; i<q_msg->data.size(); i++){
              state << q, qp/0.02, qpp;
              y(0) = u(i)/scale_output(i);
              models[i]->update(state, y);
            }
          }

        }
        
        if (first_iter && activatePrediction || !activatePrediction){
          // Output message and init time
            auto output_msg = pr_msgs::msg::PRArrayH();
            output_msg.init_time = this->get_clock()->now();

            for(int i=0; i<4; i++)
            output_msg.data[i] = 0;

            output_msg.header.frame_id = q_msg->header.frame_id;
            output_msg.header.stamp = q_msg->header.stamp;
            output_msg.current_time = this->get_clock()->now();
            publisher_->publish(output_msg);
            
          
          first_iter = false;
        }

        q_ant = q;
        qp_ant = qp;
        
    }
    
    void LWPRInv::u_callback(const pr_msgs::msg::PRArrayH::SharedPtr u_msg)
    {

      //Convert to Eigen
      PRUtils::ArRMsg2Eigen(u_msg, u);

    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_lwpr::LWPRInv)