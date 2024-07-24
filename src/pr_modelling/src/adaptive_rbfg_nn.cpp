#include "pr_modelling/adaptive_rbfg_nn.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "pr_msgs/msg/pr_array_h.hpp"
#include "pr_msgs/msg/pr_mat_h.hpp"

#include "pr_lib/pr_model.hpp"
#include "pr_lib/pr_utils.hpp"

using std::placeholders::_1;

namespace pr_modelling
{
    /**** Radial Base Gaussian Fuction for Neural Network****/
    AdaptiveRBFGNN::AdaptiveRBFGNN(const rclcpp::NodeOptions & options)
    : Node("adaptive_rbfg_nn", options)
    {
        //Parameter declaration
        this->declare_parameter<double>("ts", 0.01);
        //this->declare_parameter<int>("N_Hlayers",5);

        this->declare_parameter<double>("c_gain",10.0);
        this->declare_parameter<double>("gama_rbf",100.0);

        this->declare_parameter<std::vector<double>>("c_festo", {-2.0, -1.0, 0.0, 1.0, 2.0});
        this->declare_parameter<std::vector<double>>("c_niasa", {-2.0, -1.0, 0.0, 1.0, 2.0});

        // Read parameter from the yaml file
        this->get_parameter("ts", ts);
        //this->get_parameter("N_Hlayers", N_Hlayers);

        this->get_parameter("c_gain", c_gain);
        this->get_parameter("gama_rbf", gama_rbf);

        this->get_parameter("c_festo", c_festo);
        this->get_parameter("c_niasa", c_niasa);

        // Topic subsciption definition
        sub_q_ref.subscribe(this, "ref_pos");
        sub_q_mea.subscribe(this, "pos");

        /**** Initial definition of matrices****/
        
        // Number of Hidden neural network layers
        if(c_festo.size()!=c_niasa.size()){
             RCLCPP_ERROR(this->get_logger(), "Centres c_festo and c_niasa have different dimensions");
             return;
        }
        N_Hlayers=c_festo.size(); 

        //Print for debuggin
        if (print_debug) std::cout << "Number of Hidden neural network layers: " << N_Hlayers << std::endl;

        //Change variable tipe to eigen
        c_festo_mat.resize(N_Hlayers);
        c_niasa_mat.resize(N_Hlayers);
        for (int i=0; i<N_Hlayers; i++){
            c_festo_mat(i)=c_festo[i];
            c_niasa_mat(i)=c_niasa[i];
        }

        //Matrix c definition
        c.resize(8,N_Hlayers);
        for (int i=0; i<4; i++){
            if (i!=3){
                c.row(i)=c_festo_mat;
                c.row(i+4)=c_festo_mat;
            }
            else{
                c.row(i)=c_niasa_mat;
                c.row(i+4)=c_niasa_mat;
            }

        }
        c = c/c_gain;

        if (print_debug) std::cout << "Matrix c: " << c << std::endl;
        
        //Matrix B definition
        B.block<4,4>(0,0) = Eigen::Matrix<double,4,4>::Zero();
        B.block<4,4>(4,0) = Eigen::Matrix<double,4,4>::Identity();
        if (print_debug) std::cout << "Matrix B: " << B << std::endl;

        //Matrix P definition
        P << 1.0053,         0,         0,         0,         0,         0,         0,         0,
             0,    1.0053,         0,         0,         0,         0,         0,         0,
             0,         0,    1.0053,         0,         0,         0,         0,         0,
             0,         0,         0,    1.2652,         0,         0,         0,         0,
             0,         0,         0,         0,    0.0001,         0,         0,         0,
             0,         0,         0,         0,         0,    0.0001,         0,         0,
             0,         0,         0,         0,         0,         0,    0.0001,         0,
             0,         0,         0,         0,         0,         0,         0,    0.0001;
        P = 10000*P;
        if (print_debug) std::cout << "Matrix P: " << P << std::endl;


        //Initialization for variables used in integration and derivation
        e_ant = Eigen::Vector4d::Zero(4);
        
        //Matriz h resizing 
        h.resize(N_Hlayers,1);

        //Matriz dW resizing 
        aux_dW.resize(N_Hlayers,4);
        dW.resize(4,N_Hlayers);

        auxX.resize(4,N_Hlayers);

        //Matrix X resixing
        y.resize(4*N_Hlayers);
        y_ant.resize(4*N_Hlayers);
        y_ant=Eigen::VectorXd::Ones(4*N_Hlayers);
        y_ant = 0.1*y_ant;
        dy.resize(4*N_Hlayers);
        dy_ant=Eigen::VectorXd::Zero(4*N_Hlayers);
        
        if (print_debug) std::cout << "Vector x_ant: " << dy_ant << std::endl;
        /*************************************/

        sync_.reset(new Synchronizer(SyncPolicy(1), sub_q_ref, sub_q_mea));
        sync_->registerCallback(std::bind(&AdaptiveRBFGNN::topic_callback, this, std::placeholders::_1, std::placeholders::_2));
        //sync_->setMaxIntervalDuration(rclcpp::Duration(0, 1000000));

        publisher_ = this->create_publisher<pr_msgs::msg::PRArrayH>("Tau_adaptw", 1);

    }

    void AdaptiveRBFGNN::topic_callback(const pr_msgs::msg::PRArrayH::ConstPtr& q_ref_msg,
                               const pr_msgs::msg::PRArrayH::ConstPtr& q_mea_msg)
    {

        // Adaptive Tau message and init time
        auto tau_adaptw_msg = pr_msgs::msg::PRArrayH();
        tau_adaptw_msg.init_time = this->get_clock()->now();

        //Convert position mesages in variables for calculation
        PRUtils::ArRMsg2Eigen(q_ref_msg, q_ref);
        PRUtils::ArRMsg2Eigen(q_mea_msg, q_mea);

        //Pose error
        e = q_mea-q_ref;
        if (print_debug) std::cout << "Pose error: " << e.transpose() << std::endl;

        //Pose derivative error
        d_e = PRUtils::derivation(e, e_ant, ts);
        //Upadte previous pose error
        e_ant = e;
        if (print_debug) std::cout << "Derivative pose error: " << d_e.transpose() << std::endl;

        //xi filling
        xi.block<4,1>(0,0) = e;
        xi.block<4,1>(4,0) = d_e;
        // xi.block<4,1>(0,0) = d_e;
        // xi.block<4,1>(4,0) = e;

        if (print_debug) std::cout << "Vector xi: " << xi << std::endl;
        

        // h calculation
        for (int i=0; i<N_Hlayers; i++){
            double auxval = (xi-c.col(i)).norm();
            double num_hi = -pow(auxval,2); 
            double den_hi = 2*pow(b,2); 
            if (print_debug) std::cout << "auxval: " << auxval << "num_hi: " << num_hi << "den_hi: " << den_hi << std::endl;
            h(i,0)=exp(num_hi/den_hi); //-auxH.norm()
        }
        if (print_debug) std::cout << "Matrix h: " << h << std::endl;

        // dW calculation
        aux_dW = gama_rbf*(h*(xi.transpose())*P*B);
        dW=aux_dW.transpose();
        if (print_debug) std::cout << "Matrix dW: " << dW << std::endl;

        //dy calculation
        for (int i=0; i<4; i++){
            int aux = i*N_Hlayers;
            dy.segment(aux,N_Hlayers)=dW.row(i);
        }
        if (print_debug) std::cout << "Vector dy: " << dy << std::endl;

        // Integration of signal (trapezoidal method)
        //y = y_ant + ts/2*(dy+dy_ant);
        // Integration of signal (Forward Euler method)
        y = y_ant + ts*dy_ant;

        //Update the derivative of y signal
        dy_ant=dy;
        y_ant=y;

        // Reshape y_ant in matrix
        for (int i=0; i<4; i++){
            int aux = i*N_Hlayers;
            auxX.row(i)=y_ant.segment(aux,N_Hlayers);
        }

        if (print_debug) std::cout << "Matrix auxX: " << auxX << std::endl;
        
        //Update the previous integration result
        

        //Adative control actions calculation
        adaptive_actions=-auxX*h;
        if (print_debug) std::cout << "Pose ref: " << q_ref.transpose() << std::endl;
        if (print_debug) std::cout << "Pose med: " << q_mea.transpose() << std::endl;
        if (print_debug) std::cout << "Vector xi: " << xi.transpose() << std::endl;
        if (print_debug) std::cout << "Vector y: " << y.transpose() << std::endl;
        if (print_debug) std::cout << "Vector y_ant: " << y_ant.transpose() << std::endl;
        if (!print_debug) std::cout << "Adaptive_actions: " << adaptive_actions.transpose() << std::endl;

        //Publish the control actions 
        PRUtils::Eigen2ArMsg(adaptive_actions, tau_adaptw_msg);

        tau_adaptw_msg.header.stamp = q_ref_msg->header.stamp;
        tau_adaptw_msg.header.frame_id = q_ref_msg->header.frame_id + ", " + q_mea_msg->header.frame_id; 

        tau_adaptw_msg.current_time = this->get_clock()->now();
        publisher_->publish(tau_adaptw_msg);
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_modelling::AdaptiveRBFGNN)