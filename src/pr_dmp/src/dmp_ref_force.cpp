#include "pr_dmp/dmp_ref_force.hpp"

#include <chrono>
#include <memory>

using std::placeholders::_1;


namespace pr_dmp
{
    /**** DmpRefForce COMPONENT ****/
    DmpRefForce::DmpRefForce(const rclcpp::NodeOptions & options)
    : Node("dmp_ref_force", options)
    {
        // Parameter declaration
        // Trajectory file of DMP (cart or prism, indicated by the bool "isCart")
        this->declare_parameter<std::string>("ref_path", 
        "/home/paralelo4dofnew/ros2_eloquent_ws/pr_4dof/references/ref_cart_TRR0_CF1_IdV1.txt");
        // Trajectory file of Force DMP
        this->declare_parameter<std::string>("force_path", "");
        // Where to save the DMP learned parameters
        this->declare_parameter<std::string>("save_directory", 
        "");
        // Sample time
        this->declare_parameter<double>("ts", 0.01);
        // Number of basis functions
        this->declare_parameter<int>("n_basis_functions", 100);
        // True if the file is in cartesian coordinates (false if it is in prismatic)
        this->declare_parameter<bool>("isCart", false);
        // In case of a prismatic file, whether to calculate the cartesian (otherwise the topic will send 0)
        // This is useful when we only have the prismatic file (for example for moving disassembled legs)
        // Or when we do not have a cartesian file (since we need ref_x_init to start calculating it)
        this->declare_parameter<bool>("calcCart", true);
        // Init cartesian position. Only needed if calcCart is in true
        this->declare_parameter<std::vector<double>>("ref_x_init", {0.069190, 0.632000, 0.078714, -0.135787});
        // Robot config params for calculating the Direct/Inverse Kinematics
        this->declare_parameter<std::vector<double>>(
            "robot_config_params", 
            {0.4, 0.4, 0.4, 0.15, 90*(M_PI/180), 45*(M_PI/180), 0.3, 0.3, 0.3, 50*(M_PI/180), 90*(M_PI/180)});
        // DMP Dynamic parameters
        this->declare_parameter<std::vector<double>>("damping_coefficient_pos", {894.0, 894.0, 89.4, 89.4});
        this->declare_parameter<std::vector<double>>("spring_constant_pos", {250.0, 500.0, 25.0, 25.0});
        this->declare_parameter<std::vector<double>>("mass_pos", {200.0, 200.0, 20.0, 20.0});
        // DMP Dynamic parameters
        this->declare_parameter<std::vector<double>>("damping_coefficient_force", {894.0, 894.0, 89.4, 89.4});
        this->declare_parameter<std::vector<double>>("spring_constant_force", {250.0, 500.0, 25.0, 25.0});
        this->declare_parameter<std::vector<double>>("mass_force", {200.0, 200.0, 20.0, 20.0});
        // Speed of the execution
        this->declare_parameter<double>("speed", 1.0);
        // Gain of the slowdown
        this->declare_parameter<double>("gain_slowdown", 0.0);

        // Assignment of previous parameters to variables
        this->get_parameter("ref_path", ref_path);
        this->get_parameter("force_path", force_path);
        this->get_parameter("ts", ts);
        this->get_parameter("n_basis_functions", n_basis_functions);
        this->get_parameter("save_directory", save_directory);
        this->get_parameter("isCart", isCart);
        this->get_parameter("calcCart", calcCart);
        this->get_parameter("ref_x_init", ref_x_init);
        this->get_parameter("robot_config_params", robot_params);
        this->get_parameter("damping_coefficient_pos", damping_coefficient_pos);
        this->get_parameter("spring_constant_pos", spring_constant_pos);
        this->get_parameter("mass_pos", mass_pos);
        this->get_parameter("damping_coefficient_force", damping_coefficient_force);
        this->get_parameter("spring_constant_force", spring_constant_force);
        this->get_parameter("mass_force", mass_force);
        this->get_parameter("speed", speed);
        this->get_parameter("gain_slowdown", gain_slowdown);


        // Avoid 0 speed
        if (speed==0) speed=1;

        // Publisher of joint position
        publisher_q_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
			"dmp_ref_gen_q", 
			1);
        // Publisher of cart position (will be zeros if isCart and calcCart are both set to false)
        publisher_x_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
			"dmp_ref_gen_x", 
			1);
        // Publisher of the Init Force trajectory
        publisher_force_init_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
			"force_traj_init", 
			1);
        // Publisher of the Changed Force trajectory
        publisher_force_changed_ = this->create_publisher<pr_msgs::msg::PRArrayH>(
			"force_traj_changed", 
			1);
        // Publisher that ends the streaming
        publisher_end_ = this->create_publisher<std_msgs::msg::Bool>(
            "end_flag",
            1
        );

        // Subscriptor for the force for the DMP of position (will be added in the term of the acceleration)
        subscription_force_pos_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "dmp_force_pos",
            1,
            std::bind(&DmpRefForce::force_pos_callback, this, _1));
        // Subscriptor for the force for the DMP of force (will be added in the term of the acceleration)    
        subscription_force_force_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "dmp_force_force",
            1,
            std::bind(&DmpRefForce::force_force_callback, this, _1));
        // Subscription to the joint position
        subscription_pos_ = this->create_subscription<pr_msgs::msg::PRArrayH>(
            "joint_position",
            1,
            std::bind(&DmpRefForce::pos_callback, this, _1));

        // Trajectory generation for DMP
        Trajectory trajectory_dmp = Trajectory::readFromPosFile(ref_path, ts);
        if (trajectory_dmp.length()==0)
        {
            cerr << "The TXT file " << ref_path << " could not be found. Aborting." << endl << endl;
            return;
        }
        else ref_read=true; // We read the trajectory successfully
        int n_dims = trajectory_dmp.dim(); // Number of inputs
        tau = trajectory_dmp.duration();

        // Trajectory generation for force
        Trajectory trajectory_force;
        if (force_path != ""){
             trajectory_force = Trajectory::readFromPosFile(force_path, ts);
             if (trajectory_force.length()==0)
            {
                cerr << "The TXT file " << force_path << " could not be found. Aborting." << endl << endl;
                return;
            }
            else force_read = true; // We read the GKA trajectory succesfully
        } 

        // MAKE THE FUNCTION APPROXIMATORS
        // Initialize some meta parameters for training LWR function approximator
        // We choose 1 instead of four because we are going to stack four identical functions approximators
        // in one vector
        int input_dim = 1;
        // Eigen::Vector4i n_basis_functions{100, 100, 100, 100};
        double intersection = 0.5;
        // alpha for the phase exponential system
        double alpha_phase = 2.0;


        // DEFINITION OF DMP
        // Transformcion de los coeficientes a vectores
        // Despues estos valores son modificados usando tau de la trayectoria para que se comporte
        // como un spring-damper normal sin ser afectado por el tau
        damping_coefficient_pos_vec.resize(n_dims);
        spring_constant_pos_vec.resize(n_dims);
        mass_pos_vec.resize(n_dims);
        damping_coefficient_force_vec.resize(n_dims);
        spring_constant_force_vec.resize(n_dims);
        mass_force_vec.resize(n_dims);
        for (int i=0; i<n_dims;i++){
            damping_coefficient_pos_vec(i) = damping_coefficient_pos[i]/tau;
            spring_constant_pos_vec(i) = spring_constant_pos[i];
            mass_pos_vec[i] = mass_pos[i]/pow(tau,2);
            damping_coefficient_force_vec(i) = damping_coefficient_force[i]/tau;
            spring_constant_force_vec(i) = spring_constant_force[i];
            mass_force_vec[i] = mass_force[i]/pow(tau,2);
        }

        // Vector de aproximadores para el DMP
        // Now we declare the parameters and fill the function approximators
        // The alpha_phase takes the parameter alpha from the phase dynamical system of the DMP (in case it is exponential)
        // and uses it to calculate the centers of the DMP (instead of the min-max range approach, which gives poorer results)
        // This is an ad-hoc change in the library
        vector<FunctionApproximator*> function_approximators_dmp(n_dims);
        MetaParametersLWR* meta_parameters_dmp = new MetaParametersLWR(input_dim,n_basis_functions,intersection, 0.0, false, alpha_phase);      
        FunctionApproximatorLWR* fa_lwr_dmp = new FunctionApproximatorLWR(meta_parameters_dmp);

        for (int dd=0; dd<n_dims; dd++)
            function_approximators_dmp[dd] = fa_lwr_dmp->clone();

        // CONSTRUCT AND TRAIN THE DMP
  
        // Initialize the DMP
        dmp_pos = new Dmp(n_dims, function_approximators_dmp, damping_coefficient_pos_vec, spring_constant_pos_vec, mass_pos_vec, alpha_phase);

        // Train the DMP. Passing the directory will make sure the results are saved to file.
        bool overwrite = true;
        dmp_pos->train(trajectory_dmp,save_directory,overwrite);

        // Adjust the speed
        dmp_pos->set_tau(tau/speed);

        // DEFINITION OF GKA
        if (force_read){
            // Vector de aproximadores para el DMP de fuerza
            vector<FunctionApproximator*> function_approximators_dmp_force(n_dims);
            MetaParametersLWR* meta_parameters_dmp_force = new MetaParametersLWR(input_dim,n_basis_functions,intersection, 0.0, false, alpha_phase);      
            FunctionApproximatorLWR* fa_lwr_dmp_force = new FunctionApproximatorLWR(meta_parameters_dmp_force);

            for (int dd=0; dd<n_dims; dd++)
                function_approximators_dmp_force[dd] = fa_lwr_dmp_force->clone();

            // Initialize the DMP
            dmp_force = new Dmp(n_dims, function_approximators_dmp_force, damping_coefficient_force_vec, spring_constant_force_vec, mass_force_vec, alpha_phase);
            // Training
            dmp_force->train(trajectory_force);
            // Adjust the speed
            dmp_force->set_tau(tau/speed);

        }

        // Initialization of the states and its derivatives
        s.resize(dmp_pos->dim());
        sd.resize(dmp_pos->dim());
        s_updated.resize(dmp_pos->dim());
        s_force_init.resize(dmp_force->dim());
        sd_force_init.resize(dmp_force->dim());
        s_updated_force_init.resize(dmp_force->dim());
        s_force_changed.resize(dmp_force->dim());
        sd_force_changed.resize(dmp_force->dim());
        s_updated_force_changed.resize(dmp_force->dim());

        // Limit phase so that the exponential system lasts as much as the trajectory file (without stop conditions)
        if (dmp_pos->getAlphaPhase() != 0){
            int nn = trajectory_dmp.length();
            double t_f = (nn+1)*ts;
            phase_limit = exp(-dmp_pos->getAlphaPhase()*t_f/tau);
        }
        else phase_limit = 0.01; // If the phase system is not exponential or alpha=0
        
        // Previous value for direct kinematics
        x_dir_kin = ref_x_init;

        
        
    }

    DmpRefForce::~DmpRefForce(){
        delete dmp_pos;
        delete dmp_force;
    }

    // Callback for the force for position dmp
    void DmpRefForce::force_pos_callback(const pr_msgs::msg::PRArrayH::SharedPtr force_pos_msg)
    {
        for(int i=0; i<4; i++)
            force_pos[i] = force_pos_msg->data[i];
    }

    // Callback for the force for force dmp
    void DmpRefForce::force_force_callback(const pr_msgs::msg::PRArrayH::SharedPtr force_force_msg)
    {
        for(int i=0; i<4; i++)
            force_force[i] = force_force_msg->data[i];

    }

    // Callback for the joint position
    void DmpRefForce::pos_callback(const pr_msgs::msg::PRArrayH::SharedPtr pos_msg)
    {
        //DMP message and init time
        auto ref_dmp_q_msg = pr_msgs::msg::PRArrayH();
        ref_dmp_q_msg.init_time = this->get_clock()->now();
        auto ref_dmp_x_msg = pr_msgs::msg::PRArrayH();
        ref_dmp_x_msg.init_time = this->get_clock()->now();
        // Force message and init time
        auto force_init_msg = pr_msgs::msg::PRArrayH();
        force_init_msg.init_time = this->get_clock()->now();
        auto force_changed_msg = pr_msgs::msg::PRArrayH();
        force_changed_msg.init_time = this->get_clock()->now();

        // Make sure we read the trajectory
        if (ref_read){

            // If this is the first iter, we make the first integration of the system
            if (first_iter){
                dmp_pos->integrateStart(s,sd);
                s_pos = s.head(4);
                if (force_read){
                    dmp_force->integrateStart(s_force_changed, sd_force_changed);
                    dmp_force->integrateStart(s_force_init, sd_force_init);
                    forces_init = s_force_init.head(4);
                    forces_changed = s_force_changed.head(4);
                    //sd_force_init = sd_force_changed;
                }
                first_iter = false;
            }
            
            else{
                // The fase is obtained via the following formula (borrowed from PHASE macro in DMP lib)
                phase_current = s.segment(3*dmp_pos->dim_orig()+0, 1);
                //std::cout << phase_current << std::endl;
                
                if (phase_current[0] > phase_limit){

                    // Ahora mismo se define el slowdown a partir del error en la fuerza de aceleracion 
                    // (lo que le está costado llegar a la referencia)
                    // La dinámica dependerá de alpha_x y tau, por lo que se puede diseñar (modificar)
                    // el gain_slowdown para que la ganancia sea independiente de la dinámica

                    slowdown = gain_slowdown*force_pos.norm();
                    Eigen::Vector4d force_vel = Eigen::Vector4d::Zero();

                    // Keep integrating as long as the phase is over the limit and update the data
                    dmp_pos->integrateStepWithForceAndSlowDown(ts,s,s_updated,sd,force_pos,force_vel,slowdown);
                    s = s_updated;
                    s_pos = s.head(4);
                    if (force_read){
                        dmp_force->integrateStepWithForceAndSlowDown(ts,s_force_init,s_updated_force_init,sd_force_init,Eigen::Vector4d::Zero(), Eigen::Vector4d::Zero(),slowdown);
                        s_force_init = s_updated_force_init;
                        forces_init = s_force_init.head(4);
                        dmp_force->integrateStepWithForceAndSlowDown(ts,s_force_changed,s_updated_force_changed,sd_force_changed,force_force,force_vel,slowdown);
                        s_force_changed = s_updated_force_changed;
                        forces_changed = s_force_changed.head(4);
                        // for (int i=0; i<4; i++){
                        //     if (abs(forces_init(i)) < abs(forces_changed(i))){
                        //         std::cout << "init: " << forces_init(i) << std::endl;
                        //         std::cout << "changed: " << forces_changed(i) << std::endl;
                        //         s_force_changed(i) = s_force_init(i);
                        //         s_updated_force_changed(i) = s_updated_force_init(i);
                        //         sd_force_changed(i) = sd_force_init(i);
                        //         forces_changed(i) = forces_init(i);
                        //     }
                        // }
                    }
                }
                else{
                    // End the task and publish it
                    end_task = true;
                    auto end_msg = std_msgs::msg::Bool();
                    end_msg.data = true;
                    publisher_end_->publish(end_msg);
                }
            }

            // Current iteration (is not used anywhere else and is just for visual purposes)
            n_iter++;
            std::cout << n_iter << std::endl;

            if (!end_task){
                // s_pos can be either prismatic or cartesian according to "isCart". Now we obtain the other

                if (isCart){
                    // We performe the inverse kinematics

                    for (int i=0; i<4; i++){
                        ref_dmp_x_msg.data[i] = s_pos[i];
                    }

                    PRModel::InverseKinematics(q_inv_kin, ref_dmp_x_msg.data, robot_params);
                    ref_dmp_q_msg.data[0] = q_inv_kin(0,2);
                    ref_dmp_q_msg.data[1] = q_inv_kin(1,2);
                    ref_dmp_q_msg.data[2] = q_inv_kin(2,2);
                    ref_dmp_q_msg.data[3] = q_inv_kin(3,1);

                }

                else{
                    // We perform the direct kinematics

                    for (int i=0; i<4; i++){
                        ref_dmp_q_msg.data[i] = s_pos[i];
                    }

                    if (calcCart){
                        ref_dmp_x_msg.data = PRModel::ForwardKinematics(ref_dmp_q_msg.data, x_dir_kin, robot_params, tol, iter);
                        PRUtils::array2vector(ref_dmp_x_msg.data, x_dir_kin);
                    }
                    else{
                        for (int i=0; i<4; i++) ref_dmp_x_msg.data[i] = 0;
                    }

                }

                if (force_read){
                    for (int i=0; i<4; i++){
                        force_init_msg.data[i] = forces_init[i];
                        force_changed_msg.data[i] = forces_changed[i];
                    }
                    force_init_msg.header.stamp = pos_msg->header.stamp;
                    force_init_msg.header.frame_id = pos_msg->header.frame_id;

                    force_changed_msg.header.stamp = pos_msg->header.stamp;
                    force_changed_msg.header.frame_id = pos_msg->header.frame_id;

                    force_init_msg.current_time = this->get_clock()->now();
                    publisher_force_init_->publish(force_init_msg);

                    force_changed_msg.current_time = this->get_clock()->now();
                    publisher_force_changed_->publish(force_changed_msg);
                }

                // Prepare messages for publication
                ref_dmp_q_msg.header.stamp = pos_msg->header.stamp;
                ref_dmp_q_msg.header.frame_id = pos_msg->header.frame_id;

                ref_dmp_x_msg.header.stamp = pos_msg->header.stamp;
                ref_dmp_x_msg.header.frame_id = pos_msg->header.frame_id;
                
                ref_dmp_q_msg.current_time = this->get_clock()->now();
                publisher_q_->publish(ref_dmp_q_msg);

                ref_dmp_x_msg.current_time = this->get_clock()->now();
                publisher_x_->publish(ref_dmp_x_msg);
            }
        }
        else
        {
            // If the trajectory was not properly read, publish the end
            end_task = true;
            auto end_msg = std_msgs::msg::Bool();
            end_msg.data = true;
            publisher_end_->publish(end_msg);
        }
        
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_dmp::DmpRefForce)