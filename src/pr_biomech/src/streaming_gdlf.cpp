#include "pr_biomech/streaming_gdlf.hpp"
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace pr_biomech
{
    /**** TEST COMPONENT ****/
    StreamingGDLF::StreamingGDLF(const rclcpp::NodeOptions & options)
    : Node("streaming_gdlf", options)
    {

        //Parameter declaration
        this->declare_parameter<int>("num_samples", 1);
        this->declare_parameter<std::string>("cal_data_file","/home/paralelo4dofnew/ros2_eloquent_ws/pr_4dof/patient_data/Amparo_calibration6GDL.txt");
        this->declare_parameter<std::string>("gdlf_data_file","/home/paralelo4dofnew/ros2_eloquent_ws/pr_4dof/patient_data/Amparo_GDLF6GDL.txt");
        this->declare_parameter<std::string>("output_data_file","/home/paralelo4dofnew/ros2_eloquent_ws/pr_4dof/patient_data/Amparo_output_Data");
        this->declare_parameter<int>("robot_option",2);
        this->declare_parameter<int>("force_sensor_option",1);
        this->declare_parameter<bool>("human_option",true);
        //this->declare_parameter<std::string>("musculo_Obj","");


        this->get_parameter("num_samples", num_samples);
        this->get_parameter("cal_data_file", cal_data_file);
        this->get_parameter("gdlf_data_file", gdlf_data_file);
        this->get_parameter("output_data_file", output_data_file);
        this->get_parameter("robot_option", robot_option);
        this->get_parameter("force_sensor_option", force_sensor_option);
        this->get_parameter("human_option", human_option);
        //this->get_parameter("musculo_Obj",musculo_Obj);

        try{
            mocap_object = std::make_unique<PRMocap::Mocap>(robot_option, force_sensor_option, human_option);
            cal_data = std::make_unique<PRJsonData::PRJsonCal::Calibration_data_struct>(cal_data_file);
            gdlf_data = std::make_unique<PRJsonData::PRJsonGdlf::Gdlf_data_struct>(gdlf_data_file);
        }
        catch(const std::exception &e){
            std::cout << "Los datos del modelo no se han cargado correctamente" << std::endl;
            std::cerr << e.what() << std::endl;
            throw;
        }

        // Suscriptor a la fuerza
        subscription_ = this->create_subscription<pr_msgs::msg::PRForceState>(
            "force_state_sync",
            1,
            std::bind(&StreamingGDLF::force_callback, this, _1)
        );

        publisher_gen_force_knee = this->create_publisher<pr_msgs::msg::PRFloatH>("generalized_force_knee", 1);
        //publisher_F_opt_ref = this->create_publisher<pr_msgs::msg::PRArrayH>("f_opt_ref", 1);

        // Load Cal data
        DH_parameters = cal_data->DH_parameters;
        d = DH_parameters.col(0);
        theta = DH_parameters.col(1);
        alpha = DH_parameters.col(2);
        a = DH_parameters.col(3);
        // Calculamos los gdl de libertad del sistema
        dof = std::min(std::min(a.size(), alpha.size()), std::min(theta.size(), d.size())) - 1;
        //Matrices de rotacion
        R.u_R_LMO4 = Plane4Bar.u_R_LMO4;
        R_local.Femur_O3 = cal_data->R_local.Femur_O3;
        R_local.Tibia_O5 = cal_data->R_local.Tibia_O5;
        R_local.Pie_O7 = cal_data->R_local.Pie_O7;

        ACSpelvis_r_PelAvg_ACSpelvis = cal_data->ACSpelvis_r_PelAvg_ACSpelvis;
        Plane4Bar = cal_data->Plane4Bar;
        R.u_R_LMO4 = Plane4Bar.u_R_LMO4;

        MusPos << 1, 0, 2, 3, 4, 67, 7, 8, 9, 10;

        // Initialize dynamic matrices according to num_samples
        initialization();

        // This function is also auxiliar and initializes with input data for the algorithm
        // Can only be activated if num_samples=1
        // AUXILIAR_initialization(); //.....................................................................................................................AUXILIAR
    
        // Auxiliar assignment of Obj for debugging purposes
        //Obj = 10;
    }

    void StreamingGDLF::force_callback(const pr_msgs::msg::PRForceState::SharedPtr force_state_msg){

        // Ref mod message and init time
        auto gen_force_knee_msg = pr_msgs::msg::PRFloatH();
        gen_force_knee_msg.init_time = this->get_clock()->now();

        // rclcpp::Time init = this->get_clock()->now();

        if (first_iter) {
            // Initial time in ms
            Tiempo_0 = gen_force_knee_msg.init_time.sec*1000 + gen_force_knee_msg.init_time.nanosec*1e-6;
            //Frame_0 = mocap_object->num_frame;
            first_iter = false;
        }


        if (GlobalCnt < num_samples){

            
            for (int i=0; i<3; i++){
                force(i) = force_state_msg->force[i];
                torque(i) = force_state_msg->momentum[i];

                Data.input.forces.col(GlobalCnt)(i) = force(i);
                Data.input.forces.col(GlobalCnt)(i+3) = torque(i);
            }

            // Save passed time (in ms) and Frame data
            Data.Time(GlobalCnt) = gen_force_knee_msg.init_time.sec*1000 + gen_force_knee_msg.init_time.nanosec*1e-6 - Tiempo_0;
            //Data.Frame(GlobalCnt) = mocap_object->num_frame-Frame_0;

            Data.input.LASIS.col(GlobalCnt) = mocap_object->human_data->MarkersMatrix.col(0);
            Data.input.RASIS.col(GlobalCnt) = mocap_object->human_data->MarkersMatrix.col(1);
            Data.input.LPSIS.col(GlobalCnt) = mocap_object->human_data->MarkersMatrix.col(2);
            Data.input.RPSIS.col(GlobalCnt) = mocap_object->human_data->MarkersMatrix.col(3);
            Data.input.RLE.col(GlobalCnt) = mocap_object->human_data->MarkersMatrix.col(4);
            Data.input.RME.col(GlobalCnt) = mocap_object->human_data->MarkersMatrix.col(5);
            Data.input.RHF.col(GlobalCnt) = mocap_object->human_data->MarkersMatrix.col(6);
            Data.input.RLM.col(GlobalCnt) = mocap_object->human_data->MarkersMatrix.col(7);
            Data.input.RMM.col(GlobalCnt) = mocap_object->human_data->MarkersMatrix.col(8);
            Data.input.RCA.col(GlobalCnt) = mocap_object->human_data->MarkersMatrix.col(9);
            Data.input.RFM.col(GlobalCnt) = mocap_object->human_data->MarkersMatrix.col(10);
            Data.input.RVM.col(GlobalCnt) = mocap_object->human_data->MarkersMatrix.col(11);
            Data.input.P_Movil_1.col(GlobalCnt) = mocap_object->robot_data->MarkersMatrix.col(0);
            Data.input.P_Movil_2.col(GlobalCnt) = mocap_object->robot_data->MarkersMatrix.col(1);
            Data.input.P_Movil_3.col(GlobalCnt) = mocap_object->robot_data->MarkersMatrix.col(2);
            Data.input.P_Fija_1.col(GlobalCnt) = mocap_object->robot_data->MarkersMatrix.col(3);
            Data.input.P_Fija_2.col(GlobalCnt) = mocap_object->robot_data->MarkersMatrix.col(4);
            Data.input.P_Fija_3.col(GlobalCnt) = mocap_object->robot_data->MarkersMatrix.col(5);

            // Recoger R_sensor, r_sensor, marcadores humano

            // 1. Calcular G_Fext y G_Mext (con R_sensor)
            G_Fext = -mocap_object->force_sensor_data->R_Sensor * force;
            G_Mext = -mocap_object->force_sensor_data->R_Sensor * torque;

            // 2. Markers del humano se guardan en Piel (vector vertical). Este tiene los datos de todas las iteraciones
                    //(excepto Piel.Fext)
            Piel.LASIS.col(GlobalCnt) = mocap_object->human_data->MarkersMatrix.col(0);
            Piel.RASIS.col(GlobalCnt) = mocap_object->human_data->MarkersMatrix.col(1);
            Piel.LPSIS.col(GlobalCnt) = mocap_object->human_data->MarkersMatrix.col(2);
            Piel.RPSIS.col(GlobalCnt) = mocap_object->human_data->MarkersMatrix.col(3);
            Piel.LFE.col(GlobalCnt) = mocap_object->human_data->MarkersMatrix.col(4);
            Piel.MFE.col(GlobalCnt) = mocap_object->human_data->MarkersMatrix.col(5);
            Piel.FH.col(GlobalCnt) = mocap_object->human_data->MarkersMatrix.col(6);
            Piel.LM.col(GlobalCnt) = mocap_object->human_data->MarkersMatrix.col(7);
            Piel.MM.col(GlobalCnt) = mocap_object->human_data->MarkersMatrix.col(8);
            Piel.CAL.col(GlobalCnt) = mocap_object->human_data->MarkersMatrix.col(9);
            Piel.MH1.col(GlobalCnt) = mocap_object->human_data->MarkersMatrix.col(10);
            Piel.MH5.col(GlobalCnt) = mocap_object->human_data->MarkersMatrix.col(11);
            
            // 3. G_r_G igual que piel pero solo con los de la ultima iteracion (excepto G_r_G.Fext)
            G_r_G.LASIS = Piel.LASIS.col(GlobalCnt); //.........................................................................................................Add Piel.Marker.col(i) = mocap_object...
            G_r_G.RASIS = Piel.RASIS.col(GlobalCnt);
            G_r_G.LPSIS = Piel.LPSIS.col(GlobalCnt);
            G_r_G.RPSIS = Piel.RPSIS.col(GlobalCnt);
            G_r_G.LFE = Piel.LFE.col(GlobalCnt);
            G_r_G.MFE = Piel.MFE.col(GlobalCnt);
            G_r_G.FH = Piel.FH.col(GlobalCnt);
            G_r_G.LM = Piel.LM.col(GlobalCnt);
            G_r_G.MM = Piel.MM.col(GlobalCnt);
            G_r_G.CAL = Piel.CAL.col(GlobalCnt);
            G_r_G.MH1 = Piel.MH1.col(GlobalCnt);
            G_r_G.MH5 = Piel.MH5.col(GlobalCnt);

            // 4. Con r_sensor, calcular Piel.Fext, G_r_G.Fext
            Piel.Fext.col(GlobalCnt) = mocap_object->force_sensor_data->r_Sensor;
            G_r_G.Fext = Piel.Fext.col(GlobalCnt);

            // if (GlobalCnt %100 == 0){
            //     mocap_object->human_data->print_data();
            // }  //G_r_G.print_data();

            // This part will be substituted by the mocap and force sensor readings......................................................................AUXILIAR
            // r_Sensor = Piel.Fext.col(GlobalCnt); //..............................................................................................................Substitute by mocap reading

            // // // // // // // Piel.Fext.col(GlobalCnt) = r_Sensor;
            // // // // // // G_r_G.Fext = Piel.Fext.col(GlobalCnt);

            // // // // // // // Piel is already initialized auxiliarly. In ROS, it will update in the next lines according to the mocap sensor readings from the human
            // // // // // // G_r_G.LASIS = Piel.LASIS.col(GlobalCnt); //.........................................................................................................Add Piel.Marker.col(i) = mocap_object...
            // // // // // // G_r_G.RASIS = Piel.RASIS.col(GlobalCnt);
            // // // // // // G_r_G.LPSIS = Piel.LPSIS.col(GlobalCnt);
            // // // // // // G_r_G.RPSIS = Piel.RPSIS.col(GlobalCnt);
            // // // // // // G_r_G.LFE = Piel.LFE.col(GlobalCnt);
            // // // // // // G_r_G.MFE = Piel.MFE.col(GlobalCnt);
            // // // // // // G_r_G.FH = Piel.FH.col(GlobalCnt);
            // // // // // // G_r_G.LM = Piel.LM.col(GlobalCnt);
            // // // // // // G_r_G.MM = Piel.MM.col(GlobalCnt);
            // // // // // // G_r_G.CAL = Piel.CAL.col(GlobalCnt);
            // // // // // // G_r_G.MH1 = Piel.MH1.col(GlobalCnt);
            // // // // // // G_r_G.MH5 = Piel.MH5.col(GlobalCnt);



            // // // // // // // This will be substituted by the readings from the force sensors........................................................................Add G_Fext = callback_force_message...
            // // // // // // G_Fext = AUXILIAR_ForceSensorin.Force.col(GlobalCnt);
            // // // // // // G_Mext = AUXILIAR_ForceSensorin.Torque.col(GlobalCnt);

            //std::cout << "Posiciones cargadas" << std::endl;
            // Calculo de las posiciones y orientaciones de los segmentos
            Seg_Kinematics(); // Validado

            //std::cout << "Posiciones calculadas " << std::endl;
            // Calculo de la cinemática inversa
            Inv_Kinematics(); // Validado

            //std::cout << "Cinemática inversa calculada" << std::endl;
            // Cálculo de los parámetros de Denavit Hartenberg
            Denavit(a, alpha, theta + q, d, dof, O0, i_1_r_Oi_1_Oi, i_r_Oi_Oj, i_1_R_i, i_R_j);
            std::cout << "DH calculada" << std::endl;
            // Cálculamos la dinámica inversa
            Inv_Dynamics(); // Validado
            std::cout << "Calculamos la dinámica inversa" << std::endl;
            // Buscamos las posiciones intermedias de q5
            //std::cout << "q(5): " << q[4] << std::endl;

            // Buscamos las posiciones intermedias de q[4]
            FindQ(gdlf_data->q5, q[4], n_q5);
            //FindQ(gdlf_data->q5, -0.490041, n_q5);
            q5_int[0] = gdlf_data->q5[n_q5[0]];
            q5_int[1] = gdlf_data->q5[n_q5[1]];
 /*           std::cout << "Valor q5: " << q[4] << std::endl;
            std::cout << "Valores de q5: " << q5_int[0] << q5_int[1] << std::endl;*/
            // Interpolamos los parámetros
            TauGrav = Interpol(q[4], q5_int, gdlf_data->AllTauGrav.middleCols(n_q5[0], 2));
            CoefMus1 = Interpol(q[4], q5_int, gdlf_data->MusCoef1.middleCols(n_q5[0], 2));
            CoefMus2 = Interpol(q[4], q5_int, gdlf_data->MusCoef2.middleCols(n_q5[0], 2));
            CoefMus3 = Interpol(q[4], q5_int, gdlf_data->MusCoef3.middleCols(n_q5[0], 2));
            CoefMus4 = Interpol(q[4], q5_int, gdlf_data->MusCoef4.middleCols(n_q5[0], 2));
            CoefMus5 = Interpol(q[4], q5_int, gdlf_data->MusCoef5.middleCols(n_q5[0], 2));
            CoefMus6 = Interpol(q[4], q5_int, gdlf_data->MusCoef6.middleCols(n_q5[0], 2));
            PatCoef = Interpol(q[4], q5_int, gdlf_data->AllPatCoef.middleCols(n_q5[0], 2));
            KneeCoef = Interpol_3D(q[4], q5_int, gdlf_data->AllKneeCoef[n_q5[0]], gdlf_data->AllKneeCoef[n_q5[1]]);
            // Cálculo la Tau que debe compensar´los músculos
            TauMus = -TauGrav - Tau;
            //std::cout << "TauMus: " << TauMus << std::endl;
            // Cálculo de las fuerzas músculares para compensar la Tau
            MusForce(); // Validado
            // Cálculo de las fuerzas de reacción en la rodilla
            //gdlf_data->r_Local.print_data();
            CalcKneeForces();

            gen_force_knee_msg.header.stamp = force_state_msg->header.stamp;
            gen_force_knee_msg.header.frame_id = force_state_msg->header.frame_id;
            gen_force_knee_msg.data = 0.0; // Change

            // Fill data
            Data.Time(GlobalCnt) = gen_force_knee_msg.init_time.sec*1000 + gen_force_knee_msg.init_time.nanosec*1e-6 - Tiempo_0;
            Data.G_Fext.col(GlobalCnt) =  G_Fext;
            Data.G_Mext.col(GlobalCnt) =  G_Mext;
            Data.MuscleForce.col(GlobalCnt) =  MuscleForce;
            Data.F_Knee.col(GlobalCnt) = F_Knee;
            Data.M_Knee.col(GlobalCnt) = M_Knee;
            Data.q.col(GlobalCnt) = q;
            Data.PatForce(GlobalCnt) = PatForce;
            Data.TauMus.col(GlobalCnt) = TauMus;
            Data.PosKnee(GlobalCnt) = PosKnee;
            
            GlobalCnt++;
            // Ponemos a 0 el origen
            O0.setZero(3, 1);

            gen_force_knee_msg.current_time = this->get_clock()->now();
            publisher_gen_force_knee->publish(gen_force_knee_msg);

            // f_opt_ref_msg.current_time = this->get_clock()->now();
            // publisher_F_opt_ref->publish(f_opt_ref_msg);

            // double time_init = gen_force_knee_msg.init_time.sec + gen_force_knee_msg.init_time.nanosec*1e-9;
            // double time_current = gen_force_knee_msg.current_time.sec + gen_force_knee_msg.current_time.nanosec*1e-9;
            // double msec = (time_current - time_init)*1000;
            // std::cout << msec << std::endl;
            //double dur1 = time1.sec*1000 + time1.nanosec*1e-6 - time0.sec*1000 - time0.nanosec*1e-6;
            // double dur1 = time1.nanoseconds()*1e-6 - init.nanoseconds()*1e-6;
            // double dur2 = time2.nanoseconds()*1e-6 - time1.nanoseconds()*1e-6;
            // double dur3 = time3.nanoseconds()*1e-6 - time2.nanoseconds()*1e-6;
            // double dur4 = time4.nanoseconds()*1e-6 - time3.nanoseconds()*1e-6;
            // double dur5 = time5.nanoseconds()*1e-6 - time4.nanoseconds()*1e-6;
            // double dur6 = time6.nanoseconds()*1e-6 - time5.nanoseconds()*1e-6;
            // std::cout << "dur1 " << dur1 << std::endl;
            // std::cout << "dur2 " << dur2 << std::endl;
            // std::cout << "dur3 " << dur3 << std::endl;
            // std::cout << "dur4 " << dur4 << std::endl;
            // std::cout << "dur5 " << dur5 << std::endl;
            // std::cout << "dur6 " << dur6 << std::endl;
        }

    }

    StreamingGDLF::~StreamingGDLF() {

        //Data.print_data();
        std::stringstream ss;
        ss << "{";
        vector2ss("Time", Data.Time, ss); ss << ",";
        matrix2ss("MuscleForce", Data.MuscleForce, ss); ss << ",";
        matrix2ss("F_Knee", Data.F_Knee, ss); ss << ",";
        matrix2ss("M_Knee", Data.M_Knee, ss); ss << ",";
        matrix2ss("q", Data.q, ss); ss << ",";
        vector2ss("PatForce", Data.PatForce, ss); ss << ",";
        vector2ss("PosKnee", Data.PosKnee, ss); ss << ",";
        matrix2ss("TauMus", Data.TauMus, ss); ss << ",";

        ss << "\"input\":{";
        matrix2ss("forces", Data.input.forces, ss); ss << ",";
        matrix2ss("LASIS", Data.input.LASIS, ss); ss << ",";
        matrix2ss("RASIS", Data.input.RASIS, ss); ss << ",";
        matrix2ss("LPSIS", Data.input.LPSIS, ss); ss << ",";
        matrix2ss("RPSIS", Data.input.RPSIS, ss); ss << ",";
        matrix2ss("RLE", Data.input.RLE, ss); ss << ",";
        matrix2ss("RME", Data.input.RME, ss); ss << ",";
        matrix2ss("RHF", Data.input.RHF, ss); ss << ",";
        matrix2ss("RLM", Data.input.RLM, ss); ss << ",";
        matrix2ss("RMM", Data.input.RMM, ss); ss << ",";
        matrix2ss("RCA", Data.input.RCA, ss); ss << ",";
        matrix2ss("RFM", Data.input.RFM, ss); ss << ",";
        matrix2ss("RVM", Data.input.RVM, ss); ss << ",";
        matrix2ss("P_Movil_1", Data.input.P_Movil_1, ss); ss << ",";
        matrix2ss("P_Movil_2", Data.input.P_Movil_2, ss); ss << ",";
        matrix2ss("P_Movil_3", Data.input.P_Movil_3, ss); ss << ",";
        matrix2ss("P_Fija_1", Data.input.P_Fija_1, ss); ss << ",";
        matrix2ss("P_Fija_2", Data.input.P_Fija_2, ss); ss << ",";
        matrix2ss("P_Fija_3", Data.input.P_Fija_3, ss);
        // matrix2ss("robot_markers", Data.input.robot_markers, ss); ss << ",";
        // matrix2ss("human_markers", Data.input.human_markers, ss);
        ss << "}}";

        // std::cout << ss.str() << std::endl;

        // Document d;
        // d.SetObject();
        // d.Parse(ss.str().c_str());


        //char filename[1000];// = Data.Date.c_str();//"Data_Jose" +Data.Date + ".txt";
        //snprintf(filename, sizeof(filename), "%s_%s.txt", output_data_file.c_str(), Data.Date.c_str());
        

        // FILE* fp = fopen(filename, "wb"); // non-Windows use "w"

        // char writeBuffer[65536];
        // FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));

        // Writer<FileWriteStream> writer(os);
        // d.Accept(writer);

        // fclose(fp);

        std::string filename_str = output_data_file + "_" + Data.Date + ".txt";
        std::ofstream out_file;
        out_file.open(filename_str);
        if (out_file.is_open()){
            out_file << ss.str();
            out_file.close();
            std::cout << "Writen Bio data in: " << filename_str << std::endl;
        }
        else {
            std::cout << "Could not open file to save the data" << std::endl;
        }

    }    
}

void pr_biomech::StreamingGDLF::vector2ss(const std::string &name, const Eigen::VectorXd &v, std::stringstream &ss) {
    // Escritura en formato JSON, dado un stringstream, lo amplía con la nueva variable llamada "name" y su contenido en el vector de doubles

    std::stringstream v_ss;

    v_ss << "[";
    for (int i = 0; i < v.size(); i++) {
        v_ss << v[i] << ",";
    }

    std::string v_s = v_ss.str();
    v_s.erase(v_s.end()-1, v_s.end());
    v_s += "]";

    ss << "\"" << name << "\":" << v_s;
};

void pr_biomech::StreamingGDLF::vector2ss(const std::string& name, const Eigen::VectorXi& v, std::stringstream& ss) {
    // Escritura en formato JSON, dado un stringstream, lo amplía con la nueva variable llamada "name" y su contenido en el vector de enteros


    std::stringstream v_ss;

    v_ss << "[";
    for (int i = 0; i < v.size(); i++) {
        v_ss << v[i] << ",";
    }

    std::string v_s = v_ss.str();
    v_s.erase(v_s.end() - 1, v_s.end());
    v_s += "]";

    ss << "\"" << name << "\":" << v_s;
};

void pr_biomech::StreamingGDLF::matrix2ss(const std::string& name, const Eigen::MatrixXd& m, std::stringstream& ss) {
    // Escritura en formato JSON, dado un stringstream, lo amplía con la nueva variable llamada "name" y su contenido en la matriz de doubles


    std::stringstream m_ss;

    m_ss << "[";

    for (int i = 0; i < m.rows(); i++) {
        m_ss << "[";
        for (int j = 0; j < m.cols()-1; j++) {
            m_ss << m(i,j) << ",";
        }
        m_ss << m(i, m.cols() - 1);
        m_ss << "],";
    }
    std::string m_s = m_ss.str();
    m_s.erase(m_s.end()-1);
    m_s += "]";


    ss << "\"" << name << "\":" << m_s;
};



void pr_biomech::StreamingGDLF::initialization() {
    G_Fext.resize(3, num_samples);
    G_Mext.resize(3, num_samples);

    Piel.LASIS = Eigen::MatrixXd::Zero(3, num_samples);
    Piel.RASIS = Eigen::MatrixXd::Zero(3, num_samples);
    Piel.LPSIS = Eigen::MatrixXd::Zero(3, num_samples);
    Piel.RPSIS = Eigen::MatrixXd::Zero(3, num_samples);
    Piel.LFE = Eigen::MatrixXd::Zero(3, num_samples);
    Piel.MFE = Eigen::MatrixXd::Zero(3, num_samples);
    Piel.FH = Eigen::MatrixXd::Zero(3, num_samples);
    Piel.LM = Eigen::MatrixXd::Zero(3, num_samples);
    Piel.MM = Eigen::MatrixXd::Zero(3, num_samples);
    Piel.CAL = Eigen::MatrixXd::Zero(3, num_samples);
    Piel.MH1 = Eigen::MatrixXd::Zero(3, num_samples);
    Piel.MH5 = Eigen::MatrixXd::Zero(3, num_samples);
    Piel.Fext = Eigen::MatrixXd::Zero(3, num_samples);

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    Data.Date = oss.str();

    Data.Time = Eigen::VectorXd::Zero(num_samples);
    Data.G_Fext = Eigen::MatrixXd::Zero(3, num_samples);
    Data.G_Mext = Eigen::MatrixXd::Zero(3, num_samples);
    Data.MuscleForce = Eigen::MatrixXd::Zero(73, num_samples);
    Data.F_Knee = Eigen::MatrixXd::Zero(3, num_samples);
    Data.M_Knee = Eigen::MatrixXd::Zero(3, num_samples);
    Data.q = Eigen::MatrixXd::Zero(8, num_samples);
    Data.PatForce = Eigen::VectorXd::Zero(num_samples);
    Data.PosKnee = Eigen::VectorXd::Zero(num_samples);
    Data.TauMus = Eigen::MatrixXd::Zero(6, num_samples);

    Data.input.forces = Eigen::MatrixXd::Zero(6, num_samples);
    Data.input.LASIS = Eigen::MatrixXd::Zero(3, num_samples);
    Data.input.RASIS = Eigen::MatrixXd::Zero(3, num_samples);
    Data.input.LPSIS = Eigen::MatrixXd::Zero(3, num_samples);
    Data.input.RPSIS = Eigen::MatrixXd::Zero(3, num_samples);
    Data.input.RLE = Eigen::MatrixXd::Zero(3, num_samples);
    Data.input.RME = Eigen::MatrixXd::Zero(3, num_samples);
    Data.input.RHF = Eigen::MatrixXd::Zero(3, num_samples);
    Data.input.RLM = Eigen::MatrixXd::Zero(3, num_samples);
    Data.input.RMM = Eigen::MatrixXd::Zero(3, num_samples);
    Data.input.RCA = Eigen::MatrixXd::Zero(3, num_samples);
    Data.input.RFM = Eigen::MatrixXd::Zero(3, num_samples);
    Data.input.RVM = Eigen::MatrixXd::Zero(3, num_samples);
    Data.input.P_Movil_1 = Eigen::MatrixXd::Zero(3, num_samples);
    Data.input.P_Movil_2 = Eigen::MatrixXd::Zero(3, num_samples);
    Data.input.P_Movil_3 = Eigen::MatrixXd::Zero(3, num_samples);
    Data.input.P_Fija_1 = Eigen::MatrixXd::Zero(3, num_samples);
    Data.input.P_Fija_2 = Eigen::MatrixXd::Zero(3, num_samples);
    Data.input.P_Fija_3 = Eigen::MatrixXd::Zero(3, num_samples);

}

void pr_biomech::StreamingGDLF::AUXILIAR_initialization() { //..................................................................................AUXILIAR

    // Resize
    AUXILIAR_ForceSensorin.Force.resize(3, num_samples);
    AUXILIAR_ForceSensorin.Torque.resize(3, num_samples);

    Piel.LASIS.resize(3, num_samples);
    Piel.RASIS.resize(3, num_samples);
    Piel.LPSIS.resize(3, num_samples);
    Piel.RPSIS.resize(3, num_samples);
    Piel.LFE.resize(3, num_samples);
    Piel.MFE.resize(3, num_samples);
    Piel.LM.resize(3, num_samples);
    Piel.MM.resize(3, num_samples);
    Piel.FH.resize(3, num_samples);
    Piel.CAL.resize(3, num_samples);
    Piel.MH1.resize(3, num_samples);
    Piel.MH5.resize(3, num_samples);
    Piel.Fext.resize(3, num_samples);

    // AUXILIAR_ForceSensorin.Force <<
    //     -2.69200000000000,
    //     35.0730000000000,
    //     294.575000000000;

    // AUXILIAR_ForceSensorin.Torque <<
    //     0,
    //     0,
    //     0.00112332100000000;

    // Piel.LASIS <<
    //     0.309118633518190,
    //     0.507384384518173,
    //     0.876350081679902;
    // Piel.RASIS <<
    //     0.318646242478893,
    //     0.236414116280524,
    //     0.863542113266964;
    // Piel.LPSIS <<
    //     0.133961173630123,
    //     0.407876647102307,
    //     0.906380220709934;
    // Piel.RPSIS <<
    //     0.146207495194841,
    //     0.285209326369309,
    //     0.906621180631798;
    // Piel.LFE <<
    //     0.275177863347457,
    //     0.224258293148821,
    //     0.433858387408652;
    // Piel.MFE <<
    //     0.294452267184151,
    //     0.305708114557396,
    //     0.417302638974376;
    // Piel.FH <<
    //     0.232860947322651,
    //     0.214078867510318,
    //     0.404086227469735;
    // Piel.LM <<
    //     0.203462804049729,
    //     0.207047621498908,
    //     0.0737606031020175;
    // Piel.MM <<
    //     0.234046066905469,
    //     0.265185093663577,
    //     0.0688883798317411;
    // Piel.CAL <<
    //     0.173734580409545,
    //     0.238503121188524,
    //     0.0409408442069703;
    // Piel.MH1 <<
    //     0.338996311988788,
    //     0.215773384695226,
    //     0.0312895036406646;
    // Piel.MH5 <<
    //     0.315935084260580,
    //     0.163400319081938,
    //     0.0253501415906924;

    // Piel.Fext <<
    //     0.292624000000000,
    //     0.217876000000000,
    //     0.0000;

    AUXILIAR_ForceSensorin.Force <<
        -6.3010000000,        -13.0820000000,        -0.7610000000,        1.1910000000,        -0.2810000000,        -1.7860000000,        -7.1350000000,        1.5690000000,        -5.7190000000,        -10.0780000000,
        32.5270000000,        7.4000000000,        32.5130000000,        19.9130000000,        33.8810000000,        34.8660000000,        7.4440000000,        32.9500000000,        32.0260000000,        5.9580000000,
        243.5770000000,        236.6860000000,        292.3760000000,        204.5380000000,        240.9480000000,        259.9070000000,        267.6720000000,        267.1050000000,        239.5320000000,        276.0290000000;

    AUXILIAR_ForceSensorin.Torque <<
        0.0000000000,        0.0000000000,        0.0000000000,        0.0000000000,        0.0000000000,        0.0000000000,        0.0000000000,        0.0000000000,        0.0000000000,        0.0000000000,
        -0.0000000000,        -0.0000000000,        -0.0000000000,        -0.0000000000,        -0.0000000000,        -0.0000000000,        -0.0000000000,        -0.0000000000,        -0.0000000000,        -0.0000000000,
        0.0006522640,        0.0049807490,        0.0013821830,        0.0010984560,        0.0010490870,        0.0008717080,        0.0045111920,        0.0011909660,        0.0012314290,        0.0056581810;

    Piel.LASIS <<
        0.2477474868,        0.1294811412,        0.3078131276,        0.1398599938,        0.2467349348,        0.2818644788,        0.1327360223,        0.3148678708,        0.2267737035,        0.1191837034,
        0.5261883379,        0.5536346778,        0.5101820852,        0.5451305435,        0.5254499398,        0.5231932276,        0.5405428565,        0.5227991203,        0.5362658775,        0.5403450438,
        0.8248222508,        0.5633657244,        0.8783236890,        0.6724587284,        0.7808474160,        0.8702772926,        0.5437766915,        0.8790229173,        0.7465820657,        0.5434259701;
    Piel.RASIS <<
        0.2580239169,        0.1332195931,        0.3108956989,        0.1305558443,        0.2609498485,        0.2773621574,        0.1356880800,        0.3162873388,        0.2293512888,        0.1258548291,
        0.2568067871,        0.2910107590,        0.2404573485,        0.2824840632,        0.2575302726,        0.2547068250,        0.2773421334,        0.2525709141,        0.2679549735,        0.2762431262,
        0.8181614892,        0.5521839643,        0.8610538854,        0.6621698658,        0.7753252564,        0.8572383616,        0.5290286291,        0.8619762536,        0.7380008167,        0.5298936069;
    Piel.LPSIS <<
        0.0806048827,        -0.0078336360,        0.1334151967,        -0.0099203742,        0.0881206701,        0.1041954863,        0.0006538987,        0.1380151821,        0.0686560753,        -0.0111743539,
        0.4315706723,        0.4660243568,        0.4157809731,        0.4639590567,        0.4319678563,        0.4351126744,        0.4485969645,        0.4289697429,        0.4479601696,        0.4465665950,
        0.8734426276,        0.6418610581,        0.9221722071,        0.7484298172,        0.8476652489,        0.9143821905,        0.6212690483,        0.9170291193,        0.8190598657,        0.6178268888;
    Piel.RPSIS <<
        0.0929189083,        0.0084930865,        0.1437986522,        -0.0004170648,        0.1034357977,        0.1118940704,        0.0188010480,        0.1477258550,        0.0799661170,        0.0077510425,
        0.3081276128,        0.3487275611,        0.2922556420,        0.3430987287,        0.3070496123,        0.3104966545,        0.3325781977,        0.3052453011,        0.3230612682,        0.3311895606,
        0.8790820801,        0.6503538002,        0.9210273134,        0.7580985835,        0.8547277951,        0.9158006667,        0.6277071726,        0.9167742903,        0.8256145095,        0.6241019456;
    Piel.LFE <<
        0.3417116869,        0.3827507363,        0.2527146932,        0.3459950739,        0.3774480170,        0.2797898277,        0.3852419496,        0.2758731381,        0.3844683981,        0.3826903046,
        0.2128620609,        0.2140790171,        0.2336588127,        0.2068552541,        0.2013674419,        0.2367690840,        0.1983372251,        0.2393340956,        0.2085788324,        0.2055884073,
        0.4180735842,        0.3890032087,        0.4355896843,        0.4076560073,        0.4016538120,        0.4341157479,        0.3851049554,        0.4341291414,        0.3972811910,        0.3877356422;
    Piel.MFE <<
        0.3621197364,        0.4049232662,        0.2665260127,        0.3702246713,        0.4000691703,        0.2975529216,        0.4082080977,        0.2922181798,        0.4067707294,        0.4046439576,
        0.2977966399,        0.3167019903,        0.3148573546,        0.3041527848,        0.2907614851,        0.3171153258,        0.3005865782,        0.3184394848,        0.3011795810,        0.3069410092,
        0.4063254387,        0.3797438430,        0.4203904716,        0.3997886009,        0.3918361458,        0.4182462228,        0.3787237568,        0.4169589376,        0.3874394320,        0.3828445985;
    Piel.FH <<
        0.3009333318,        0.3483453780,        0.2131753259,        0.3132736343,        0.3365882418,        0.2394402270,        0.3553050950,        0.2338770581,        0.3443088560,        0.3511106354,
        0.2035339639,        0.2048745514,        0.2198425374,        0.1996089217,        0.1942483421,        0.2233001423,        0.1902027849,        0.2258232849,        0.2012044812,        0.1949869398,
        0.3859011409,        0.3543535560,        0.4075592116,        0.3696276387,        0.3708345183,        0.4032197385,        0.3470095707,        0.4061197550,        0.3662999912,        0.3513118602;
    Piel.LM <<
        0.2126760055,        0.2186616108,        0.2010347308,        0.2145414880,        0.2153114763,        0.2060835122,        0.2185987710,        0.2038672758,        0.2168762546,        0.2187422344,
        0.2067377720,        0.2225434590,        0.2125107570,        0.2149426485,        0.2116608128,        0.2128163541,        0.2201480256,        0.2136134320,        0.2143417206,        0.2221835104,
        0.0721142674,        0.0739595419,        0.0749397125,        0.0713057809,        0.0723625415,        0.0736502769,        0.0722239810,        0.0751069193,        0.0724303415,        0.0724783236;
    Piel.MM <<
        0.2409659134,        0.2501510120,        0.2320749395,        0.2450379943,        0.2448550985,        0.2368854535,        0.2493971761,        0.2350187083,        0.2466551361,        0.2503192612,
        0.2672066427,        0.2811594282,        0.2693883178,        0.2750232366,        0.2721976402,        0.2707535871,        0.2794522562,        0.2705782801,        0.2746173601,        0.2807539855,
        0.0634129639,        0.0573300672,        0.0707861696,        0.0602560553,        0.0618588337,        0.0667803135,        0.0587250319,        0.0686323987,        0.0598323590,        0.0573320671;
    Piel.CAL <<
        0.1762569243,        0.1831064069,        0.1734986912,        0.1778914159,        0.1786686596,        0.1750224059,        0.1826954005,        0.1745634236,        0.1795943969,        0.1830844937,
        0.2404700593,        0.2597913819,        0.2441267709,        0.2512759026,        0.2477646299,        0.2454789345,        0.2583878616,        0.2455410273,        0.2504961372,        0.2609160986,
        0.0450622584,        0.0485863128,        0.0404893483,        0.0462520181,        0.0484870773,        0.0420278153,        0.0492940846,        0.0416600393,        0.0484970486,        0.0485830552;
    Piel.MH1 <<
        0.3399323624,        0.3424081818,        0.3394145212,        0.3406678089,        0.3403568670,        0.3396077662,        0.3416913455,        0.3394133352,        0.3406451344,        0.3415778584,
        0.2163798812,        0.2181208772,        0.2161538546,        0.2170263179,        0.2164138455,        0.2160530751,        0.2176920629,        0.2157774851,        0.2168082127,        0.2174913343,
        0.0307783424,        0.0297230737,        0.0311783984,        0.0305143328,        0.0304206184,        0.0308676690,        0.0296055958,        0.0311469712,        0.0301196691,        0.0299118225;
    Piel.MH5 <<
        0.3172476117,        0.3170907480,        0.3149158782,        0.3168493926,        0.3153387858,        0.3145167390,        0.3165682843,        0.3144290527,        0.3159556565,        0.3158313515,
        0.1648413208,        0.1701887378,        0.1644086503,        0.1695682648,        0.1669460497,        0.1644577102,        0.1704337765,        0.1644574975,        0.1686606506,        0.1712084522,
        0.0264592712,        0.0281076649,        0.0254538267,        0.0292280663,        0.0268007930,        0.0259632646,        0.0281221157,        0.0260288754,        0.0276497388,        0.0292536895;
    Piel.Fext <<
        0.2911380000,        0.3128510000,        0.2932580000,        0.2614130000,        0.3290970000,        0.2808630000,        0.3289390000,        0.2929940000,        0.3347960000,        0.3081720000,
        0.2281170000,        0.2349360000,        0.2226390000,        0.2426380000,        0.2192940000,        0.2288320000,        0.2285070000,        0.2244560000,        0.2231170000,        0.2347640000,
        0.0000000000,        0.0000000000,        0.0000000000,        0.0000000000,        0.0000000000,        0.0000000000,        0.0000000000,        0.0000000000,        0.0000000000,        0.0000000000;

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_biomech::StreamingGDLF)