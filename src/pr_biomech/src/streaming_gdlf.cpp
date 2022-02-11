#include "pr_biomech/streaming_gdlf.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace pr_biomech
{
    /**** TEST COMPONENT ****/
    StreamingGDLF::StreamingGDLF(const rclcpp::NodeOptions & options)
    : Node("test", options)
    {

        //Parameter declaration
        this->declare_parameter<int>("num_samples", 2000);
        this->declare_parameter<std::string>("cal_data_file","/home/paralelo4dofnew/ros2_eloquent_ws/pr_4dof/patient_data/Jose_calibration.txt");
        this->declare_parameter<std::string>("gdlf_data_file","/home/paralelo4dofnew/ros2_eloquent_ws/pr_4dof/patient_data/Jose_CoefOffline_Data.txt");
        this->declare_parameter<std::string>("output_data_file","/home/paralelo4dofnew/ros2_eloquent_ws/pr_4dof/patient_data/Jose_output_Data");
        this->declare_parameter<int>("robot_option",1);
        this->declare_parameter<int>("force_sensor_option",1);
        this->declare_parameter<bool>("human_option",true);


        this->get_parameter("num_samples", num_samples);
        this->get_parameter("cal_data_file", cal_data_file);
        this->get_parameter("gdlf_data_file", gdlf_data_file);
        this->get_parameter("output_data_file", output_data_file);
        this->get_parameter("robot_option", robot_option);
        this->get_parameter("force_sensor_option", force_sensor_option);
        this->get_parameter("human_option", human_option);

        try{
            mocap_object = std::make_unique<PRMocap::Mocap>(robot_option, force_sensor_option, human_option);
            cal_data = std::make_unique<PRJsonData::PRJsonCal::Calibration_data_struct>(cal_data_file);
            gdlf_data = std::make_unique<PRJsonData::PRJsonGdlf::Gdlf_data_struct>(gdlf_data_file);
        }
        catch(const std::exception &e){
            std::cerr << e.what() << std::endl;
            throw;
        }

        // Suscriptor a la fuerza
        subscription_ = this->create_subscription<pr_msgs::msg::PRForceState>(
            "force_state_sync",
            1,
            std::bind(&StreamingGDLF::force_callback, this, _1)
        );

        publisher_ = this->create_publisher<pr_msgs::msg::PRFloatH>("generalized_force_knee", 1);

        // Load Widths data
        Widths(0) = cal_data->Widths.Fem;
        Widths(1) = cal_data->Widths.Tib;

        // Load rotation matrix data
        R.u_R_LMO4 = cal_data->Plane4Bar.u_R_LMO4;
        R.LMO4_R_u = R.u_R_LMO4.transpose();

        // Load L, A, theta0, DELTA from Plane4Bar
        L = cal_data->Plane4Bar.L;
        A.head(2) = cal_data->Plane4Bar.A; A(2) = 0;
        theta0 = cal_data->Plane4Bar.theta0;
        DELTA = cal_data->Plane4Bar.DELTA;

        // Initialize dynamic matrices according to num_samples
        initialization();

        // This function is also auxiliar and initializes with input data for the algorithm
        // Can only be activated if num_samples=4
        //AUXILIAR_initialization(); //.....................................................................................................................AUXILIAR
    }

    void StreamingGDLF::force_callback(const pr_msgs::msg::PRForceState::SharedPtr force_state_msg){

        // Ref mod message and init time
        auto gen_force_knee_msg = pr_msgs::msg::PRFloatH();
        gen_force_knee_msg.init_time = this->get_clock()->now();

        // rclcpp::Time init = this->get_clock()->now();

        if (first_iter) {
            // Initial time in ms
            Tiempo_0 = gen_force_knee_msg.init_time.sec*1000 + gen_force_knee_msg.init_time.nanosec*1e-6;
            Frame_0 = mocap_object->num_frame;
            first_iter = false;
        }

        if (GlobalCnt < num_samples){

            // Save passed time (in ms) and Frame data
            Data.Time(GlobalCnt) = gen_force_knee_msg.init_time.sec*1000 + gen_force_knee_msg.init_time.nanosec*1e-6 - Tiempo_0;
            Data.Frame(GlobalCnt) = mocap_object->num_frame-Frame_0;

            for (int i=0; i<3; i++){
                force(i) = force_state_msg->force[i];
                torque(i) = force_state_msg->momentum[i];
            }

            // // Recoger R_sensor, r_sensor, marcadores humano

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

            // // This part will be substituted by the mocap and force sensor readings......................................................................AUXILIAR
            // r_Sensor = Piel.Fext.col(GlobalCnt); //..............................................................................................................Substitute by mocap reading

            // Piel.Fext.col(GlobalCnt) = r_Sensor;
            // G_r_G.Fext = r_Sensor;

            // // Piel is already initialized auxiliarly. In ROS, it will update in the next lines according to the mocap sensor readings from the human
            // G_r_G.LASIS = Piel.LASIS.col(GlobalCnt); //.........................................................................................................Add Piel.Marker.col(i) = mocap_object...
            // G_r_G.RASIS = Piel.RASIS.col(GlobalCnt);
            // G_r_G.LPSIS = Piel.LPSIS.col(GlobalCnt);
            // G_r_G.RPSIS = Piel.RPSIS.col(GlobalCnt);
            // G_r_G.LFE = Piel.LFE.col(GlobalCnt);
            // G_r_G.MFE = Piel.MFE.col(GlobalCnt);
            // G_r_G.FH = Piel.FH.col(GlobalCnt);
            // G_r_G.LM = Piel.LM.col(GlobalCnt);
            // G_r_G.MM = Piel.MM.col(GlobalCnt);
            // G_r_G.CAL = Piel.CAL.col(GlobalCnt);
            // G_r_G.MH1 = Piel.MH1.col(GlobalCnt);
            // G_r_G.MH5 = Piel.MH5.col(GlobalCnt);


            // // This will be substitued by the readings from the force sensors........................................................................Add G_Fext = callback_force_message...
            // G_Fext = AUXILIAR_ForceSensorin.Force.col(GlobalCnt);
            // G_Mext = AUXILIAR_ForceSensorin.Torque.col(GlobalCnt);

            // Here begins the calculus using previous data
            Vector3d G_r_G_KJC_temp = (Piel.LFE.col(GlobalCnt) + Piel.MFE.col(GlobalCnt)) / 2;
            Vector3d G_u_LFE_MFE = unit((Piel.MFE.col(GlobalCnt) - Piel.LFE.col(GlobalCnt)));
            Piel.LFE.col(GlobalCnt) = G_r_G_KJC_temp - G_u_LFE_MFE * Widths(0) / 2;
            Piel.MFE.col(GlobalCnt) = G_r_G_KJC_temp + G_u_LFE_MFE * Widths(0) / 2;
            G_r_G.LFE = Piel.LFE.col(GlobalCnt);
            G_r_G.MFE = Piel.MFE.col(GlobalCnt);

            // // Rotation matrices of the hip
            PelCalcs();
            G_r_G.HJC = HJC.col(GlobalCnt);
            // rclcpp::Time time1 = this->get_clock()->now();

            // Calculation of SCS
            CalConst_Pau2();
            // rclcpp::Time time2 = this->get_clock()->now();
                
            // Matriz de rotacion de LMO4 (Rodilla)
            R.G_R_LMO4 = GetRotMat(G_r_G.HJC, G_r_G.LFE, G_r_G.MFE, 2, 1, 3);
            R.LMO4_R_G = R.G_R_LMO4.transpose();
            
            // Calcula posiciones de los angulos auxiliares de la rodilla theta2, theta3 y theta4, necesarios para el calculo de q5
            CalTheta2Theta3(); // Asigna valores a las variables theta2, theta3, theta4
            // rclcpp::Time time3 = this->get_clock()->now();

            // Obtener la dinamica de la rodilla resultante
            DynamicsHerz_Pau();
            // rclcpp::Time time4 = this->get_clock()->now();

            //Calculo Muscular
            MusForceOpt2_Anta_Coef_Guard_Red();
            // rclcpp::Time time5 = this->get_clock()->now();
                
            // Calculo ligamentos
            CalLigForceAnta_Pau();
            //Data.print_data();
            // rclcpp::Time time6 = this->get_clock()->now();

            gen_force_knee_msg.header.frame_id = force_state_msg->header.frame_id;
            gen_force_knee_msg.data = Data.GenForceKneeFlxExt(GlobalCnt);

            GlobalCnt++;

            gen_force_knee_msg.current_time = this->get_clock()->now();
            publisher_->publish(gen_force_knee_msg);

            double time_init = gen_force_knee_msg.init_time.sec + gen_force_knee_msg.init_time.nanosec*1e-9;
            double time_current = gen_force_knee_msg.current_time.sec + gen_force_knee_msg.current_time.nanosec*1e-9;
            double msec = (time_current - time_init)*1000;
            std::cout << msec << std::endl;
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

        std::stringstream ss;
        ss << "{\"ang\":{\"cad\":{";
        vector2ss("FlxExt", Data.ang.cad.FlxExt, ss); ss << ",";
        vector2ss("AbdAduc", Data.ang.cad.AbdAduc, ss); ss << ",";
        vector2ss("IntExt", Data.ang.cad.IntExt, ss);
        ss << "},\"rod\":{";

        vector2ss("FlxExt", Data.ang.rod.FlxExt, ss);
        ss << "},\"pie\":{";

        vector2ss("FlxExt", Data.ang.pie.FlxExt, ss);
        ss << "}";

        ss << "},";
        vector2ss("Time", Data.Time, ss); ss << ",";
        vector2ss("Frame", Data.Frame, ss); ss << ",";
        vector2ss("GenForceHipFlxExt", Data.GenForceHipFlxExt, ss); ss << ",";
        vector2ss("GenForceKneeFlxExtGrav", Data.GenForceKneeFlxExtGrav, ss); ss << ",";
        vector2ss("GenForceKneeFlxExtFext", Data.GenForceKneeFlxExtFext, ss); ss << ",";
        vector2ss("GenForceKneeFlxExtMext", Data.GenForceKneeFlxExtMext, ss); ss << ",";
        vector2ss("GenForceKneeFlxExt", Data.GenForceKneeFlxExt, ss); ss << ",";
        matrix2ss("q", Data.q, ss); ss << ",";
        vector2ss("BicFemCB", Data.BicFemCB, ss); ss << ",";
        vector2ss("GastLat", Data.GastLat, ss); ss << ",";
        vector2ss("GastMed", Data.GastMed, ss); ss << ",";
        vector2ss("BicFemCL", Data.BicFemCL, ss); ss << ",";
        vector2ss("SemTend", Data.SemTend, ss); ss << ",";
        vector2ss("SemMem", Data.SemMem, ss); ss << ",";
        vector2ss("Sat", Data.Sat, ss); ss << ",";
        vector2ss("Gra", Data.Gra, ss); ss << ",";
        vector2ss("VasInt123", Data.VasInt123, ss); ss << ",";
        vector2ss("VasMedInf12", Data.VasMedInf12, ss); ss << ",";
        vector2ss("VasMedMed12", Data.VasMedMed12, ss); ss << ",";
        vector2ss("VasMedSup34", Data.VasMedSup34, ss); ss << ",";
        vector2ss("VasLatSup12", Data.VasLatSup12, ss); ss << ",";
        vector2ss("VasInt456", Data.VasInt456, ss); ss << ",";
        vector2ss("VasLatInf4", Data.VasLatInf4, ss); ss << ",";
        vector2ss("RecFem12_1", Data.RecFem12_2, ss); ss << ",";
        vector2ss("TenFacLat", Data.TenFacLat, ss); ss << ",";
        matrix2ss("FlxCoefMatRed", Data.FlxCoefMatRed, ss); ss << ",";
        matrix2ss("ExtCoefMatRed", Data.ExtCoefMatRed, ss); ss << ",";
        vector2ss("F_OnTibUc", Data.F_OnTibUc, ss); ss << ",";
        vector2ss("F_OnTibUt", Data.F_OnTibUt, ss); ss << ",";
        vector2ss("F_ACL", Data.F_ACL, ss); ss << ",";
        vector2ss("F_PCL", Data.F_PCL, ss);

        ss << "}";

        Document d;
        d.SetObject();
        d.Parse(ss.str().c_str());


        char filename[1000];// = Data.Date.c_str();//"Data_Jose" +Data.Date + ".txt";
        snprintf(filename, sizeof(filename), "%s_%s.txt", output_data_file.c_str(), Data.Date.c_str());
        

        FILE* fp = fopen(filename, "wb"); // non-Windows use "w"

        char writeBuffer[65536];
        FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));

        Writer<FileWriteStream> writer(os);
        d.Accept(writer);

        fclose(fp);
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

    Data.ang.cad.AbdAduc = Eigen::VectorXd::Zero(num_samples);
    Data.ang.cad.FlxExt = Eigen::VectorXd::Zero(num_samples);
    Data.ang.cad.IntExt = Eigen::VectorXd::Zero(num_samples);
    Data.ang.rod.FlxExt = Eigen::VectorXd::Zero(num_samples);
    Data.ang.pie.FlxExt = Eigen::VectorXd::Zero(num_samples);
    Data.Time = Eigen::VectorXd::Zero(num_samples);
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    Data.Date = oss.str();
    Data.GenForceHipFlxExt = Eigen::VectorXd::Zero(num_samples);
    Data.GenForceKneeFlxExtGrav = Eigen::VectorXd::Zero(num_samples);
    Data.GenForceKneeFlxExtFext = Eigen::VectorXd::Zero(num_samples);
    Data.GenForceKneeFlxExtMext = Eigen::VectorXd::Zero(num_samples);
    Data.GenForceKneeFlxExt = Eigen::VectorXd::Zero(num_samples);
    Data.q = Eigen::MatrixXd::Zero(6,num_samples);
    Data.BicFemCB = Eigen::VectorXd::Zero(num_samples);
    Data.GastLat = Eigen::VectorXd::Zero(num_samples);
    Data.GastMed = Eigen::VectorXd::Zero(num_samples);
    Data.BicFemCL = Eigen::VectorXd::Zero(num_samples);
    Data.SemTend = Eigen::VectorXd::Zero(num_samples);
    Data.SemMem = Eigen::VectorXd::Zero(num_samples);
    Data.Sat = Eigen::VectorXd::Zero(num_samples);
    Data.Gra = Eigen::VectorXd::Zero(num_samples);
    Data.VasInt123 = Eigen::VectorXd::Zero(num_samples);
    Data.VasMedInf12 = Eigen::VectorXd::Zero(num_samples);
    Data.VasMedMed12 = Eigen::VectorXd::Zero(num_samples);
    Data.VasMedMed12 = Eigen::VectorXd::Zero(num_samples);
    Data.VasMedSup34 = Eigen::VectorXd::Zero(num_samples);
    Data.VasLatSup12 = Eigen::VectorXd::Zero(num_samples);
    Data.VasInt456 = Eigen::VectorXd::Zero(num_samples);
    Data.VasLatInf4 = Eigen::VectorXd::Zero(num_samples);
    Data.RecFem12_1 = Eigen::VectorXd::Zero(num_samples);
    Data.RecFem12_2 = Eigen::VectorXd::Zero(num_samples);
    Data.TenFacLat = Eigen::VectorXd::Zero(num_samples);
    Data.FlxCoefMatRed = Eigen::MatrixXd::Zero(8, num_samples);
    Data.ExtCoefMatRed = Eigen::MatrixXd::Zero(10, num_samples);
    Data.F_OnTibUc = Eigen::VectorXd::Zero(num_samples);
    Data.F_OnTibUt = Eigen::VectorXd::Zero(num_samples);
    Data.F_ACL = Eigen::VectorXd::Zero(num_samples);
    Data.F_PCL = Eigen::VectorXd::Zero(num_samples);
    Data.Frame = Eigen::VectorXi::Zero(num_samples);

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

    HJC = Eigen::MatrixXd::Zero(3, num_samples);

}

void pr_biomech::StreamingGDLF::AUXILIAR_initialization() { //..................................................................................AUXILIAR

    AUXILIAR_ForceSensorin.Force.resize(3, num_samples);
    AUXILIAR_ForceSensorin.Torque.resize(3, num_samples);

    AUXILIAR_ForceSensorin.Force <<
        0.22196887348544, 12.278994778356, -3.89805690727794, 4.33142972673402,
        95.3377414997786, -30.8715768146965, 51.3105360965565, 74.8833625505454,
        23.0808269450729, -54.4707206629079, -3.86025931546142, 25.5973116884979;

    AUXILIAR_ForceSensorin.Torque <<
        -3.35140884563206, -16.9072163835842, -10.3047041861824, 10.807649335197,
        1.72294171615608, 3.02304501267983, 0.823035132584989, 0.639216624105116,
        -0.219808524037625, 4.11242991780717, 2.74755888430851, -2.2637694719948;

    Piel.LASIS <<
        1.164245, 1.164047, 1.164949, 1.164001,
        1.335371, 1.335517, 1.331638, 1.332329,
        -1.350255, -1.353063, -1.364925, -1.363151;

    Piel.RASIS <<
        0.829182, 0.829037, 0.830712, 0.829375,
        1.33496, 1.334313, 1.336947, 1.336816,
        -1.324737, -1.326898, -1.330643, -1.332763;

    Piel.LPSIS <<
        1.035284, 1.03566, 1.031021, 1.031988,
        1.22817, 1.217651, 1.213356, 1.213734,
        -1.502076, -1.497275, -1.503653, -1.503438;


    Piel.RPSIS <<
        0.945335, 0.945752, 0.941373, 0.942194,
        1.225082, 1.214609, 1.212139, 1.212296,
        -1.491387, -1.486225, -1.490374, -1.491204;

    Piel.LFE <<
        0.852027, 0.836302, 0.851037, 0.840061,
        1.274141, 1.246399, 1.268532, 1.288775,
        -0.959213, -0.95265, -0.955833, -0.962246;

    Piel.MFE <<
        0.998688, 0.983424, 0.998393, 0.98677,
        1.300224, 1.26268, 1.289405, 1.312557,
        -0.948915, -0.933013, -0.94374, -0.947871;

    Piel.FH <<
        0.87377, 0.846863, 0.867401, 0.85856,
        1.21064, 1.185322, 1.205837, 1.227225,
        -0.936859, -0.937646, -0.937368, -0.93369;

    Piel.LM <<
        1.068577, 0.990411, 1.041451, 1.029757,
        0.991182, 0.909304, 0.958348, 1.03035,
        -0.663538, -0.684616, -0.674172, -0.629145;

    Piel.MM <<
        1.160969, 1.091492, 1.137375, 1.127268,
        1.028383, 0.940144, 0.992311, 1.057997,
        -0.733811, -0.745362, -0.741282, -0.696864;

    Piel.CAL <<
        0.984366999999963, 0.987096, 1.059916, 1.053149,
        0.963262999999927, 0.885379, 0.947322, 1.008945,
        -0.679830000000038, -0.758732, -0.765965, -0.718059;

    Piel.MH1 <<
        1.10293799999999, 1.107187, 1.112112, 1.091627,
        0.975294000000019, 0.853177, 0.897411, 0.985896,
        -0.49541499999998, -0.578819, -0.559538, -0.504014;

    Piel.MH5 <<
        0.998428999999987, 1.018215, 1.017569, 0.997843,
        0.980027000000007, 0.889765, 0.945738, 1.03938,
        -0.448546999999991, -0.528978, -0.537976, -0.494923;

    Piel.Fext <<
        0.984055701207706, 0.991114889982437, 0.991877875377595, 0.976891679071567,
        1.03790007518413, 0.949808836209316, 0.992001247821963, 1.06767612577389,
        -0.650543720667812, -0.692379943828314, -0.682209593218062, -0.631651551694311;
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pr_biomech::StreamingGDLF)