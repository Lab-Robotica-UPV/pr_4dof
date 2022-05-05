#include "pr_lib_biomech/pr_mocap.hpp"

// Constructor for Data (human)
PRMocap::Data::Data(std::vector<std::string> names) : markers_name{names}{}

// Constructor for Robot Data
PRMocap::Robot_data::Robot_data(std::vector<std::string> robot_markers, RobotOptions robot_option) : Data(robot_markers){
    robot_5p = (robot_option == RobotOptions::Robot5p);
}

// Constructor for Sensor Data
PRMocap::Force_sensor_data::Force_sensor_data(std::vector<std::string> force_sensor_markers, RobotOptions robot_option, ForceSensorOptions force_sensor_option) : Data(force_sensor_markers){
    robot_5p = (robot_option == RobotOptions::Robot5p);
    onRobot = (force_sensor_option == ForceSensorOptions::OnRobot);
    if (!onRobot) r_2_0 << 90.0/1000, 130.0/1000, 24.0/1000;
    else{
        if (!robot_5p) r_2_0 << 0.0, 200.0/1000, -75.7/1000;
        else r_2_0 << -320.44/1000.0, 150.0/1000.0, -13.4/1000.0;
    }
}

// Print data for Data structure
void PRMocap::Data::print_data(){
    std::cout << "HUMAN DATA" << std::endl;
    for (int i=0; i<markers_name.size(); i++){
        std::cout << markers_name[i] << ": " << markers[i] << std::endl;
    }
    std::cout << "MarkersMatrix: " << std::endl;
    std::cout << MarkersMatrix << std::endl;
    std::cout << "\n" << std::endl;
}

// Print data for Robot
void PRMocap::Robot_data::print_data(){
    std::cout << "ROBOT DATA " << std::endl;
    for (int i=0; i<markers_name.size(); i++){
        std::cout << markers_name[i] << ": " << markers[i] << std::endl;
    }
    std::cout << "Robot Cartesian coordinates:\n" << XCoords << std::endl;
    std::cout << "robot_5p: " << robot_5p << std::endl;
}

// Print data for Force Sensor
void PRMocap::Force_sensor_data::print_data(){
    std::cout << "FORCE SENSOR DATA " << std::endl;
    if (!onRobot){
        for (int i=0; i<markers_name.size(); i++){
            std::cout << markers_name[i] << ": " << markers[i] << std::endl;
        }
    }
    std::cout << "onRobot: " << onRobot << std::endl;
    std::cout << "robot_5p: " << robot_5p << std::endl;
    std::cout << "r_2_0: " << r_2_0.transpose() << std::endl;
    std::cout << "r_Sensor: " << r_Sensor.transpose() << std::endl;
    std::cout << "R_Sensor: " << R_Sensor << std::endl;
}

// Constructor which creates the structures (according to the arguments), the connection and defines the markers
PRMocap::Mocap::Mocap(int robot = 0, int force_sensor = 0, bool human = false) :
    human_option{human}{

    // Throw exceptions if the user does not use the proper arguments
    if (robot<0 || robot>=static_cast<int>(RobotOptions::RobotOptions_EnumSize)){
        std::cout << "Robot parameter not correct (must be between 0 and " << static_cast<int>(RobotOptions::RobotOptions_EnumSize) << std::endl;
        throw std::string("Robot parameter not correct");
    }

    if (force_sensor<0 || force_sensor>=static_cast<int>(ForceSensorOptions::ForceSensorOptions_EnumSize)){
        std::cout << "Force sensor parameter not correct (must be between 0 and " << static_cast<int>(ForceSensorOptions::ForceSensorOptions_EnumSize) << std::endl;
        throw std::string("Force sensor parameter not correct");
    }

    // Casting to the enum after making sure the arguments were correct
    robot_option = static_cast<RobotOptions>(robot);
    force_sensor_option = static_cast<ForceSensorOptions>(force_sensor);

    robot_data = nullptr;
    force_sensor_data = nullptr;
    human_data = nullptr;

    // Initialize the data in runtime
    if (robot_option == RobotOptions::Robot3ups){
        robot_data = new Robot_data(robot_markers_3ups, robot_option);
        robot_data->robot_5p = false;
    }
    else if (robot_option == RobotOptions::Robot5p){
        robot_data = new Robot_data(robot_markers_5p, robot_option);
        robot_data->robot_5p = true;
    }
    if (force_sensor_option != ForceSensorOptions::NoSensor)
        force_sensor_data = new Force_sensor_data(force_sensor_markers, robot_option, force_sensor_option);
    if (human_option == true)
        human_data = new Data(human_markers);

    // If force sensor is on Robot but robot was set to NoRobot, throw an error
    if (force_sensor_option == ForceSensorOptions::OnRobot && robot_option == RobotOptions::NoRobot){
        std::cout << "If the sensor is on the robot (sensor argument 1), the robot argument cannot be 0 (no robot)";
        throw std::string("If the sensor is on the robot (sensor argument 1), the robot argument cannot be 0 (no robot)");
    }
 
    if (robot_option!=RobotOptions::NoRobot) robot_data->MarkersMatrix.resize(3,robot_data->markers_name.size());
    if (force_sensor_option==ForceSensorOptions::OffRobot) force_sensor_data->MarkersMatrix.resize(3,force_sensor_data->markers_name.size());
    if (human_option) human_data->MarkersMatrix.resize(3,human_data->markers_name.size());


    //NatNet client
    g_pClient = new NatNetClient();
    g_pClient->SetFrameReceivedCallback(DataHandler, this);

    g_connectParams.connectionType = ConnectionType_Multicast;
    g_connectParams.serverCommandPort = server_command_port;
    g_connectParams.serverDataPort = server_data_port;
    g_connectParams.serverAddress = server_address.c_str();
    g_connectParams.localAddress = "";
    g_connectParams.multicastAddress = "";

    // Release previous server if exists
    g_pClient->Disconnect();

    // Init Client and connect to NatNet server
    int retCode = g_pClient->Connect( g_connectParams );
    if (retCode != ErrorCode_OK)
    {
        std::cout << "Unable to connect to server. Error code: " << retCode << ". Exiting"<< std::endl;
        throw retCode;
    }
    else
        std::cout << "NatNet client initialized and ready." << std::endl;

    // Retrieve Data Descriptions from Motive
    std::cout << "[SampleClient] Requesting Data Descriptions..." << std::endl;
    sDataDescriptions* pDataDefs = NULL;
    int iResult = g_pClient->GetDataDescriptionList(&pDataDefs);
    if (iResult != ErrorCode_OK || pDataDefs == NULL)
    {
        std::cout << "[SampleClient] Unable to retrieve Data Descriptions." << std::endl;
        throw iResult;
    }
    else
    {
        for (int i=0; i < pDataDefs->nDataDescriptions; i++)
        {
            if(pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet)
            {
                // MarkerSet
                sMarkerSetDescription* pMS = pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription;
                if (strcmp(pMS->szName, "all")==0){
                    bool found;
                    printf("\nMarkerSet Name : %s\n", pMS->szName);
                    for(int i=0; i < pMS->nMarkers; i++){
                        printf("%s\n", pMS->szMarkerNames[i]);
                        found = false;
                        if (robot_option!=RobotOptions::NoRobot && !found){
                            found = read_marker(robot_data, pMS->szMarkerNames[i], i);
                        }
                            
                        if (force_sensor_option==ForceSensorOptions::OffRobot && !found){
                            found = read_marker(force_sensor_data, pMS->szMarkerNames[i], i);
                        }

                        if (human_option && !found){
                            std::string markername_str = std::string(pMS->szMarkerNames[i]);
                            std::size_t last_underscore = markername_str.find_last_of('_');
                            std::string sub_str = markername_str.substr(last_underscore+1);
                            found = read_marker(human_data, sub_str.c_str(), i);
                                
                        }   
                    }
                    if (robot_option!=RobotOptions::NoRobot && std::all_of(robot_data->markers.begin(), robot_data->markers.end(), [](int i) { return i==0; })){
                        std::cout << "Robot markers cannot be read" << std::endl;
                        throw std::string("Robot markers cannot be read");
                    }
                    if (force_sensor_option==ForceSensorOptions::OffRobot && std::all_of(force_sensor_data->markers.begin(), force_sensor_data->markers.end(), [](int i) { return i==0; })){
                        std::cout << "Force sensor platform markers cannot be read" << std::endl;
                        throw std::string("Force sensor platform markers cannot be read");
                    }
                    if (human_option && std::all_of(human_data->markers.begin(), human_data->markers.end(), [](int i) { return i==0; })){
                        std::cout << "Human markers cannot be read" << std::endl;
                        throw std::string("Human markers cannot be read");
                    }
                }
            }
        }
    }  
}

// Destructor to null the pointer
PRMocap::Mocap::~Mocap(){

    if (g_pClient){
        g_pClient->Disconnect();
        delete g_pClient;
        g_pClient = NULL;
    }

    delete robot_data;
    delete force_sensor_data;
    delete human_data;

}

// Read the marker and place in order (integer indices)
bool PRMocap::Mocap::read_marker(Data* data, const char* marker_name, const int &loc){
    data->markers.resize(data->markers_name.size());
    for(int j=0; j<data->markers_name.size(); j++){
        if(strcmp(marker_name, data->markers_name[j].c_str())==0){
            data->markers[j] = loc;
            return true;
        }
    }
    return false;
}

// Fill the matrix with the data
void PRMocap::Mocap::fillMarkersMatrix(Data* data, const sMarker &marker, const int &loc){
    for (int j=0; j<data->markers_name.size(); j++){
        if (loc == data->markers[j]){
            data->MarkersMatrix(0,j) = marker.x;
            data->MarkersMatrix(1,j) = marker.y;
            data->MarkersMatrix(2,j) = marker.z;
        }
    }
}

// Print all the data
void PRMocap::Mocap::print_data(){
    if (robot_option!=RobotOptions::NoRobot) robot_data->print_data();
    if (force_sensor_option!=ForceSensorOptions::NoSensor) force_sensor_data->print_data();
    if (human_option) human_data->print_data();
    std::cout << "Num frame: " << num_frame << std::endl;
}

// Set the robot origin
void PRMocap::Mocap::robot_origin(){
    Eigen::Vector3d mf1 = robot_data->MarkersMatrix.col(3);
    Eigen::Vector3d mf2 = robot_data->MarkersMatrix.col(4);
    Eigen::Vector3d mf3 = robot_data->MarkersMatrix.col(5);
    Eigen::Vector3d mm1 = robot_data->MarkersMatrix.col(0);
    Eigen::Vector3d mm2 = robot_data->MarkersMatrix.col(1);
    Eigen::Vector3d mm3 = robot_data->MarkersMatrix.col(2);
            
    //Sistema de referencia fijo
            
    //Matriz de rotación del sistema fijo al sistema local
    Eigen::Matrix<double, 3, 3> Rlf;
    //eje xf
    Eigen::Vector3d rmf1mf2 = mf2-mf1;
    Rlf.col(0) = rmf1mf2/sqrt(rmf1mf2.transpose()*rmf1mf2);
    //eje yf
    Eigen::Vector3d rmf1mf3 = mf3-mf1;
    Rlf.col(1) = rmf1mf3/sqrt(rmf1mf3.transpose()*rmf1mf3);
    //eje zf
    Eigen::Vector3d rzf_l = rmf1mf2.cross(rmf1mf3);
    Rlf.col(2) = rzf_l/sqrt(rzf_l.transpose()*rzf_l);
            
    //Matriz de rotación del sistema local al sistema fijo
    Eigen::Matrix<double, 3, 3> Rfl = Rlf.transpose();

    //Sistema de referencia móvil
            
    //Matriz de rotación del sistema movil al sistema local
    Eigen::Matrix<double, 3, 3> Rlm;
    //eje xm
    Eigen::Vector3d rmm1mm2_l = mm2 - mm1;
    if(robot_data->robot_5p)
        rmm1mm2_l = -rmm1mm2_l;
            
    Rlm.col(0) = rmm1mm2_l/sqrt(rmm1mm2_l.transpose()*rmm1mm2_l);
    //eje ym
    Eigen::Vector3d rmm1mm3_l = mm3 - mm1;
    Rlm.col(1) = rmm1mm3_l/sqrt(rmm1mm3_l.transpose()*rmm1mm3_l);
    //eje zm
    Eigen::Vector3d rzm_l = rmm1mm2_l.cross(rmm1mm3_l);
    Rlm.col(2) = rzm_l/sqrt(rzm_l.transpose()*rzm_l);
            
    //Matriz de rotación entre el sistema móvil y el sistema fijo
    Eigen::Matrix<double, 3, 3> Rfm = Rfl*Rlm;

    //Orientación de la plataforma móvil
    robot_data->XCoords(1,1) = atan2(Rfm(0,2), Rfm(2,2));
    robot_data->XCoords(2,1) = atan2(Rfm(1,0), Rfm(1,1));

    //Posición de la plataforma móvil
    Eigen::Vector3d rdf_f;
    Eigen::Vector3d rdm_m;
    if(robot_data->robot_5p){
        rdf_f << -0.350, 0, 0;
        rdm_m << -0.320, 0.150, -0.098;
    } else {
        rdf_f << -0.276, 0.064, 0.018;
        //rdm_m << 0,0,-0.1605;
        rdm_m << 0,0.2,-0.1605;
    }
            
    robot_data->XCoords.col(0) = Rfl*(mm1 + Rlm*rdm_m - (mf1 + Rlf*rdf_f));
}

void PRMocap::Mocap::force_sensor_origin(){

    Eigen::MatrixXd r_markers;
    Eigen::Vector3d unit_i, unit_j, unit_k;

    if (force_sensor_option != ForceSensorOptions::NoSensor){
        // If the sensor is on the robot, use the robot markers to know the position and orientation
        if (force_sensor_option == ForceSensorOptions::OnRobot){
            r_markers = robot_data->MarkersMatrix;  
        }
        // If the sensor is on the auxiliary platform, use those markers
        else if (force_sensor_option == ForceSensorOptions::OffRobot){
            r_markers = force_sensor_data->MarkersMatrix;
        }

        // The markers used are the 3 of the robot mobile platform (if onRobot) or the first three markers
        // of the auxiliary force platform (offRobot)
        Eigen::Vector3d col1 = r_markers.col(0);
        Eigen::Vector3d col2 = r_markers.col(1);
        Eigen::Vector3d col3 = r_markers.col(2);
        if (force_sensor_option == ForceSensorOptions::OnRobot && robot_option == RobotOptions::Robot3ups){
            unit_i = (col2-col1)/(col2-col1).norm();
            unit_j = (col3-col1)/(col3-col1).norm();
            unit_k = unit_i.cross(unit_j);
        }
        else if (force_sensor_option == ForceSensorOptions::OnRobot && robot_option == RobotOptions::Robot5p){
            unit_i = (col1-col2)/(col1-col2).norm();
            Eigen::Vector3d aux = (col1-col3).cross(unit_i);
            unit_k = aux/(aux.norm());
            unit_j = unit_k.cross(unit_i);
        }
        else if (force_sensor_option == ForceSensorOptions::OffRobot){
            unit_i = (col1-col2)/(col1-col2).norm();
            unit_j = (col3-col2)/(col3-col2).norm();
            unit_k = unit_i.cross(unit_j);
        }
        

        // Rotation matrix
        force_sensor_data->R_Sensor << unit_i, unit_j, unit_k;

        //Throw an exception if the matrix R is not orthogonal
        if (force_sensor_data->R_Sensor.determinant() >= 1+force_sensor_data->error_R || 
            force_sensor_data->R_Sensor.determinant() <= 1-force_sensor_data->error_R){
            std::cout << "Hay una gran falta de ortogonalidad entre los ejes X e Y";
            throw std::string("Hay una gran falta de ortogonalidad entre los ejes X e Y");
        }

        //Vector of the location of the force sensor
        if (force_sensor_option == ForceSensorOptions::OnRobot){
            force_sensor_data->r_Sensor = r_markers.col(0) + (force_sensor_data->R_Sensor*force_sensor_data->r_2_0);
        }
        else if (force_sensor_option == ForceSensorOptions::OffRobot){
            force_sensor_data->r_Sensor = r_markers.col(1) + (force_sensor_data->R_Sensor*force_sensor_data->r_2_0);
        }
    }
    
}

// Thread that reads the cameras and uses the previous functions
void PRMocap::NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData){

    Mocap* mocap_object = (Mocap*) pUserData;
    NatNetClient* pClient = mocap_object->g_pClient;

    for(int i=0; i < data->nLabeledMarkers; i++)
    {
        sMarker marker = data->LabeledMarkers[i];

        //Fill Markers Matrix of robot
        if (mocap_object->robot_option != RobotOptions::NoRobot){
            mocap_object->fillMarkersMatrix(mocap_object->robot_data, marker, i);
        }

        //Fill Markers Matrix of sensor
        if (mocap_object->force_sensor_option == ForceSensorOptions::OffRobot){
            mocap_object->fillMarkersMatrix(mocap_object->force_sensor_data, marker, i);
        }

        //Fill Markers Matrix of man
        if (mocap_object->human_option){
            mocap_object->fillMarkersMatrix(mocap_object->human_data, marker, i);
        }
            
    }

    // Location of the robot (mobile platform relative to fixed platform)
    // Location of the force_sensor relative to the global frame of the cameras
    if (mocap_object->robot_option != RobotOptions::NoRobot) mocap_object->robot_origin();
    if (mocap_object->force_sensor_option != ForceSensorOptions::NoSensor) mocap_object->force_sensor_origin();

    mocap_object->num_frame++;

}