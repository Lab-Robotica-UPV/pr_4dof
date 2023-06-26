#ifndef PR_LIB__MOCAP_HPP_
#define PR_LIB__MOCAP_HPP_

#include <string>
#include <vector>
#include <memory>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>

#include <iostream>

namespace PRMocap{

    // Enums for the Robot and Sensor options
    // NoRobot: No data will be collected from the robot
    // Robot3ups: Data from Robot3ups will be collected
    // Robot5p: Data from Robot5p will be collected
    // NoSensor: No data from sensor will be collected from cameras
    // SensorOnRobot: No data from sensor platform will be collected from cameras, but it will be calculated using the
        //robot position (it needs Robot3ups or Robot5p. Currently working only on Robot3ups)
    // SensorOffRobot: Data from sensor external platform will be collected. This is for auxiliary tests.
    // ENUMSIZE is a trick to be able to know the number of elements of enum.
    enum RobotOptions {NoRobot, Robot3ups, Robot5p, RobotOptions_EnumSize};
    enum ForceSensorOptions {NoSensor, OnRobot, OffRobot, ForceSensorOptions_EnumSize};

    // Function running in another Thread to collect data from cameras. The data comes from
    // Robot
    // Force sensor position
    // Human
    void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData);    // receives data from the server

    // Generic struct for data from cameras
    // Saves the name of the markers to search in the cameras
    struct Data{
        std::vector<std::string> markers_name;
        std::vector<int> markers;
        Eigen::Matrix<double, 3, -1> MarkersMatrix;
        Data(std::vector<std::string> names);
        void print_data();
    };

    // Derived struct for the robot, which needs type and cartesian coordinates
    struct Robot_data : Data{
        bool robot_5p; // False if robot is 3ups_rpu
        Eigen::Matrix<double, 3, 2> XCoords;
        Eigen::Matrix<double, 3, 3> Rlf;
        Eigen::Matrix<double, 3, 3> Rfl;
        Robot_data(std::vector<std::string> robot_markers, RobotOptions robot_option);
        void print_data();
    };

    // Structure with the position of the platform with the sensor data (for auxiliary off-robot task)
    struct Force_sensor_data : Data{
        bool robot_5p; // False if robot is 3ups_rpu
        bool onRobot; // False if force sensor is not attached to the robot (auxiliar platform)
        Eigen::Matrix3d R_Sensor; // Rotation of the force sensor
        double error_R = 1e-3; // Maximum deviation fron unit determinant
        Eigen::Vector3d r_Sensor; // Vector from origin (reference frame)
        Eigen::Vector3d r_2_0 = Eigen::Vector3d::Zero(); // Distance from the sensor center to the marker 2
        // This is suposing the sensor attached to the robot
        Force_sensor_data(std::vector<std::string> sensor_markers, RobotOptions robot_option, ForceSensorOptions force_sensor_option);
        void print_data();
    };

    // Class to read from cameras and assign values to the structures
    class Mocap{

        public:

            // The constructor consider reading only from the robot
            // The int variables will be casted to the enum variables (see enum meanings)
            // The bool human variable indicates whether the human data will be collected or not
            explicit Mocap(int robot, int force_sensor, bool human);
            ~Mocap();
            void print_data();

            // Variables for the connection
            NatNetClient* g_pClient = NULL;
            sServerDescription g_serverDescription;

            // Variables with the names (if not fixed, can be an argument to pass to the constructor)
            std::vector<std::string> robot_markers_3ups = {"P_Movil1_1", "P_Movil1_2", "P_Movil1_3", "P_Fija1_1", "P_Fija1_2", "P_Fija1_3"};
            std::vector<std::string> robot_markers_5p = {"P_Movil_5P_1", "P_Movil_5P_2", "P_Movil_5P_3", "P_Fija_5P_1", "P_Fija_5P_2", "P_Fija_5P_3"};
            std::vector<std::string> force_sensor_markers = {"ForceSensor_1", "ForceSensor_2", "ForceSensor_3", "ForceSensor_4"};
            std::vector<std::string> human_markers = {"LASIS","RASIS","LPSIS","RPSIS","RLE","RME","RHF","RLM","RMM","RCA","RFM","RVM"};

            // Creation of structs
            Robot_data *robot_data;
            Force_sensor_data *force_sensor_data;
            Data *human_data;

            // Variables assigned during construction to know the markers to seek
            RobotOptions robot_option;
            ForceSensorOptions force_sensor_option; 
            bool human_option;

            // Number of frames
            int num_frame;

        protected:
            // Function to read the markers
            bool read_marker(Data* data, const char* marker_name, const int &loc);
            // Function to fill the marker matrix (3 coords of each marker)
            void fillMarkersMatrix(Data* data, const sMarker &marker, const int &loc);
            // Function to calculate the origin of the mobile platform for the robot
            void robot_origin();
            // Function to calculate the rotation matrix of the force sensor
            void force_sensor_origin();

        private:
            // Connection variables (if can change, can be set from outside)
            sNatNetClientConnectParams g_connectParams;
            int g_analogSamplesPerMocapFrame = 0;
            std::string server_address = "158.42.21.85";
            int server_command_port = 1510;
            int server_data_port = 1511;
            // Make function friend so it can access private data
            friend void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData);
            
    };

}

#endif // PR_LIB__MOCAP_HPP_