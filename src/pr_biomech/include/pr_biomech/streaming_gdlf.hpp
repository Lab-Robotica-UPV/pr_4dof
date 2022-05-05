#ifndef PR_BIOMECH__StreamingGDLF_HPP_
#define PR_BIOMECH__StreamingGDLF_HPP_

#include "rclcpp/rclcpp.hpp"

#include "eigen3/Eigen/Dense"

#include "pr_lib_biomech/pr_json_calibration.hpp"
#include "pr_lib_biomech/pr_json_gdlf.hpp"
#include "pr_lib_biomech/pr_mocap.hpp"
#include "pr_lib_biomech/pr_data_structures.hpp"
#include "pr_lib_biomech/pr_algebra_fun.hpp"
#include "pr_lib_biomech/pr_biomech_fun.hpp"

// JSON writer from rapidjson library
#include "filewritestream.h"
#include "writer.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <iomanip>
#include <ctime>
#include <math.h>

#include "pr_msgs/msg/pr_force_state.hpp"
#include "pr_msgs/msg/pr_float_h.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"


namespace pr_biomech
{
    class StreamingGDLF : public rclcpp::Node
    {
        public:
            //PR_COONTROLLERS_PUBLIC
            explicit StreamingGDLF(const rclcpp::NodeOptions & options);
            // DESTRUCTOR
            ~StreamingGDLF();

            void AUXILIAR_initialization(); //..............................................................................................................AUXILIAR
            void initialization(); //Memory allocation of matrices according to num_samples
            void PelCalcs(); // Calculation of rotation matrices of the hip
            void CalConst_Pau2(); // Calculation of Subject Central System (SCS) of the model segments
            void CalTheta2Theta3(); // Calculation of Theta2 and Theta3
            void CalLigForceAnta_Pau(); // Calculation of ligaments
            void FOpt(); // Calculation of Optimal Force
            
            //Calculos de dinamica
            void DynamicsHerz_Pau();

            //Calculo Muscular
            void MusForceOpt2_Anta_Coef_Guard_Red();

            // Metodos para escritura en formato JSON (vector de doubles y de enteros, y matriz de doubles)
            void vector2ss(const std::string &name, const Eigen::VectorXd &v, std::stringstream& ss);
            void vector2ss(const std::string &name, const Eigen::VectorXi &v, std::stringstream& ss);
            void matrix2ss(const std::string &name, const Eigen::MatrixXd &m, std::stringstream& ss);

        protected:
            void force_callback(const pr_msgs::msg::PRForceState::SharedPtr force_state_msg);

        private:
            rclcpp::Subscription<pr_msgs::msg::PRForceState>::SharedPtr subscription_;
            rclcpp::Publisher<pr_msgs::msg::PRFloatH>::SharedPtr publisher_gen_force_knee;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_F_opt_ref;

            // PARAMETERS
            // Archivos de lectura y escritura de la struct Data
            std::string cal_data_file, gdlf_data_file;
            std::string output_data_file;
            // Number of samples for the whole experiment
            int num_samples;
            // Recogida de datos de las camaras
            int robot_option, force_sensor_option;
            bool human_option;
            // Musculo que se desea controlar
            std::string musculo_Obj;

            // Datos de entrada
            std::unique_ptr<PRJsonData::PRJsonCal::Calibration_data_struct> cal_data;
            std::unique_ptr<PRJsonData::PRJsonGdlf::Gdlf_data_struct> gdlf_data;
            std::unique_ptr<PRMocap::Mocap> mocap_object;

            // Fuerza y momento medidos
            Eigen::Vector3d force, torque;
            
            // Coactivacion, por ahora lo dejamos a 0
            double per = 0;

            const std::vector<std::string> markers_Nidal{ "LASIS","RASIS","LPSIS","RPSIS","LFE","MFE","FH","LM","MM","CAL","MH1","MH5" };
            const std::vector<std::string> markers_Rizzoli{ "LASIS","RASIS","LPSIS","RPSIS","RLE","RME","RHF","RLM","RMM","RCA","RFM","RVM" };

            // Widths vector
            Vector2d Widths;

            // Rotation matrix of the knee
            PRDataStructures::R_struct R;

            // Struct with info from last iteration
            PRDataStructures::G_r_G_struct G_r_G;

            // Variables L, A, theta0, DELTA from Plane4Bar (A has an extra 0)
            Eigen::Matrix<double, 6, 1> L;
            Eigen::Vector3d A;
            double theta0;
            double DELTA;

            // Variables for Forces and Moments
            Eigen::Matrix<double, 3, Dynamic> G_Fext;
            Eigen::Matrix<double, 3, Dynamic> G_Mext;
            
            // Data struct
            PRDataStructures::Data_struct Data;

            // Piel struct
            PRDataStructures::Piel_struct Piel;

            // This data will come from the force and camera sensors. ForceSensorIn is an auxiliary variable
            // AUXILIAR (to remove) force input struct
            PRDataStructures::AUXILIAR_ForceSensorin_struct AUXILIAR_ForceSensorin; //........................................................................AUXILIAR

            // Vector of force sensor with respect to the global (camera) frame
            Vector3d r_Sensor;
            

            // Initial time and frame
            double Tiempo_0;
            int Frame_0;

            // Bool for first iteration
            bool first_iter = true;

            // Counter for iteration
            int GlobalCnt=0;

            // Variable for hip
            MatrixXd HJC;

            // Vector of positions for DynamicsHerz_Pau
            Matrix<double, 6, 1> q = Matrix<double, 6, 1>::Zero();

            // Structs of angles for DynamicsHerz_Pau
            PRDataStructures::ang_struct ang;

            // Outputs of DynamicsHerz_Pau
            double Tau_TOT_4;
            Matrix<double, 5, 1> Tau_TOT_7;
            Vector3d u_n;

            // Obtenemos el vector director de los musculos
            Vector2i q5ind;
            
            // Posiciones de los angulos auxiliares de la rodilla
            double theta2, theta3, theta4;

            // Salida del calculo muscular
            PRDataStructures::forLigForceCal_struct forLigForceCal;

            // Nombre de los musculos para FOpt
            std::vector<std::string> Flx_name {"BicFemCB","GastLat","GastMed","BicFemCL","SemTend","SemMem","Sat","Gra"};
            std::vector<std::string> Ext_name {"TenFacLat","VasInt123","VasMedInf12","VasMedMed12","VasMedSup34","VasLatSup12","VasInt456","VasLatInf4","RecFem12_1","RecFem12_2"};

            // Fuerza o par generalizado para FOpt
            double Obj;

            // Salida de FOpt
            // Respecto del sistema de camaras global
            Eigen::Vector3d Fext_Opt = Eigen::Vector3d::Zero();
            // Respecto del sistema del sensor. 
            Eigen::Vector3d Fext_Opt_Sensor;
            // Fuerza de referencia (formato PRArrayH)
            Eigen::Vector4d F_Opt_ref;
    
    };
}

#endif // PR_BIOMECH__StreamingGDLF_HPP_