#ifndef PR_BIOMECH__StreamingGDLF_HPP_
#define PR_BIOMECH__StreamingGDLF_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pr_msgs/msg/pr_force_state.hpp"
#include "pr_msgs/msg/pr_float_h.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"

#include "pr_lib_biomech/pr_json_calibration.hpp"
#include "pr_lib_biomech/pr_json_gdlf.hpp"
#include "pr_lib_biomech/pr_mocap.hpp"
#include "pr_lib_biomech/pr_data_structures.hpp"
#include "pr_lib_biomech/pr_algebra_fun.hpp"



#include "eigen3/Eigen/Dense"

// JSON writer from rapidjson library
#include "filewritestream.h"
#include "writer.h"

#include <chrono>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <iostream>
#include <memory>
#include <math.h>



namespace pr_biomech
{
    class StreamingGDLF : public rclcpp::Node
    {
        public:
            // PUBLIC_CONSTRUCTOR
            explicit StreamingGDLF(const rclcpp::NodeOptions & options);
            // DESTRUCTOR
            ~StreamingGDLF();
            //void auxiliar_initialization(); //..............................................................................................................auxiliar
            void AUXILIAR_initialization();
            void initialization(); //Memory allocation of matrices according to num_samples
            void PelCalcs(); // Calculation of rotation matrices of the hip
            //Vector3d Cross(Vector3d P1, Vector3d P2); // Funcion que calcula el producto vectorial
            //void CalConst_Pau2(); // Calculation of Subject Central System (SCS) of the model segments
            void CalTheta2Theta3(); // Calculation of Theta2 and Theta3
            //void Denavit(Matrix<double, 8, 1> a, Matrix<double, 8, 1> alpha, Matrix<double, 8, 1> theta, Matrix<double, 8, 1> d, int dof);
            void Denavit(Matrix<double, 8, 1> a, Matrix<double, 8, 1> alpha, Matrix<double, 8, 1> theta, Matrix<double, 8, 1> d, int dof, Vector3d O0, Matrix<double, 3, Dynamic>& i_1_r_Oi_1_Oi, std::vector<std::vector<Eigen::Vector3d>>& i_r_Oi_Oj, std::vector<Eigen::Matrix3d>& i_1_R_i, std::vector<std::vector<Eigen::Matrix3d>>& i_R_j);
            //void Interpol(double x, Vector <double, 2> X, Matrix <double, Dynamic, 2> Y, Vector <double, Dynamic> y);
            Matrix <double, Dynamic, 1>  Interpol(double x, Matrix <double, 2, 1> X, Matrix <double, Dynamic, 2> Y);
            Matrix <double, 6, 11>  Interpol_3D(double x, Matrix <double, 2, 1> X, Eigen::Matrix <double, 6, 11> y1, Eigen::Matrix <double, 6, 11> y2);
            void FindQ(Matrix <double, Dynamic, 1> Q5, double q5, Matrix <int, 2, 1>& n_q5);
            //Eigen::Vector<double, 8> Dirt_Kinematics(PRDataStructures::R_struct& R, PRDataStructures::G_r_G_struct& G_r_G);
            void Seg_Kinematics();
            void Inv_Kinematics();
            void Dir_Kinematics();
            void Inv_Dynamics();
            void MusForce();
            void CalcKneeForces();
            Vector4d VectDir();
            //void CalLigForceAnta_Pau(); // Calculation of ligaments
            //
            ////Calculos de dinamica
            //void DynamicsHerz_Pau();

            ///* METODOS  DEFINIDOS POR JOSE */
            ////Calculo Muscular
            //void MusForceOpt2_Anta_Coef_Guard_Red();

            //// Metodos para escritura en formato JSON (vector de doubles y de enteros, y matriz de doubles)
            void vector2ss(const std::string &name, const Eigen::VectorXd &v, std::stringstream& ss);
            void vector2ss(const std::string &name, const Eigen::VectorXi &v, std::stringstream& ss);
            void matrix2ss(const std::string &name, const Eigen::MatrixXd &m, std::stringstream& ss);

        protected:
            void force_callback(const pr_msgs::msg::PRForceState::SharedPtr force_state_msg);

        private:

            rclcpp::Subscription<pr_msgs::msg::PRForceState>::SharedPtr subscription_;
            rclcpp::Publisher<pr_msgs::msg::PRFloatH>::SharedPtr pub_gen_force_knee;
            rclcpp::Publisher<pr_msgs::msg::PRFloatH>::SharedPtr pub_muscle_force;
            rclcpp::Publisher<pr_msgs::msg::PRFloatH>::SharedPtr pub_ref_muscle_force;
            rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr pub_muscle_dir;
            //rclcpp::Publisher<pr_msgs::msg::PRArrayH>::SharedPtr publisher_F_opt_ref;

            // Variables to publish
            double gen_force_knee, muscle_force, ref_muscle_force;
            Eigen::Vector4d muscle_dir;
            
            // PARAMETERS
            // Archivos de lectura y escritura de la struct Data
            std::string cal_data_file, gdlf_data_file;
            std::string output_data_file;
            // Number of samples for the whole experiment
            int num_samples;
            // Recogida de datos de las camaras
            int robot_option, force_sensor_option;
            bool human_option;

            // Datos de entrada
            std::unique_ptr<PRJsonData::PRJsonCal::Calibration_data_struct> cal_data;
            std::unique_ptr<PRJsonData::PRJsonGdlf::Gdlf_data_struct> gdlf_data;
            std::unique_ptr<PRMocap::Mocap> mocap_object;

            // Fuerza y momento medidos
            Eigen::Vector3d force, torque;

        //    // Coactivacion, por ahora lo dejamos a 0
        //    double per = 0;

        //    const std::vector<std::string> markers_Nidal{ "LASIS","RASIS","LPSIS","RPSIS","LFE","MFE","FH","LM","MM","CAL","MH1","MH5" };
        //    const std::vector<std::string> markers_Rizzoli{ "LASIS","RASIS","LPSIS","RPSIS","RLE","RME","RHF","RLM","RMM","RCA","RFM","RVM" };

            // Declare Cal variables
            Eigen::Matrix <double,8,4> DH_parameters;
            Eigen::Vector3d ACSpelvis_r_PelAvg_ACSpelvis;
            PRJsonData::PRJsonCal::Plane4Bar_struct Plane4Bar;
            PRJsonData::PRJsonCal::r_Local_struct r_Local;
            PRJsonData::PRJsonCal::R_local_struct R_local;

            // Rotation matrix of the knee
            PRDataStructures::R_struct R;

            // Struct with info from last iteration
            PRDataStructures::G_r_G_struct G_r_G;

            //Variables para la funcion Denavit
            Matrix<double, 3, Dynamic> i_1_r_Oi_1_Oi;
            std::vector<std::vector<Eigen::Vector3d>> i_r_Oi_Oj;
            std::vector<Eigen::Matrix3d> i_1_R_i;
            std::vector<std::vector<Eigen::Matrix3d>> i_R_j;
            Vector3d O0;
            Eigen::Matrix <double, 8, 1> q;
            int dof;
            // Angulos mecanismo 4 barras
            //double theta2, theta3, theta4;
            //double theta1, theta2, theta3, theta6, theta7, q_temp = 0;
            double theta1, theta2, theta3, theta4, theta6, theta7, q_temp = 0;
            // Longitud desde 3 hasta IC
            double L_O3_IC;
            // Fuerza generalizada
            Matrix <double, 6, 1> Tau;
            // Fuerza externa (Fuerza + Momento)
            Matrix <double, 6, 1> F_Ext;

            // Piel struct
            PRDataStructures::Piel_struct Piel;

        //    // This data will come from the force and camera sensors. ForceSensorIn is an auxiliary variable
        //    // AUXILIAR (to remove) force input struct
            PRDataStructures::AUXILIAR_ForceSensorin_struct AUXILIAR_ForceSensorin; //........................................................................AUXILIAR

        //    // Vector of force sensor with respect to the global (camera) frame
        //    Vector3d r_Sensor;


        //    // Initial time and frame
        //    int Tiempo_0, Frame_0;

        //    // Bool for first iteration
        //    bool first_iter = true;

        //    // Variable for hip
        //    MatrixXd HJC;

        //    // Vector of positions for DynamicsHerz_Pau
        //    Matrix<double, 6, 1> q = Matrix<double, 6, 1>::Zero();

        //    // Structs of angles for DynamicsHerz_Pau
        //    PRDataStructures::ang_struct ang;

        //    // Outputs of DynamicsHerz_Pau
        //    double Tau_TOT_4;
        //    Matrix<double, 5, 1> Tau_TOT_7;
        //    Vector3d u_n;

        //    // Obtenemos el vector director de los musculos
        //    Vector2i q5ind;
        //    
            // Parámetros de DH
            Eigen::Matrix<double, 8, 1 > d;
            Eigen::Matrix<double, 8, 1 > theta;
            Eigen::Matrix<double, 8, 1 > alpha;
            Eigen::Matrix<double, 8, 1 > a;

            // Posicion n_q5
            Eigen::Matrix <int, 2, 1> n_q5;
            Eigen::Matrix <double, 2, 1> q5_int;
            // Coeficientes interpolación
            Eigen::Matrix <double, 6, 1> TauGrav;
            Eigen::Matrix <double, 73, 1> CoefMus1;
            Eigen::Matrix <double, 73, 1> CoefMus2;
            Eigen::Matrix <double, 73, 1> CoefMus3;
            Eigen::Matrix <double, 73, 1> CoefMus4;
            Eigen::Matrix <double, 73, 1> CoefMus5;
            Eigen::Matrix <double, 73, 1> CoefMus6;
            Eigen::Matrix <double, 9, 1>  PatCoef;
            Eigen::Matrix <double, 6, 11> KneeCoef;
            Eigen::Matrix <double, 3, 11> KneeCoef_Force;
            Eigen::Matrix <double, 3, 11> KneeCoef_Torque;
            // Vector Tau
            Eigen::Matrix<double, 6, 1> TauMus;
            //Vector de las fuerzas musculares
            Eigen::Matrix <double, 73, 1> MuscleForce;
            // Fuerza en el ligamento patelar
            double PatForce;
            // Posición de la rodilla
            double PosKnee;
            // Fuerza en la rodilla
            Eigen::Vector3d F_Knee;
            Eigen::Vector3d M_Knee;
            // Posiciones musculares rodilla
            Eigen::Matrix <int, 10, 1> MusPos; //{ 1, 0, 2, 3, 4, 67, 7, 8, 9, 10 };
            // Vectores de 3 componentes auxiliares
            Eigen::Vector3d rF;
            Eigen::Vector3d F;


        //    // Salida del calculo muscular
        //    PRDataStructures::forLigForceCal_struct forLigForceCal;

            //bool nombre;
            // Initial time and frame
            double Tiempo_0;
            int Frame_0;

            // Bool for first iteration
            bool first_iter = true;

            // Counter for iteration
            int GlobalCnt=0;

            // Data struct
            PRDataStructures::Data_struct Data;

            // Variables for Forces and Moments
            Eigen::Vector3d G_Fext;
            Eigen::Vector3d G_Mext;

            // Musculo objetivo de control
            int n_mus;

            // Length of tibia and foot
            double length_tibia, length_foot;

            // Unit vector of muscle
            //Eigen::Vector4d u_m;
    };

}

#endif // PR_BIOMECH__StreamingGDLF_HPP_