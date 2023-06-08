#ifndef PR_LIB__JSON_DATA_
#define PR_LIB__JSON_DATA_

#include "document.h"     // rapidjson's DOM-style API
#include <cstdio>
#include <iostream>
#include <vector>
#include "eigen3/Eigen/Dense"
#include <fstream>

using namespace rapidjson;

namespace PRJsonData{

    struct Data_struct{

        // correct_reading remains true if all readings were right
        bool correct_reading = true;
        // Vector of non correct readings
        std::vector<std::string> non_correct;
        // JSON document
        Document document;

        // Constructor. Receives a path and extracts the JSON content
        Data_struct(std::string path);

        // Prints all the data
        void print_data();

        // Generic function that reads from a rapidjson document or value (substructure) and a generic var
        template<typename D, typename T>
        void read_member(const D &doc, const std::string member, T &var);

        // Function overloading if the var is int
        template<typename D>
        void assign_value(const D &doc, const Value::ConstMemberIterator &itr, int &var);

        // Function overloading if the var is double
        template<typename D>
        void assign_value(const D &doc, const Value::ConstMemberIterator &itr, double &var);

        template<typename D>
        void assign_value(const D &doc, const Value::ConstMemberIterator &itr, std::string &var);

        // Function overloading if the var is a Eigen Matrix
        template<typename D, typename Derived>
        void assign_value(const D &doc, const Value::ConstMemberIterator &itr, Eigen::MatrixBase<Derived> &var);

        // Function overloading if the var is a vector of double Eigen Matrices
        template<typename D, int rows, int cols>
        void assign_value(const D& doc, const Value::ConstMemberIterator& itr, std::vector<Eigen::Matrix<double, rows, cols>>& var);

        // Function overload if the Eigen matrix is of integers
        void assign_value(int &var, const Value &val);

        // Function overload if the Eigen matrix is of doubles
        void assign_value(double &var, const Value &val);

        // Extracts substructures from the document (or another substructure) and places in a Value variable
        template<typename D>
        const Value& struct_value(const D &doc, const std::string member);

    };

    // Constructor. Receives a path and extracts the JSON content
    inline Data_struct::Data_struct(std::string path){
        // Complete JSON string
        std::string json_string;
        std::ifstream file(path);
        if (file.is_open()){
            // JSON is encoded in just one line
            getline(file, json_string);
            //std::cout << json_string << std::endl;
            file.close();
            // document parses all JSON content
            if (document.Parse(json_string.c_str()).HasParseError()){
                std::cout << "Error while parsing JSON string " << std::endl;
                correct_reading = false;
                throw std::string("Error while parsing JSON string");
            }
        }
    }

    // Prints all the data
    inline void Data_struct::print_data() {

        std::cout << "correct_reading: " << correct_reading << std::endl;
        std::cout << "non correct: ";
        for (int i = 0; i < non_correct.size(); i++) std::cout << non_correct[i] << " ";
        std::cout << "\n";
    };

    // Generic function that reads from a rapidjson document or value (substructure) and a generic var
    template<typename D, typename T>
    inline void Data_struct::read_member(const D &doc, const std::string member, T &var){
        Value::ConstMemberIterator itr = doc.FindMember(member.c_str());
        if (itr != doc.MemberEnd()){ 
            assign_value(doc, itr, var);
        }
        else{ 
            correct_reading = false;
            non_correct.push_back(member);
        }    
    }

    // Function overloading if the var is int
    template<typename D>
    inline void Data_struct::assign_value(const D &doc, const Value::ConstMemberIterator &itr, int &var){
        var = itr->value.GetInt();
    }

    // Function overloading if the var is double
    template<typename D>
    inline void Data_struct::assign_value(const D &doc, const Value::ConstMemberIterator &itr, double &var){
        var = itr->value.GetDouble();
    }

    template<typename D>
    inline void Data_struct::assign_value(const D &doc, const Value::ConstMemberIterator &itr, std::string &var){
        var = itr->value.GetString();
    }

    // Function overloading if the var is a Eigen Matrix
    template<typename D, typename Derived>
    inline void Data_struct::assign_value(const D &doc, const Value::ConstMemberIterator &itr, Eigen::MatrixBase<Derived> &var){
        const Value &vec = itr->value.GetArray();
        int dim1 = vec.Size();
        if (vec[0].IsArray()){
            int dim2 = vec[0].GetArray().Size();
            if (var.rows()==0 && var.cols()==0){
                var.derived().resize(dim1,dim2);
            }
            for (int i=0; i<dim1; i++){
                for (int j=0; j<dim2; j++){
                    assign_value(var(i,j), vec[i].GetArray()[j]);
                }
            }
        }
        else{
            if (var.size() == 0){ var.derived().resize(dim1,1);//resize_matrix(var,dim1);
            }
            for (int i=0; i<dim1; i++){
                assign_value(var(i), vec[i]);
            }  
        }

    }

    // Function overloading if the var is a vector of double Eigen Matrices
    template<typename D, int rows, int cols>
    inline void Data_struct::assign_value(const D& doc, const Value::ConstMemberIterator& itr, std::vector<Eigen::Matrix<double, rows, cols>>& var) {
        const Value& vec = itr->value.GetArray();
        int dim1 = itr->value.Size();
        const Value& vec2 = vec[0].GetArray();
        int dim2 = vec2.Size();
        const Value& vec3 = vec2[0].GetArray();
        int dim3 = vec3.Size();

        assert(dim2 == rows && dim3 == cols);

        Eigen::Matrix<double, rows, cols> mat;

        for (int i = 0; i < dim1; i++) {
            for (int j = 0; j < dim2; j++) {
                for (int k = 0; k < dim3; k++) {
                    assign_value(mat(j, k), vec[i].GetArray()[j].GetArray()[k]);
                }
            }
            var.push_back(mat);
        }
    }

    // Function overload if the Eigen matrix is of integers
    inline void Data_struct::assign_value(int &var, const Value &val){
        var = val.GetInt();
    }

    // Function overload if the Eigen matrix is of doubles
    inline void Data_struct::assign_value(double &var, const Value &val){
        var = val.GetDouble();
    }

    // Extracts substructures from the document (or another substructure) and places in a Value variable
    template<typename D>
    inline const Value& Data_struct::struct_value(const D &doc, const std::string member){ 
        Value::ConstMemberIterator itr = doc.FindMember(member.c_str());
        if (itr == doc.MemberEnd()){  
            correct_reading = false;
        }
        const Value& val = doc[member.c_str()];
        return val;
    }

    

}

#endif // PR_LIB__JSON_DATA_