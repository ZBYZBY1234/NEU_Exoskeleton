#include <memory>
#include <fstream>
#include <string>
#include <iostream>
#include <streambuf>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;
using namespace std;

#define Exoskeleton_Sensor_Topic          "Sensor_Exoskeleton"
#define Human_Sensor_Topic          "Sensor_Human"
#define Joint_Angle_Topic           "Joint_State_Send"
#define Interaction_Force_Topic     "Interaction_Force"

#define CSV_File_Path       "/home/hemingshan/exo_ws/src/security_module/csv_File/4.csv"
class Joint_Record :
    public rclcpp::Node
{
public:
    Joint_Record()
    : Node("Joint_Record")
    {
        oFile.open(CSV_File_Path, ios::out | ios::trunc);
        oFile   << "Exoskeleton_Left_Thigh (Ang)" << "," << "Exoskeleton_Left_Thigh (Vel)"<< "," <<"Exoskeleton_Left_Thigh (Acc)"<< ","
                << "Exoskeleton_Left_Calf (Ang)" << "," << "Exoskeleton_Left_Calf (Vel)"<< "," <<"Exoskeleton_Left_Calf (Acc)"<< ","
                << "Exoskeleton_Right_Thigh (Ang)" << "," << "Exoskeleton_Right_Thigh (Vel)"<< "," <<"Exoskeleton_Right_Thigh (Acc)"<< ","
                << "Exoskeleton_Right_Calf (Ang)" << "," << "Exoskeleton_Right_Calf (Vel)"<< "," <<"Exoskeleton_Right_Calf (Acc)"<< ","

                << "Human_Left_Thigh (Ang)" << "," << "Human_Left_Thigh (Vel)"<< "," <<"Human_Left_Thigh (Acc)"<< ","
                << "Human_Left_Calf (Ang)" << "," << "Human_Left_Calf (Vel)"<< "," <<"Human_Left_Calf (Acc)"<< ","
                << "Human_Right_Thigh (Ang)" << "," << "Human_Right_Thigh (Vel)"<< "," <<"Human_Right_Thigh (Acc)"<< ","
                << "Human_Right_Calf (Ang)" << "," << "Human_Right_Calf (Vel)"<< "," <<"Human_Right_Calf (Acc)"<< ","

                << "Angle_Driver (Output)"<< endl;

        /* Define Callback Groups*/
        // Sensor_Exoskeleton
        Sensor_Exoskeleton_Callback_Group = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );
        auto Sensor_Exoskeleton_Sub_opt   = rclcpp::SubscriptionOptions();
        Sensor_Exoskeleton_Sub_opt.callback_group = Sensor_Exoskeleton_Callback_Group;
        Sensor_Exoskeleton_SHuman = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            Exoskeleton_Sensor_Topic,
            rclcpp::QoS(10),
            std::bind(
                &Joint_Record::Sensor_Exoskeleton_Cb,
                this,
                _1),
            Sensor_Exoskeleton_Sub_opt
        );
        // Sensor_Human
        Sensor_Human_Callback_Group = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );
        auto Sensor_Human_Sub_opt   = rclcpp::SubscriptionOptions();
        Sensor_Human_Sub_opt.callback_group = Sensor_Human_Callback_Group;
        Sensor_Human_Subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            Human_Sensor_Topic,
            rclcpp::QoS(10),
            std::bind(
                &Joint_Record::Sensor_Human_Cb,
                this,
                _1),
            Sensor_Human_Sub_opt
        );
        // Angle_Driver
        Joint_Angle_Callback_Group = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );
        auto Joint_Angle_Sub_opt   = rclcpp::SubscriptionOptions();
        Joint_Angle_Sub_opt.callback_group = Joint_Angle_Callback_Group;
        Joint_Angle_Subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            Joint_Angle_Topic,
            rclcpp::QoS(10),
            std::bind(
                &Joint_Record::callback3,
                this,
                _1),
            Joint_Angle_Sub_opt
        );
        // Interaction_Force
        Interaction_Force_Callback_Group = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );
        auto Interaction_Force_Sub_opt  = rclcpp::SubscriptionOptions();
        Interaction_Force_Sub_opt.callback_group = Interaction_Force_Callback_Group;
        Interaction_Force_Subscription  = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            Interaction_Force_Topic,
            rclcpp::QoS(10),
            std::bind(
                &Joint_Record::callback2,
                this,
                _1
            ),
            Interaction_Force_Sub_opt
        );
    }
    ~Joint_Record()
    {
        oFile.close();
    }

private:
    /* Sensor Data Accept */
    void Sensor_Exoskeleton_Cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        auto Sensor_Data      = msg->data;
        Sensor_Exoskeleton[0] = Sensor_Data[0];
        Sensor_Exoskeleton[1] = Sensor_Data[1];
        Sensor_Exoskeleton[2] = Sensor_Data[2];
        Sensor_Exoskeleton[3] = Sensor_Data[3];

        Sensor_Exoskeleton[4] = Sensor_Data[4];
        Sensor_Exoskeleton[5] = Sensor_Data[5];
        Sensor_Exoskeleton[6] = Sensor_Data[6];
        Sensor_Exoskeleton[7] = Sensor_Data[7];

        Sensor_Exoskeleton[8] = Sensor_Data[8];
        Sensor_Exoskeleton[9] = Sensor_Data[9];
        Sensor_Exoskeleton[10] = Sensor_Data[10];
        Sensor_Exoskeleton[11] = Sensor_Data[11];
        // std::cout<<"Sensor_Angle: "<<Sensor_Angle[0]<<","<<Sensor_Angle[2]<<std::endl;
    }
    /* Human Sensor Data Accept*/
    void Sensor_Human_Cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        auto Sensor_Data        = msg->data;
        Sensor_Human[0] = Sensor_Data[0];
        Sensor_Human[1] = Sensor_Data[1];
        Sensor_Human[2] = Sensor_Data[2];
        Sensor_Human[3] = Sensor_Data[3];

        Sensor_Human[4] = Sensor_Data[4];
        Sensor_Human[5] = Sensor_Data[5];
        Sensor_Human[6] = Sensor_Data[6];
        Sensor_Human[7] = Sensor_Data[7];

        Sensor_Human[8] = Sensor_Data[8];
        Sensor_Human[9] = Sensor_Data[9];
        Sensor_Human[10] = Sensor_Data[10];
        Sensor_Human[11] = Sensor_Data[11];
    }
    /* Force Data Accept */
    void callback2(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        auto Force = msg->data;
        Force_Left[0]   = Force[0] - Force[1];
        Force_Left[1]   = Force[2] - Force[3];

        Force_Right[0]  = Force[4] - Force[5];
        Force_Right[1]  = Force[6] - Force[7];
    }
    /* Angle Publish Data*/
    void callback3(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        auto Angle = msg->data;
        // oFile   << Angle[0] << "," << Angle[1] << ","
        //         << Sensor_Angle[0] << "," << Sensor_Angle[2] <<","
        //         << Force_Left[0]<<","<<Force_Left[1]<<","
        //         << Force_Right[0]<<","<<Force_Right[1]<<endl;
        oFile   << Sensor_Exoskeleton[0] << "," << Sensor_Exoskeleton[4] << Sensor_Exoskeleton[8] << ","
                << Sensor_Exoskeleton[1] << "," << Sensor_Exoskeleton[5] << Sensor_Exoskeleton[9] << ","
                << Sensor_Exoskeleton[2] << "," << Sensor_Exoskeleton[6] << Sensor_Exoskeleton[10] << ","
                << Sensor_Exoskeleton[3] << "," << Sensor_Exoskeleton[7] << Sensor_Exoskeleton[11] << ","

                << Sensor_Human[0] << "," << Sensor_Human[4] << Sensor_Human[8] << ","
                << Sensor_Human[1] << "," << Sensor_Human[5] << Sensor_Human[9] << ","
                << Sensor_Human[2] << "," << Sensor_Human[6] << Sensor_Human[10] << ","
                << Sensor_Human[3] << "," << Sensor_Human[7] << Sensor_Human[11] << ","

                << Angle[0]<<","<<Angle[1]<<","<<Angle[2]<<","<<Angle[3]<<endl;
    }
    rclcpp::CallbackGroup::SharedPtr                                    Sensor_Exoskeleton_Callback_Group;
    rclcpp::CallbackGroup::SharedPtr                                    Sensor_Human_Callback_Group;
    rclcpp::CallbackGroup::SharedPtr                                    Joint_Angle_Callback_Group;
    rclcpp::CallbackGroup::SharedPtr                                    Interaction_Force_Callback_Group;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   Sensor_Exoskeleton_SHuman;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   Sensor_Human_Subscription;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   Joint_Angle_Subscription;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   Interaction_Force_Subscription;

    float                                                               Exoskeleton_Left_Thigh_Angle;
    float                                                               Exoskeleton_Right_Thigh_Angle;

    float                                                               Sensor_Exoskeleton[12];
    float                                                               Sensor_Human[12];

    float                                                               Force_Left[2];
    float                                                               Force_Right[2];
    ofstream                                                            oFile;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Joint_Record>());
    rclcpp::shutdown();
    return 0;
}