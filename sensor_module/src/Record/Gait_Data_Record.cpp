#include <memory>
#include <fstream>
#include <string>
#include <iostream>
#include <streambuf>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;
using namespace std;

#define Human_Sensor_Topic          "Sensor_Human"
#define Interaction_Force_Topic     "Plantar_Pressure"

#define CSV_File_Path       "/home/hemingshan/exo_ws/src/sensor_module/csv_File/1.csv"
class Joint_Record :
    public rclcpp::Node
{
public:
    Joint_Record()
    : Node("Joint_Record")
    {
        oFile.open(CSV_File_Path, ios::out | ios::trunc);
        oFile   << "Human_Left_Thigh (Ang)"  << "," << "Human_Left_Thigh (Vel)" << "," << "Human_Left_Thigh (Acc)"  << ","
                << "Human_Left_Calf (Ang)"   << "," << "Human_Left_Calf (Vel)"  << "," << "Human_Left_Calf (Acc)"   << ","
                << "Human_Right_Thigh (Ang)" << "," << "Human_Right_Thigh (Vel)"<< "," << "Human_Right_Thigh (Acc)" << ","
                << "Human_Right_Calf (Ang)"  << "," << "Human_Right_Calf (Vel)" << "," << "Human_Right_Calf (Acc)"  << ","

                << "Left_Plantar_Pressure (1)" << "," << "Left_Plantar_Pressure (2)" << "," << "Left_Plantar_Pressure (3)"<<","
                << "Right_Plantar_Pressure (1)"<< "," << "Right_Plantar_Pressure (2)"<< "," << "Right_Plantar_Pressure (3)"<<endl;

        /* Define Callback Groups*/
        // Sensor_Human
        Sensor_Human_Callback_Group = this->create_callback_group
        (
            rclcpp::CallbackGroupType::MutuallyExclusive
        );

        auto Sensor_Human_Sub_opt   = rclcpp::SubscriptionOptions();
        Sensor_Human_Sub_opt.callback_group = Sensor_Human_Callback_Group;
        Sensor_Human_Subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>
        (
            Human_Sensor_Topic,
            rclcpp::QoS(10),
            std::bind(
                &Joint_Record::Sensor_Human_Cb,
                this,
                _1),
            Sensor_Human_Sub_opt
        );

        // Interaction_Force
        Interaction_Force_Callback_Group = this->create_callback_group
        (
            rclcpp::CallbackGroupType::MutuallyExclusive
        );

        auto Interaction_Force_Sub_opt  = rclcpp::SubscriptionOptions();
        Interaction_Force_Sub_opt.callback_group = Interaction_Force_Callback_Group;
        Interaction_Force_Subscription  = this->create_subscription<std_msgs::msg::Float64MultiArray>
        (
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
        Left_Plantar_Pressure[0]   = Force[0];
        Left_Plantar_Pressure[1]   = Force[1];
        Left_Plantar_Pressure[2]   = Force[2];

        Right_Plantar_Pressure[0]   = Force[3];
        Right_Plantar_Pressure[1]   = Force[4];
        Right_Plantar_Pressure[2]   = Force[5];
    }
    /* Angle Publish Data*/
    void callback3(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        auto Angle = msg->data;
        // oFile   << Angle[0] << "," << Angle[1] << ","
        //         << Sensor_Angle[0] << "," << Sensor_Angle[2] <<","
        //         << Force_Left[0]<<","<<Force_Left[1]<<","
        //         << Force_Right[0]<<","<<Force_Right[1]<<endl;
        oFile   << Sensor_Human[0] << "," << Sensor_Human[4] << "," << Sensor_Human[8] << ","
                << Sensor_Human[1] << "," << Sensor_Human[5] << "," << Sensor_Human[9] << ","
                << Sensor_Human[2] << "," << Sensor_Human[6] << "," << Sensor_Human[10] << ","
                << Sensor_Human[3] << "," << Sensor_Human[7] << "," << Sensor_Human[11] << ","

                << Left_Plantar_Pressure[0]  << "," << Left_Plantar_Pressure[1]  << "," << Left_Plantar_Pressure[2]  <<","
                << Right_Plantar_Pressure[0] << "," << Right_Plantar_Pressure[1] << "," << Right_Plantar_Pressure[2] <<endl;
        // oFile   <<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","
        //         <<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","
        //         <<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","
        //         <<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","
        //         <<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<1<<","<<endl;
    }
    rclcpp::CallbackGroup::SharedPtr                                    Sensor_Human_Callback_Group;
    rclcpp::CallbackGroup::SharedPtr                                    Interaction_Force_Callback_Group;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   Sensor_Human_Subscription;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   Interaction_Force_Subscription;

    float                                                               Exoskeleton_Left_Thigh_Angle;
    float                                                               Exoskeleton_Right_Thigh_Angle;

    float                                                               Sensor_Human[12];

    float                                                               Left_Plantar_Pressure[3];
    float                                                               Right_Plantar_Pressure[3];
    ofstream                                                            oFile;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Joint_Record>());
    rclcpp::shutdown();
    return 0;
}