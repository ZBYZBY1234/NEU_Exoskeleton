#include <memory>
#include <fstream>
#include <string>
#include <iostream>
#include <streambuf>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;
using namespace std;

#define Joint_Sensor_Topic  "Sensor_Exoskeleton"
#define Joint_Angle_Topic   "Joint_State_Send"
#define CSV_File_Path       "/home/hemingshan/exo_ws/src/security_module/csv_File/2.csv"
class Joint_Record :
    public rclcpp::Node
{
public:
    Joint_Record()
    : Node("Joint_Record")
    {
        oFile.open(CSV_File_Path, ios::out | ios::trunc);
        oFile   << "Exoskeleton_Left_Thigh (Input)" << "," << "Exoskeleton_Right_Thigh (Input)"<< ","
                << "Exoskeleton_Left_Thigh (Feedback)" << "," << "Exoskeleton_Right_Thigh (Feedback)" << ","
                << "Joint_Error (Left_Thigh)" << "," <<"Joint_Error (Right_Thigh)" << endl;

        /* Define Callback Groups*/
        Joint_Sensor_Callback_Group = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );
        auto Joint_Sensor_Sub_opt   = rclcpp::SubscriptionOptions();
        Joint_Sensor_Sub_opt.callback_group = Joint_Sensor_Callback_Group;
        Joint_Sensor_Subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            Joint_Sensor_Topic,
            rclcpp::QoS(10),
            std::bind(
                &Joint_Record::callback1,
                this,
                _1),
            Joint_Sensor_Sub_opt
        );


        Joint_Angle_Callback_Group = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );
        auto Joint_Angle_Sub_opt   = rclcpp::SubscriptionOptions();
        Joint_Angle_Sub_opt.callback_group = Joint_Angle_Callback_Group;
        Joint_Angle_Subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            Joint_Angle_Topic,
            rclcpp::QoS(10),
            std::bind(
                &Joint_Record::callback2,
                this,
                _1),
            Joint_Angle_Sub_opt
        );
    }
    ~Joint_Record()
    {
        oFile.close();
    }

private:
    /* Sensor Data Accept */
    void callback1(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        auto Angle      = msg->data;
        Sensor_Angle[0] = Angle[0]-90;
        Sensor_Angle[1] = Angle[1]-90;
        Sensor_Angle[2] = Angle[2]-90;
        Sensor_Angle[3] = Angle[3]-90;
        // std::cout<<"Sensor_Angle: "<<Sensor_Angle[0]<<","<<Sensor_Angle[2]<<std::endl;
    }
    /* Angle Publish Data*/
    void callback2(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        auto Angle = msg->data;
        oFile   << Angle[0] << "," << Angle[1] << ","
                << Sensor_Angle[0] << "," << Sensor_Angle[2] <<endl;
    }
    rclcpp::CallbackGroup::SharedPtr                                    Joint_Sensor_Callback_Group;
    rclcpp::CallbackGroup::SharedPtr                                    Joint_Angle_Callback_Group;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   Joint_Sensor_Subscription;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   Joint_Angle_Subscription;

    float                                                               Exoskeleton_Left_Thigh_Angle;
    float                                                               Exoskeleton_Right_Thigh_Angle;

    float                                                               Sensor_Angle[4];
    ofstream                                                            oFile;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Joint_Record>());
    rclcpp::shutdown();
    return 0;
}