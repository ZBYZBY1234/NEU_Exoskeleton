/* * @Author: Beal.MS
   * @Date: 2021-04-21 10:55:36
 * @Last Modified by: Beal.MS
 * @Last Modified time: 2021-04-26 22:19:52
   * @Description: This is for Hardware to control exoskeleton
*/

#include <iostream>
#include "exoskeleton_control/Admittance_control_Hardware.hpp"
#include <unistd.h>
#include <math.h>
#include <memory>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

/* Topic Name Definition */
#define Joint_Subscription_Topic                "Joint_State_Accept"
#define Joint_State_Publisher_Topic             "Joint_State_Send"
#define Joint_Error_Publisher_Topic             "Joint_Error"
#define Human_Sensor_Subscription_Topic         "Sensor_Human"
#define Exoskeleton_Sensor_Subscription_Topic   "Sensor_Exoskeleton"
#define Interaction_Force_Subscription_Topic    "Interaction_Force"

#define Joint_Thigh_Offset          -0.7046042369967704
#define JOint_Calf_Offset           -0.7046042369967704+0.4744191163880517

class Admittance_Control_Subscription :
    public rclcpp::Node,
    public Admittance_control
{
public:
    Admittance_Control_Subscription()
    : Node("Admittance_Control_Subscription"),Admittance_control()
    {
        /* Subscription Node Initialized */
        /* Callback Group Subscriber Initialization */
        Human_callback_group_subscriber = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );
        Exoskeleton_callback_group_subscriber = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );
        Interaction_force_callback_group_subscriber = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );
        auto Human_sub_opt = rclcpp::SubscriptionOptions();
        Human_sub_opt.callback_group = Human_callback_group_subscriber;
        auto Exoskeleton_sub_opt = rclcpp::SubscriptionOptions();
        Exoskeleton_sub_opt.callback_group = Exoskeleton_callback_group_subscriber;
        auto Interaction_sub_opt = rclcpp::SubscriptionOptions();
        Interaction_sub_opt.callback_group = Interaction_force_callback_group_subscriber;

        Human_Joint_State_Subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            Human_Sensor_Subscription_Topic,
            rclcpp::QoS(2),
            std::bind(
                &Admittance_Control_Subscription::Human_Joint_State_Callback,
                this,
                std::placeholders::_1
            ),
            Human_sub_opt
        );
        Exoskeleton_Joint_State_Subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            Exoskeleton_Sensor_Subscription_Topic,
            rclcpp::QoS(2),
            std::bind(
                &Admittance_Control_Subscription::Exoskeleton_Joint_State_Callback,
                this,
                std::placeholders::_1
            ),
            Exoskeleton_sub_opt
        );
        Interaction_Force_Subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            Interaction_Force_Subscription_Topic,
            rclcpp::QoS(2),
            std::bind(
                &Admittance_Control_Subscription::Interaction_Force_Callback,
                this,
                std::placeholders::_1
            ),
            Interaction_sub_opt
        );
        /* Publish Node Initialized */
        Joint_State_Publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(Joint_State_Publisher_Topic, 1);
        Joint_Error_Publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(Joint_Error_Publisher_Topic, 1);

        /* Data Preinitialized */
        Expected_Angle_Left << 0.0,0.0,0.0,0.0;
        Expected_Velocity_Left << 0.0,0.0,0.0,0.0;
        Expected_Acceleration_Left << 0.0,0.0,0.0,0.0;

        Expected_Angle_Right << 0.0,0.0,0.0,0.0;
        Expected_Velocity_Right << 0.0,0.0,0.0,0.0;
        Expected_Acceleration_Right << 0.0,0.0,0.0,0.0;

        Force_Left << 0.0, 0.0, 0.0, 0.0;
        Force_Right << 0.0, 0.0, 0.0, 0.0;

        Thigh_Front = 1.0;
        Thigh_Back  = 1.0;
        Calf_Front  = 1.0;
        Calf_Back   = 1.0;
    }

private:
    /* Human Joint Angle Callback*/

    void Human_Joint_State_Callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        auto Sensor_Data = msg->data;
        /* Sensor Data Initialization*/
        // Angle Data
        float Human_Left_Thigh_Angle            = Sensor_Data[0]-90;
        float Human_Left_Calf_Angle             = Sensor_Data[1]-90;
        float Human_Right_Thigh_Angle           = Sensor_Data[2]-90;
        float Human_Right_Calf_Angle            = Sensor_Data[3]-90;
        // Velocity Data
        float Human_Left_Thigh_Velocity         = Sensor_Data[4];
        float Human_Left_Calf_Velocity          = Sensor_Data[5];
        float Human_Right_Thigh_Velocity        = Sensor_Data[6];
        float Human_Right_Calf_Velocity         = Sensor_Data[7];
        // Acceleration Data
        float Human_Left_Thigh_Acceleration     = Sensor_Data[8];
        float Human_Left_Calf_Acceleration      = Sensor_Data[9];
        float Human_Right_Thigh_Acceleration    = Sensor_Data[10];
        float Human_Right_Calf_Acceleration     = Sensor_Data[11];

        /* Force/Torque Data */
        //TODO: Force
        // Force_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;


        /* Sensor Data Pre-Treatment*/
        // Angle Data
        Expected_Angle_Left(0,1) = Human_Left_Thigh_Angle/180*PI;
        Expected_Angle_Left(0,2) = (Human_Left_Calf_Angle - Human_Left_Thigh_Angle)/180*PI;
        if(Expected_Angle_Left(0,2) < 0)
        {
            Expected_Angle_Left(0,2) = 0;
        }
        Expected_Angle_Right(0,1) = Human_Right_Thigh_Angle/180*PI;
        Expected_Angle_Right(0,2) = (Human_Right_Calf_Angle - Human_Right_Thigh_Angle)/180*PI;
        if(Expected_Angle_Right(0,2) < 0)
        {
            Expected_Angle_Right(0,2) = 0;
        }
        // Velocity Data
        Expected_Velocity_Left << 0.0,
        Human_Left_Thigh_Velocity,
        Human_Left_Calf_Velocity,
        0.0;
        Expected_Velocity_Right << 0.0,
        Human_Right_Thigh_Velocity,
        Human_Right_Calf_Velocity,
        0.0;
        // Acceleration Data
        Expected_Acceleration_Left << 0.0,
        Human_Left_Thigh_Acceleration,
        Human_Left_Calf_Acceleration,
        0.0;
        Expected_Acceleration_Right << 0.0,
        Human_Right_Thigh_Acceleration,
        Human_Right_Calf_Acceleration,
        0.0;

        /*Calculate the Position for Publish.*/
        /*
         *@Name: main Function
         *@Input: Feedback_Angle, Feedback_Velocity, Feedback_Acceleration
         *        Expected_Angle, Expected_Velocity, Expected_Acceleration
         *        Force
         */

        /* Please choose one of Mode or none*/
        // Force Test
        // Expected_Angle_Left << 0.0,0.0,0.0,0.0;
        // Expected_Velocity_Left << 0.0,0.0,0.0,0.0;
        // Expected_Acceleration_Left << 0.0,0.0,0.0,0.0;

        // Feedback_Angle_Left << 0.0,0.0,0.0,0.0;
        // Feedback_Velocity_Left << 0.0,0.0,0.0,0.0;
        // Feedback_Acceleration_Left << 0.0,0.0,0.0,0.0;

        // Expected_Angle_Right << 0.0,0.0,0.0,0.0;
        // Expected_Velocity_Right << 0.0,0.0,0.0,0.0;
        // Expected_Acceleration_Right << 0.0,0.0,0.0,0.0;

        // Feedback_Angle_Right << 0.0,0.0,0.0,0.0;
        // Feedback_Velocity_Right << 0.0,0.0,0.0,0.0;
        // Feedback_Acceleration_Right << 0.0,0.0,0.0,0.0;

        //Angle Test
        Force_Left  << 0.0, 0.0, 0.0, 0.0;
        Force_Right << 0.0, 0.0, 0.0, 0.0;

        Left_Angle = main(
            Feedback_Angle_Left,
            Feedback_Velocity_Left,
            Feedback_Acceleration_Left,
            Expected_Angle_Left,
            Expected_Velocity_Left,
            Expected_Acceleration_Left,
            Force_Left
            );
        Right_Angle = main(
            Feedback_Angle_Right,
            Feedback_Velocity_Left,
            Feedback_Acceleration_Left,
            Expected_Angle_Right,
            Expected_Velocity_Right,
            Expected_Acceleration_Right,
            Force_Right
            );

        auto message = std_msgs::msg::Float64MultiArray();
        auto error_message = std_msgs::msg::Float64MultiArray();
        if (Left_Angle(2,0)<0)
        {
            Left_Angle(2,0) = 0;
        }
        if (Right_Angle(2,0)<0)
        {
            Right_Angle(2,0)=0;
        }
        message.data = {
            Left_Angle(1,0)/PI*180,
            Right_Angle(1,0)/PI*180,
            Left_Angle(2,0)/PI*180,
            Right_Angle(2,0)/PI*180
        };
        error_message.data = {
            0.0
        };

        /* Publish */
        Joint_State_Publisher->publish(message);
        Joint_Error_Publisher->publish(error_message);
    }

    void Exoskeleton_Joint_State_Callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        auto Sensor_Data = msg->data;

        float Exoskeleton_Left_Thigh_Angle;
        float Exoskeleton_Left_Calf_Angle;
        float Exoskeleton_Right_Thigh_Angle;
        float Exoskeleton_Right_Calf_Angle;

        float Exoskeleton_Left_Thigh_Velocity;
        float Exoskeleton_Left_Calf_Velocity;
        float Exoskeleton_Right_Thigh_Velocity;
        float Exoskeleton_Right_Calf_Velocity;

        float Exoskeleton_Left_Thigh_Acceleration;
        float Exoskeleton_Left_Calf_Acceleration;
        float Exoskeleton_Right_Thigh_Acceleration;
        float Exoskeleton_Right_Calf_Acceleration;

        /* Sensor Data Initialization*/
        // Angle Data

        Exoskeleton_Left_Thigh_Angle    = Sensor_Data[0];
        Exoskeleton_Left_Calf_Angle     = Sensor_Data[1];
        Exoskeleton_Right_Thigh_Angle   = Sensor_Data[0];
        Exoskeleton_Right_Calf_Angle    = Sensor_Data[1];

        // Velocity Data
        Exoskeleton_Left_Thigh_Velocity     = Sensor_Data[4];
        Exoskeleton_Left_Calf_Velocity      = Sensor_Data[5];
        Exoskeleton_Right_Thigh_Velocity    = Sensor_Data[6];
        Exoskeleton_Right_Calf_Velocity     = Sensor_Data[7];

        // Acceleration Data
        Exoskeleton_Left_Thigh_Acceleration   = Sensor_Data[8];
        Exoskeleton_Left_Calf_Acceleration    = Sensor_Data[9];
        Exoskeleton_Right_Thigh_Acceleration  = Sensor_Data[10];
        Exoskeleton_Right_Calf_Acceleration   = Sensor_Data[11];

        /* Sensor Data Pre-Treatment*/
        // Angle Data
        Feedback_Angle_Left << 0.0,
        Exoskeleton_Left_Thigh_Angle/180*PI,
        (Exoskeleton_Left_Calf_Angle - Exoskeleton_Left_Thigh_Angle)/180*PI,
        0.0;
        Feedback_Angle_Right << 0.0,
        Exoskeleton_Right_Thigh_Angle/180*PI,
        (Exoskeleton_Right_Calf_Angle - Exoskeleton_Right_Thigh_Angle)/180*PI,
        0.0;

        // Velocity Data
        Feedback_Velocity_Left << 0.0,
        Exoskeleton_Left_Thigh_Velocity,
        Exoskeleton_Left_Calf_Velocity,
        0.0;
        Feedback_Velocity_Right << 0.0,
        Exoskeleton_Right_Thigh_Velocity,
        Exoskeleton_Right_Calf_Velocity,
        0.0;

        // Acceleration Data
        Feedback_Acceleration_Left << 0.0,
        Exoskeleton_Left_Thigh_Acceleration,
        Exoskeleton_Left_Calf_Acceleration,
        0.0;
        Feedback_Acceleration_Right << 0.0,
        Exoskeleton_Right_Thigh_Acceleration,
        Exoskeleton_Right_Calf_Acceleration,
        0.0;
    }

    /* Interaction Force Initialization */
    void Interaction_Force_Callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        auto Interaction_Force = msg->data;
        Force_Left[0] = 0.0;
        Force_Left[1] = (   Interaction_Force[0]*Thigh_Front -
                            Interaction_Force[1]*Thigh_Back
                        )/180*PI*0.1;
        Force_Left[2] = (   Interaction_Force[2]*Calf_Front -
                            Interaction_Force[3]*Calf_Back
                        )/180*PI*0.1;
        Force_Left[3] = 0.0;

        Force_Right[0] = 0.0;
        Force_Right[1] = (  Interaction_Force[4]*Thigh_Front -
                            Interaction_Force[5]*Thigh_Back
                        )/180*PI*0.1;
        Force_Right[2] = (  Interaction_Force[6]*Calf_Front -
                            Interaction_Force[7]*Calf_Back
                        )/180*PI*0.1;
        Force_Right[3] = 0.0;
    }
private:

    /* Callback Group: Human & Exoskeleton Sensors*/
    rclcpp::CallbackGroup::SharedPtr                                    Human_callback_group_subscriber;
    rclcpp::CallbackGroup::SharedPtr                                    Exoskeleton_callback_group_subscriber;
    rclcpp::CallbackGroup::SharedPtr                                    Interaction_force_callback_group_subscriber;

    /* Subscription Node */
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   Human_Joint_State_Subscription;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   Exoskeleton_Joint_State_Subscription;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   Interaction_Force_Subscription;

    /* Publisher Node */
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr      Joint_State_Publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr      Joint_Error_Publisher;

    /* Data */
    Eigen::Matrix<float,4,1> Left_Angle, Right_Angle;

    Eigen::Matrix<float,1,4> Feedback_Angle_Left;
    Eigen::Matrix<float,1,4> Feedback_Angle_Right;
    Eigen::Matrix<float,1,4> Feedback_Velocity_Left;
    Eigen::Matrix<float,1,4> Feedback_Velocity_Right;
    Eigen::Matrix<float,1,4> Feedback_Acceleration_Left;
    Eigen::Matrix<float,1,4> Feedback_Acceleration_Right;

    Eigen::Matrix<float,1,4> Expected_Angle_Left;
    Eigen::Matrix<float,1,4> Expected_Angle_Right;
    Eigen::Matrix<float,1,4> Expected_Velocity_Left;
    Eigen::Matrix<float,1,4> Expected_Velocity_Right;
    Eigen::Matrix<float,1,4> Expected_Acceleration_Left;
    Eigen::Matrix<float,1,4> Expected_Acceleration_Right;

    Eigen::Matrix<float,4,1> Force_Left;
    Eigen::Matrix<float,4,1> Force_Right;

    float                    Thigh_Front;
    float                    Thigh_Back;
    float                    Calf_Front;
    float                    Calf_Back;

};

int main(int argc, char * argv[])
{
    /* ROS Node */
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Admittance_Control_Subscription>());
    rclcpp::shutdown();
    return 0;
}