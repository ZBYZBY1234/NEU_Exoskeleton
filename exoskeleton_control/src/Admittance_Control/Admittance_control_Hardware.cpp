/* * @Author: Beal.MS
   * @Date: 2021-04-21 10:55:36
 * @Last Modified by: Beal.MS
 * @Last Modified time: 2021-04-26 22:19:52
   * @Description: This is for Hardware to control exoskeleton
*/

#include <iostream>
#include "exoskeleton_control/Admittance_control.hpp"
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

Eigen::Matrix<float,1,4> Feedback_Angle_;
Eigen::Matrix<float,1,4> Expected_Angle_;
Eigen::Matrix<float,1,4> Expected_Velocity_;
Eigen::Matrix<float,1,4> Expected_Acceleration_;
Eigen::Matrix<float,6,1> Force_;
bool flag_;

#define Joint_Subscription_Topic                "Joint_State_Accept"
#define Joint_Publisher_Topic                   "Joint_State_Send"
#define Human_Sensor_Subscription_Topic         "Sensor_Human"
#define Exoskeleton_Sensor_Subscription_Topic   "Sensor_Exoskeleton"

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
        // Sensor_Subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        //     Sensor_Subscription_Topic,
        //     10,
        //     std::bind(&Admittance_Control_Subscription::Sensor_Callback, this, _1)
        // );
        /* Callback Group Subscriber Initialization */
        Human_callback_group_subscriber = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );
        Exoskeleton_callback_group_subscriber = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive
        );
        auto Human_sub_opt = rclcpp::SubscriptionOptions();
        Human_sub_opt.callback_group = Human_callback_group_subscriber;
        auto Exoskeleton_sub_opt = rclcpp::SubscriptionOptions();
        Exoskeleton_sub_opt.callback_group = Exoskeleton_callback_group_subscriber;

        Human_Joint_Angle_Subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            Human_Sensor_Subscription_Topic,
            rclcpp::QoS(10),
            std::bind(
                &Admittance_Control_Subscription::Human_Joint_Angle_Callback,
                this,
                std::placeholders::_1
            ),
            Human_sub_opt
        );
        Exoskeleton_Joint_Angle_Subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            Exoskeleton_Sensor_Subscription_Topic,
            rclcpp::QoS(10),
            std::bind(
                &Admittance_Control_Subscription::Exoskeleton_Joint_Angle_Callback,
                this,
                std::placeholders::_1
            ),
            Exoskeleton_sub_opt
        );
        /* Publish Node Initialized */
        Joint_Publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(Joint_Publisher_Topic, 1);
    }

private:

    void Sensor_Callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        auto Sensor_Data = msg->data;

        /* Force/Torque Data */
        //TODO: Force
        Force_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        /*IMU Data Pretreatment*/

        Human_Left_Thigh_Angle = Sensor_Data[3]-90;
        Human_Left_Calf_Angle  = Sensor_Data[6]-90;
        Exoskeleton_Left_Thigh_Angle = Sensor_Data[9]-90;
        Exoskeleton_Left_Calf_Angle  = Sensor_Data[12]-90;

        Feedback_Angle_ << 0.0,
        Exoskeleton_Left_Thigh_Angle/180*PI,
        (Exoskeleton_Left_Calf_Angle - Exoskeleton_Left_Thigh_Angle)/180*PI,
        0.0;

        Expected_Angle_(0,1) = Human_Left_Thigh_Angle/180*PI;
        Expected_Angle_(0,2) = (Human_Left_Calf_Angle - Human_Left_Thigh_Angle)/180*PI;
        if(Expected_Angle_(0,2) < 0)
        {
            Expected_Angle_(0,2) = 0;
        }
        std::cout<<"Feedback_Angle_: "<<Feedback_Angle_<<std::endl;
        std::cout<<"Expected_Angle_: "<<Expected_Angle_<<std::endl;

        /*Calculate the Position for Publish.*/

        Left_Angle = main(
            Feedback_Angle_,
            Expected_Angle_,
            Expected_Velocity_,
            Expected_Acceleration_,
            Force_
            );
        Right_Angle = main(
            Feedback_Angle_,
            Expected_Angle_,
            Expected_Velocity_,
            Expected_Acceleration_,
            Force_
            );

        auto message = std_msgs::msg::Float64MultiArray();
        if (Left_Angle(2,0)<0)
        {
            Left_Angle(2,0) = 0;
        }
        if (Right_Angle(2,0)<0)
        {
            Right_Angle(2,0)=0;
        }
        message.data = {
            Left_Angle(1,0),
            Right_Angle(1,0),
            Left_Angle(2,0),
            Right_Angle(2,0)
        };
        Joint_Publisher->publish(message);

        // std::cout<<"Left: " << Left_Angle(0,0) <<
        // " " << Left_Angle(1,0)*180/PI <<
        // " " << Left_Angle(2,0)*180/PI <<
        // " " << Left_Angle(3,0)  <<
        // " ,Published!!!"<<std::endl;
        // std::cout<<"Right: "<<  Right_Angle(0,0) <<
        // " " << Right_Angle(1,0)*180/PI <<
        // " " << Right_Angle(2,0)*180/PI <<
        // " " << Right_Angle(3,0) <<
        // " ,Published!!!"<<std::endl;
    }

    void Human_Joint_Angle_Callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        auto Sensor_Data = msg->data;
        Human_Left_Thigh_Angle = Sensor_Data[0];
        Human_Left_Calf_Angle  = Sensor_Data[1];
    }
    void Exoskeleton_Joint_Angle_Callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        auto Sensor_Data = msg->data;
        Exoskeleton_Left_Thigh_Angle = Sensor_Data[0];
        Exoskeleton_Left_Calf_Angle = Sensor_Data[1];
    }
private:

    /* Callback Group: Human & Exoskeleton Sensors*/
    rclcpp::CallbackGroup::SharedPtr                                    Human_callback_group_subscriber;
    rclcpp::CallbackGroup::SharedPtr                                    Exoskeleton_callback_group_subscriber;
    /* Subscription Node */
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr Sensor_Subscription;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   Human_Joint_Angle_Subscription;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   Exoskeleton_Joint_Angle_Subscription;

    /* Publisher Node */
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr Joint_Publisher;

    /* Data */
    Eigen::Matrix<float,4,1> Left_Angle, Right_Angle;
    float Force;

    float Human_Left_Thigh_Angle;
    float Human_Left_Calf_Angle;
    float Exoskeleton_Left_Thigh_Angle;
    float Exoskeleton_Left_Calf_Angle;
};

int main(int argc, char * argv[])
{
    /* Data Preinitialized */
    Expected_Angle_ << 0.0,0.0,0.0,0.0;
    Expected_Velocity_ << 0.0,0.0,0.0,0.0;
    Expected_Acceleration_ << 0.0,0.0,0.0,0.0;

    /* ROS Node */
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Admittance_Control_Subscription>());
    rclcpp::shutdown();
    return 0;
}