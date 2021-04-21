/* * @Author: Beal.MS
   * @Date: 2021-03-20 12:01:47
 * @Last Modified by: Beal.MS
 * @Last Modified time: 2021-03-20 23:32:53
   * @Description: Admittance Control
   * @input: Excepted_Angle, Excepted_Velosity, Force
   * @output: Position_Angle
*/
#include <iostream>
#include "exoskeleton_control/Admittance_control.hpp"
#include <unistd.h>

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

class Admittance_Control_Subscription :
    public rclcpp::Node,
    public Admittance_control
{
public:
    Admittance_Control_Subscription()
    : Node("minimal_subscriber"),Admittance_control()
    {
        /*Subscription Node Initialized*/
        Joint_Subscription = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&Admittance_Control_Subscription::Joint_Callback, this, _1)
        );

        Force_Subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "Piezoelectric", 10, std::bind(&Admittance_Control_Subscription::callback2, this, _1)
        );

        Angle_Subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "MPU6050_Thigh", 10, std::bind(&Admittance_Control_Subscription::Angle_Callback, this, _1)
        );
        Joint_Publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("topic", 1);
    }

private:

    void Joint_Callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        auto position = msg->position;
        //Feedback Joint Message
        RCLCPP_INFO(this->get_logger(),
        "Left: '%f','%f','%f','%f' Right: '%f','%f','%f','%f'",
        -position[0],-position[1],-position[2],-position[3],
        -position[4],-position[5],-position[6],-position[7]);

        Feedback_Angle_ << -position[0], -position[1], -position[2], -position[3];
        Expected_Angle_(0,1) = (Angle_Thigh/180)*PI;
        Force_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

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
        message.data = {  -Left_Angle(0,0),  -Left_Angle(1,0),  -Left_Angle(2,0),  -Left_Angle(3,0),
                        -Right_Angle(0,0), -Right_Angle(1,0), -Right_Angle(2,0), -Right_Angle(3,0) };

        Joint_Publisher->publish(message);

      //延迟函数
      // sleep(1);

        std::cout<<"Left: " << -Left_Angle(0,0)  << -Left_Angle(1,0)  << -Left_Angle(2,0)  << -Left_Angle(3,0)  <<" ,Published!!!"<<std::endl;
        std::cout<<"Right: "<< -Right_Angle(0,0) << -Right_Angle(1,0) << -Right_Angle(2,0) << -Right_Angle(3,0) <<" ,Published!!!"<<std::endl;
    }

    void callback2(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        auto force = msg->data;
        RCLCPP_INFO(this->get_logger(), "Force: '%f','%f','%f'",
        force[0], force[1], force[2]);
        Force = force[0];
    }

    void Angle_Callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        auto Angle = msg->data;
        RCLCPP_INFO(this->get_logger(), "Angle: '%f','%f','%f'",
        Angle[0], Angle[1], Angle[2]);
        Angle_Thigh = Angle[0];
    }
private:
    /*Subscription Node*/
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr Joint_Subscription;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr Force_Subscription;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr Angle_Subscription;

    /*Publisher Node*/
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr Joint_Publisher;

    /*Data*/
    Eigen::Matrix<float,4,1> Left_Angle, Right_Angle;
    float Force;
    float Angle_Thigh;
};

int main(int argc, char * argv[])
{

    Expected_Angle_ << 0.0,0.0,0.0,0.0;
    Expected_Velocity_ << 0.0,0.0,0.0,0.0;
    Expected_Acceleration_ << 0.0,0.0,0.0,0.0;

    /*For Test*/

  // Force_ << 0.0, -10000.0, 0.0, 0.0, 0.0, 0.0;
  // Admittance_control admittance_control();
  // float a,b,c,d;
  // for (int i = 0; i < 5; i++)
  // {
  //   std::cin>>a>>b>>c>>d;
  //   admittance_control.main(PI*a,PI*b,PI*c,PI*d, Force_);
  // }

    /*ROS Node*/

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Admittance_Control_Subscription>());
    rclcpp::shutdown();
    return 0;
}