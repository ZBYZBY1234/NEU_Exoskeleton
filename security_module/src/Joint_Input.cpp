#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#define Joint_Input_Topic   "Joint_State_Send"
#define Timer_Config        100ms

using namespace std::chrono_literals;

class Joint_Input : public rclcpp::Node
{
public:
    Joint_Input()
    : Node("Joint Inpput")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>
        (
            Joint_Input_Topic, 10
        );
        timer_     = this->create_wall_timer(
            Timer_Config,
            std::bind(
                &Joint_Input::timer_callback, this
            )
        );
        Exoskeleton_Left_Thigh_Angle    = 0;
        Exoskeleton_Right_Thigh_Angle   = 0;
        Exoskeleton_Left_Calf_Angle     = 0;
        Exoskeleton_Right_Calf_Angle    = 0;
    }
private:
    void timer_callback()
    {
        auto Joint_State = std_msgs::msg::Float64MultiArray();
        Joint_State.data[0] = Exoskeleton_Left_Thigh_Angle;
        Joint_State.data[1] = Exoskeleton_Right_Thigh_Angle;
        Joint_State.data[2] = Exoskeleton_Left_Calf_Angle;
        Joint_State.data[3] = Exoskeleton_Right_Calf_Angle;
        if(Exoskeleton_Left_Thigh_Angle> -30)
        {
            Exoskeleton_Left_Thigh_Angle += 1;
        }
        if(Exoskeleton_Right_Thigh_Angle> -30)
        {
            Exoskeleton_Right_Thigh_Angle += 1;
        }

        publisher_->publish(Joint_State);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

    float               Exoskeleton_Left_Thigh_Angle;
    float               Exoskeleton_Right_Thigh_Angle;
    float               Exoskeleton_Left_Calf_Angle;
    float               Exoskeleton_Right_Calf_Angle;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Joint_Input>());
    rclcpp::shutdown();
    return 0;
}