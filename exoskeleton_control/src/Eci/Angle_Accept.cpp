/* * @Author: Beal.MS
   * @Date: 2021-04-23 12:41:33
 * @Last Modified by: Beal.MS
 * @Last Modified time: 2021-04-23 12:49:54
   * @Description: This Executable Programm is mainly for the basement of the Motors which can
   * be used to Accept the Motor's Position and Send the Position to the Motors
*/
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "eci/EciDemo113.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

using namespace std::chrono_literals;

extern WORD Get_position[12][64];

BYTE Rec_pos_lower_position[12][8]      = {
    {0x40,0x64,0x60,0x00,0x00,0x00,0x00,0x00},/* read_position */
    {0x40,0x64,0x60,0x00,0x00,0x00,0x00,0x00},
    {0x40,0x64,0x60,0x00,0x00,0x00,0x00,0x00},
    {0x40,0x64,0x60,0x00,0x00,0x00,0x00,0x00},

    {0x40,0x64,0x60,0x00,0x00,0x00,0x00,0x00},/* read_position */
    {0x40,0x64,0x60,0x00,0x00,0x00,0x00,0x00},
    {0x40,0x64,0x60,0x00,0x00,0x00,0x00,0x00},
    {0x40,0x64,0x60,0x00,0x00,0x00,0x00,0x00},

    {0x40,0x64,0x60,0x00,0x00,0x00,0x00,0x00},/* read_position */
    {0x40,0x64,0x60,0x00,0x00,0x00,0x00,0x00},
    {0x40,0x64,0x60,0x00,0x00,0x00,0x00,0x00},
    {0x40,0x64,0x60,0x00,0x00,0x00,0x00,0x00}
};
BYTE TX_Active_Move[12][8] = {
    {0x00,0X01,0x00,0x00,0x00,0x00,0x00,0x00},/* ACTIVE */
    {0x00,0X01,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0X01,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0X01,0x00,0x00,0x00,0x00,0x00,0x00},

    {0x22,0x60,0x60,0x00,0x01,0x00,0x00,0x00},/* PPM */
    {0x22,0x60,0x60,0x00,0x01,0x00,0x00,0x00},
    {0x22,0x60,0x60,0x00,0x01,0x00,0x00,0x00},
    {0x22,0x60,0x60,0x00,0x01,0x00,0x00,0x00},

    {0x22,0x40,0x60,0x00,0x06,0x00,0x00,0x00},/* DIASBALE */
    {0x22,0x40,0x60,0x00,0x06,0x00,0x00,0x00},
    {0x22,0x40,0x60,0x00,0x06,0x00,0x00,0x00},
    {0x22,0x40,0x60,0x00,0x06,0x00,0x00,0x00}
};

BYTE TX_pos_upper_follow_[12][8] = {
    {0x22,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},/* ENABLE */
    {0x22,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},
    {0x22,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},
    {0x22,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},

    {0x22,0x7A,0x60,0x00,0x00,0x00,0x00,0x00},
    {0x22,0x7A,0x60,0x00,0x00,0x00,0x00,0x00},
    {0x22,0x7A,0x60,0x00,0x00,0x00,0x00,0x00},
    {0x22,0x7A,0x60,0x00,0x00,0x00,0x00,0x00},

    {0x22,0x40,0x60,0x00,0x3F,0x00,0x00,0x00},/* MOVE */
    {0x22,0x40,0x60,0x00,0x3F,0x00,0x00,0x00},
    {0x22,0x40,0x60,0x00,0x3F,0x00,0x00,0x00},
    {0x22,0x40,0x60,0x00,0x3F,0x00,0x00,0x00}
};
DWORD Move_lower_motorID[12]   = {0x605,0x607,0x606,0x608,0x605,0x607,0x606,0x608,0x605,0x607,0x606,0x608};
ECI_RESULT hResult;

/*
    *@Description: The Angle_Accept Class for Accepting the Position of Motors
    * And Send the result of Position to the topic "Joint_State_Accept"
*/
class Angle_Accept : public rclcpp::Node
{
public:
    Angle_Accept()
    : Node("Angle_Accept"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("Joint_State_Accept", 10);

        auto timer_callback =[this]() -> void {
            auto message = std_msgs::msg::Float64MultiArray();

            Can_Rx_Position( hResult, Rec_pos_lower_position, Move_lower_motorID);
            //OS_Sleep(10);
            if(Get_position[0][0] == 0 && Get_position[1][0] == 0 && Get_position[2][0] == 0 && Get_position[3][0] == 0)
            {
                //OS_Sleep(1);
                Can_Rx_Position( hResult, Rec_pos_lower_position, Move_lower_motorID);
            }
            for(i = 0;i < 4;++i)
            {
                for(j = 0;j < 4;++j)
                {
                    v|=((unsigned int)Get_position[i][j]&0xFFu)<<(j*8);
                }
                angle[i] = 360 * v /1638400;
                v = 0;
            }
            //OS_Sleep(50);
            message.data = {angle[0],angle[1],angle[2],angle[3]};
            publisher_->publish(message);
        };

        timer_ = this->create_wall_timer(20ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    size_t count_;

    int i;
    int j;
    int v;
    float angle[4];
};

/*
    *@Description: The Angle_Send Class which is mainly for Sending the Motor Position which is got by
    * The topic "Joint_State_Send"
*/
class Angle_Send : public rclcpp::Node
{
public:
    Angle_Send()
    : Node("Angle_Send")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "Joint_State_Send", 10, std::bind(&Angle_Send::topic_callback, this, std::placeholders::_1)
        );
    }

private:
    void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        auto angle = msg->data;
        for (int i = 0; i < 4; i++)
        {
            motor_angle[i] = angle[i];
            motor_qc[i] = 1638400/360*motor_angle[i];
            TX_pos_upper_follow_[4][i+4] = (-motor_qc[0]>>(8*i)&0xff);
            TX_pos_upper_follow_[5][i+4] = ( motor_qc[1]>>(8*i)&0xff);
            TX_pos_upper_follow_[6][i+4] = (-motor_qc[2]>>(8*i)&0xff);
            TX_pos_upper_follow_[7][i+4] = ( motor_qc[3]>>(8*i)&0xff);
        }
        //Motive
        Can_Tx_Data( hResult, TX_pos_upper_follow_, Move_lower_motorID);
        Can_Tx_Data( hResult, TX_pos_upper_follow_, Move_lower_motorID);
    }
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;


    float motor_angle[4];
    int motor_qc[4];
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    hResult = ECI_OK;
    hResult = EciDemo113();
    Can_Tx_Data( hResult, TX_Active_Move, Move_lower_motorID);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto Angle_Accept_Node = std::make_shared<Angle_Accept>();
    auto Angle_Send_Node = std::make_shared<Angle_Send>();
    executor.add_node(Angle_Send_Node);
    executor.add_node(Angle_Accept_Node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

