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
float angle[4];

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
DWORD Move_lower_motorID[12]   = {0x605,0x607,0x606,0x608,0x605,0x607,0x606,0x608,0x605,0x607,0x606,0x608};

class Angle_Accept : public rclcpp::Node
{
public:
    Angle_Accept()
    : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "topic", 10
        );
        timer_ = this->create_wall_timer(
            500ms, std::bind(&Angle_Accept::timer_callback, this)
        );
        hResult = ECI_OK;
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::Float64MultiArray();
        Can_Rx_Position( hResult, Rec_pos_lower_position, Move_lower_motorID);
        OS_Sleep(10);
        if(Get_position[0][0] == 0 && Get_position[1][0] == 0 && Get_position[2][0] == 0 && Get_position[3][0] == 0)
        {
            OS_Sleep(1);
            Can_Rx_Position( hResult, Rec_pos_lower_position, Move_lower_motorID);
        }
        for(i = 0;i < 4;++i)
        {
            //printf("%d:  ",i);
            for(j = 0;j < 4;++j)
            {
                v|=((unsigned int)Get_position[i][j]&0xFFu)<<(j*8);
                //printf("%02x ",Get_position[i][j]);
            }
            //printf("%d:  ",v);
            angle[i] = 360 * v /1638400;
            v = 0;
            //printf("\n");
        }
        for(i = 0;i < 4;++i)
            printf("%.2f\n",angle[i]);
        OS_Sleep(50);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    size_t count_;

    ECI_RESULT hResult;
    int i;
    int j;
    int v;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Angle_Accept>());
    rclcpp::shutdown();
    return 0;
}