#include "eci/Motor.hpp"


///////////////////////////////////////////////////////////////////////////////////////////////////

int   main()
{
    DWORD Active_MotorID[12]  = {0x00,0x00,0x00,0x00,0x605,0x607,0x606,0x608,0x605,0x607,0x606,0x608};
    Motor_Position motor_position(Active_MotorID);
    motor_position.PositionControl();
}