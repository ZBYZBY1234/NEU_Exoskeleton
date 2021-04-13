#ifndef MPU6050_H
#define MPU6050_H
#include "sensor_module/Serial.hpp"
#include <eigen3/Eigen/Dense>

class MPU6050 : public Serial
{
public:
    float angle_x;
    float angle_y;
    float angle_z;
private:
    int Serial_nFd;

    unsigned char Serial_k;
    int Serial_rxflag;
    int Serial_len;

    unsigned char leftknee_recdata[1];
    unsigned char leftknee_data[11];

    char * Name;
public:
    MPU6050(char * Device_, int Baudrate_, char * Name_);
    ~MPU6050(){};
public:
    Eigen::Matrix<float,3,1> Read();
    float * Read_Data();
};

MPU6050::MPU6050(char * Device_, int Baudrate_, char * Name_) : Serial()
{
    // Serial serial;
    // Serial_nFd = serial.Init(Device_, Baudrate_);
    Serial_nFd = this->Init(Device_,Baudrate_);

    Serial_k = 0;
    Serial_rxflag = 1;
    Serial_len=0;

    Name = Name_;
}

Eigen::Matrix<float,3,1> MPU6050::Read()
{
    Serial_len = read(Serial_nFd, leftknee_recdata, 1);

    leftknee_data[Serial_k] = leftknee_recdata[0];

    if(Serial_k == 0 && leftknee_data[0] !=0x55)
    {
        Serial_rxflag = 0;
        Serial_k = 0;
    }
    if(Serial_rxflag)
    {
        Serial_k++;
        if(Serial_k == 11)
        {
            Serial_k = 0;
            if(leftknee_data[0] == 0x55)
            {
                if(leftknee_data[1]==0x53)
                {

                    angle_x=((short)(leftknee_data[3]<<8|leftknee_data[2]))/32768.0*180;
                    angle_y=((short)(leftknee_data[5]<<8|leftknee_data[4]))/32768.0*180;
                    angle_z=((short)(leftknee_data[7]<<8|leftknee_data[6]))/32768.0*180;
                    // std::cout<<"anglex: "<<angle_x<<std::endl;
                    Eigen::Matrix<float,3,1> angle;
                    angle<<angle_x,angle_y,angle_z;
                    // std::cout<<"Heard"<<std::endl;
                    std::cout<<"Angle: "<<angle(0,0)<<","<<angle(1,0)<<","<<angle(2,0)<<std::endl;
                    return angle;
                }
            }
        }
    }
}
float * MPU6050::Read_Data()
{

    Serial_len = read(Serial_nFd, leftknee_recdata, 1);

    leftknee_data[Serial_k] = leftknee_recdata[0];

    if(Serial_k == 0 && leftknee_data[0] !=0x55)
    {
        Serial_rxflag = 0;
        Serial_k = 0;
    }
    if(Serial_rxflag)
    {
        Serial_k++;
        if(Serial_k == 11)
        {
            Serial_k = 0;
            if(leftknee_data[0] == 0x55)
            {
                if(leftknee_data[1]==0x53)
                {

                    angle_x=((short)(leftknee_data[3]<<8|leftknee_data[2]))/32768.0*180;
                    angle_y=((short)(leftknee_data[5]<<8|leftknee_data[4]))/32768.0*180;
                    angle_z=((short)(leftknee_data[7]<<8|leftknee_data[6]))/32768.0*180;

                    float angle[3] = {angle_x,angle_y,angle_z};
                    std::cout<<"Angle: "<<angle_x<<","<<angle_y<<","<<angle_z<<std::endl;
                    return angle;
                }
            }
        }
    }
}
#endif