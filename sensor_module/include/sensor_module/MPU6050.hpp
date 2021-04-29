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

    float           Ang[3];
    float           Vel[3];
    float           Acc[3];

    int             num;
private:
    int Serial_nFd;

    unsigned char Serial_k;
    int Serial_rxflag;
    int Serial_len;

    unsigned char leftknee_recdata[1];
    unsigned char Re_buf[11];

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

    Re_buf[Serial_k] = leftknee_recdata[0];

    if(Serial_k == 0 && Re_buf[0] !=0x55)
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
            if(Re_buf[0] == 0x55)
            {
                if(Re_buf[1]==0x53)
                {

                    angle_x=((short)(Re_buf[3]<<8|Re_buf[2]))/32768.0*180;
                    angle_y=((short)(Re_buf[5]<<8|Re_buf[4]))/32768.0*180;
                    angle_z=((short)(Re_buf[7]<<8|Re_buf[6]))/32768.0*180;
                    // std::cout<<"anglex: "<<angle_x<<std::endl;
                    Eigen::Matrix<float,3,1> angle;
                    angle<<angle_x,angle_y,angle_z;
                    // std::cout<<"Heard"<<std::endl;
                    // std::cout<<"Angle: "<<angle(0,0)<<","<<angle(1,0)<<","<<angle(2,0)<<std::endl;
                    return angle;
                }
            }
        }
    }
}
float * MPU6050::Read_Data()
{
    Serial_len = read(Serial_nFd, leftknee_recdata, 1);
    Re_buf[Serial_k] = leftknee_recdata[0];

    if(Serial_k == 0 && Re_buf[0] !=0x55)
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
            if(Re_buf[0] == 0x55)
            {
                num = 0;
                switch (Re_buf[1])
                {
                case 0x51:
                    Acc[0] = ((short)(Re_buf[3]<<8|Re_buf[2]))/32768.0*16;
                    Acc[1] = ((short)(Re_buf[5]<<8|Re_buf[4]))/32768.0*16;
                    Acc[2] = ((short)(Re_buf[7]<<8|Re_buf[6]))/32768.0*16;
                    break;
                case 0x52:
                    Vel[0] = ((short)(Re_buf[3]<<8|Re_buf[2]))/32768.0*2000;
                    Vel[1] = ((short)(Re_buf[5]<<8|Re_buf[4]))/32768.0*2000;
                    Vel[2] = ((short)(Re_buf[7]<<8|Re_buf[6]))/32768.0*2000;
                    break;
                case 0x53:
                    Ang[0] = ((short)(Re_buf[3]<<8|Re_buf[2]))/32768.0*180;
                    Ang[1] = ((short)(Re_buf[5]<<8|Re_buf[4]))/32768.0*180;
                    Ang[2] = ((short)(Re_buf[7]<<8|Re_buf[6]))/32768.0*180;
                    break;
                }
            }
            float Ang_Vel_Acc[3];
            Ang_Vel_Acc[0] = Ang[0];
            Ang_Vel_Acc[1] = Vel[0];
            Ang_Vel_Acc[2] = Acc[0];
            // std::cout<<Ang[0]<<","<<Vel[0]<<","<<Acc[0]<<std::endl;
            return Ang_Vel_Acc;
        }
    }
}
#endif