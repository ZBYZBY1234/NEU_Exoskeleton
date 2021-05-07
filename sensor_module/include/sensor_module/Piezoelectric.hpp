#ifndef PIEZOELECTRIC_H
#define PIEZOELECTRIC_H
#include "sensor_module/Serial.hpp"
#include <eigen3/Eigen/Dense>

#define BAUDRATE B115200 ///Baud rate : 115200
#define DEVICE "/dev/ttyUSB2"//设置你的端口号


#define footlimitl 400  //设置足底压力启动值
#define footlimitf 300
#define avcount 5

class Piezoelectric : public Serial
{
private:
    int Serial_nFd;

    int Serial_len;
    unsigned char hexdata[1];
    int datamid;
    int countk;
    int count;
    int decdata[10];

    int i;

    int forcedata[8];
    int flag1;
    int flagout;
    int flagagain;
    int contrustk;

    int condatal[avcount];
    int sumcondatal, sumcondatalb[avcount];
    int condataf[avcount];
    int sumcondataf, sumcondatafb[avcount];
public:
    Piezoelectric(char * Device_, int Baudrate_)
    {
        Serial serial;
        Serial_nFd = serial.Init(Device_, Baudrate_);
        countk = 0;
        count=0;
        flag1=0;
        flagout = 0;
        flagagain =0;
        contrustk=0;

        condatal[avcount] = {0};
        condataf[avcount] = {0};
    };
    ~Piezoelectric(){};
public:
    Eigen::Matrix<float,8,1> Read_8_ADC();
    Eigen::Matrix<float,6,1> Read_6_ADC();
    int * Read_Data();
};

Eigen::Matrix<float,8,1> Piezoelectric::Read_8_ADC()
{
    Serial_len = read(Serial_nFd, hexdata, 1);

    datamid = (int)hexdata[0];
    if(datamid==254)
    {
        count = 0;
        decdata[count] = datamid;
        count++;

        Serial_len = read(Serial_nFd, hexdata, 1);
        datamid = (int)hexdata[0];

        if (datamid==255)
        {
            decdata[count] = datamid;
            for (i = 2; i < 10; i++)
            {
                count++;
                Serial_len = read(Serial_nFd, hexdata, 1);
                datamid = (int)hexdata[0];
                decdata[count] = datamid;
            }
                forcedata[0] = decdata[2];
                forcedata[1] = decdata[3];
                forcedata[2] = decdata[4];
                forcedata[3] = decdata[5];

                forcedata[4] = decdata[6];
                forcedata[5] = decdata[7];
                forcedata[6] = decdata[8];
                forcedata[7] = decdata[9];

                // for (i = 0; i < 4; i++)
                // {
                //     printf("数据=%d\t", forcedata[i]);
                // }
                // printf("\n");
                flag1 = 0;
        }
    }
    Eigen::Matrix<float,8,1> data;
    data << forcedata[0], forcedata[1], forcedata[2], forcedata[3],
            forcedata[4], forcedata[5], forcedata[6], forcedata[7];
    return data;

}

Eigen::Matrix<float,6,1> Piezoelectric::Read_6_ADC()
{
    Serial_len = read(Serial_nFd, hexdata, 1);

    datamid = (int)hexdata[0];
    if(datamid==254)
    {
        count = 0;
        decdata[count] = datamid;
        count++;

        Serial_len = read(Serial_nFd, hexdata, 1);
        datamid = (int)hexdata[0];

        if (datamid==255)
        {
            decdata[count] = datamid;
            for (i = 2; i < 8; i++)
            {
                count++;
                Serial_len = read(Serial_nFd, hexdata, 1);
                datamid = (int)hexdata[0];
                decdata[count] = datamid;
            }
                forcedata[0] = decdata[2];
                forcedata[1] = decdata[3];
                forcedata[2] = decdata[4];

                forcedata[3] = decdata[5];
                forcedata[4] = decdata[6];
                forcedata[5] = decdata[7];

                flag1 = 0;
        }
    }
    Eigen::Matrix<float,6,1> data;
    data << forcedata[0], forcedata[1], forcedata[2],
            forcedata[3], forcedata[4], forcedata[5];
    return data;

}

int * Piezoelectric::Read_Data()
{
    Serial_len = read(Serial_nFd, hexdata, 1);

    datamid = (int)hexdata[0];
    if(datamid==254)
    {
        count = 0;
        decdata[count] = datamid;
        count++;

        Serial_len = read(Serial_nFd, hexdata, 1);
        datamid = (int)hexdata[0];

        if (datamid==255)
        {
            decdata[count] = datamid;
            for (i = 2; i < 5; i++)
            {
                count++;
                Serial_len = read(Serial_nFd, hexdata, 1);
                datamid = (int)hexdata[0];
                decdata[count] = datamid;
            }
                forcedata[0] = decdata[2];
                forcedata[1] = decdata[3];
                forcedata[2] = decdata[4];
                forcedata[3] = decdata[5];

                forcedata[4] = decdata[6];
                forcedata[5] = decdata[7];
                forcedata[6] = decdata[8];
                forcedata[7] = decdata[9];
                // for (i = 0; i < 3; i++)
                // {
                //     printf("数据=%d\t", forcedata[i]);
                // }
                // printf("\n");
                flag1 = 0;
        }
    }
    return forcedata;
}

#endif