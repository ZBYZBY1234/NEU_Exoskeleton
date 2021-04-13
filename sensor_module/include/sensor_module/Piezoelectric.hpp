#include "sensor_module/Serial.hpp"
#include <eigen3/Eigen/Dense>

#define BAUDRATE B115200 ///Baud rate : 115200
#define DEVICE "/dev/ttyUSB0"//设置你的端口号


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
    int decdata[8];

    int i;

    int forcedata[3];
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
    Eigen::Matrix<float,3,1> Read();
};

Eigen::Matrix<float,3,1> Piezoelectric::Read()
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
                forcedata[0] = decdata[3] * 256 + decdata[2];
                forcedata[1] = decdata[5] * 256 + decdata[4];
                forcedata[2] = decdata[7] * 256 + decdata[6];

                // for (i = 0; i < 3; i++)
                // {
                //     printf("数据=%d\t", forcedata[i]);
                // }
                // printf("\n");
                flag1 = 0;
        }
    }

    sumcondatal = 0;
    sumcondataf = 0;
    condatal[countk] = forcedata[2];
    condataf[countk]=forcedata[0] + forcedata[1];
    countk++;

    for ( i = 0; i < avcount; i++)
    {
        sumcondatal += condatal[i];
        sumcondataf += condataf[i];
    }

    sumcondatalb[countk] = sumcondatal;
    sumcondatafb[countk] = sumcondataf;

    contrustk=countk-3;

    if (contrustk==-1)
    {
        contrustk = 4;
    }
    if (contrustk==-2)
    {
        contrustk = 3;
    }
    if (contrustk==-3)
    {
        contrustk = 2;
    }


    if (( (sumcondatal/avcount) < footlimitl) &&(sumcondatalb[contrustk]>sumcondatalb[countk] ) &&(sumcondatafb[contrustk]<sumcondatafb[countk]) )
    {
        flagout = 1;
    }
    if ((sumcondatal/avcount) > 1.5*footlimitl)
    {
        flagagain = 1;
        flagout = 0;
    }

    if (flagout&&flagagain)
    {
        printf("start>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
        flagout = 0;
        flagagain = 0;
    }


    if (countk==avcount)
        countk = 0;
    if (contrustk==avcount)
        contrustk = 0;
    Eigen::Matrix<float,3,1> data;
    data << forcedata[0], forcedata[1], forcedata[2];
    return data;

}