#include "powered_exoskeleton/EciDemo113.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>


#define BAUDRATE B115200 ///Baud rate : 115200
#define BAUDRATE B115200 ///Baud rate : 115200
#define BAUDRATE B115200 ///Baud rate : 115200
#define BAUDRATE B115200 ///Baud rate : 115200
#define LKDEVICE "/dev/ttyUSB0"//设置你的端口号
#define LHDEVICE "/dev/ttyUSB1"//设置你的端口号
#define RKDEVICE "/dev/ttyUSB2"//设置你的端口号
#define RHDEVICE "/dev/ttyUSB3"//设置你的端口号

//////////////////////////////////////////485收到数据转16进制定义
int leftknee_nFd = 0;
int lefthip_nFd = 0;
int rightknee_nFd = 0;
int righthip_nFd = 0;

struct termios leftknee_stNew;
struct termios lefthip_stNew;
struct termios rightknee_stNew;
struct termios righthip_stNew;
struct termios leftknee_stOld;
struct termios lefthip_stOld;
struct termios rightknee_stOld;
struct termios righthip_stOld;

float leftknee_anglex;
float lefthip_anglex;
float rightknee_anglex;
float righthip_anglex;

float leftknee_angle,lefthip_angle,rightknee_angle,righthip_angle;

float left_knee_anglex;
float left_hip_anglex;
float right_knee_anglex;
float right_hip_anglex;



char lefthip_buffer[33];
char leftknee_buffer[33];
char righthip_buffer[33];
char rightknee_buffer[33];
//////////////////////////////////////////485收到数据转16进制定义
double num = 0.0;
double PI = 3.14159265;
int a, b;

/////////////////////////////////////////////////////////////////////////////////////
////////////////////////485串口接受激光传感器数据
/*
    函数功能：串口初始化
*/
int LKSerialInit()
{
    leftknee_nFd = open(LKDEVICE, O_RDWR|O_NOCTTY|O_NDELAY);   //打开串口USB0
    if(-1 == leftknee_nFd)
    {
        perror("leftknee:Open Serial Port Error!\n");
        return -1;
    }
    if( (fcntl(leftknee_nFd, F_SETFL, 0)) < 0 )
    {
        perror("leftknee:Fcntl F_SETFL Error!\n");
        return -1;
    }
    if(tcgetattr(leftknee_nFd, &leftknee_stOld) != 0)
    {
        perror("leftknee:tcgetattr error!\n");
        return -1;
    }

    leftknee_stNew = leftknee_stOld;
    cfmakeraw(&leftknee_stNew);//将终端设置为原始模式，该模式下全部的输入数据以字节为单位被处理
    /**1. tcgetattr函数用于获取与终端相关的参数。
    *参数fd为终端的文件描述符，返回的结果保存在termios结构体中
    */
    //set speed
    cfsetispeed(&leftknee_stNew, BAUDRATE);   //设置波特率115200s
    //set databits
    leftknee_stNew.c_cflag |= (CLOCAL|CREAD); //设置控制模式状态，本地连接，接收使能
    leftknee_stNew.c_cflag &= ~CSIZE;         //字符长度，设置数据位之前一定要屏掉这个位
    leftknee_stNew.c_cflag |= CS8;            //8位数据长度
    //set parity
    leftknee_stNew.c_cflag &= ~PARENB;
    leftknee_stNew.c_iflag &= ~INPCK;         //无奇偶检验位
    //set stopbits
    leftknee_stNew.c_cflag &= ~CSTOPB;        //1位停止位

    //stNew.c_oflag = 0; //输出模式
    //stNew.c_lflag = 0; //不激活终端模式

    leftknee_stNew.c_cc[VTIME]=0;             //指定所要读取字符的最小数量
    leftknee_stNew.c_cc[VMIN]=1;              //指定读取第一个字符的等待时间，时间的单位为n*100ms
    //假设设置VTIME=0，则无字符输入时read（）操作无限期的堵塞
    /**3. 设置新属性，TCSANOW：所有改变立即生效*/
    tcflush(leftknee_nFd,TCIFLUSH);           //清空终端未完毕的输入/输出请求及数据。

    if( tcsetattr(leftknee_nFd,TCSANOW,&leftknee_stNew) != 0 )
    {
        perror("leftknee:tcsetattr Error!\n");
        return -1;
    }
    return leftknee_nFd;
}

int LHSerialInit()
{
    lefthip_nFd = open(LHDEVICE, O_RDWR|O_NOCTTY|O_NDELAY);   //打开串口USB1
    if(-1 == lefthip_nFd)
    {
        perror("lefthip:Open Serial Port Error!\n");
        return -1;
    }
    if( (fcntl(lefthip_nFd, F_SETFL, 0)) < 0 )
    {
        perror("lefthip:Fcntl F_SETFL Error!\n");
        return -1;
    }
    if(tcgetattr(lefthip_nFd, &lefthip_stOld) != 0)
    {
        perror("lefthip:tcgetattr error!\n");
        return -1;
    }

    lefthip_stNew = lefthip_stOld;
    cfmakeraw(&lefthip_stNew);//将终端设置为原始模式，该模式下全部的输入数据以字节为单位被处理
    /**1. tcgetattr函数用于获取与终端相关的参数。
    *参数fd为终端的文件描述符，返回的结果保存在termios结构体中
    */
    //set speed
    cfsetispeed(&lefthip_stNew, BAUDRATE);   //设置波特率115200s
    //set databits
    lefthip_stNew.c_cflag |= (CLOCAL|CREAD); //设置控制模式状态，本地连接，接收使能
    lefthip_stNew.c_cflag &= ~CSIZE;         //字符长度，设置数据位之前一定要屏掉这个位
    lefthip_stNew.c_cflag |= CS8;            //8位数据长度
    //set parity
    lefthip_stNew.c_cflag &= ~PARENB;
    lefthip_stNew.c_iflag &= ~INPCK;         //无奇偶检验位
    //set stopbits
    lefthip_stNew.c_cflag &= ~CSTOPB;        //1位停止位

    //stNew.c_oflag = 0; //输出模式
    //stNew.c_lflag = 0; //不激活终端模式

    lefthip_stNew.c_cc[VTIME]=0;             //指定所要读取字符的最小数量
    lefthip_stNew.c_cc[VMIN]=1;              //指定读取第一个字符的等待时间，时间的单位为n*100ms
    //假设设置VTIME=0，则无字符输入时read（）操作无限期的堵塞
    /**3. 设置新属性，TCSANOW：所有改变立即生效*/
    tcflush(lefthip_nFd,TCIFLUSH);           //清空终端未完毕的输入/输出请求及数据。

    if( tcsetattr(lefthip_nFd,TCSANOW,&lefthip_stNew) != 0 )
    {
        perror("lefthip:tcsetattr Error!\n");
        return -1;
    }
    return lefthip_nFd;
}
int RKSerialInit()
{
   rightknee_nFd = open(RKDEVICE, O_RDWR|O_NOCTTY|O_NDELAY);   //打开串口USB0
    if(-1 == rightknee_nFd)
    {
        perror("rightknee:Open Serial Port Error!\n");
        return -1;
    }
    if( (fcntl(rightknee_nFd, F_SETFL, 0)) < 0 )
    {
        perror("rightknee:Fcntl F_SETFL Error!\n");
        return -1;
    }
    if(tcgetattr(rightknee_nFd, &rightknee_stOld) != 0)
    {
        perror("rightknee:tcgetattr error!\n");
        return -1;
    }

    rightknee_stNew = rightknee_stOld;
    cfmakeraw(&rightknee_stNew);//将终端设置为原始模式，该模式下全部的输入数据以字节为单位被处理
    /**1. tcgetattr函数用于获取与终端相关的参数。
    *参数fd为终端的文件描述符，返回的结果保存在termios结构体中
    */
    //set speed
    cfsetispeed(&rightknee_stNew, BAUDRATE);   //设置波特率115200s
    //set databits
    rightknee_stNew.c_cflag |= (CLOCAL|CREAD); //设置控制模式状态，本地连接，接收使能
    rightknee_stNew.c_cflag &= ~CSIZE;         //字符长度，设置数据位之前一定要屏掉这个位
    rightknee_stNew.c_cflag |= CS8;            //8位数据长度
    //set parity
    rightknee_stNew.c_cflag &= ~PARENB;
    rightknee_stNew.c_iflag &= ~INPCK;         //无奇偶检验位
    //set stopbits
    rightknee_stNew.c_cflag &= ~CSTOPB;        //1位停止位

    //stNew.c_oflag = 0; //输出模式
    //stNew.c_lflag = 0; //不激活终端模式

    rightknee_stNew.c_cc[VTIME]=0;             //指定所要读取字符的最小数量
    rightknee_stNew.c_cc[VMIN]=1;              //指定读取第一个字符的等待时间，时间的单位为n*100ms
    //假设设置VTIME=0，则无字符输入时read（）操作无限期的堵塞
    /**3. 设置新属性，TCSANOW：所有改变立即生效*/
    tcflush(rightknee_nFd,TCIFLUSH);           //清空终端未完毕的输入/输出请求及数据。

    if( tcsetattr(rightknee_nFd,TCSANOW,&rightknee_stNew) != 0 )
    {
        perror("rightknee:tcsetattr Error!\n");
        return -1;
    }
    return rightknee_nFd;
}

int RHSerialInit()
{
    righthip_nFd = open(RHDEVICE, O_RDWR|O_NOCTTY|O_NDELAY);   //打开串口USB0
    if(-1 == righthip_nFd)
    {
        perror("righthip:Open Serial Port Error!\n");
        return -1;
    }
    if( (fcntl(righthip_nFd, F_SETFL, 0)) < 0 )
    {
        perror("righthip:Fcntl F_SETFL Error!\n");
        return -1;
    }
    if(tcgetattr(righthip_nFd, &righthip_stOld) != 0)
    {
        perror("righthip:tcgetattr error!\n");
        return -1;
    }

    righthip_stNew = righthip_stOld;
    cfmakeraw(&righthip_stNew);//将终端设置为原始模式，该模式下全部的输入数据以字节为单位被处理
    /**1. tcgetattr函数用于获取与终端相关的参数。
    *参数fd为终端的文件描述符，返回的结果保存在termios结构体中
    */
    //set speed
    cfsetispeed(&righthip_stNew, BAUDRATE);   //设置波特率115200s
    //set databits
    righthip_stNew.c_cflag |= (CLOCAL|CREAD); //设置控制模式状态，本地连接，接收使能
    righthip_stNew.c_cflag &= ~CSIZE;         //字符长度，设置数据位之前一定要屏掉这个位
    righthip_stNew.c_cflag |= CS8;            //8位数据长度
    //set parity
    righthip_stNew.c_cflag &= ~PARENB;
    righthip_stNew.c_iflag &= ~INPCK;         //无奇偶检验位
    //set stopbits
    righthip_stNew.c_cflag &= ~CSTOPB;        //1位停止位

    //stNew.c_oflag = 0; //输出模式
    //stNew.c_lflag = 0; //不激活终端模式

    righthip_stNew.c_cc[VTIME]=0;             //指定所要读取字符的最小数量
    righthip_stNew.c_cc[VMIN]=1;              //指定读取第一个字符的等待时间，时间的单位为n*100ms
    //假设设置VTIME=0，则无字符输入时read（）操作无限期的堵塞
    /**3. 设置新属性，TCSANOW：所有改变立即生效*/
    tcflush(righthip_nFd,TCIFLUSH);           //清空终端未完毕的输入/输出请求及数据。

    if( tcsetattr(righthip_nFd,TCSANOW,&righthip_stNew) != 0 )
    {
        perror("righthip:tcsetattr Error!\n");
        return -1;
    }
    return righthip_nFd;
}
/* 
    函数功能：10进制转换16进制位置
    输入：n：需要转化的十进制数   
    输出：buffer1十六进制字符
 */
char *LHdec2hexa(int n)
{

    memset(lefthip_buffer,0,33*sizeof(int)); 
	int i = 0;
    int d = n;
	int m = 0;
    int t = 0;
	char lefthip_hexa4[64];
	const char *hmap = "0123456789ABCDEF";
	while(d > 0)
	{
		m = d % 16;
		lefthip_hexa4[t] = hmap[m];
		d = d / 16;
		t++;
	}
	//printf("t = %d \n",t);
	for(i = 0; i < t; i++)
	{
		lefthip_buffer[i]=lefthip_hexa4[t - i - 1 ];
       // putchar(hexa[i]);	
	}
	//printf("\n");
    return lefthip_buffer;
}
char *LKdec2hexa(int n)
{

    memset(leftknee_buffer,0,33*sizeof(int)); 
	int i = 0;
    int d = n;
	int m = 0;
    int t = 0;
	char leftknee_hexa4[64];
	const char *hmap = "0123456789ABCDEF";
	while(d > 0)
	{
		m = d % 16;
		leftknee_hexa4[t] = hmap[m];
		d = d / 16;
		t++;
	}
	//printf("t = %d \n",t);
	for(i = 0; i < t; i++)
	{
		leftknee_buffer[i]=leftknee_hexa4[t - i - 1 ];
       // putchar(hexa[i]);	
	}
	//printf("\n");
    return leftknee_buffer;
}
char *RHdec2hexa(int n)
{

    memset(righthip_buffer,0,33*sizeof(int)); 
	int i = 0;
    int d = n;
	int m = 0;
    int t = 0;
	char righthip_hexa4[64];
	const char *hmap = "0123456789ABCDEF";
	while(d > 0)
	{
		m = d % 16;
		righthip_hexa4[t] = hmap[m];
		d = d / 16;
		t++;
	}
	//printf("t = %d \n",t);
	for(i = 0; i < t; i++)
	{
		righthip_buffer[i]=righthip_hexa4[t - i - 1 ];
       // putchar(hexa[i]);	
	}
	//printf("\n");
    return righthip_buffer;
}
char *RKdec2hexa(int n)
{

    memset(rightknee_buffer,0,33*sizeof(int)); 
	int i = 0;
    int d = n;
	int m = 0;
    int t = 0;
	char rightknee_hexa4[64];
	const char *hmap = "0123456789ABCDEF";
	while(d > 0)
	{
		m = d % 16;
		rightknee_hexa4[t] = hmap[m];
		d = d / 16;
		t++;
	}
	//printf("t = %d \n",t);
	for(i = 0; i < t; i++)
	{
		rightknee_buffer[i]=rightknee_hexa4[t - i - 1 ];
       // putchar(hexa[i]);	
	}
	//printf("\n");
    return rightknee_buffer;
}

//将16进制字符串转换成16进制数
int tolower(int c)  
{  
    if (c >= 'A' && c <= 'Z')  
    {  
        return c + 'a' - 'A';  
    }  
    else  
    {  
        return c;  
    }  
} 
/* 
    函数功能：将16进制字符串转换成16进制数
    输入：s【】十六进制字符   
    输出：n十六进制数
 */ 
int htoi(char s[])  
{  
    int i;  
    int n = 0;  
    if (s[0] == '0' && (s[1]=='x' || s[1]=='X'))  
    {  
        i = 2;  
    }  
    else  
    {  
        i = 0;  
    }  
    for (; (s[i] >= '0' && s[i] <= '9') || (s[i] >= 'a' && s[i] <= 'z') || (s[i] >='A' && s[i] <= 'Z');++i)  
    {  
        if (tolower(s[i]) > '9')  
        {  
            n = 16 * n + (10 + tolower(s[i]) - 'a');  
        }  
        else  
        {  
            n = 16 * n + (tolower(s[i]) - '0');  
        }  
    }  
    return n;  
}
///////////////////////////////////////////////////////////////////////////////////////////////////

int   main( int        argc,
            char**     argv,
            char**     envp )
{
    ECI_RESULT hResult = ECI_OK;

    BYTE lefthip_ox11,lefthip_ox12,lefthip_ox13,lefthip_ox14;
    BYTE leftknee_ox11,leftknee_ox12,leftknee_ox13,leftknee_ox14;
    BYTE righthip_ox11,righthip_ox12,righthip_ox13,righthip_ox14;
    BYTE rightknee_ox11,rightknee_ox12,rightknee_ox13,rightknee_ox14;
   
    char *leftknee_sixteen,*lefthip_sixteen,*rightknee_sixteen,*righthip_sixteen;
    int leftknee_qc,lefthip_qc,rightknee_qc,righthip_qc;
    int leftknee_num,lefthip_num,rightknee_num,righthip_num;

    unsigned char leftknee_k =0;
    unsigned char lefthip_k =0;
    unsigned char rightknee_k =0;
    unsigned char righthip_k =0;

    int leftknee_rxflag=1;
    int lefthip_rxflag=1;
    int rightknee_rxflag=1;
    int righthip_rxflag=1;

    int left_knee_rxflag=1;
    int left_hip_rxflag=1;
    int right_knee_rxflag=1;
    int right_hip_rxflag=1;

    int LK_rxflag=1;
    int LH_rxflag=1;
    int RK_rxflag=1;
    int RH_rxflag=1;

    int lk_stopflag=1;
    int lh_stopflag=1;
    int rk_stopflag=1;
    int rh_stopflag=1;

    int lkstopflag=1;
    int lhstopflag=1;
    int rkstopflag=1;
    int rhstopflag=1;
    
    int mpuflag=1;
    int mpu6050flag=1;

    int lefthip_len=0;
    int leftknee_len=0;
    int righthip_len=0;
    int rightknee_len=0;

    int left_hip_len=0;
    int left_knee_len=0;
    int right_hip_len=0;
    int right_knee_len=0;

    float leftknee_deviation;
    float lefthip_deviation;
    float rightknee_deviation;
    float righthip_deviation;

    float lefthip_angle_move;
    float leftknee_angle_move;
    float righthip_angle_move;
    float rightknee_angle_move;

    unsigned char leftknee_recdata[1];
    unsigned char lefthip_recdata[1];
    unsigned char rightknee_recdata[1];
    unsigned char righthip_recdata[1];

    unsigned char left_knee_recdata[1];
    unsigned char left_hip_recdata[1];
    unsigned char right_knee_recdata[1];
    unsigned char right_hip_recdata[1];

    unsigned char leftknee_data[11];
    unsigned char lefthip_data[11];
    unsigned char rightknee_data[11];
    unsigned char righthip_data[11];

    unsigned char left_knee_data[11];
    unsigned char left_hip_data[11];
    unsigned char right_knee_data[11];
    unsigned char right_hip_data[11];
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////can转驱动器定义///////////////////////////////////////////////////
    /*MOTOR ID */
    DWORD active_upper_motorID[12] = {0x00, 0x00, 0x00, 0x00, 0x601,0x602,0x603,0x604,0x601,0x602,0x603,0x604};
    DWORD active_lower_motorID[12] = {0x00, 0x00, 0x00, 0x00, 0x605,0x606,0x607,0x608,0x605,0x606,0x607,0x608};
    DWORD Move_upper_motorID[12]   = {0x601,0x603,0x602,0x604,0x601,0x603,0x602,0x604,0x601,0x603,0x602,0x604};
    DWORD Move_lower_motorID[12]   = {0x605,0x607,0x606,0x608,0x605,0x607,0x606,0x608,0x605,0x607,0x606,0x608};
    /*自由位置定义*/
////////上肢微调
    BYTE TX_pos_move[12][8]      = {                                      
                                {0x22,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},/* ENABLE */
                                {0x22,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},
                                {0x22,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},
                                {0x22,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},

                                {0x22,0x7A,0x60,0x00,0x00,0x00,0x00,0x00},/* left   shoulder肩  */
                                {0x22,0x7A,0x60,0x00,0x00,0x00,0x00,0x00},/* right  shoulder  */
                                {0x22,0x7A,0x60,0x00,0x00,0x00,0x00,0x00},/* left   elbow肘     */
                                {0x22,0x7A,0x60,0x00,0x00,0x00,0x00,0x00},/* right  elbow     */

                                {0x22,0x40,0x60,0x00,0x3F,0x00,0x00,0x00},/* MOVE */
                                {0x22,0x40,0x60,0x00,0x3F,0x00,0x00,0x00},
                                {0x22,0x40,0x60,0x00,0x3F,0x00,0x00,0x00},
                                {0x22,0x40,0x60,0x00,0x3F,0x00,0x00,0x00}
                        };
   BYTE TX_pos_upper_follow[12][8]      = {                                      
                                {0x22,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},/* ENABLE */
                                {0x22,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},
                                {0x22,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},
                                {0x22,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},

                                {0x22,0x7A,0x60,0x00,0x00,0x00,0x00,0x00},/* left   shoulder肩  */
                                {0x22,0x7A,0x60,0x00,0x00,0x00,0x00,0x00},/* right  shoulder  */
                                {0x22,0x7A,0x60,0x00,0x00,0x00,0x00,0x00},/* left   elbow肘     */
                                {0x22,0x7A,0x60,0x00,0x00,0x00,0x00,0x00},/* right  elbow     */

                                {0x22,0x40,0x60,0x00,0x3F,0x00,0x00,0x00},/* MOVE */
                                {0x22,0x40,0x60,0x00,0x3F,0x00,0x00,0x00},
                                {0x22,0x40,0x60,0x00,0x3F,0x00,0x00,0x00},
                                {0x22,0x40,0x60,0x00,0x3F,0x00,0x00,0x00}
                        };
                        
////////驱动器激活---s
    BYTE TX_active_move[12][8]      = {                                   
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
////////紧急停止---p
    BYTE TX_pos_stop[12][8]      = {                                      
                                {0x22,0x40,0x60,0x00,0x0F,0x01,0x00,0x00},/*  */
                                {0x22,0x40,0x60,0x00,0x0F,0x01,0x00,0x00},
                                {0x22,0x40,0x60,0x00,0x0F,0x01,0x00,0x00},
                                {0x22,0x40,0x60,0x00,0x0F,0x01,0x00,0x00},

                                {0x22,0x40,0x60,0x00,0x0F,0x01,0x00,0x00},/*  */
                                {0x22,0x40,0x60,0x00,0x0F,0x01,0x00,0x00},
                                {0x22,0x40,0x60,0x00,0x0F,0x01,0x00,0x00},
                                {0x22,0x40,0x60,0x00,0x0F,0x01,0x00,0x00},

                                {0x22,0x40,0x60,0x00,0x0F,0x01,0x00,0x00},/*  */
                                {0x22,0x40,0x60,0x00,0x0F,0x01,0x00,0x00},
                                {0x22,0x40,0x60,0x00,0x0F,0x01,0x00,0x00},
                                {0x22,0x40,0x60,0x00,0x0F,0x01,0x00,0x00}

                        };
    ////////上肢复位---h
    BYTE TX_pos_upper_home[12][8]      = {                                
                                {0x22,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},/* ENABLE */
                                {0x22,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},
                                {0x22,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},
                                {0x22,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},

                                {0x22,0x7A,0x60,0x00,0x00,0x00,0x00,0x00},/* left   shoulder  */
                                {0x22,0x7A,0x60,0x00,0x00,0x00,0x00,0x00},/* right  shoulder  */
                                {0x22,0x7A,0x60,0x00,0x00,0x00,0x00,0x00},/* left   elbow     */
                                {0x22,0x7A,0x60,0x00,0x00,0x00,0x00,0x00},/* right  elbow     */

                                {0x22,0x40,0x60,0x00,0x3F,0x00,0x00,0x00},/* MOVE */
                                {0x22,0x40,0x60,0x00,0x3F,0x00,0x00,0x00},
                                {0x22,0x40,0x60,0x00,0x3F,0x00,0x00,0x00},
                                {0x22,0x40,0x60,0x00,0x3F,0x00,0x00,0x00}
                        };

////////下肢复位---t
    BYTE TX_pos_lower_home[12][8]      = {                               
                                {0x22,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},/* ENABLE */
                                {0x22,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},
                                {0x22,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},
                                {0x22,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},

                                {0x22,0x7A,0x60,0x00,0x00,0x00,0x00,0x00},/* left   hip  */
                                {0x22,0x7A,0x60,0x00,0x00,0x00,0x00,0x00},/* right  hip  */
                                {0x22,0x7A,0x60,0x00,0x00,0x00,0x00,0x00},/* left   keen  */
                                {0x22,0x7A,0x60,0x00,0x00,0x00,0x00,0x00},/* right  keen  */

                                {0x22,0x40,0x60,0x00,0x3F,0x00,0x00,0x00},/* MOVE */
                                {0x22,0x40,0x60,0x00,0x3F,0x00,0x00,0x00},
                                {0x22,0x40,0x60,0x00,0x3F,0x00,0x00,0x00},
                                {0x22,0x40,0x60,0x00,0x3F,0x00,0x00,0x00}
                        };
///////////////////////////////////////////////////////////////////////////////////////////////////////
    hResult = EciDemo113();
    char ch;
    while(scanf(" %c",&ch) != EOF)
    {
        switch (ch)
        {
            case 's':    //激活上电 start
                printf("%c\n",ch);
                Can_Tx_Data( hResult, TX_active_move, active_upper_motorID);
                Can_Tx_Data( hResult, TX_active_move, active_lower_motorID);
                printf("激活\n");
                OS_Sleep(100); 
                Can_Tx_Data( hResult, TX_pos_lower_home, Move_lower_motorID);   //下肢归零上电
                OS_Sleep(100);
                Can_Tx_Data( hResult, TX_pos_upper_home, Move_upper_motorID);   //上肢归零上电
                OS_Sleep(100);
                printf("复位\n");
                break;
            
            case 'q': //上肢复位
                printf("%c\n",ch);
                Can_Tx_Data( hResult, TX_pos_upper_home, Move_upper_motorID);   //上肢归零上电
                OS_Sleep(100);
                
                break;
            case 'w': //下肢复位

                printf("%c\n",ch);
                Can_Tx_Data( hResult, TX_pos_lower_home, Move_lower_motorID);   //下肢归零上电
                OS_Sleep(100);
                break;
            
            case 'p':   //停止
                printf("%c\n",ch);
                Can_Tx_Data( hResult, TX_pos_stop, active_upper_motorID);
                Can_Tx_Data( hResult, TX_pos_stop, active_lower_motorID);
                OS_Sleep(100); 
                break;
            case 'n':
                
                LHSerialInit();
                LKSerialInit();
                RHSerialInit();
                RKSerialInit();
                lk_stopflag=1;
                lh_stopflag=1;
                rk_stopflag=1;
                rh_stopflag=1;
                leftknee_k = 0;
                lefthip_k = 0;
                mpuflag = 1;
                //printf("1");
                while(mpuflag)
                {  
                    //printf("2");
                    left_knee_rxflag=1;
                    left_hip_rxflag=1;
                    right_knee_rxflag=1;
                    right_hip_rxflag=1;

                    left_knee_len = read(leftknee_nFd, left_knee_recdata, 1);
                    left_knee_data[leftknee_k] = left_knee_recdata[0];

                    left_hip_len = read(lefthip_nFd, left_hip_recdata, 1);
                    left_hip_data[lefthip_k] = left_hip_recdata[0];

                    right_knee_len = read(rightknee_nFd, right_knee_recdata, 1);
                    right_knee_data[rightknee_k] = right_knee_recdata[0];

                    right_hip_len = read(righthip_nFd, right_hip_recdata, 1);
                    right_hip_data[righthip_k] = right_hip_recdata[0];
                    //printf("len: %d(bytes)\n", leftknee_len); 
                    if(leftknee_k == 0 && left_knee_data[0] !=0x55)
                    {
                        left_knee_rxflag = 0;
                        leftknee_k = 0;
                    }
                    if(left_knee_rxflag && lk_stopflag)
                    {
                        leftknee_k++;
                        if(leftknee_k == 11)
                        {
                            leftknee_k = 0;
                            if(left_knee_data[0] == 0x55)
                            {
                                
                                if(left_knee_data[1]==0x53)
                                {  
                                    left_knee_anglex=((short)(left_knee_data[3]<<8|left_knee_data[2]))/32768.0*180;
                                    //left_knee_angley=((short)(left_knee_data[5]<<8|left_knee_data[4]))/32768.0*180;
                                    //left_knee_anglez=((short)(left_knee_data[7]<<8|left_knee_data[6]))/32768.0*180;
                                    printf("1:left_knee_anglex = %.2f\n",left_knee_anglex);
                                    lk_stopflag = 0;
                                    leftknee_deviation = left_knee_anglex;
                                    printf("1:leftknee_deviation = %.2f\n",leftknee_deviation);
                                } 
                            }
                        }
                    }
                    if(lefthip_k == 0 && left_hip_data[0] !=0x55)
                    {
                        left_hip_rxflag = 0;
                        lefthip_k = 0;
                    }
                    if(left_hip_rxflag && lh_stopflag)
                    {
                        lefthip_k++;
                        if(lefthip_k == 11)
                        {
                            lefthip_k = 0;
                            if(left_hip_data[0] == 0x55)
                            {
                                if(left_hip_data[1]==0x53)
                                {  
                                    left_hip_anglex=((short)(left_hip_data[3]<<8|left_hip_data[2]))/32768.0*180;
                                    //left_hip_angley=((short)(left_hip_data[5]<<8|left_hip_data[4]))/32768.0*180;
                                    //left_hip_anglez=((short)(left_hip_data[7]<<8|left_hip_data[6]))/32768.0*180;
                                    printf("2:left_hip_anglex = %.2f\n",left_hip_anglex);
                                    lh_stopflag = 0;  
                                    lefthip_deviation = left_hip_anglex;
                                    printf("2:lefthip_deviation = %.2f\n",lefthip_deviation);
                                } 
                            }
                        }
                    }
                    if(rightknee_k == 0 && right_knee_data[0] !=0x55)
                    {
                        right_knee_rxflag = 0;
                        rightknee_k = 0;
                    }
                    if(right_knee_rxflag && rk_stopflag)
                    {
                        rightknee_k++;
                        if(rightknee_k == 11)
                        {
                            rightknee_k = 0;
                            if(right_knee_data[0] == 0x55)
                            {
                                if(right_knee_data[1]==0x53)
                                {  
                                    
                                    right_knee_anglex=((short)(right_knee_data[3]<<8|right_knee_data[2]))/32768.0*180;
                                    //rightknee_angley=((short)(rightknee_data[5]<<8|rightknee_data[4]))/32768.0*180;
                                    //rightknee_anglez=((short)(rightknee_data[7]<<8|rightknee_data[6]))/32768.0*180;
                                    printf("3:rightknee_anglex = %.2f,\n",right_knee_anglex);
                                    rk_stopflag = 0;
                                    rightknee_deviation = right_knee_anglex;
                                    printf("3:rightknee_deviation = %.2f\n",rightknee_deviation);
                                } 
                            }
                        }
                    }
                    if(righthip_k == 0 && right_hip_data[0] !=0x55)
                    {
                        right_hip_rxflag = 0;
                        righthip_k = 0;
                    }
                    if(right_hip_rxflag && rh_stopflag)
                    {
                        righthip_k++;
                        if(righthip_k == 11)
                        {
                            righthip_k = 0;
                            if(right_hip_data[0] == 0x55)
                            {
                                if(right_hip_data[1]==0x53)
                                {  
            
                                    right_hip_anglex=((short)(right_hip_data[3]<<8|right_hip_data[2]))/32768.0*180;
                                    //righthip_angley=((short)(righthip_data[5]<<8|righthip_data[4]))/32768.0*180;
                                    //righthip_anglez=((short)(righthip_data[7]<<8|righthip_data[6]))/32768.0*180;
                                    printf("4:righthip_anglex = %.2f\n",right_hip_anglex);
                                    rh_stopflag=0;
                                    righthip_deviation = right_hip_anglex;
                                    printf("4:righthip_deviation = %.2f\n",righthip_deviation);
                                }
                            }
                        } 
                    }
                    if(rk_stopflag==0 && rh_stopflag==0 && lh_stopflag==0 && lk_stopflag==0)
                    {
                        mpuflag = 0;
                    }
                    /*if(lh_stopflag==0 && lk_stopflag==0)
                    {
                        mpuflag = 0;
                    }*/
                }
                close(leftknee_nFd);
                close(lefthip_nFd);
                close(rightknee_nFd);
                close(righthip_nFd);
                break;
            case 'm':
                
                LHSerialInit();
                LKSerialInit();
                RHSerialInit();
                RKSerialInit();
                //leftknee_deviation = 1.0;
                //lefthip_deviation = -3.6;
                //rightknee_deviation = 40;
                //righthip_deviation = 40;
                leftknee_k = 0;
                lefthip_k = 0;
                rightknee_k = 0;
                righthip_k = 0;
                mpuflag = 1;
                while(1)
                {
                    mpu6050flag = 1;
                    lkstopflag=1;
                    lhstopflag=1;
                    rkstopflag=1;
                    rhstopflag=1;
                    while(mpu6050flag)
                    { 
                        leftknee_rxflag=1;
                        lefthip_rxflag=1;
                        rightknee_rxflag=1;
                        righthip_rxflag=1;
                    
                        righthip_len = read(righthip_nFd, righthip_recdata, 1);
                        righthip_data[righthip_k] = righthip_recdata[0];

                        rightknee_len = read(rightknee_nFd, rightknee_recdata, 1);
                        rightknee_data[rightknee_k] = rightknee_recdata[0];

                        lefthip_len = read(lefthip_nFd, lefthip_recdata, 1);
                        lefthip_data[lefthip_k] = lefthip_recdata[0];   

                        leftknee_len = read(leftknee_nFd, leftknee_recdata, 1);
                        leftknee_data[leftknee_k] = leftknee_recdata[0];

                        //righthip_len = read(righthip_nFd, righthip_recdata, 1);
                        //righthip_data[++righthip_k] = righthip_recdata[0];

                        if(righthip_k == 0 && righthip_data[0] !=0x55)
                        {
                            righthip_rxflag = 0;
                            righthip_k = 0;
                        }
                        if(righthip_rxflag && rhstopflag)
                        {
                            righthip_k++;
                            if(righthip_k == 11)
                            {
                                righthip_k = 0;
                                
                                if(righthip_data[0] == 0x55)
                                {
                                    //printf("%x\n",righthip_data[1]);
                                    if(righthip_data[1]==0x53)
                                    {  
                                        righthip_anglex=((short)(righthip_data[3]<<8|righthip_data[2]))/32768.0*180;
                                        printf("4:righthip_anglex = %02f\n",righthip_anglex);
                                        rhstopflag=0;
                                        memset(righthip_data,0,33*sizeof(int)); 
                                    } 
                                }
                            }
                        }              
                        if(rightknee_k == 0 && rightknee_data[0] !=0x55)
                        {
                            rightknee_rxflag = 0;
                            rightknee_k = 0;
                        }
                        if(rightknee_rxflag && rkstopflag)
                        {
                            rightknee_k++;
                            if(rightknee_k == 11)
                            {
                                rightknee_k = 0;
                                if(rightknee_data[0] == 0x55)
                                {
                                    
                                    if(rightknee_data[1]==0x53)
                                    {  
                                        rightknee_anglex=((short)(rightknee_data[3]<<8|rightknee_data[2]))/32768.0*180;     
                                        printf("3:rightknee_anglex = %.02f\n",rightknee_anglex);
                                        rkstopflag = 0;
                                        memset(rightknee_data,0,33*sizeof(int));
                                        
                                    } 
                                }
                            }
                        }
                        if(lefthip_k == 0 && lefthip_data[0] !=0x55)
                        {
                            lefthip_rxflag = 0;
                            lefthip_k = 0;
                        }
                        if(lefthip_rxflag && lhstopflag)
                        {
                            lefthip_k++;
                            if(lefthip_k == 11)
                            {
                                lefthip_k = 0;
                                if(lefthip_data[0] == 0x55)
                                {
                                    if(lefthip_data[1]==0x53)
                                    {  
                                        lefthip_anglex=((short)(lefthip_data[3]<<8|lefthip_data[2]))/32768.0*180;
                                        //lefthip_angley=((short)(lefthip_data[5]<<8|lefthip_data[4]))/32768.0*180;
                                        //lefthip_anglez=((short)(lefthip_data[7]<<8|lefthip_data[6]))/32768.0*180;
                                        //printf("lefthip_anglex = %.2f,lefthip_angley = %.2f,lefthip_anglez = %.2f\n",lefthip_anglex,lefthip_angley,lefthip_anglez);
                                        printf("1:lefthip_anglex = %02f\n",lefthip_anglex);
                                        lhstopflag = 0;
                                        memset(lefthip_data,0,33*sizeof(int)); 
                                    } 
                                }
                            }
                        }                       
                        if(leftknee_k == 0 && leftknee_data[0] !=0x55)
                        {
                            leftknee_rxflag = 0;
                            leftknee_k = 0;
                        }
                        if(leftknee_rxflag && lkstopflag)
                        {
                            leftknee_k++;
                            if(leftknee_k == 11)
                            {
                                leftknee_k = 0;
                                if(leftknee_data[0] == 0x55)
                                {
                                    
                                    if(leftknee_data[1]==0x53)
                                    {  
                                        leftknee_anglex=((short)(leftknee_data[3]<<8|leftknee_data[2]))/32768.0*180;
                                        //leftknee_angley=((short)(leftknee_data[5]<<8|leftknee_data[4]))/32768.0*180;
                                        //leftknee_anglez=((short)(leftknee_data[7]<<8|leftknee_data[6]))/32768.0*180;
                                        //printf("leftknee_anglex = %.2f,leftknee_angley = %.2f,leftknee_anglez = %.2f\n\n",leftknee_anglex,leftknee_angley,leftknee_anglez);
                                        printf("2:leftknee_anglex = %.02f\n",leftknee_anglex);
                                        lkstopflag = 0;
                                        memset(leftknee_data,0,33*sizeof(int));
                                        
                                    } 
                                }
                            }
                        }
                        if(rkstopflag==0 && rhstopflag==0 && lhstopflag==0 && lkstopflag==0)
                        {
                            mpu6050flag = 0;
                        }
                    }  
                    printf("2:lefthip_deviation = %.2f\n",lefthip_deviation);
                    lefthip_angle = lefthip_anglex - lefthip_deviation;
                    printf("2:lefthip_angle = %.2f\n",lefthip_angle);
                    if(lefthip_angle >= 0)
                    {
                        if(lefthip_angle > 90.0)//40
                        {
                            lefthip_angle = 90.0;//40
                        }
                        lefthip_angle_move = lefthip_angle;
                        //printf("1lefthip_angle_move = %.2f\n",lefthip_angle_move);
                        lefthip_qc = 1638400/360*lefthip_angle;
                        lefthip_sixteen = LHdec2hexa(lefthip_qc);
                        lefthip_num = htoi(lefthip_sixteen);
                    }
                    else
                    {
                        if(lefthip_angle < -15.0)
                        {
                            lefthip_angle = -15.0;
                        }
                        lefthip_angle_move = lefthip_angle;
                        //printf("2lefthip_angle_move = %.2f\n",lefthip_angle_move);
                        lefthip_angle = -lefthip_angle;
                        lefthip_qc = 1638400/360*lefthip_angle;
                        lefthip_sixteen = LHdec2hexa(lefthip_qc);
                        lefthip_num = htoi(lefthip_sixteen) -1 ; 
                        lefthip_num = ~lefthip_num; 
                    }
                    
                    printf("1:leftknee_deviation = %.2f,lefthip_angle_move = %.2f\n",leftknee_deviation,lefthip_angle_move);
                    leftknee_angle = leftknee_anglex - leftknee_deviation - lefthip_angle_move;
                    printf("1:leftknee_angle = %.2f\n",leftknee_angle);
                    if(leftknee_angle >= 0)
                    {
                        /*if(leftknee_angle > 25.0)
                        {
                            leftknee_angle = 25.0;
                        }*/
                        leftknee_angle = 0.0;
                        leftknee_qc = 1638400/360*leftknee_angle;
                        leftknee_sixteen = LKdec2hexa(leftknee_qc);
                        leftknee_num = htoi(leftknee_sixteen); 
                    }
                    else
                    {
                        if(leftknee_angle < -90.0)//40
                        {
                            leftknee_angle = -90.0;
                        }
                        leftknee_angle = -leftknee_angle;
                        leftknee_qc = 1638400/360*leftknee_angle;
                        leftknee_sixteen = LKdec2hexa(leftknee_qc);
                        leftknee_num = htoi(leftknee_sixteen) -1 ; 
                        leftknee_num = ~leftknee_num; 
                    }    

                    printf("4:righthip_deviation = %.2f\n",righthip_deviation);               
                    righthip_angle = righthip_anglex - righthip_deviation;
                    printf("4:righthip_angle = %.2f\n",righthip_angle);
                    if(righthip_angle <= 0)
                    {
                        if(righthip_angle < -90.0)//45
                        {
                            righthip_angle = -90.0;
                        }
                        righthip_angle_move =righthip_angle;
                        righthip_angle = -righthip_angle;
                        righthip_qc = 1638400/360*righthip_angle;
                        righthip_sixteen = RHdec2hexa(righthip_qc);
                        righthip_num = htoi(righthip_sixteen) -1 ; 
                        righthip_num = ~righthip_num;
                    }
                    else
                    {
                        if(righthip_angle > 15.0)
                        {
                            righthip_angle = 15.0;
                        }
                        righthip_angle_move = righthip_angle;
                        righthip_qc = 1638400/360*righthip_angle;
                        righthip_sixteen = RHdec2hexa(righthip_qc);
                        righthip_num = htoi(righthip_sixteen);   
                    }

                    printf("3:rightknee_deviation = %.2f,righthip_angle_move = %.2f\n",rightknee_deviation,righthip_angle_move);
                    rightknee_angle = rightknee_anglex -rightknee_deviation - righthip_angle_move;
                    printf("3:rightknee_angle = %.2f\n\n",rightknee_angle);
                    if(rightknee_angle <= 0)
                    {
                        /*if(rightknee_angle < -25.0)
                        {
                            rightknee_angle = -25.0;
                        }*/
                        rightknee_angle = 0.0; 
                        rightknee_qc = 1638400/360*rightknee_angle;
                        rightknee_sixteen = RKdec2hexa(rightknee_qc);
                        rightknee_num = htoi(rightknee_sixteen) ;  
                    }
                    else
                    {
                        if(rightknee_angle > 90.0)//45
                        {
                            rightknee_angle = 90.0;//45
                        }
                        rightknee_qc = 1638400/360*rightknee_angle;
                        rightknee_sixteen = RKdec2hexa(rightknee_qc);
                        rightknee_num = htoi(rightknee_sixteen);
                    }
                    
                    rightknee_ox11 = (rightknee_num&0xff);
                    rightknee_ox12 = (rightknee_num>>8&0xff);
                    rightknee_ox13 = (rightknee_num>>16&0xff);
                    rightknee_ox14 = (rightknee_num>>24&0xff);
                    printf("3:rightknee：0x%x,%x,%x,%x\n",rightknee_ox11,rightknee_ox12,rightknee_ox13,rightknee_ox14);
       
                    lefthip_ox11 = (lefthip_num&0xff);
                    lefthip_ox12 = (lefthip_num>>8&0xff);
                    lefthip_ox13 = (lefthip_num>>16&0xff);
                    lefthip_ox14 = (lefthip_num>>24&0xff);
                    printf("2:lefthip：0x%x,%x,%x,%x\n",lefthip_ox11,lefthip_ox12,lefthip_ox13,lefthip_ox14);

                    leftknee_ox11 = (leftknee_num&0xff);
                    leftknee_ox12 = (leftknee_num>>8&0xff);
                    leftknee_ox13 = (leftknee_num>>16&0xff);
                    leftknee_ox14 = (leftknee_num>>24&0xff);
                    printf("1:leftknee：0x%x,%x,%x,%x\n",leftknee_ox11,leftknee_ox12,leftknee_ox13,leftknee_ox14);

                    righthip_ox11 = (righthip_num&0xff);
                    righthip_ox12 = (righthip_num>>8&0xff);
                    righthip_ox13 = (righthip_num>>16&0xff);
                    righthip_ox14 = (righthip_num>>24&0xff);
                    printf("4:righthip：0x%x,%x,%x,%x\n\n",righthip_ox11,righthip_ox12,righthip_ox13,righthip_ox14);

                    TX_pos_upper_follow[4][4]=lefthip_ox11;
                    TX_pos_upper_follow[4][5]=lefthip_ox12;
                    TX_pos_upper_follow[4][6]=lefthip_ox13;
                    TX_pos_upper_follow[4][7]=lefthip_ox14;

                    TX_pos_upper_follow[5][4]=righthip_ox11;
                    TX_pos_upper_follow[5][5]=righthip_ox12;
                    TX_pos_upper_follow[5][6]=righthip_ox13;
                    TX_pos_upper_follow[5][7]=righthip_ox14;

                    TX_pos_upper_follow[6][4]=leftknee_ox11;
                    TX_pos_upper_follow[6][5]=leftknee_ox12;
                    TX_pos_upper_follow[6][6]=leftknee_ox13;
                    TX_pos_upper_follow[6][7]=leftknee_ox14;

                    TX_pos_upper_follow[7][4]=rightknee_ox11;
                    TX_pos_upper_follow[7][5]=rightknee_ox12;
                    TX_pos_upper_follow[7][6]=rightknee_ox13;
                    TX_pos_upper_follow[7][7]=rightknee_ox14;
                
                    Can_Tx_Data( hResult, TX_pos_upper_follow, Move_lower_motorID);  
                }
                break;
            default:
                printf("%c\n",ch);
                printf("Error input , please check out\n");
                break;
            } 
        }
        return hResult;
    }