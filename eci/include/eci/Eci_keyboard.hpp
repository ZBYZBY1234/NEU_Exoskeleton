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


const char* HEX = "0123456789abcdef";
char    motor_buffer[8];

class Motor_Position
{
    public:
        Motor_Position(){};
        ~Motor_Position(){};
    public:
        void PositionControl();
    private:
        void  htoi(char s[]);
        void tohex(int n);
    private:
        /*MOTOR ID*/
        DWORD motorID;
        /*自由位置定义*/
        BYTE TX_init_pose;
        /*关节角度*/
        float   motor_angle;
        int     motor_qc;

        int    motor_sixteen;
};

void Motor_Position::tohex(int n)
{
    memset(motor_buffer,0,33*sizeof(int)); 
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
    printf("%d\n",t);
	for(i = 0; i < t; i++)
	{
		motor_buffer[i]=righthip_hexa4[t - i - 1 ];
	}
}
void Motor_Position::htoi(char s[])
{
    int i = 0;
    Motor_Position::motor_sixteen = 0;
    for (; (s[i] >= '0' && s[i] <= '9') || (s[i] >= 'a' && s[i] <= 'z') || (s[i] >='A' && s[i] <= 'Z');++i)
    {
        int x;
        if (s[i] >= 'A' && s[i] <= 'Z')
        {
            x =  s[i] + 'a' - 'A';
        }
        else
        {
            x = s[i];
        }
        if (x> '9')
        {
            Motor_Position::motor_sixteen = 16 * Motor_Position::motor_sixteen + (10 + x - 'a');
        }
        else
        {
            Motor_Position::motor_sixteen = 16 * Motor_Position::motor_sixteen + (x - '0');
        }
    }
}
void Motor_Position::PositionControl()
{
    DWORD active_upper_motorID[12] = {0x00, 0x00, 0x00, 0x00, 0x601,0x602,0x603,0x608,0x601,0x602,0x603,0x608};
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
    ECI_RESULT hResult = ECI_OK;
    hResult = EciDemo113();
    Can_Tx_Data( hResult, TX_active_move, active_upper_motorID);
    printf("Motived!!!\n");
    while (scanf("%f",&motor_angle) != EOF)
    {
        printf("The motor angle you have entered: %f\n",motor_angle);
        // printf("1");
        motor_qc  = 1638400/360*motor_angle;
        tohex(motor_qc);
        for (int i = 0; i < 8; i++)
        {
            printf("%c",motor_buffer[i]);
        }
        htoi(motor_buffer);
        printf("\n--\n");
        printf("motor_sixteen:%x\n",Motor_Position::motor_sixteen);
        BYTE righthip_ox11 = (motor_sixteen&0xff);
        BYTE righthip_ox12 = (motor_sixteen>>8&0xff);
        BYTE righthip_ox13 = (motor_sixteen>>16&0xff);
        BYTE righthip_ox14 = (motor_sixteen>>24&0xff);
        printf("4:righthip：0x%x,%x,%x,%x\n\n",righthip_ox11,righthip_ox12,righthip_ox13,righthip_ox14);
        ECI_RESULT hResult = ECI_OK;
        hResult = EciDemo113();
        

        BYTE TX_pos_upper_follow[12][8]      = {                                      
                                        {0x22,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},/* ENABLE */
                                        {0x22,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},
                                        {0x22,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},
                                        {0x22,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},

                                        {0x22,0x7A,0x60,0x00,0x00,0x00,0x00,0x00},//righthip_ox11,righthip_ox12,righthip_ox13,righthip_ox14},/* left   shoulder肩  */
                                        {0x22,0x7A,0x60,0x00,0x00,0x00,0x00,0x00},//righthip_ox11,righthip_ox12,righthip_ox13,righthip_ox14},/* right  shoulder  */
                                        {0x22,0x7A,0x60,0x00,0x00,0x00,0x00,0x00},//righthip_ox11,righthip_ox12,righthip_ox13,righthip_ox14},/* left   elbow肘     */
                                        {0x22,0x7A,0x60,0x00,righthip_ox11,righthip_ox12,righthip_ox13,righthip_ox14},//righthip_ox11,righthip_ox12,righthip_ox13,righthip_ox14},/* right  elbow     */

                                        {0x22,0x40,0x60,0x00,0x3F,0x00,0x00,0x00},/* MOVE */
                                        {0x22,0x40,0x60,0x00,0x3F,0x00,0x00,0x00},
                                        {0x22,0x40,0x60,0x00,0x3F,0x00,0x00,0x00},
                                        {0x22,0x40,0x60,0x00,0x3F,0x00,0x00,0x00}
                                };
        

        DWORD Move_upper_motorID[12]   = {0x601,0x603,0x602,0x608,0x601,0x603,0x602,0x608,0x601,0x603,0x602,0x608};

        //Motive
        
        
        Can_Tx_Data( hResult, TX_pos_upper_follow, Move_upper_motorID);
        sleep(1);
        Can_Tx_Data( hResult, TX_pos_upper_follow, Move_upper_motorID);
        sleep(1);
    }
}