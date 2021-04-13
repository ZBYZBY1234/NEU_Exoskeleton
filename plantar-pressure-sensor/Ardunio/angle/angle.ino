//GY-25  ARDUINO
//   GY25                  arduino UNO
//   VCC----------------------VCC
//   RX-----------------------TX
//   TX-----------------------RX
//   GND----------------------GND
#include <SoftwareSerial.h>
#include <Wire.h>

SoftwareSerial mySerial_down(2,3);
SoftwareSerial mySerial_up(4,5);
int YPR_down[3];
int YPR_up[3];
unsigned char Re_buf_down[8],counter_down=0;
unsigned char Re_buf_up[8],counter_up=0;
unsigned char sign_down=0;
unsigned char sign_up=0;

void setup()
{ 
  Serial.begin(115200);  
  mySerial_down.begin(115200);  
  mySerial_up.begin(115200);  
  delay(2000);  
  mySerial_up.write(0XA5);
  mySerial_up.write(0X52);    //初始化GY25,连续输出模式 
  mySerial_down.write(0XA5);
  mySerial_down.write(0X52);    //初始化GY25,连续输出模式
}
//-------------------------------------------------------------
void loop() {
  serialEvent_up();
  serialEvent_down();
  if(sign_up  && sign_down)
  {  
     sign_up=0;
     sign_down=0;
     if(Re_buf_up[0]==0xAA && Re_buf_up[7]==0x55 && Re_buf_down[0]==0xAA && Re_buf_down[7]==0x55)        //检查帧头，帧尾
     {                 
//            YPR_up[0]=(Re_buf_up[1]<<8|Re_buf_up[2])/100;   //合成数据，去掉小数点后2位
            YPR_up[1]=(Re_buf_up[3]<<8|Re_buf_up[4])/100;
            YPR_down[1]=(Re_buf_down[3]<<8|Re_buf_down[4])/100;
//            YPR_up[2]=(Re_buf_up[5]<<8|Re_buf_up[6])/100;
              
//            Serial.print("YPR_up[0]: ");
//            Serial.print(YPR_up[0]);      //显示航向
//            Serial.print(" "); 
            Serial.print("YPR_up[1]: ");                    
            Serial.print(YPR_up[1]);
            Serial.print(" ");
            Serial.print("YPR_down[1]: ");                    
            Serial.println(YPR_down[1]);    
//            Serial.print(" ");
//            Serial.print("YPR_up[2]: ");  
//            Serial.print(YPR_up[2]);   
//            Serial.println("");   
//            delay(100);           
    }
  }
  
//  
//  if(sign_down)
//  {
//     sign_down=0;
//     if(Re_buf_down[0]==0xAA && Re_buf_down[7]==0x55)        //检查帧头，帧尾
//     {                 
////            YPR_down[0]=(Re_buf_down[1]<<8|Re_buf_down[2])/100;   //合成数据，去掉小数点后2位
//            YPR_down[1]=(Re_buf_down[3]<<8|Re_buf_down[4])/100;
////            YPR_down[2]=(Re_buf_down[5]<<8|Re_buf_down[6])/100;
//              
////            Serial.print("YPR_down[0]: ");
////            Serial.print(YPR_down[0]);      //显示航向
////            Serial.print(" "); 
//            Serial.print("YPR_down[1]: ");                    
//            Serial.println(YPR_down[1]);     
////            Serial.print(" ");
////            Serial.print("YPR_down[2]: ");  
////            Serial.print(YPR_down[2]);   
////            Serial.println("");   
////            delay(100);           
//     }
//  }
}
//---------------------------------------------------------------
void serialEvent_up() {
  mySerial_up.listen();
  delay(10);
  while (mySerial_up.available()) {   
    Re_buf_up[counter_up]=(unsigned char)mySerial_up.read();
    if(counter_up==0&&Re_buf_up[0]!=0xAA) return;      // 检查帧头         
    counter_up++;      
    if(counter_up==8)                //接收到数据
    {   
       counter_up=0;                 //重新赋值，准备下一帧数据的接收
       sign_up=1;
    }      
  }
}

void serialEvent_down() {
  mySerial_down.listen();
  delay(10);
  while (mySerial_down.available()) {   
    Re_buf_down[counter_down]=(unsigned char)mySerial_down.read();
    if(counter_down==0&&Re_buf_down[0]!=0xAA) return;      // 检查帧头         
    counter_down++;      
    if(counter_down==8)                //接收到数据
    {   
       counter_down=0;                 //重新赋值，准备下一帧数据的接收
       sign_down=1;
    }      
  }
}
