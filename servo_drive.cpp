#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
uint8_t start_flag = 0;
uint8_t index = 0;
int incomedate = 0;
String inputString="";
int angle_pluse[6]={0};

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");

  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

//  Serial.println(78, BIN);// "1001110"
//  Serial.println(78, OCT);// "116"
//  Serial.println(78, DEC);// "78"
//  Serial.println(78, HEX);// "4E"
//  Serial.println(1.23456, 0);// "1"
//  Serial.println(1.23456, 2);// "1.23"
//  Serial.println(1.23456, 4);// "1.2346"
//  Serial.println('N');// "N"
  Serial.println("Hello world.");// "Hello world."

  yield();
}

// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

int str_to_num(char a)
{
  int num=0;
  if(a=='0'){ num=0;}else if(a=='1'){ num=1;}else if(a=='2'){ num=2;}else if(a=='3'){ num=3;}else if(a=='4'){ num=4;}
  else if(a=='5'){ num=5;}else if(a=='6'){ num=6;}else if(a=='7'){ num=7;}else if(a=='8'){ num=8;}else if(a=='9'){ num=9;}
  else{ num=0;}
  return num;
}

void loop() {
  if(start_flag==0)
  {
    start_flag=1;
  }
  while(Serial.available())
  {
     inputString=inputString+char(Serial.read());
     delay(2);
  }
  if(inputString.length()>0)
  {
      //Serial.println(inputString);
      if(inputString[0] == 'a')
      {
        index=0;
        angle_pluse[index]=100*str_to_num(inputString[5*index+2])+10*str_to_num(inputString[5*index+3])+str_to_num(inputString[5*index+4]);
        if(angle_pluse[index]<130) angle_pluse[index]=130; 
        if(angle_pluse[index]>330) angle_pluse[index]=330;
        if(inputString[5*index+1]=='1') 
        {
          pwm.setPWM(index, 0, angle_pluse[index]);
          Serial.print("a-new success:");
          Serial.println(angle_pluse[index]);
        }
        else
        {
          Serial.println("a-default");
        }
      }
	  delay(500);
      if(inputString[5] == 'b')
      {
        index=1;
        angle_pluse[index]=100*str_to_num(inputString[5*index+2])+10*str_to_num(inputString[5*index+3])+str_to_num(inputString[5*index+4]);
        if(angle_pluse[index]<130) angle_pluse[index]=130; 
        if(angle_pluse[index]>330) angle_pluse[index]=330;
        if(inputString[5*index+1]=='1') 
        {
          pwm.setPWM(index, 0, angle_pluse[index]);
          Serial.print("b-new success:");
          Serial.println(angle_pluse[index]);
        }
        else
        {
          Serial.println("b-default");
        }
      }
	  delay(500);
      if(inputString[10] == 'c')
      {
        index=2;
        angle_pluse[index]=100*str_to_num(inputString[5*index+2])+10*str_to_num(inputString[5*index+3])+str_to_num(inputString[5*index+4]);
//        if(angle_pluse[index]<130) angle_pluse[index]=130; 
//        if(angle_pluse[index]>330) angle_pluse[index]=330;
        if(inputString[5*index+1]=='1') 
        {
          pwm.setPWM(index, 0, angle_pluse[index]);
          Serial.print("c-new success:");
          Serial.println(angle_pluse[index]);
        }
        else
        {
          Serial.println("c-default");
        }
      }
	  delay(2000);
      if(inputString[15] == 'd')
      {
        index=3;
        angle_pluse[index]=100*str_to_num(inputString[5*index+2])+10*str_to_num(inputString[5*index+3])+str_to_num(inputString[5*index+4]);
//        if(angle_pluse[index]<130) angle_pluse[index]=130; 
//        if(angle_pluse[index]>330) angle_pluse[index]=330;
        if(inputString[5*index+1]=='1') 
        {
          pwm.setPWM(index, 0, angle_pluse[index]);
          Serial.print("d-new success:");
          Serial.println(angle_pluse[index]);
        }
        else
        {
          Serial.println("d-default");
        }
      }
	  delay(2000);
      if(inputString[20] == 'e')
      {
        index=4;
        angle_pluse[index]=100*str_to_num(inputString[5*index+2])+10*str_to_num(inputString[5*index+3])+str_to_num(inputString[5*index+4]);
//        if(angle_pluse[index]<130) angle_pluse[index]=130; 
//        if(angle_pluse[index]>330) angle_pluse[index]=330;
        if(inputString[5*index+1]=='1') 
        {
          pwm.setPWM(index, 0, angle_pluse[index]);
          Serial.print("e-new success:");
          Serial.println(angle_pluse[index]);
        }
        else
        {
          Serial.println("e-default");
        }
      }
	  delay(2000);
      if(inputString[25] == 'f')
      {
        index=5;
        angle_pluse[index]=100*str_to_num(inputString[5*index+2])+10*str_to_num(inputString[5*index+3])+str_to_num(inputString[5*index+4]);
//        if(angle_pluse[index]<130) angle_pluse[index]=130; 
//        if(angle_pluse[index]>330) angle_pluse[index]=330;
        if(inputString[5*index+1]=='1') 
        {
          pwm.setPWM(index, 0, angle_pluse[index]);
          Serial.print("f-new success:");
          Serial.println(angle_pluse[index]);
        }
        else
        {
          Serial.println("f-default");
        }
      }
      delay(2000);
      inputString="";
  }
  delay(100);
}
