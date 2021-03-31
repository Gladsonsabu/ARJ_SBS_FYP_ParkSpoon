
#include <String.h>
#include<Wire.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "RTClib.h"
#include <Sim800l.h>

RTC_DS1307 RTC;
Sim800l Sim800l;
Servo Servo_P, Servo_R, Servo_Med, Servo_MedGate;
TinyGPSPlus gps;
SoftwareSerial gps_comm(5, 4, 9600);

bool SET_RTC(int);
String link(void);                    //link function
void Stable(void);

void gsm_trx(char data1, char data2);
void gsm_trx_local();
bool SMS_Valid(void);
int nonZero(void);


int day, month, year, minute, second, hour;

//int State = 0;

const int Button_Panic = 12;
const int Button_Spoon = 7;
const int Buzz = 8;

const int MPU_addr = 0x68;                    // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float delta_t = 0.005;
float pitchAcc, rollAcc, pitch, roll, pitched;
float P_CompCoeff = 0.98;

char* text = "Sample Text";
char* SMS_Key = "###";
char* text_Panic = "Please come to me I am panicking \n";
char* number1 = "7907595401";                           // Reciver number
//uint8_t index;
bool error;

//int timeArray[12][4] = {{6,12,18,22}, {0,30,0,30}, {30,0,30,0}};
int timeArray[12][3] = {{6,0,30}, {12,30,0}, {18,0,30}, {22,30,0}};
int array_max_entry = 3;
//int array_max_entry = nonZero();
char charBuf[125];
char time_char[125];


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(Button_Panic, INPUT);
  pinMode(Button_Spoon, INPUT);
  pinMode(Buzz, OUTPUT);
  
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);                           // PWR_MGMT_1 register
  Wire.write(0);                              // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Sim800l.begin();
  boolean notConnected = true;                                         // Connection state assumed to exist
  while (notConnected)
  {
    notConnected = Sim800l.delAllSms();
    Serial.println(notConnected);
    if (notConnected)
    { notConnected = false; }                                            // If not made to false
    else
    { Serial.println(F("Not connected")); }
    delay(1000);
  }
  error = Sim800l.delAllSms();


  while (! RTC.isrunning()) {
    digitalWrite(LED_BUILTIN, LOW);
  }
  Serial.println(F("RTC is running!"));
  SET_RTC(1);

  Servo_MedGate.attach(6);
  Servo_Med.attach(9);
  Servo_P.attach(10);
  Servo_R.attach(11);
}

void loop()
{
  //bool SMS_Update = SMS_Valid();
  DateTime now = RTC.now();
  if (digitalRead(Button_Panic))
  {
    String locn = String(text_Panic + link());
    locn.toCharArray(charBuf, 100);
    char* ptext = charBuf;
    error = Sim800l.sendSms(number1, ptext);
  }

  else if (digitalRead(Button_Spoon))
  {
    delay(100);
    do {
      Stable();
    } while (digitalRead(Button_Spoon));
  }
  
//  else if (SMS_Update)
//  {
//    delay(100);
//    
//  }
  
  else
  {
    delay(1000);
    for(int i=0;i<=array_max_entry;i++)
    {
      if(now.hour() == timeArray[i][0] && now.minute() == timeArray[i][1] && now.second() == timeArray[i][2])
        {
          MedSerShift((i*60),((i*60)+60));//********* put limits here
          //MedSerShift((i*(270/nonZero())),((i*(270/nonZero()))+(270/nonZero())));
          Serial.println(F("SERVO HAS RESETED"));
          do{
            Serial.println(F("waiting for trigger"));//buzzer loop @ do
            digitalWrite(Buzz,HIGH);
            delay(200);
            digitalWrite(Buzz,LOW);
            delay(200);
            }while(!digitalRead(Button_Spoon));
            //===================================//gate open/close here
          for (int Pos = 0; Pos<=90 ; Pos++) 
          {
            Servo_MedGate.write(Pos);
            delay(15); 
          }
          Serial.println(F("SERVO 1 HAS RESETED"));
          for (int Pos = 90; Pos>=0 ; Pos--) 
          {
            Servo_MedGate.write(Pos);
            delay(15); 
          }

            
          
        }
       else if(now.hour() == 23 && now.minute() == 59 && now.second() == 59)
        {
          for (int Z = 180; Z>=0 ; Z--) 
            {
              Servo_Med.write(Z);
              delay(15); 
            }
            Serial.println(F("SERVO HAS RESETED"));
        }
       else
       {
          Serial.println(F("NO PILL DISPENSED"));
       }
    }
    
  }

}




//============================================================================================================
//============================================================================================================
//============================================================================================================
//============================================================================================================
//============================================================================================================
//============================================================================================================
//============================================================================================================
//============================================================================================================
//============================================================================================================
//============================================================================================================




bool SET_RTC(int type)
{
  if (type == 1)
  {
    RTC.adjust(DateTime(__DATE__, __TIME__));  //automatic adjust
    return 0;
  }
  else if (type == 2)
  {
    RTC.adjust(DateTime(2021, 02, 20, 6, 30, 0));  //manual adjust @ 20/02/2021---6:30:00 AM
    return 0;
  }
  else if (type == 3)
  {
    Sim800l.updateRtc(-3);  //GSM Adjust
    Sim800l.RTCtime(&day, &month, &year, &hour, &minute, &second);
    RTC.adjust(DateTime(year, month, day, hour, minute, second));
    return 0;
  }
  else {
    return 1;
  }
}



void Stable()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);                                     // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);                 // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read();                 // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read();                 // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read();                 // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  GyX = Wire.read() << 8 | Wire.read();                 // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read();                 // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read();                 // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  //Complementary filter
  long squaresum_P = ((long)GyY * GyY + (long)AcY * AcY);
  long squaresum_R = ((long)GyX * GyX + (long)AcX * AcX);
  pitch += ((-AcY / 40.8f) * (delta_t));
  roll += ((-AcX / 45.8f) * (delta_t));                 //32.8
  pitchAcc = atan((AcY / sqrt(squaresum_P)) * RAD_TO_DEG);
  rollAcc = atan((AcX / sqrt(squaresum_R)) * RAD_TO_DEG);
  pitch = (P_CompCoeff * pitch + (1.0f - P_CompCoeff) * pitchAcc);
  //pitch = P_CompCoeff*pitch + (1.0f-P_CompCoeff)*pitchAcc;
  roll = (P_CompCoeff * roll + (1.0f - P_CompCoeff) * rollAcc);


  if (pitch < -158)
  {
    pitched = abs(pitch + 158);
    pitched = pitched - 158;
  }
  else if (pitch > -156)
  {
    pitched = abs(156 + pitch);
    pitched = -156 - pitched;
  }
  //locked movement for upward direction of pitch
  if (pitched < -240)
  {
    pitched = -240;
  }
  //Servo commands, roll/pitch + nr, where nr is compensation for mounting to start horizontally
  Servo_P.write((roll + 120));
  Servo_R.write(pitched + 340);
}


String link()
{
  String comm = ",";
  String lnk = "http://www.google.com/maps/?q=";
  String lattitude = String(gps.location.lat(), DEC);
  String longitude = String(gps.location.lng(), DEC);
  String str1 = lnk + lattitude + comm + longitude;
  String str2 = String("http://www.google.com/maps/?q=" + String(gps.location.lat(), DEC) + "," + String(gps.location.lng(), DEC));
  Serial.print(str1);
  Serial.print(str2);
  return (str1);
}

int MedSerShift(int pos1,int pos2)
{
  Serial.print(F("MED SERVO HAS INITIATED"));
  for (int Z = pos1; Z <= pos2; Z++) 
  {
    Servo_Med.write(Z);
    delay(15); 
  }
  Serial.print(F("MED SERVO HAS TERMINATED"));
  delay(100);
}

int nonZero()
{
  int count = 0;
  int sum = 0;
  for(int arrindex=0;arrindex<=12;arrindex++) //sizeof timeArray / sizeof timeArray[0]
  {
    sum = timeArray[arrindex][0] + timeArray[arrindex][1] + timeArray[arrindex][2];
    sum=0 ? count++ : sum=0;
  }
  return count;
}




//format for SMS_Valid
//###set*HH:MM:SS*HH:MM:SS*HH:MM:SS*..... till 12 indexes
//###get ---> returns current location and send to preset number


bool SMS_Valid(void) 
{
  String result1 = Sim800l.readSms(1);
  if (result1.indexOf(SMS_Key) != -1)       //SMS_Key = "###";
  {
    if (result1.indexOf("set", 3) != -1)
    {
      int timeArrayentry = 0;
      result1.toCharArray(time_char, 100);
      for (int j = 0; j <= 100; j++)
      {
        if (time_char[j] == '*') {
          timeArray[timeArrayentry] [0] = (((Toint(time_char[j + 1])) * 10) + Toint(time_char[j + 2]));
          timeArray[timeArrayentry] [1] = (((Toint(time_char[j + 4])) * 10) + Toint(time_char[j + 5]));
          timeArray[timeArrayentry] [2] = (((Toint(time_char[j + 7])) * 10) + Toint(time_char[j + 8]));
//          timeArray[timeArrayentry] [0] = ((int)(time_char[j + 1]) * 10) + (int)(time_char[j + 2]);
//          timeArray[timeArrayentry] [1] = ((int)(time_char[j + 4]) * 10) + (int)(time_char[j + 5]);
//          timeArray[timeArrayentry] [2] = ((int)(time_char[j + 7]) * 10) + (int)(time_char[j + 8]);
          timeArrayentry++;
          j = j + 7;
        }
        else if (time_char[j] == '\0')
        {
          j = 100;
          //break;
        }
      }
      error = Sim800l.delAllSms();
      return (error);
    }
    if (result1.indexOf("get", 3) != -1)
    {
      String Req_locn = link();
      Req_locn.toCharArray(charBuf, 100);
      char* Req_locntext = charBuf;
      error = Sim800l.sendSms(number1, Req_locntext);
    }
  }
}

int Toint(char x)
{
  if (x = "0") {return (0);}
  else if (x = "1") {return (1);}
  else if (x = "2") {return (2);}
  else if (x = "3") {return (3);}
  else if (x = "4") {return (4);}
  else if (x = "5") {return (5);}
  else if (x = "6") {return (6);}
  else if (x = "7") {return (7);}
  else if (x = "8") {return (8);}
  else if (x = "9") {return (9);}
  else {return (0);}
}
