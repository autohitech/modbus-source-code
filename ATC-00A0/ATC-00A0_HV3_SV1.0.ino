/*
 * ATC-00A0_HV3_SV1.0      
 * 
 * Model Name : ATC-00A0
 * HardWare Version : 3.0
 * SoftWare Version : 1.0
 * Communication 1: Modbus Slave
 * Creation Date : 2017-02-06
 */
/****************************************************************************
 * This program is for ATC-00A0 controller.
 * Below is assigned I/O pin number.
 * 
 * Analog Input RTD (8 channels)
 * Analog Input (2 channels) : A7, A6
 * 
 * ID select pin (4 pin) : A2(b0), A3(b1), A4(b2), A5(b3) 
 * Communication 1 : RxD, TxD
 * Communication 2 : None
 * **************************************************************************
 * 
 * Modbus Address Map
 * 30000 : Digital Input (IX00 ~ IX03)
 * 30001 : Digital Input 
 * 
 * 30002 : Digital Output (QX00 ~ QX03)
 * 30003 : Digital Output
 * 
 * 30004 : Analog Input (IW00)
 * 30005 : Analog Input (IW01)
 * 30006 : Analog Input
 * 30007 : Analog Input
 * 30008 : Analog Input
 * 30009 : Analog Input
 * 30010 : Analog Input
 * 30011 : Analog Input
 * 
 * 30012 : Analog Output (QW00)
 * 30013 : Analog Output (QW01)
 * 30014 : Analog Output
 * 30015 : Analog Output
 * 30016 : Analog Output
 * 30017 : Analog Output
 * 30018 : Analog Output
 * 30019 : Analog Output
 * 
 */
 
#include <Arduino.h>
#include "LT_SPI.h"
#include "UserInterface.h"
#include <ModbusRtu.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <SimpleTimer.h>
#include "configuration_constants_LTC2983.h"
#include "support_functions_LTC2983.h"
#include "table_coeffs_LTC2983.h"
#define CHIP_SELECT   10 

int Ch_Value[10];
void configure_channels();
void configure_global_parameters();

///////////union test///////////////////////////
typedef union ftest{
  float fdata;
  unsigned int udata[2]; 
}U_SEND ;
///////////////////////////////////////////////////////
// ID 번호 선택

bool id1 = !digitalRead(A5);
bool id2 = !digitalRead(A4);
bool id3 = !digitalRead(A3);
bool id4 = !digitalRead(A2);
int id = id1 + (id2 * 2) + (id3 * 4); 

//모드버스 마스터의  통신포트 및 통신텔레그램 오브젝트 생성
Modbus slave(id,0,0); // 0 maeans master(1~247 slave), 0 means serial port, 0 means  RS-232 or USB-FTDI
Modbus myslave(id);

// 소프트시리얼 열기
SoftwareSerial mySerial(2, 3);

//모드버스 데이터 어레이 변수
u16 _D[50]; // 데이터 어레이

//입출력 및 메모리 변수
int  IW00,IW01,IW02,IW03,IW04,IW05,IW06,IW07,IW08,IW09;              // 마스터의 아날로그 입력변수
int i;

//통신 속도 및 타이머 변수
int baud_rate = 19200; // 시리얼 통신속도
int timer001,timer002,timer003,timer004,timer005; // 타이머 진행시간 변수
boolean t001,t002,t003,t004,t005; // 타이머 동작 변수

//온도센서 용 변수
int fChannel0,fChannel1,fChannel2,fChannel3,fChannel4,fChannel5,fChannel6,fChannel7;
int igCheck;
extern float fgAve0, fgAve1, fgAve2, fgAve3, fgAve4, fgAve5, fgAve6, fgAve7;

//아날로그 입력(평균값) 처리 변수
const int numReadings00 = 10;
const int numReadings01 = 10;
int readings00[numReadings00];      // the readings from the Analg 0 input 
int readings01[numReadings01];      // the readings from the Analog 1 input
int readIndex00 = 0;                // the index of the current Analog 0 reading
int readIndex01 = 0;                // the index of the current Analog 1 reading
int total00 = 0;                    // the running total of Analog 0
int total01 = 0;                    // the running total of Analog 1
int average00 = 0;                  // the average of Analog 0
int average01 = 0;                  // the average of Analog 1

//타이머 및 기타변수
void fn(); 
void fntemp(); 

// 타이머 오브젝트 생성
SimpleTimer timer ;
SimpleTimer timertemp ;

void setup() {

//통신속도 디폴드 설정//
 if (id4 == id) { EEPROM.put(21,19200); _D[21] = 19200;}
   EEPROM.get(21,baud_rate);
   _D[21] = baud_rate;   
   
  SPI.setClockDivider(SPI_CLOCK_DIV16); 
  SPI.begin();

  slave.begin(19200);         // Initialize the serial port to the PC
  configure_channels();
  configure_global_parameters();  
    
// 통신속도 및 타임아웃 시간 설정
 myslave.begin( &mySerial, baud_rate ); 

// 타이머 오브젝트 시간 설정
  timer.setInterval(100,fn);
  timertemp.setInterval(300,fntemp);
}

void loop() {

// 통신속도 변경
  if (_D[22] == 1234) { EEPROM.put(21,_D[21]); _D[22] = 0;} //비밀번호 1234를 D[22]에 입력하면 통신속도 변경
 //  Serial.println("AAAA0");
 // 아날로그 입력 읽기
   total00 = total00 - readings00[readIndex00]; // subtract the last reading:
   total01 = total01 - readings01[readIndex01]; // subtract the last reading:
   readings00[readIndex00] = analogRead(A7); // read from the sensor:
   readings01[readIndex01] = analogRead(A6); // read from the sensor:
   total00 = total00 + readings00[readIndex00]; // add the reading to the total:
   total01 = total01 + readings01[readIndex01]; // add the reading to the total:
   readIndex00 = readIndex00 + 1; // advance to the next position in the array:
   readIndex01 = readIndex01 + 1; // advance to the next position in the array:
   if (readIndex00 >= numReadings00) { readIndex00 = 0;}
   if (readIndex01 >= numReadings01) { readIndex01 = 0;}
   IW08 = total00 / numReadings00;                   //아날로그입력 00
   IW09 = total01 / numReadings01;                   //아날로그입력 01  
   
    if(fChannel0==STIMER){
      fChannel0 = SVALUE;
      measure_channel_f (CHIP_SELECT, 4, TEMPERATURE,0); 
    }
    if(fChannel1==STIMER){
      fChannel1= SVALUE;
      measure_channel_f(CHIP_SELECT, 6, TEMPERATURE,0);
    }
    if(fChannel2==STIMER){
      fChannel2= SVALUE;  
      measure_channel_f(CHIP_SELECT, 8, TEMPERATURE,0);  
    }
    if(fChannel3==STIMER){
      fChannel3= SVALUE;
      measure_channel_f(CHIP_SELECT, 10, TEMPERATURE,0);
    }
    if(fChannel4==STIMER){
      fChannel4= SVALUE;
      measure_channel_f(CHIP_SELECT, 12, TEMPERATURE,0);  
    }
    
    if(fChannel5==STIMER){
      fChannel5= SVALUE;
      measure_channel_f(CHIP_SELECT, 14, TEMPERATURE,0);  
    }
    if(fChannel6==STIMER){
      fChannel6= SVALUE;
      measure_channel_f(CHIP_SELECT, 16, TEMPERATURE,0);  
    }
    if(fChannel7==STIMER){
      fChannel7= SVALUE;
      measure_channel_f(CHIP_SELECT, 18, TEMPERATURE,0);  
    }
////////////////////////////////////////////////////////
  U_SEND u_send0,u_send1,u_send2,u_send3,u_send4,u_send5,u_send6,u_send7 ;
  u_send0.fdata= fgAve0;
  
  _D[4]=u_send0.udata[1];
  _D[5]=u_send0.udata[0];

  u_send1.fdata=fgAve1;
  _D[6]=u_send1.udata[1];
  _D[7]=u_send1.udata[0]; 

  u_send2.fdata=fgAve2;
_D[8]=u_send2.udata[1];
_D[9]=u_send2.udata[0];

  u_send3.fdata=fgAve3;
  _D[10]=u_send3.udata[1];
  _D[11]=u_send3.udata[0];

  u_send4.fdata=fgAve4;
  _D[12]=u_send4.udata[1];
  _D[13]=u_send4.udata[0];

  u_send5.fdata=fgAve5;
  _D[14]=u_send5.udata[1];
  _D[15]=u_send5.udata[0];

  u_send6.fdata=fgAve6;
  _D[16]=u_send6.udata[1];
  _D[17]=u_send6.udata[0];

  u_send7.fdata=fgAve7;
  _D[18]=u_send7.udata[1];
  _D[19]=u_send7.udata[0]; 
    
///////////////////////////////////////////////////////////////////////////////
// 로직 프로그램 시작
///////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////
// 로직 프로그램 끝
//////////////////////////////////////////////////////////////////////////////////

// 모드버스 어드레스 mapping  
//    _D[0] = IX00 + (IX01*2) + (IX02*4) + (IX03*8);
//    _D[1] = 0;
     _D[2] = IW08;
     _D[3] = IW09;
//   _D[4] = IW00;
//   _D[5] = IW01;
//   _D[6] = IW02;
//   _D[7] = IW03;
//   _D[8] = IW04;
//   _D[9] = IW05;
//   _D[10] = IW06;
//   _D[11] = IW07;
//   _D[12] = IW08;
//   _D[13] = IW09;   

    slave.poll( _D, 20 );
    myslave.poll( _D, 20 );    
    
    timer.run();
    timertemp.run();
    delay(10);
}

void fn() {  
  if (t001 == 1) {timer001 = timer001 + 1;}
  if (t001 == 0) {timer001 = 0;}
  if (t002 == 1) {timer002 = timer002 + 1;}
  if (t002 == 0) {timer002 = 0;}
  if (t003 == 1) {timer003 = timer003 + 1;}
  if (t003 == 0) {timer003 = 0;}
  if (t004 == 1) {timer004 = timer004 + 1;}
  if (t004 == 0) {timer004 = 0;}
  if (t005 == 1) {timer005 = timer005 + 1;}  
  if (t005 == 0) {timer005 = 0;}  
 }
 
void fntemp() {   
  if(igCheck==0) fChannel0= STIMER;
  if(igCheck==1) fChannel1= STIMER;
  if(igCheck==2) fChannel2= STIMER;
  if(igCheck==3) fChannel3= STIMER;
  if(igCheck==4) fChannel4= STIMER;
  if(igCheck==5) fChannel5= STIMER;
  if(igCheck==6) fChannel6= STIMER;
  if(igCheck==7) fChannel7= STIMER;  
  igCheck++;
  if(igCheck >= 8) { igCheck=0;}
}

void configure_global_parameters()
{
  // -- Set global parameters
  transfer_byte(CHIP_SELECT, WRITE_TO_RAM, 0xF0, TEMP_UNIT__C |
                REJECTION__50_60_HZ);
  // -- Set any extra delay between conversions (in this case, 0*100us)
  transfer_byte(CHIP_SELECT, WRITE_TO_RAM, 0xFF, 0);
}

void configure_channels()
{
  uint8_t channel_number;
  uint32_t channel_assignment_data;

  // 입출력 핀 정의
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(A1, INPUT);
  pinMode(A0, INPUT);
  
 delay(100);    

  Ch_Value[0] =digitalRead(4);  //4, channel 0=RTD, 1=TC 
  Ch_Value[1] = digitalRead(5);  //6, channel 0=RTD, 1=TC
  Ch_Value[2] = digitalRead(6);  //8, channel 0=RTD, 1=TC
  Ch_Value[3] = digitalRead(7);  //10, channel 0=RTD, 1=TC
  Ch_Value[4] = digitalRead(8);  //12, channel 0=RTD, 1=TC
  Ch_Value[5] = digitalRead(9);  //14, channel 0=RTD, 1=TC
  Ch_Value[6] = digitalRead(A1); //16, channel 0=RTD, 1=TC
  Ch_Value[7] = digitalRead(A0); //18, channel 0=RTD, 1=TC
  
for(int i=0;i<8;i++){
  if(Ch_Value[i]==1){
    channel_assignment_data =
    SENSOR_TYPE__TYPE_K_THERMOCOUPLE |
    TC_COLD_JUNCTION_CH__20 |
    TC_DIFFERENTIAL |
    TC_OPEN_CKT_DETECT__NO |
    TC_OPEN_CKT_DETECT_CURRENT__10UA;   
    assign_channel(CHIP_SELECT, 4+i*2, channel_assignment_data);    
  }  
  else{ 
    // ----- Channel 4: Assign RTD PT-100 -----
    channel_assignment_data =
    SENSOR_TYPE__RTD_PT_100 |
    RTD_RSENSE_CHANNEL__2 |
    RTD_NUM_WIRES__3_WIRE |
    RTD_EXCITATION_MODE__NO_ROTATION_SHARING |
    RTD_EXCITATION_CURRENT__100UA |
    RTD_STANDARD__EUROPEAN; //    
    assign_channel(CHIP_SELECT,4+i*2, channel_assignment_data);
  }
}
 // ----- Channel 2: Assign Sense Resistor -----
    channel_assignment_data =
    SENSOR_TYPE__SENSE_RESISTOR |
    (uint32_t) 0x1F4000 << SENSE_RESISTOR_VALUE_LSB;    // sense resistor - value: 2000.
    assign_channel(CHIP_SELECT, 2, channel_assignment_data);
  

  // ----- Channel 20: Assign Off-Chip Diode -----
  channel_assignment_data =
    SENSOR_TYPE__OFF_CHIP_DIODE |
    DIODE_SINGLE_ENDED |
    DIODE_NUM_READINGS__2 |
    DIODE_AVERAGING_OFF |
    DIODE_CURRENT__20UA_80UA_160UA |
    
    (uint32_t) 0x100C49 << DIODE_IDEALITY_FACTOR_LSB;   // diode - ideality factor(eta): 1.00299930572509765625
  assign_channel(CHIP_SELECT,20, channel_assignment_data); 
}
