/*
 * ATC-FAF0R_HV3_SV1.0      
 * 
 * Model Name : ATC-FAF0R
 * HardWare Version : 3.0
 * SoftWare Version : 1.0
 * Communication 1: Modbus Slave
 * Creation Date : 2017-02-16
 */
/****************************************************************************
 * This program is for ATC-FAF0R controller.
 * Below is assigned I/O pin number.
 * 
 * Digital Input (16 points) : Pin 5 ~ Pin 2
 * Digital Output (10 points) : Pin 9 ~ Pin 6
 * Analog Input (16 channels RTD) : A0 ~ A1
 * 
 * ID select pin (4 pin) : D6, D7, D8, D9 
 * Communication 1 : RxD, TxD
 * Communication 2 : None
 * **************************************************************************
 * 
 * Modbus Address Map
 * 30000 : Digital Input (IX00 ~ IX15)
 * 30001 : Digital Input 
 * 
 * 30002 : Digital Output (QX00 ~ QX09)
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
 * 30012 : Analog Output
 * 30013 : Analog Output
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
//#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <SimpleTimer.h>
#include "configuration_constants_LTC2983.h"
#include "support_functions_LTC2983_di.h"
#include "table_coeffs_LTC2983.h"

int Ch_Value[10];
void configure_channels();
void configure_channels1();
void configure_global_parameters();
void configure_global_parameters1();
 void measure_ss0();
 void measure_ss1();
 
///////////union test///////////////////////////
typedef union ftest{
  float fdata;
  unsigned int udata[2]; 
}U_SEND ;
///////////////////////////////////////////////////////
// ID 번호 선택

bool id1 = !digitalRead(6);
bool id2 = !digitalRead(7);
bool id3 = !digitalRead(8);
bool id4 = !digitalRead(9);
int id = id1 + (id2 * 2) + (id3 * 4); 

//모드버스 슬레이브의 오브젝트 생성
Modbus slave_E(id,0,0);  // 0 maeans master(1~247 slave), 0 means serial port, 0 means  RS-232 or USB-FTDI
Modbus slave(id,1,0);    // 0 maeans master(1~247 slave)


// 소프트시리얼 열기
//SoftwareSerial mySerial(18, 19);

//모드버스 데이터 어레이 변수
u16 _D[50]; // 데이터 어레이

//입출력 및 메모리 변수
bool IX00,IX01,IX02,IX03,IX04,IX05,IX06,IX07,IX08,IX09,IX10,IX11,IX12,IX13,IX14,IX15;   // 마스터의 디지털 입력변수
bool QX00,QX01,QX02,QX03,QX04,QX05,QX06,QX07,QX08,QX09,QX10,QX11,QX12,QX13,QX14,QX15;   // 마스터의 디지털 출력변수
int  IW00,IW01,IW02,IW03,IW04,IW05,IW06,IW07,IW08,IW09;              // 마스터의 아날로그 입력변수
bool MX00,MX01,MX02,MX03,MX04,MX05,MX06,MX07,MX08,MX09,MX10,MX11,MX12,MX13,MX14,MX15;    // 메모리변수
int i;

//통신 속도 및 타이머 변수
int baud_rate1 = 19200;          // LAN모듈 통신속도
int baud_rate2 = 19200;          // 시리얼(WiFi) 통신속도
int timer001,timer002,timer003,timer004,timer005; // 타이머 진행시간 변수
boolean t001,t002,t003,t004,t005; // 타이머 동작 변수

//온도센서 용 변수
int fChannel0,fChannel1,fChannel2,fChannel3,fChannel4,fChannel5,fChannel6,fChannel7;
int fChannel8,fChannel9,fChannel10,fChannel11,fChannel12,fChannel13,fChannel14,fChannel15;
int igCheck;
extern float fgAve0, fgAve1, fgAve2, fgAve3, fgAve4, fgAve5, fgAve6, fgAve7;
extern float fgAve8, fgAve9, fgAve10, fgAve11, fgAve12, fgAve13, fgAve14, fgAve15;


//타이머 및 기타변수
void fn(); 
void fntemp(); 

// 타이머 오브젝트 생성
SimpleTimer timer ;
SimpleTimer timertemp ;

void setup() {

//통신속도 디폴드 설정//
 if (id4 == 1) {                /////////// 수정
  EEPROM.put(0,19200); 
  EEPROM.put(2,19200); 
  _D[40] = 19200;
  _D[41] = 19200;
 }
  EEPROM.get(0,baud_rate1);
  EEPROM.get(2,baud_rate2);
  _D[40] = baud_rate1;
  _D[41] = baud_rate2;    
  
  pinMode(4, INPUT);
  pinMode(2, INPUT);
  pinMode(14, INPUT);
  pinMode(16, INPUT);
  pinMode(12, INPUT);
  pinMode(49, INPUT);
  pinMode(47, INPUT);
  pinMode(45, INPUT);
  
  pinMode(5, INPUT);
  pinMode(3, INPUT);
  pinMode(15, INPUT);
  pinMode(17, INPUT);
  pinMode(13, INPUT);
  pinMode(48, INPUT);
  pinMode(46, INPUT);
  pinMode(44, INPUT);
  
  
  pinMode(38, OUTPUT);
  pinMode(41, OUTPUT);
  pinMode(40, OUTPUT);
  pinMode(37, OUTPUT);
  pinMode(36, OUTPUT);
  
  pinMode(35, OUTPUT);
  pinMode(34, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(31, OUTPUT);
  
  pinMode(53, OUTPUT);  //ss0
  pinMode(10, OUTPUT);  //ss1
  digitalWrite(53, HIGH);
  digitalWrite(10, HIGH);
  
  SPI.setClockDivider(SPI_CLOCK_DIV32); 
  SPI.begin();
  
  configure_channels();  
  configure_global_parameters();  
  delay(10); 
  configure_global_parameters1(); 
  configure_channels1();
  
  // 통신속도 및 타임아웃 시간 설정  
  slave_E.begin( baud_rate1 );  
  slave.begin( baud_rate2 );  
  
  // 타이머 오브젝트 시간 설정
  timer.setInterval(100,fn);
  timertemp.setInterval(50,fntemp);
}

void loop() {
// 통신속도 변경
  if (_D[42] == 1234) {            //비밀번호 1234를 D[22]에 입력하면 통신속도 변경 
    EEPROM.put(0,_D[40]);
    EEPROM.put(2,_D[41]);
    _D[42] = 0;
    } 
    
  // Digital Input//
  IX00 = digitalRead(4);
  IX01 = digitalRead(2);
  IX02 = digitalRead(14);
  IX03 = digitalRead(16);
  
  IX04 = digitalRead(12);
  IX05 = digitalRead(49);
  IX06 = digitalRead(47);
  IX07 = digitalRead(45);
  
  IX08 = digitalRead(5);
  IX09 = digitalRead(3);
  IX10 = digitalRead(15);
  IX11 = digitalRead(17);
  
  IX12 = digitalRead(13);
  IX13 = digitalRead(48);
  IX14 = digitalRead(46);
  IX15 = digitalRead(44);
  
  // 마스터로부터 비트쓰기
  MX00 = _D[2] & 0b1; 
  MX01 = _D[2] & 0b10; 
  MX02 = _D[2] & 0b100; 
  MX03 = _D[2] & 0b1000;  
  MX04 = _D[2] & 0b10000; 
  
  MX05 = _D[2] & 0b100000; 
  MX06 = _D[2] & 0b1000000; 
  MX07 = _D[2] & 0b10000000; 
  MX08 = _D[2] & 0b100000000;  
  MX09 = _D[2] & 0b1000000000; 
  
  
  // 디지털 출력
  digitalWrite(38,QX00);          // 디지털출력 00
  digitalWrite(41,QX01);          // 디지털출력 01 
  digitalWrite(40,QX02);          // 디지털출력 02 
  digitalWrite(37,QX03);          // 디지털출력 03
  digitalWrite(36,QX04); 
  
  digitalWrite(35,QX05);
  digitalWrite(34,QX06);
  digitalWrite(33,QX07);
  digitalWrite(32,QX08);
  digitalWrite(31,QX09);
  
  measure_ss0();
  measure_ss1();

  //////////////// 0 ////////////////////////////////////////
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
  
  //////////////// 0 ////////////////////////////////////////
  U_SEND u_send8,u_send9,u_send10,u_send11,u_send12,u_send13,u_send14,u_send15;
  
  u_send8.fdata= fgAve8;  
  _D[20]=u_send8.udata[1];
  _D[21]=u_send8.udata[0];
  
  u_send9.fdata=fgAve9;
  _D[22]=u_send9.udata[1];
  _D[23]=u_send9.udata[0]; 
  
  u_send10.fdata=fgAve10;
  _D[24]=u_send10.udata[1];
  _D[25]=u_send10.udata[0];
  
  u_send11.fdata=fgAve11;
  _D[26]=u_send11.udata[1];
  _D[27]=u_send11.udata[0];
  
  u_send12.fdata=fgAve12;
  _D[28]=u_send12.udata[1];
  _D[29]=u_send12.udata[0];
  
  u_send13.fdata=fgAve13;
  _D[30]=u_send13.udata[1];
  _D[31]=u_send13.udata[0];
  
  u_send14.fdata=fgAve14;
  _D[32]=u_send14.udata[1];
  _D[33]=u_send14.udata[0];
  
  u_send15.fdata=fgAve15;
  _D[34]=u_send15.udata[1];
  _D[35]=u_send15.udata[0];
  
  
  
///////////////////////////////////////////////////////////////////////////////
// 로직 프로그램 시작
///////////////////////////////////////////////////////////////////////////////
  
  QX00 = MX00; 
  QX01 = MX01;
  QX02 = MX02;
  QX03 = MX03;
  QX04 = MX04; 
  QX05 = MX05;
  QX06 = MX06;
  QX07 = MX07;
  QX08 = MX08;
  QX09 = MX09;

//////////////////////////////////////////////////////////////////////////////////
// 로직 프로그램 끝
//////////////////////////////////////////////////////////////////////////////////

// 모드버스 어드레스 mapping  
  _D[0] = IX00 + (IX01<<1) + (IX02<<2) + (IX03<<3) + (IX04<<4) + (IX05<<5) + (IX06<<6) + (IX07<<7) + 
          (IX08<<8) + (IX09<<9) + (IX10<<10) + (IX11<<11) + (IX12<<12) + (IX13<<13) + (IX14<<14) + (IX15<<15) ;
  _D[2] =  QX00 + (QX01<<1) + (QX02<<2) + (QX03<<3) + (QX04<<4) + (QX05<<5) + (QX06<<6) + (QX07<<7) + 
          (QX08<<8) + (QX09<<9) + (QX10<<10) + (QX11<<11) + (QX12<<12) + (QX13<<13) + (QX14<<14) + (QX15<<15) ;
//    _D[2] = IW08;
//    _D[3] = IW09;
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
  
  slave_E.poll( _D, 50 );
  slave.poll( _D, 50 );
  timer.run();
  delay(1);
  
  timer.run();
  timertemp.run();
  delay(1);
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
  
  if(igCheck==8) fChannel8= STIMER;
  if(igCheck==9) fChannel9= STIMER;
  if(igCheck==10) fChannel10= STIMER;
  if(igCheck==11) fChannel11= STIMER;
  if(igCheck==12) fChannel12= STIMER;
  if(igCheck==13) fChannel13= STIMER;
  if(igCheck==14) fChannel14= STIMER;
  if(igCheck==15) fChannel15= STIMER;  
  igCheck++;
  if(igCheck >= 16) { igCheck=0;}
}
void measure_ss0(){
////////////////////// 0 ////////////////////  
  if(fChannel0==STIMER){
    fChannel0 = SVALUE;
    measure_channel_f (CHIP_SELECT0, 4, TEMPERATURE,0); 
    //  Serial.println("0========================");
   //Serial.println(fgAve0);
  }
  
  if(fChannel1==STIMER){
    fChannel1= SVALUE;
    measure_channel_f(CHIP_SELECT0, 6, TEMPERATURE,0);
    //Serial.println(fgAve1);
  }    
  if(fChannel2==STIMER){
    fChannel2= SVALUE;  
    measure_channel_f(CHIP_SELECT0, 8, TEMPERATURE,0);  
    //Serial.println(fgAve2);
  }
  if(fChannel3==STIMER){
    fChannel3= SVALUE;
    measure_channel_f(CHIP_SELECT0, 10, TEMPERATURE,0);
    //Serial.println(fgAve3);
  }
  if(fChannel4==STIMER){
    fChannel4= SVALUE;
    measure_channel_f(CHIP_SELECT0, 12, TEMPERATURE,0);  
    //Serial.println(fgAve4);
  }
  
  if(fChannel5==STIMER){
    fChannel5= SVALUE;
    measure_channel_f(CHIP_SELECT0, 14, TEMPERATURE,0);  
      //Serial.println(fgAve5   );
  }
  if(fChannel6==STIMER){
    fChannel6= SVALUE;
    measure_channel_f(CHIP_SELECT0, 16, TEMPERATURE,0);  
    //Serial.println(fgAve6);
  }
  if(fChannel7==STIMER){
    fChannel7= SVALUE;
    measure_channel_f(CHIP_SELECT0, 18, TEMPERATURE,0);  
    //Serial.println(fgAve7);
  }
}

void measure_ss1(){
/////////// ss1 ////////////////////  
  if(fChannel8==STIMER){
    fChannel8 = SVALUE;
    measure_channel_g (CHIP_SELECT1, 4, TEMPERATURE,0); 
        //Serial.println("1========================");
    //Serial.println(fgAve8);
  }

  if(fChannel9==STIMER){
    fChannel9= SVALUE;
    measure_channel_g(CHIP_SELECT1, 6, TEMPERATURE,0);
    //Serial.println(fgAve9);
  }   
     
  if(fChannel10==STIMER){
    fChannel10= SVALUE;  
    measure_channel_g(CHIP_SELECT1, 8, TEMPERATURE,0);  
    //Serial.println(fgAve10);
  }

  if(fChannel11==STIMER){
    fChannel11= SVALUE;
    measure_channel_g(CHIP_SELECT1, 10, TEMPERATURE,0);
    //Serial.println(fgAve11);
  }
  if(fChannel12==STIMER){
    fChannel12= SVALUE;
    measure_channel_g(CHIP_SELECT1, 12, TEMPERATURE,0);  
   // Serial.println(fgAve12);
  }    
  if(fChannel13==STIMER){
    fChannel13= SVALUE;
    measure_channel_g(CHIP_SELECT1, 14, TEMPERATURE,0);  
      //Serial.println(fgAve13 );
  }
  if(fChannel14==STIMER){
    fChannel14= SVALUE;
    measure_channel_g(CHIP_SELECT1, 16, TEMPERATURE,0);  
    //Serial.println(fgAve14);
  }
  if(fChannel15==STIMER){
    fChannel15= SVALUE;
    measure_channel_g(CHIP_SELECT1, 18, TEMPERATURE,0);  
    //Serial.println(fgAve15);
   //  Serial.println("2========================");
  }
}
/////////////ss0////////////////
void configure_global_parameters()
{
  // -- Set global parameters
  transfer_byte(CHIP_SELECT0, WRITE_TO_RAM, 0xF0, TEMP_UNIT__C |
                REJECTION__50_60_HZ);
  // -- Set any extra delay between conversions (in this case, 0*100us)
  transfer_byte(CHIP_SELECT0, WRITE_TO_RAM, 0xFF, 0);
}

void configure_channels()
{
  uint8_t channel_number;
  uint32_t channel_assignment_data;
  for(int i=0;i<8;i++){
  
    // ----- Channel 4: Assign RTD PT-100 -----
    channel_assignment_data =
    SENSOR_TYPE__RTD_PT_100 |
    RTD_RSENSE_CHANNEL__2 |
    RTD_NUM_WIRES__3_WIRE |
    RTD_EXCITATION_MODE__NO_ROTATION_SHARING |
    RTD_EXCITATION_CURRENT__100UA |
    RTD_STANDARD__EUROPEAN; //    
    assign_channel(CHIP_SELECT0,4+i*2, channel_assignment_data);
 
}
 // ----- Channel 2: Assign Sense Resistor -----
    channel_assignment_data =
    SENSOR_TYPE__SENSE_RESISTOR |
    (uint32_t) 0x1F4000 << SENSE_RESISTOR_VALUE_LSB;    // sense resistor - value: 2000.
    assign_channel(CHIP_SELECT0, 2, channel_assignment_data);
  

  // ----- Channel 20: Assign Off-Chip Diode -----
  channel_assignment_data =
    SENSOR_TYPE__OFF_CHIP_DIODE |
    DIODE_SINGLE_ENDED |
    DIODE_NUM_READINGS__2 |
    DIODE_AVERAGING_OFF |
    DIODE_CURRENT__20UA_80UA_160UA |
    
    (uint32_t) 0x100C49 << DIODE_IDEALITY_FACTOR_LSB;   // diode - ideality factor(eta): 1.00299930572509765625
  assign_channel(CHIP_SELECT0,20, channel_assignment_data); 
}

/////////////////ss1//////////////
void configure_global_parameters1()
{  
  // -- Set global parameters
  transfer_byte(CHIP_SELECT1, WRITE_TO_RAM, 0xF0, TEMP_UNIT__C |
                REJECTION__50_60_HZ);
  // -- Set any extra delay between conversions (in this case, 0*100us)
  transfer_byte(CHIP_SELECT1, WRITE_TO_RAM, 0xFF, 0); 
}


void configure_channels1()
{ 
 // uint8_t channel_number;
  uint32_t channel_assignment_data;  
  for(int i=0;i<8;i++){  
    // ----- Channel 4: Assign RTD PT-100 -----
    channel_assignment_data =
    SENSOR_TYPE__RTD_PT_100 |
    RTD_RSENSE_CHANNEL__2 |
    RTD_NUM_WIRES__3_WIRE |
    RTD_EXCITATION_MODE__NO_ROTATION_SHARING |
    RTD_EXCITATION_CURRENT__100UA |
    RTD_STANDARD__EUROPEAN; //    
    assign_channel(CHIP_SELECT1,4+i*2, channel_assignment_data); 
    delay(1);
  }
 // ----- Channel 2: Assign Sense Resistor -----
    channel_assignment_data =
    SENSOR_TYPE__SENSE_RESISTOR |
    (uint32_t) 0x1F4000 << SENSE_RESISTOR_VALUE_LSB;    // sense resistor - value: 2000.
    assign_channel(CHIP_SELECT1, 2, channel_assignment_data);  

  // ----- Channel 20: Assign Off-Chip Diode -----
  channel_assignment_data =
    SENSOR_TYPE__OFF_CHIP_DIODE |
    DIODE_SINGLE_ENDED |
    DIODE_NUM_READINGS__2 |
    DIODE_AVERAGING_OFF |
    DIODE_CURRENT__20UA_80UA_160UA |
    
    (uint32_t) 0x100C49 << DIODE_IDEALITY_FACTOR_LSB;   // diode - ideality factor(eta): 1.00299930572509765625
  assign_channel(CHIP_SELECT1,20, channel_assignment_data); 
  
}
