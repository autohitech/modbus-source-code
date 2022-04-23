/*
 * ATC-0066_HV5_SV1.0      
 * 
 * Model Name : ATC-0066
 * HardWare Version : 5.0
 * SoftWare Version : 1.0
 * Communication 1: Modbus Slave
 * Creation Date : 2017-06-21
 */
/****************************************************************************
 * This program is for ATC-0066 controller.
 * Below is assigned I/O pin number.
 * 
  * Analog Input (2 channels) : A0 ~ A5
 * Analog Output (2 channels) : Pin 11,10,9,6,5,3 
 * ID select pin (4 pin) : D2,4,7,8
 * Communication 1 : RxD, TxD
 * Communication 2 : None
 * **************************************************************************
 * 
 * Modbus Address Map
 * 30000 : Digital Input
 * 30001 : Digital Input 
 * 
 * 30002 : Digital Output
 * 30003 : Digital Output
 * 
 * 30004 : Analog Input (IW00)
 * 30005 : Analog Input (IW01)
 * 30006 : Analog Input (IW02)
 * 30007 : Analog Input (IW03)
 * 30008 : Analog Input (IW04)
 * 30009 : Analog Input (IW05)
 * 30010 : Analog Input
 * 30011 : Analog Input
 * 
 * 30012 : Analog Output (QW00)
 * 30013 : Analog Output (QW01)
 * 30014 : Analog Output (QW02)
 * 30015 : Analog Output (QW03)
 * 30016 : Analog Output (QW04)
 * 30017 : Analog Output (QW05)
 * 30018 : Analog Output
 * 30019 : Analog Output
 * 
 */

#include <ModbusRtu.h>
#include <EEPROM.h>
#include <SimpleTimer.h>

// ID 번호 선택
bool id0 = !digitalRead(2);
bool id1 = !digitalRead(4);
bool id2 = !digitalRead(7);
bool id3 = !digitalRead(8);
int id = id1 + (id2 * 2) + (id3 * 4);

//모드버스 마스터의  통신포트 및 통신텔레그램 오브젝트 생성
Modbus slave(id,0,0);  // 0 maeans master(1~247 slave), 0 means serial port, 0 means  RS-232 or USB-FTDI

//모드버스 데이터 어레이 변수
u16 _D[50]; // 데이터 어레이

//입출력 및 메모리 변수
int IW00,IW01,IW02,IW03,IW04,IW05;              // 마스터의 아날로그 입력변수
int QW00,QW01,QW02,QW03,QW04,QW05;              // 마스터의 아날로그 출력변수
bool MX00,MX01,MX02,MX03,MX04,MX05,MX06,MX07;    // 메모리변수
int MW00,MW01,MW02,MW03,MW04,MW05,MW06,MW07;     // 메모리변수
int MW10,MW11,MW12,MW13,MW14,MW15,MW16,MW17;     // 메모리변수
int i;

//통신 속도 및 타이머 변수
int baud_rate = 19200; // 시리얼 통신속도
int timer001,timer002,timer003,timer004,timer005; // 타이머 진행시간 변수
boolean t001,t002,t003,t004,t005; // 타이머 동작 변수

//아날로그 입력(평균값) 처리 변수
const int numRead = 30;
const int numReadings00 = numRead;
const int numReadings01 = numRead;
const int numReadings02 = numRead;
const int numReadings03 = numRead;
const int numReadings04 = numRead;
const int numReadings05 = numRead;

int readings00[numReadings00];      // the readings from the Analog 0 input 
int readings01[numReadings01];      // the readings from the Analog 1 input
int readings02[numReadings02];      // the readings from the Analog 2 input 
int readings03[numReadings03];      // the readings from the Analog 3 input
int readings04[numReadings04];      // the readings from the Analog 4 input 
int readings05[numReadings05];      // the readings from the Analog 5 input

int readIndex00,readIndex01,readIndex02,readIndex03,readIndex04,readIndex05;  // the index of the current Analog 0-5 reading
int total00 , total01, total02, total03, total04, total05;                    // the running total of Analog 0-5
int average00,average01,average02,average03,average04,average05 ;             // the average of Analog 0-5


//타이머 및 기타변수
void fn(); 

// 타이머 오브젝트 생성
SimpleTimer timer;



void setup() {

//통신속도 디폴드 설정//
 if (id0 == 1) { EEPROM.put(21,19200); _D[21] = 19200;}
   EEPROM.get(21,baud_rate);
   _D[21] = baud_rate;


    
// 통신속도 및 타임아웃 시간 설정
  slave.begin( baud_rate );  

// 타이머 오브젝트 시간 설정
   timer.setInterval(20,fn);   
}



void loop() {

// 통신속도 변경
  if (_D[22] == 1234) { EEPROM.put(21,_D[21]); _D[22] = 0;} //비밀번호 1234를 D[22]에 입력하면 통신속도 변경



 // 아날로그 입력 읽기
   total00 = total00 - readings00[readIndex00]; // subtract the last reading:
   total01 = total01 - readings01[readIndex01]; // subtract the last reading:
   total02 = total02 - readings02[readIndex02]; // subtract the last reading:
   total03 = total03 - readings03[readIndex03]; // subtract the last reading:
   total04 = total04 - readings04[readIndex04]; // subtract the last reading:
   total05 = total05 - readings05[readIndex05]; // subtract the last reading:
   
   readings00[readIndex00] = analogRead(A0); // read from the sensor:
   readings01[readIndex01] = analogRead(A1); // read from the sensor:
   readings02[readIndex02] = analogRead(A2); // read from the sensor:
   readings03[readIndex03] = analogRead(A3); // read from the sensor:
   readings04[readIndex04] = analogRead(A4); // read from the sensor:
   readings05[readIndex05] = analogRead(A5); // read from the sensor:   
   
   total00 = total00 + readings00[readIndex00]; // add the reading to the total:
   total01 = total01 + readings01[readIndex01]; // add the reading to the total:
   total02 = total02 + readings02[readIndex02]; // add the reading to the total:
   total03 = total03 + readings03[readIndex03]; // add the reading to the total:
   total04 = total04 + readings04[readIndex04]; // add the reading to the total:
   total05 = total05 + readings05[readIndex05]; // add the reading to the total:

   
   readIndex00 = readIndex00 + 1; // advance to the next position in the array:
   readIndex01 = readIndex01 + 1; // advance to the next position in the array:
   readIndex02 = readIndex02 + 1; // advance to the next position in the array:
   readIndex03 = readIndex03 + 1; // advance to the next position in the array:
   readIndex04 = readIndex04 + 1; // advance to the next position in the array:
   readIndex05 = readIndex05 + 1; // advance to the next position in the array:

      
   if (readIndex00 >= numReadings00) { readIndex00 = 0;}
   if (readIndex01 >= numReadings01) { readIndex01 = 0;}
   if (readIndex02 >= numReadings02) { readIndex02 = 0;}
   if (readIndex03 >= numReadings03) { readIndex03 = 0;}   
   if (readIndex04 >= numReadings04) { readIndex04 = 0;}
   if (readIndex05 >= numReadings05) { readIndex05 = 0;} 

   
   IW00 = total00 / numReadings00;                   //아날로그입력 00
   IW01 = total01 / numReadings01;                   //아날로그입력 01 
   IW02 = total02 / numReadings02;                   //아날로그입력 02
   IW03 = total03 / numReadings03;                   //아날로그입력 03   
   IW04 = total04 / numReadings04;                   //아날로그입력 04
   IW05 = total05 / numReadings05;                   //아날로그입력 05 


// 마스터로부터 아날로그 쓰기
    MW12 = _D[12];
    MW13 = _D[13];    
    MW14 = _D[14];
    MW15 = _D[15];
    MW16 = _D[16];
    MW17 = _D[17]; 
    
///////////////////////////////////////////////////////////////////////////////
// 로직 프로그램 시작
///////////////////////////////////////////////////////////////////////////////

QW00 = MW12;
QW01 = MW13;
QW02 = MW14;
QW03 = MW15;
QW04 = MW16;
QW05 = MW17;

//////////////////////////////////////////////////////////////////////////////////
// 로직 프로그램 끝
//////////////////////////////////////////////////////////////////////////////////


 // 아날로그 출력
    analogWrite(11,QW00);          // 아날로그출력 00
    analogWrite(10,QW01);          // 아날로그출력 01
    analogWrite(9,QW02);          // 아날로그출력 02
    analogWrite(6,QW03);          // 아날로그출력 03
    analogWrite(5,QW04);          // 아날로그출력 04
    analogWrite(3,QW05);          // 아날로그출력 05

 // 모드버스 어드레스 mapping  
    _D[0] = 0;
    _D[1] = 0;
    _D[2] =0;
    _D[3] = 0;
    _D[4] = IW00;
    _D[5] = IW01;
    _D[6] = IW02;
    _D[7] = IW03;
    _D[8] = IW04;
    _D[9] = IW05;
    _D[10] = 0;
    _D[11] = 0; 

  slave.poll( _D, 50 );
    timer.run();
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

