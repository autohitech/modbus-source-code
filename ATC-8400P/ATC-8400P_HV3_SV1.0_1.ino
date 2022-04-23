
/*
 * ATC-8400P_HV3_SV1.0_1
 * 
 * Model Name : ATC-8400P
 * HardWare Version : 3.0
 * SoftWare Version : 1.0
 * Communication 1: Modbus Slave
 * Communication 2: Modbus Slave
 * Creation Date : 2017-03-27
 */
/****************************************************************************
 * This program is for ATC-8400P-ES controller.
 * Below is assigned I/O pin number.
 * 
 * Digital Input (4 points) : Pin 2 ~ Pin 5
 * Digital Output (4 points) : Pin 6 ~ Pin 9
 * Analog Input (2 channels) : A0 ~ A1
 * Analog Output (2 channels) : Pin 10 ~ Pin 11 
 * ID select pin (4 pin) : A2(b0), A3(b1), A4(b2), A5(b3) 
 * Communication 1 : Slave     RxD(0), TxD(1) Serial module 
 * Communication 2 : Slave_E   RxD(2), TxD(3) Ethernet Module
 * **************************************************************************
 * 
 * Modbus Address Map
 * 30000 : Digital Input (IX00 ~ IX07)
 * 30001 : Digital Input Spare
 * 
 * 30002 : Digital Output (QX00 ~ QX03)
 * 30003 : Digital Output Spare
 * 
 * 30004 : Analog Input
 * 30005 : Analog Input
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

#include <ModbusRtu.h>
#include <EEPROM.h>
#include <SimpleTimer.h>
#include <SoftwareSerial.h>

// ID 번호 선택
bool id1 = !digitalRead(13);
bool id2 = !digitalRead(10);
bool id3 = !digitalRead(11);
bool id4 = !digitalRead(12);
int id = id1 + (id2 * 2) + (id3 * 4) + (id4 * 8);

//모드버스 슬레이브의 오브젝트 생성
Modbus slave(id,0,0);  // 0 maeans master(1~247 slave), 0 means serial port, 0 means  RS-232 or USB-FTDI
Modbus slave_E(id);    // 0 maeans master(1~247 slave)

//소프트웨어 시리얼 열기
SoftwareSerial mySerial(2,3);

// 타이머 오브젝트 생성
SimpleTimer timer;

//변수선언
bool IX00,IX01,IX02,IX03;   // 디지털 입력변수
bool IX04,IX05,IX06,IX07;   // 디지털 입력변수
bool QX00,QX01,QX02,QX03;   // 디지털 출력변수
bool MX00,MX01,MX02,MX03;   // 내부변수
int i;

int baud_rate1 = 19200;          // LAN모듈 통신속도
int baud_rate2 = 19200;          // 시리얼(WiFi) 통신속도
int timer000,timer001,timer002;  // 타이머 진행시간 변수
int timer003,timer004,timer005;  // 타이머 진행시간 변수
boolean t000,t001,t002,t003;     // 타이머 동작 변수
boolean t004,t005;     // 타이머 동작 변수

u16 _D[50];       // 데이터 어레이
void fn();        //타이머 및 기타변수


void setup() {

//통신속도 디폴드 설정//
 if (id4 == 1) { 
  EEPROM.put(0,19200); 
  EEPROM.put(2,19200); 
  _D[21] = 19200;
  _D[22] = 19200;
 }
   EEPROM.get(0,baud_rate1);
   EEPROM.get(2,baud_rate2);
   _D[21] = baud_rate1;
   _D[22] = baud_rate2;  
   
// 입출력 핀 정의
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);
    pinMode(A4, INPUT);
    pinMode(A5, INPUT);
    pinMode(5, INPUT);
    pinMode(4, INPUT);   

    pinMode(9, OUTPUT);  
    pinMode(8, OUTPUT);  
    pinMode(7, OUTPUT);
    pinMode(6, OUTPUT);
    
// 통신속도 및 타임아웃 시간 설정
  slave.begin( baud_rate1 );  
 slave_E.begin( &mySerial, baud_rate2 ); 
// 타이머 오브젝트 시간 설정
   timer.setInterval(100,fn);
   
}

void loop() {

// 통신속도 변경
  if (_D[23] == 1234) {            //비밀번호 1234를 D[22]에 입력하면 통신속도 변경 
    EEPROM.put(0,_D[21]);
    EEPROM.put(2,_D[22]);
    _D[23] = 0;
    } 

// 디지털 입력 읽기
    IX00 = digitalRead(A0);    
    IX01 = digitalRead(A1);
    IX02 = digitalRead(A2);
    IX03 = digitalRead(A3);
    IX04 = digitalRead(A4);    
    IX05 = digitalRead(A5);
    IX06 = digitalRead(5);
    IX07 = digitalRead(4);


// 마스터로부터 비트쓰기
    MX00 = _D[2] & 0x0001; 
    MX01 = _D[2] & 0x0002; 
    MX02 = _D[2] & 0x0004; 
    MX03 = _D[2] & 0x0008;       
    
///////////////////////////////////////////////////////////////////////////////
// 로직 프로그램 시작
///////////////////////////////////////////////////////////////////////////////

QX00 = MX00; 
QX01 = MX01;
QX02 = MX02;
QX03 = MX03;

//////////////////////////////////////////////////////////////////////////////////
// 로직 프로그램 끝
//////////////////////////////////////////////////////////////////////////////////

// 디지털 출력
    digitalWrite(9,!QX00);          // 디지털출력 00
    digitalWrite(8,!QX01);          // 디지털출력 01 
    digitalWrite(7,!QX02);          // 디지털출력 02 
    digitalWrite(6,!QX03);          // 디지털출력 03



 // 모드버스 어드레스 mapping  
    _D[0] = IX00 + (IX01*2) + (IX02*4) + (IX03*8) + (IX04*16) + (IX05*32) + (IX06*64) + (IX07*128);
    _D[1] = 0;
    _D[2] = QX00 + (QX01*2) + (QX02*4) + (QX03*8);
    _D[3] = 0;

  slave.poll( _D, 50 );
  slave_E.poll( _D, 50 );
    timer.run();
    delay(10);
}

void fn() {
  if (t000 == 1) {timer000 = timer000 + 1;}
  if (t000 == 0) {timer000 = 0;}
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


