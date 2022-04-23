/*
 * ATC-A800_HV1_SV1.0_2     
 * 
 * Model Name : ATC-A800
 * HardWare Version : 1.0
 * SoftWare Version : 1.0
 * Communication 1: Modbus Slave
 * Creation Date : 2016-12-10
 */
/****************************************************************************
 * This program is for ATC-A800 controller.
 * Below is assigned I/O pin number.
 * 
 * Digital Input (10 points) : Pin 11 ~ Pin 2
 * Digital Output (8 points) : Pin 12 ~ 13, A0 ~ A5
 * Analog Input (0 channels) : None
 * Analog Output (0 channels) : None
 * ID select pin (4 pin) : Program
 * Communication 1 : RxD, TxD
 * Communication 2 : None
 * **************************************************************************
 * 
 * Modbus Address Map
 * 30000 : Digital Input (IX00 ~ IX09)
 * 30001 : Digital Input 
 * 
 * 30002 : Digital Output (QX00 ~ QX07)
 * 30003 : Digital Output
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

#include <ModbusRtu.h>
#include <EEPROM.h>
#include <SimpleTimer.h>

// ID 번호 선택
bool id1 = !(digitalRead(A5));
bool id2 = !(digitalRead(A4));
int id = id1 + (id2 * 2);

//모드버스 마스터의  통신포트 및 통신텔레그램 오브젝트 생성
Modbus slave(id,0,0);  // 0 maeans master(1~247 slave), 0 means serial port, 0 means  RS-232 or USB-FTDI
//모드버스 데이터 어레이 변수
u16 _D[50]; // 데이터 어레이

//입출력 및 메모리 변수
bool IX00,IX01,IX02,IX03,IX04,IX05,IX06,IX07,IX08,IX09;   // 디지털 입력변수
bool QX00,QX01,QX02,QX03,QX04,QX05,QX06,QX07;   // 디지털 출력변수
bool MX00,MX01,MX02,MX03,MX04,MX05,MX06,MX07;    // 메모리변수
int i;

//통신 속도 및 타이머 변수
int baud_rate = 19200; // 시리얼 통신속도
int timer001,timer002,timer003,timer004,timer005; // 타이머 진행시간 변수
boolean t001,t002,t003,t004,t005; // 타이머 동작 변수

//타이머 및 기타변수
void fn(); 

// 타이머 오브젝트 생성
SimpleTimer timer;



void setup() {

//통신속도 디폴드 설정//
 if (id1 == 1) { EEPROM.put(21,19200); _D[21] = 19200;}
   EEPROM.get(21,baud_rate);
   _D[21] = baud_rate;

// 입출력 핀 정의
    pinMode(2, INPUT);
    pinMode(3, INPUT);
    pinMode(4, INPUT);
    pinMode(5, INPUT);
    pinMode(6, INPUT);
    pinMode(7, INPUT);
    pinMode(8, INPUT);
    pinMode(9, INPUT);
    pinMode(10, INPUT);
    pinMode(11, INPUT);
    pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);
    pinMode(A0, OUTPUT);
    pinMode(A1, OUTPUT);
    pinMode(A2, OUTPUT);
    pinMode(A3, OUTPUT);
    pinMode(A4, OUTPUT);
    pinMode(A5, OUTPUT);
   
// Commpunication speed & timeout for Slave //
  slave.begin( baud_rate ); 

// Timer //
   timer.setInterval(100,fn);
    
// 통신속도 및 타임아웃 시간 설정
  slave.begin( baud_rate );  

// 타이머 오브젝트 시간 설정
   timer.setInterval(100,fn);
   
}


 
void loop() {

// 통신속도 변경
  if (_D[22] == 1234) { EEPROM.put(21,_D[21]); _D[22] = 0;} //비밀번호 1234를 D[22]에 입력하면 통신속도 변경

// 디지털 입력 읽기
    IX00 = digitalRead(11);
    IX01 = digitalRead(10);
    IX02 = digitalRead(9);
    IX03 = digitalRead(8);
    IX04 = digitalRead(7);
    IX05 = digitalRead(6);
    IX06 = digitalRead(5);
    IX07 = digitalRead(4);
    IX08 = digitalRead(3);
    IX09 = digitalRead(2);

// 마스터로부터 비트쓰기
    MX00 = _D[2] & 0x0001;
    MX01 = _D[2] & 0x0002;
    MX02 = _D[2] & 0x0004;
    MX03 = _D[2] & 0x0008;      
    MX04 = _D[2] & 0x0010;
    MX05 = _D[2] & 0x0020;
    MX06 = _D[2] & 0x0040;
    MX07 = _D[2] & 0x0080; 
     
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

//////////////////////////////////////////////////////////////////////////////////
// 로직 프로그램 끝
//////////////////////////////////////////////////////////////////////////////////

// 디지털 출력
    digitalWrite(12,QX00);            // 디지털출력 00  
    digitalWrite(13,QX01);            // 디지털출력 01  
    digitalWrite(A0,QX02);            // 디지털출력 02  
    digitalWrite(A1,QX03);            // 디지털출력 03 
    digitalWrite(A2,QX04);            // 디지털출력 04  
    digitalWrite(A3,QX05);            // 디지털출력 05 
    digitalWrite(A4,QX06);            // 디지털출력 06  
    digitalWrite(A5,QX07);            // 디지털출력 07 

   
  // 모드버스 어드레스 mapping  
    _D[0] = IX00 + (IX01*2) + (IX02*4) + (IX03*8) + (IX04*16) + (IX05*32) + (IX06*64) + (IX07*128) + (IX08*256) + (IX09*512);
    _D[1] = 0;
    _D[2] = QX00 + (QX01*2) + (QX02*4) + (QX03*8) + (QX04*16) + (QX05*32) + (QX06*64) + (QX07*128);
    _D[3] = 0;
    _D[4] = 0;
    _D[5] = 0;
    _D[6] = 0;
    _D[7] = 0;
    _D[8] = 0;
    _D[9] = 0;
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


