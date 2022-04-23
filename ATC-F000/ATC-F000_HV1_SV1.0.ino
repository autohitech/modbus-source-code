/*
 * ATC-F000_HV1_SV1.0      
 * 
 * Model Name : ATC-F000
 * HardWare Version : 1.0
 * SoftWare Version : 1.0
 * Communication 1: Modbus Slave
 * Creation Date : 2017-01-04
 */
/****************************************************************************
 * This program is for ATC-F000 controller.
 * Below is assigned I/O pin number.  
 * Digital Input (16 points) : Pin13 ~ Pin2, A0-A3 
 * Communication 1 : RxD, TxD
 * Communication 2 : None
 * **************************************************************************
 * 
 * Modbus Address Map
 * 30000 : Digital Input (IX00 ~ IX15)
 * 30001 : Digital Input 
 * 
 * 30002 : Digital Output
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

// ID 번호 선택
bool id1 = !digitalRead(A4);
bool id2 = !digitalRead(A5);
int id = (id1 + (id2 * 2));

//모드버스 마스터의  통신포트 및 통신텔레그램 오브젝트 생성
Modbus slave(id,0,0);  // 0 maeans master(1~247 slave), 0 means serial port, 0 means  RS-232 or USB-FTDI

//모드버스 데이터 어레이 변수
u16 _D[50]; // 데이터 어레이

//입출력 및 메모리 변수
bool IX00,IX01,IX02,IX03,IX04,IX05,IX06,IX07,IX08,IX09,IX10,IX11,IX12,IX13,IX14,IX15;   // 디지털 입력변수
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

//Communication Speed//
if (id2 == 1) {EEPROM.put(21,19200); _D[21] = 19200;}
EEPROM.get(21,baud_rate);
_D[21] = baud_rate;

// 입력 핀 정의
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
    pinMode(12, INPUT);
    pinMode(13, INPUT);
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);
// 통신속도 및 타임아웃 시간 설정
  slave.begin( baud_rate );  

// 타이머 오브젝트 시간 설정
   timer.setInterval(100,fn);
   
}


void loop() {

// 통신속도 변경
  if (_D[22] == 1234) { EEPROM.put(21,_D[21]); _D[22] = 0;} //비밀번호 1234를 D[22]에 입력하면 통신속도 변경

// 디지털 입력 읽기
    IX00 = digitalRead(13);
    IX01 = digitalRead(12);
    IX02 = digitalRead(11);
    IX03 = digitalRead(10);
    IX04 = digitalRead(9);
    IX05 = digitalRead(8);
    IX06 = digitalRead(7);
    IX07 = digitalRead(6);      
    IX08 = digitalRead(5);
    IX09 = digitalRead(4);
    IX10 = digitalRead(3);
    IX11 = digitalRead(2);
    IX12 = digitalRead(A0);
    IX13 = digitalRead(A1);
    IX14 = digitalRead(A2);
    IX15 = digitalRead(A3);


///////////////////////////////////////////////////////////////////////////////
// 로직 프로그램 시작
///////////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////////////////
// 로직 프로그램 끝
//////////////////////////////////////////////////////////////////////////////////

 // MODBUS ADDRESS//    
    _D[0] = IX00 + (IX01<<1) + (IX02<<2) + (IX03<<3) + (IX04<<4) + (IX05<<5) + (IX06<<6) + (IX07<<7) + (IX08<<8) + (IX09<<9) + (IX10<<10) + (IX11<<11) + (IX12<<12) + (IX13<<13) + (IX14<<14) + (IX15<<15) ;

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



