/*
 * ATC-C420_HV2_SV1.0      
 * 
 * Model Name : ATC-C420
 * HardWare Version : 2.0
 * SoftWare Version : 1.0
 * Communication 1: Modbus Slave
 * Creation Date : 2016-12-10
 */
/****************************************************************************
 * This program is for ATC-C420 controller.
 * Below is assigned I/O pin number.
 * 
 * Digital Input (12 points) : Pin 2 ~ Pin 13
 * Digital Output (4 points) : A0 ~ A3
 * Analog Input (2 channels) : A6, A7
 * Analog Output (0 channels) : None
 * ID select pin (4 pin) : A4(b0), A5(b1) 
 * Communication 1 : RxD, TxD
 * Communication 2 : None
 * **************************************************************************
 * 
 * Modbus Address Map
 * 30000 : Digital Input (IX00 ~ IX11)
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
bool id1 = !(digitalRead(A5));
bool id2 = !(digitalRead(A4));
int id = id1 + (id2 * 2);

//모드버스 마스터의  통신포트 및 통신텔레그램 오브젝트 생성
Modbus slave(id,0,0);  // 0 maeans master(1~247 slave), 0 means serial port, 0 means  RS-232 or USB-FTDI
//모드버스 데이터 어레이 변수
u16 _D[50]; // 데이터 어레이

//입출력 및 메모리 변수
bool IX00,IX01,IX02,IX03,IX04,IX05,IX06,IX07,IX08,IX09,IX10,IX11;   // 디지털 입력변수
bool QX00,QX01,QX02,QX03;   // 디지털 출력변수
int IW00,IW01;              // 아날로그 입력변수
bool MX00,MX01,MX02,MX03;    // 메모리변수
int i;

//통신 속도 및 타이머 변수
int baud_rate = 19200; // 시리얼 통신속도
int timer001,timer002,timer003,timer004,timer005; // 타이머 진행시간 변수
boolean t001,t002,t003,t004,t005; // 타이머 동작 변수

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

// 타이머 오브젝트 생성
SimpleTimer timer;



void setup() {

//통신속도 디폴드 설정//
 if (id2 == 1) { EEPROM.put(21,19200); _D[21] = 19200;}
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
    pinMode(12, INPUT);
    pinMode(13, INPUT);
    pinMode(A0, OUTPUT);
    pinMode(A1, OUTPUT);
    pinMode(A2, OUTPUT);
    pinMode(A3, OUTPUT);
   
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
   IW00 = total00 / numReadings00;                                    //아날로그입력 00
   IW01 = total01 / numReadings01;                                    //아날로그입력 01 

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
    digitalWrite(A0,QX00);          // 디지털출력 00 
    digitalWrite(A1,QX01);          // 디지털출력 01 
    digitalWrite(A2,QX02);          // 디지털출력 02
    digitalWrite(A3,QX03);          // 디지털출력 03
    
  // 모드버스 어드레스 mapping  
    _D[0] = IX00 + (IX01*2) + (IX02*4) + (IX03*8) + (IX04*16) + (IX05*32) + (IX06*64) + (IX07*128) + (IX08*256) + (IX09*512) + (IX10*1024) + (IX11*2048);
    _D[1] = 0;
    _D[2] = QX00 + (QX01*2) + (QX02*4) + (QX03*8);
    _D[3] = 0;
    _D[4] = IW00;
    _D[5] = IW01;
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


