/*
// **************************************************************************
// ATC-SDAE_V4.0     SP0 : Modbus Slave     SP1 : Modbus Slave
// **************************************************************************
* SM0 : DO4 -- RTD/AI -- DO4
* SM0 : DO4 -- RTD/AI -- DO4
* SM0 : DO4 -- RTD/AI -- DO4
* * 
* Communication 1: Modbus Slave  (SP00)
* Communication 2: Modbus Slave (SP01)
* 
* Creation Date : 2021-05-30
* Programer : Hosoon, Lee
* **************************************************************************
* Pin Assignment
* 
* DIP S/W : D04(SW1), D11(SW2), D12(SW3), D13(SW4) 
* SM 0 : A00, A01, D07, D08
* SM 1 : A02, A03, A06, A07
* SM 2 : D10, D09, D06, D05
* 
* ***************************************************************************
* Modbus Address Map 
* 
* 40000 : SM00 (QX00 ~ QX03)
* 40001 : SM01 (RTD10)
* 40002 : SM01 (IW11)
* 40003 : SM02 (QX20 ~ QX23)
* 
* 
* 40010 : SM1 RTD LOWER VALUE (0도)
* 40011 : SM1 RTD UPPER VALUE (65도)
* 40012 : SP00 baud 통신속도(0:4800, 1:9600, 2:19200, 3:38400)
* 40013 : SP01 baud 통신속도(0:4800, 1:9600, 2:19200, 3:38400)
* 40014 : Reserve
* 40015 : Bit write  b0(MX00). b1(MX01), b2(MX02), b3(MX03), b4(MX04). b5(MX05), b6(MX06), b7(MX07))  
* 
* 40020 : Password (통신속도 변경시 1234를 입력)
// **************************************************************************
*/


//================================================================================
//  S  T  A  R  T              DO4 - RTD/AI - DO4 
//================================================================================
#include <ModbusRtu.h>
#include <EEPROM.h>
#include <SimpleTimer.h>
#include <SoftwareSerial.h> 

// ID 번호 선택
bool id1 = !digitalRead(4);
bool id2 = !digitalRead(11);
bool id3 = !digitalRead(12);
bool id4 = !digitalRead(13);
int id = id1 + (id2 * 2) + (id3 * 4);

//통신 속도
byte baud0 = 2, baud1 = 2;          // 통신속도(0,1,2...)
word baudrate0, baudrate1;  // 통신속도(4800,9600,19200...)

//변수선언
bool QX00, QX01, QX02, QX03;   // 디지털 출력변수(SM0)
int RTD10, IW11;                           // 아날로그 입력변수(SM1)
bool QX20, QX21, QX22, QX23;   // 디지털 출력변수(SM2)

bool MX00, MX01, MX02, MX03;   // 내부변수
bool MX04, MX05, MX06, MX07;   // 내부변수
bool MX08, MX09, MX10, MX11;   // 내부변수
bool MX12, MX13, MX14, MX15;   // 내부변수
bool MX16, MX17, MX18, MX19;   // 내부변수
bool MX20, MX21, MX22, MX23;   // 내부변수

int timer001,timer002;      // 타이머 진행시간 변수
boolean t001,t002;           // 타이머 동작 변수
int d10,d11;

//모드버스 변수
u16 _D[30];                 // modbus registor array


//모드버스 마스터, 슬레이브의 오브젝트 생성
Modbus slave0(id,0,0);   //  RX0, TX0
Modbus slave1(id);         // SoftwareSerial 

//소프트웨어 시리얼 열기
SoftwareSerial mySerial(2,3);

// 타이머 오브젝트 생성
SimpleTimer timer;

//Pin 정의
int SM00 = A0, SM01 = A1, SM02 = 7, SM03 = 8;  // SM0 핀 정의
int SM10 = A2, SM11 = A3,SM12 = A6, SM13 = A7; // SM1 핀 정의
int SM20 = 10, SM21 = 9, SM22 = 6, SM23 = 5;   // SM2 핀 정의

void fn();          //타이머 및 기타변수

//==============================================================
//                     SETUP
//==============================================================
void setup() {
  
//baudrate불러오기
  EEPROM.get(2,baud0);
  EEPROM.get(6,baud1);


// RTD LOWER, UPPER 자동저장
  EEPROM.get(20,_D[10]);
  EEPROM.get(22,_D[11]);

//SP00 통신속도
  if (baud0 == 0) baudrate0 = 4800;
  if (baud0 == 1) baudrate0 = 9600;
  if (baud0 == 2) baudrate0 = 19200;    
  if (baud0 == 3) baudrate0 = 38400;   
  _D[12] = baud0;

//SP01 통신속도
  if (baud1 == 0) baudrate1 = 4800;
  if (baud1 == 1) baudrate1 = 9600;
  if (baud1 == 2) baudrate1 = 19200;    
  if (baud1 == 3) baudrate1 = 38400;   
  _D[13] = baud1;

//모드버스 슬레이브의 ID변경
  slave0.setID(id);
  slave1.setID(id);
  
// SM0(DO4) 디지털 출력핀 정의
  pinMode(SM00, OUTPUT);
  pinMode(SM01, OUTPUT);
  pinMode(SM02, OUTPUT);
  pinMode(SM03, OUTPUT);

// SM2(DO4) 디지털 출력핀 정의
  pinMode(SM20, OUTPUT);
  pinMode(SM21, OUTPUT);
  pinMode(SM22, OUTPUT);
  pinMode(SM23, OUTPUT);
 
// 통신속도 및 타임아웃 시간 설정
  slave0.begin( baudrate0 );            // USB serial port
  slave1.begin(&mySerial, baudrate1 );  // SoftWareSerial 

// 타이머 오브젝트 시간 설정
  timer.setInterval(100,fn);
}

//==============================================================
//                     LOOP
//==============================================================
void loop() {

//baudrate 변경하기
if (_D[20] == 1234) {
  EEPROM.put(2, _D[12]);
  EEPROM.put(6, _D[13]);
  _D[20] = 0;
  }

//baudrate 초기화
if (id == 0) {
  EEPROM.put(2, 2);
  EEPROM.put(6, 2);
  }
  
// SM1(AI4) 아날로그 입력 읽기
  RTD10 = map(analogRead(SM10),_D[10],_D[11],0,500);  //
  IW11 = map(analogRead(SM11),0,1000,0,1000);

// RTD LOWER, UPPER 자동저장
  if (d10 != _D[10] || d11 != _D[11]) {  
  EEPROM.put(20, _D[10]);
  EEPROM.put(22, _D[11]);
  }
   d10 = _D[10];
   d11 = _D[11]; 


// Serial(SP00 / SP01) 통신 입력 읽기
    MX00 = _D[15] & 0x0001;  // QX00 = ON/OFF
    MX01 = _D[15] & 0x0002;  // QX01 = ON/OFF 
    MX02 = _D[15] & 0x0004;  // QX02 = ON/OFF
    MX03 = _D[15] & 0x0008;  // QX03 = ON/OFF
    MX04 = _D[15] & 0x0010;  // QX20 = ON/OFF
    MX05 = _D[15] & 0x0020;  // QX21 = ON/OFF 
    MX06 = _D[15] & 0x0040;  // QX22 = ON/OFF
    MX07 = _D[15] & 0x0080;  // QX23 = ON/OFF

//----------------------------------------------------------------------------------------------------------------------
//    L O G I C      P R O G R A M     S T A R T 
//----------------------------------------------------------------------------------------------------------------------
  //SM0
  QX00 = MX00;
  QX01 = MX01;
  QX02 = MX02;
  QX03 = MX03;
  
  //SM2
  QX20 = MX04;
  QX21 = MX05;
  QX22 = MX06;
  QX23 = MX07;
    
//----------------------------------------------------------------------------------------------------------------------
//    L O G I C      P R O G R A M     E N D 
//----------------------------------------------------------------------------------------------------------------------

// SM0(DO4) 디지털 출력
    digitalWrite(SM00, QX00);    
    digitalWrite(SM01, QX01);
    digitalWrite(SM02, QX02);
    digitalWrite(SM03, QX03);
    
// SM2(DO4) 디지털 출력
    digitalWrite(SM20, QX20);    
    digitalWrite(SM21, QX21);
    digitalWrite(SM22, QX22);
    digitalWrite(SM23, QX23);

// 모드버스 slave 어드레스 mapping  
    _D[0] = QX00 + (QX01 * 2) + (QX02 * 4) + (QX03 * 8);
    _D[1] = RTD10;
    _D[2] = IW11;
    _D[3] = QX20 + (QX21 * 2) + (QX22 * 4) + (QX23 * 8);
    
    slave0.poll( _D, 30 );
    slave1.poll( _D, 30 );
    timer.run();   
}

//==============================================================
//                     FN (Timer)
//==============================================================
void fn() {
  if (t001 == 1) {timer001 = timer001 + 1;}
  if (t001 == 0) {timer001 = 0;}
  if (t002 == 1) {timer002 = timer002 + 1;}
  if (t002 == 0) {timer002 = 0;}
}
//================================================================================
//  E  N  D            DO4 - RTD/AI - DO4 
//================================================================================
