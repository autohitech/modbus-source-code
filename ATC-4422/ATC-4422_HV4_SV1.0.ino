/*
 * ATC-4422_HV4_SV1.0      
 * 
 * Model Name : ATC-4422
 * HardWare Version : 4.0
 * SoftWare Version : 1.0
 * Communication 1: Modbus Slave
 * Communication 2: Modbus Slave
 * Creation Date : 2017-04-15
 */
/****************************************************************************
 * This program is for ATC-4422-ES controller.
 * Below is assigned I/O pin number.
 * 
 * Digital Input (4 points) : Pin 7, Pin 6, Pin 5, Pin 4
 * Digital Output (4 points) : A0, A1, A2, A3
 * Analog Input (2 channels) : A6, A7
 * Analog Output (2 channels) : Pin 9, Pin 10 
 * ID select pin (4 pin) : A2(13), A3(12), A4(11), A5(8) 
 * Communication 1 : Ethernet
 * Communication 2 : RxD, TxD
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

#include <ModbusRtu.h>
#include <EEPROM.h>
#include <SimpleTimer.h>
#include <SoftwareSerial.h>

// ID 번호 선택

bool id0 = !digitalRead(13);    ////// 수정
bool id1 = !digitalRead(12);
bool id2 = !digitalRead(11);
bool id3 = !digitalRead(8);

int id = id1 + (id2 * 2) + (id3 * 4);

//모드버스 슬레이브의 오브젝트 생성
Modbus slave_E(id,0,0);  // 0 maeans master(1~247 slave), 0 means serial port, 0 means  RS-232 or USB-FTDI
Modbus slave(id);    // 0 maeans master(1~247 slave)

//소프트웨어 시리얼 열기
SoftwareSerial mySerial(2,3);

// 타이머 오브젝트 생성
SimpleTimer timer;

//변수선언
bool IX00,IX01,IX02,IX03;   // 디지털 입력변수
bool IX04,IX05,IX06,IX07;   // 디지털 입력변수
bool QX00,QX01,QX02,QX03;   // 디지털 출력변수
int IW00,IW01,QW00,QW01;    // 아날로그 입출력변수
bool MX00,MX01,MX02,MX03;   // 디지털 내부변수
int MW00,MW01;              // 아날로그 내부변수
int i;

int baud_rate1 = 19200;          // LAN모듈 통신속도
int baud_rate2 = 19200;          // 시리얼(WiFi) 통신속도
int timer000,timer001,timer002;  // 타이머 진행시간 변수
int timer003,timer004,timer005;  // 타이머 진행시간 변수
int timer006,timer007,timer008;  // 타이머 진행시간 변수
boolean t000,t001,t002,t003;     // 타이머 동작 변수
boolean t004,t005,t006,t007;     // 타이머 동작 변수


const int numReadings00 = 10;    //아날로그 입력00 평균값처리 횟수 변수
const int numReadings01 = 10;    //아날로그 입력01 평균값처리 횟수 변수
int readings00[numReadings00];   // the readings from the Analg 0 input 
int readings01[numReadings01];   // the readings from the Analog 1 input
int readIndex00 = 0;             // the index of the current Analog 0 reading
int readIndex01 = 0;             // the index of the current Analog 1 reading
int total00 = 0;                 // the running total of Analog 0
int total01 = 0;                 // the running total of Analog 1
int average00 = 0;               // the average of Analog 0
int average01 = 0;               // the average of Analog 1

u16 _D[50];       // 데이터 어레이
void fn();        //타이머 및 기타변수



void setup() {

     pinMode(13, INPUT);
     delay(10);
     id0 = !digitalRead(13);
                  
//통신속도 디폴드 설정//
 if (id0 == 1) {
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
    pinMode(7, INPUT);  
    pinMode(6, INPUT);  
    pinMode(5, INPUT);
    pinMode(4, INPUT);

    pinMode(A0, OUTPUT);  
    pinMode(A1, OUTPUT);  
    pinMode(A2, OUTPUT);
    pinMode(A3, OUTPUT);

// 통신속도 및 타임아웃 시간 설정
  slave_E.begin( baud_rate1 );  
  slave.begin( &mySerial, baud_rate2 ); 

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
    IX00 = digitalRead(7);    
    IX01 = digitalRead(6);
    IX02 = digitalRead(5);
    IX03 = digitalRead(4);

 // 아날로그 입력 읽기
   total00 = total00 - readings00[readIndex00]; // subtract the last reading:
   total01 = total01 - readings01[readIndex01]; // subtract the last reading:
   readings00[readIndex00] = analogRead(A6); // read from the sensor:
   readings01[readIndex01] = analogRead(A7); // read from the sensor:
   total00 = total00 + readings00[readIndex00]; // add the reading to the total:
   total01 = total01 + readings01[readIndex01]; // add the reading to the total:
   readIndex00 = readIndex00 + 1; // advance to the next position in the array:
   readIndex01 = readIndex01 + 1; // advance to the next position in the array:
   if (readIndex00 >= numReadings00) { readIndex00 = 0;}
   if (readIndex01 >= numReadings01) { readIndex01 = 0;}
   IW00 = total00 / numReadings00;                   //아날로그입력 00
   IW01 = total01 / numReadings01;                   //아날로그입력 01 


// 마스터로부터 비트쓰기
    MX00 = _D[2] & 0x0001; 
    MX01 = _D[2] & 0x0002; 
    MX02 = _D[2] & 0x0004; 
    MX03 = _D[2] & 0x0008;  
    
// 마스터로부터 아날로그 쓰기
    MW00 = _D[12];
    MW01 = _D[13];

    
///////////////////////////////////////////////////////////////////////////////
// 로직 프로그램 시작
///////////////////////////////////////////////////////////////////////////////

QX00 = MX00; 
QX01 = MX01;
QX02 = MX02;
QX03 = MX03;

QW00 = MW00;
QW01 = MW01;

//////////////////////////////////////////////////////////////////////////////////
// 로직 프로그램 끝
//////////////////////////////////////////////////////////////////////////////////

// 디지털 출력
    digitalWrite(A0,QX00);          // 디지털출력 00
    digitalWrite(A1,QX01);          // 디지털출력 01 
    digitalWrite(A2,QX02);          // 디지털출력 02 
    digitalWrite(A3,QX03);          // 디지털출력 03

 // 아날로그 출력
    analogWrite(9,QW00);          // 아날로그출력 00
    analogWrite(10,QW01);          // 아날로그출력 01

 // 모드버스 어드레스 mapping  
    _D[0] = IX00 + (IX01*2) + (IX02*4) + (IX03*8);
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

    _D[20] = id;
  
  slave_E.poll( _D, 50 );
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


