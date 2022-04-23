/*
 * ATC-A800_HV1_SV1.1    
 * 
 * Model Name : ATC-A800
 * HardWare Version : 1.0
 * SoftWare Version : 1.1
 * Communication 1: Modbus Master
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

//모드버스 마스터의  통신포트 및 통신텔레그램 오브젝트 생성
Modbus master(0,0,0); // 0 maeans master(1~247 slave), 0 means serial port, 0 means  RS-232 or USB-FTDI
modbus_t telegram[10]; //Schedule For Communication of Modbus Master

//모드버스 데이터 어레이 변수
uint16_t data10[20];    // 슬레이브 1번의 워드읽기 데이터
uint16_t data11[20];    // 슬레이브 1번의 비트쓰기 데이터
uint16_t data12[20];    // 슬레이브 1번의 워드쓰기 데이터
uint16_t data20[20];    // 슬레이브 2번의 워드읽기 데이터
uint16_t data21[20];    // 슬레이브 2번의 비트쓰기 데이터
uint16_t data22[20];    // 슬레이브 2번의 워드쓰기 데이터

//입출력 및 메모리 변수
bool IX00,IX01,IX02,IX03,IX04,IX05,IX06,IX07,IX08,IX09;             // 마스터의 디지털 입력변수
bool QX00,QX01,QX02,QX03,QX04,QX05,QX06,QX07;                       // 마스터의 디지털 출력변수
bool IX10S,IX11S,IX12S,IX13S;                                       // 슬레이브 1번의 디지털 입력변수
bool QX10,QX11,QX12,QX13;                                           // 슬레이브 1번의 디지털 출력변수
int IW10,IW11;                                                      // 슬레이브 1번의 아날로그 입력변수
int QW10,QW11;                                                      // 슬레이브 1번의 아날로그 출력변수
bool IX20S,IX21S,IX22S,IX23S;                                       // 슬레이브 2번의 디지털 입력변수
bool QX20,QX21,QX22,QX23;                                           // 슬레이브 2번의 디지털 출력변수
int IW20,IW21;                                                      // 슬레이브 2번의 아날로그 입력변수
int QW20,QW21;                                                      // 슬레이브 2번의 아날로그 출력변수
bool MX00,MX01,MX02,MX03,MX04,MX05,MX06,MX07;    // 메모리변수
int i;

//통신 속도 및 타이머 변수
int baud_rate = 19200; // 시리얼 통신속도
int timer001,timer002,timer003,timer004,timer005; // 타이머 진행시간 변수
boolean t001,t002,t003,t004,t005; // 타이머 동작 변수

//타이머 및 기타변수
void fn(); 
unsigned long stwait;
uint8_t state;
uint8_t format;

// 타이머 오브젝트 생성
SimpleTimer timer;



void setup() {

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
    
  // 텔레그렘 0: 슬레이브1번 워드읽기
    telegram[0].u8id = 1; // slave address
    telegram[0].u8fct = 3; // function code (this one is registers read)
    telegram[0].u16RegAdd = 0; // start address in slave
    telegram[0].u16CoilsNo = 20; // number of elements (coils or registers) to read
    telegram[0].au16reg = data10; // pointer to a memory array in the Arduino

  // 텔레그렘 1: 슬레이브1번 비트쓰기
    telegram[1].u8id = 1; // slave address
    telegram[1].u8fct = 6; // function code (this one is write a single register) //6
    telegram[1].u16RegAdd = 2; // start address in slave  //4
    telegram[1].u16CoilsNo = 1; // number of elements (coils or registers) to read  //1
    telegram[1].au16reg = data11; // au16data+4; // pointer to a memory array in the Arduino

  // 텔레그렘 2: 슬레이브1번 워드쓰기
    telegram[2].u8id = 1; // slave address
    telegram[2].u8fct = 16; // function code (this one is write a single register) //6
    telegram[2].u16RegAdd = 12; // start address in slave  //4
    telegram[2].u16CoilsNo = 2; // number of elements (coils or registers) to read  //1
    telegram[2].au16reg = data12; // au16data+4; // pointer to a memory array in the Arduino

  // 텔레그렘 3: 슬레이브2번 워드읽기
    telegram[3].u8id = 2; // slave address
    telegram[3].u8fct = 3; // function code (this one is registers read)
    telegram[3].u16RegAdd = 0; // start address in slave
    telegram[3].u16CoilsNo = 20; // number of elements (coils or registers) to read
    telegram[3].au16reg = data20; // pointer to a memory array in the Arduino

  // 텔레그렘 4: 슬레이브2번 비트쓰기
    telegram[4].u8id = 2; // slave address
    telegram[4].u8fct = 6; // function code (this one is write a single register) //6
    telegram[4].u16RegAdd = 2; // start address in slave  //4
    telegram[4].u16CoilsNo = 1; // number of elements (coils or registers) to read  //1
    telegram[4].au16reg = data21; // au16data+4; // pointer to a memory array in the Arduino

  // 텔레그렘 5: 슬레이브2번 워드쓰기
    telegram[5].u8id = 2; // slave address
    telegram[5].u8fct = 16; // function code (this one is write a single register) //6
    telegram[5].u16RegAdd = 12; // start address in slave  //4
    telegram[5].u16CoilsNo = 2; // number of elements (coils or registers) to read  //1
    telegram[5].au16reg = data22; // au16data+4; // pointer to a memory array in the Arduino

// 통신속도 및 타임아웃 시간 설정 //
  master.begin( 19200 ); //master.begin( 19200 ); // baud-rate at 19200
  master.setTimeOut( 50 ); // if there is no answer in 2000 ms, roll over
  stwait = millis() + 1000;
  state = 0; 
  format = 0;


// 타이머 오브젝트 시간 설정
   timer.setInterval(100,fn);
  
}


void loop() {

//통신 스케쥴
 switch( state ) {
  case 0: 
  if (millis() > stwait) state++; // wait state
     break;
  case 1: 
    master.query(telegram[format]); state++; format++;
    if (format > 6) format = 0;
     break;
  case 2:
    master.poll(); // check incoming messages
    if (master.getState() == COM_IDLE) { state = 1; stwait = millis() + 100; }
    break;
  }     

// 디지털 입력 읽기
    IX00 = digitalRead(11);     //마스터 디지털입력 00
    IX01 = digitalRead(10);     //마스터 디지털입력 01
    IX02 = digitalRead(9);      //마스터 디지털입력 02
    IX03 = digitalRead(8);      //마스터 디지털입력 03
    IX04 = digitalRead(7);      //마스터 디지털입력 04
    IX05 = digitalRead(6);      //마스터 디지털입력 05
    IX06 = digitalRead(5);      //마스터 디지털입력 06
    IX07 = digitalRead(4);      //마스터 디지털입력 07
    IX08 = digitalRead(3);      //마스터 디지털입력 08
    IX09 = digitalRead(2);      //마스터 디지털입력 09
    
///////////////////////////////////////////////////////////////////////////////
// 로직 프로그램 시작
///////////////////////////////////////////////////////////////////////////////

//슬레이브 1번 디지털 출력
    QX10 = IX00;
    QX11 = IX01;
    QX12 = IX02;
    QX13 = IX03;
    
//슬레이브 2번 디지털 출력
    QX20 = IX00;
    QX21 = IX01;
    QX22 = IX02;
    QX23 = IX03;

//슬레이브 1번 아날로그 값 출력
    QW10 = 200;
    QW11 = 100;

  t001 = !MX00;
  if (timer001 >= 20) {QX10 = 1; MX00 = 1; t002 = 1;}
  if (timer002 >= 20) {QX10 = 0; MX00 = 0; t002 = 0;}
  
  t003 = !MX01;
  if (timer003 >= 20) {QX20 = 1; MX01 = 1; t004 = 1;}
  if (timer004 >= 20) {QX20 = 0; MX01 = 0; t004 = 0;}

//////////////////////////////////////////////////////////////////////////////////
// 로직 프로그램 끝
//////////////////////////////////////////////////////////////////////////////////

// 디지털 출력
    digitalWrite(12,QX00);            // 마스터 디지털출력 00  
    digitalWrite(13,QX01);            // 마스터 디지털출력 01  
    digitalWrite(A0,QX02);            // 마스터 디지털출력 02  
    digitalWrite(A1,QX03);            // 마스터 디지털출력 03 
    digitalWrite(A2,QX04);            // 마스터 디지털출력 04  
    digitalWrite(A3,QX05);            // 마스터 디지털출력 05 
    digitalWrite(A4,QX06);            // 마스터 디지털출력 06  
    digitalWrite(A5,QX07);            // 마스터 디지털출력 07 
    data11[0] = QX10 + (QX11 * 2) + (QX12 * 4) + (QX13 * 8); // 슬레이브 1번 디지털 출력 00 01 02 03
    data21[0] = QX20 + (QX21 * 2) + (QX22 * 4) + (QX23 * 8); // 슬레이브 1번 디지털 출력 00 01 02 03
    
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

