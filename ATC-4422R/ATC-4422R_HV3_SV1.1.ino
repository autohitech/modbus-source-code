/*
 * ATC-4422R_HV3_SV1.1
 * 
 * Model Name : ATC-4422R
 * HardWare Version : 3.0
 * SoftWare Version : 1.1
 * Communication 1: Modbus Master
 * Creation Date : 2016-12-10
 */
/****************************************************************************
 * This program is for ATC-4422R controller.
 * Below is assigned I/O pin number.
 * 
 * Digital Input (4 points) : Pin 7, Pin 8, Pin 9, Pin 10
 * Digital Output (4 points) : Pin 2, Pin 3, Pin 11, Pin 12
 * RTD Input (2 channels) : A6, A7
 * Analog Output (2 channels) : Pin 6, Pin 5 
 * ID select pin (4 pin) : A0(b0), A1(b1), A2(b2), A3(b3) 
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
 * 30004 : Analog Input (IW00) - RTD
 * 30005 : Analog Input (IW01) - RTD
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
bool IX00,IX01,IX02,IX03;   // 마스터의 디지털 입력변수
bool QX00,QX01,QX02,QX03;   // 마스터의 디지털 출력변수
int IW00,IW01;              // 마스터의 아날로그 입력변수
int QW00,QW01;              // 마스터의 아날로그 출력변수
bool IX10,IX11,IX12,IX13;   // 슬레이브 1번의 디지털 입력변수
bool QX10,QX11,QX12,QX13;   // 슬레이브 1번의 디지털 출력변수
int IW10,IW11;              // 슬레이브 1번의 아날로그 입력변수
int QW10,QW11;              // 슬레이브 1번의 아날로그 출력변수
bool IX20,IX21,IX22,IX23;   // 슬레이브 2번의 디지털 입력변수
bool QX20,QX21,QX22,QX23;   // 슬레이브 2번의 디지털 출력변수
int IW20,IW21;              // 슬레이브 2번의 아날로그 입력변수
int QW20,QW21;              // 슬레이브 2번의 아날로그 출력변수
bool MX00,MX01,MX02,MX03,MX04,MX05,MX06,MX07;    // 메모리변수
int MW00,MW01,MW02,MW03,MW04,MW05,MW06,MW07;     // 메모리변수
int MW10,MW11,MW12,MW13,MW14,MW15,MW16,MW17;     // 메모리변수
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
unsigned long stwait;
uint8_t state;
uint8_t format;

// 타이머 오브젝트 생성
SimpleTimer timer;



void setup() {

// 통신속도 설정
bool id1 = !digitalRead(A0);
bool id2 = !digitalRead(A1);
bool id3 = !digitalRead(A2);
bool id4 = !digitalRead(A3);
  if (id1 == 0 && id2 == 0 && id3 == 0 && id4 == 0) {baud_rate = 9600;}
  if (id1 == 1 && id2 == 0 && id3 == 0 && id4 == 0) {baud_rate = 19200;}
  if (id1 == 0 && id2 == 1 && id3 == 0 && id4 == 0) {baud_rate = 38400;}
  if (id1 == 0 && id2 == 0 && id3 == 1 && id4 == 0) {baud_rate = 57600;}
  if (id1 == 0 && id2 == 0 && id3 == 0 && id4 == 1) {baud_rate = 115200;}


// 입출력 핀 정의
    pinMode(7, INPUT);
    pinMode(8, INPUT);
    pinMode(9, INPUT);
    pinMode(10, INPUT);
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(11, OUTPUT);
    pinMode(12, OUTPUT);
    
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
    IX00 = digitalRead(10);      //마스터 디지털입력 00
    IX01 = digitalRead(9);      //마스터 디지털입력 01
    IX02 = digitalRead(8);      //마스터 디지털입력 02
    IX03 = digitalRead(7);      //마스터 디지털입력 03
    IX10 = data10[0] & 0x0001;  //슬레이브1 디지털입력 00
    IX11 = data10[0] & 0x0002;  //슬레이브1 디지털입력 01
    IX12 = data10[0] & 0x0004;  //슬레이브1 디지털입력 02
    IX13 = data10[0] & 0x0008;  //슬레이브1 디지털입력 03
    IX20 = data20[0] & 0x0001;  //슬레이브2 디지털입력 00
    IX21 = data20[0] & 0x0002;  //슬레이브2 디지털입력 01
    IX22 = data20[0] & 0x0004;  //슬레이브2 디지털입력 02
    IX23 = data20[0] & 0x0008;  //슬레이브2 디지털입력 03

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
   IW00 = map((total00 / numReadings00),0,1000,-200,700);             //온도로 변환 RTD 00
   IW01 = map((total01 / numReadings01),0,1000,-200,700);             //온도로 변환 RTD 01
   IW10 = data10[4];                                //슬레이브1 아날로그입력 00
   IW11 = data10[5];                                //슬레이브1 아날로그입력 01
   IW20 = data20[4];                                //슬레이브2 아날로그입력 00
   IW21 = data20[5];                                //슬레이브2 아날로그입력 01

   
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
    digitalWrite(12,QX00);                                   // 디지털출력 00 
    digitalWrite(11,QX01);                                   // 디지털출력 01 
    digitalWrite(3,QX02);                                    // 디지털출력 02
    digitalWrite(2,QX03);                                    // 디지털출력 03
    data11[0] = QX10 + (QX11 * 2) + (QX12 * 4) + (QX13 * 8); // 슬레이브 1번 디지털 출력 00 01 02 03
    data21[0] = QX20 + (QX21 * 2) + (QX22 * 4) + (QX23 * 8); // 슬레이브 1번 디지털 출력 00 01 02 03


 // 아날로그 출력
    analogWrite(6,QW00);                                    // 마스터 아날로그출력 00
    analogWrite(5,QW01);                                    // 마스터 아날로그출력 01
    data12[0] = QW10;                                        // 슬레이브 1번 아날로그출력 00
    data12[1] = QW11;                                        // 슬레이브 1번 아날로그출력 01
    data22[0] = QW20;                                        // 슬레이브 2번 아날로그출력 00
    data22[1] = QW21;                                        // 슬레이브 2번 아날로그출력 01
 
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

