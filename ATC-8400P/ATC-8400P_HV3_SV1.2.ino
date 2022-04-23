/*
 * ATC-8400P_HV3_SV1.2     
 * 
 * Model Name : ATC-8400P
 * HardWare Version : 3.0
 * SoftWare Version : 1.2
 * Communication 1: Modbus Master
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
 * Communication 1 : Master    RxD(0), TxD(1) Serial module 
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
Modbus master(0,0,0);  // 0 maeans master(1~247 slave), 0 means serial port, 0 means  RS-232 or USB-FTDI
Modbus slave_E(id);  // 0 maeans master(1~247 slave)
modbus_t telegram[10];

//소프트웨어 시리얼 열기
SoftwareSerial mySerial(2,3);

// 타이머 오브젝트 생성
SimpleTimer timer;

//변수선언
bool IX00,IX01,IX02,IX03;   // 디지털 입력변수(마스터)
bool IX04,IX05,IX06,IX07;   // 디지털 입력변수(마스터)
bool QX00,QX01,QX02,QX03;   // 디지털 출력변수(마스터)
bool MX00,MX01,MX02,MX03;   // 내부변수
bool MX10,MX11,MX12,MX13;   // 내부변수
bool MX20,MX21,MX22,MX23;   // 내부변수
bool MW10,MW11,MW20,MW21;   // 내부변수

bool IX10,IX11,IX12,IX13;    // 슬레이브 1번의 디지털 입력변수
bool QX10,QX11,QX12,QX13;    // 슬레이브 1번의 디지털 출력변수
int IW10,IW11;               // 슬레이브 1번의 아날로그 입력변수
int QW10,QW11;               // 슬레이브 1번의 아날로그 출력변수
bool IX20,IX21,IX22,IX23;    // 슬레이브 2번의 디지털 입력변수
bool QX20,QX21,QX22,QX23;    // 슬레이브 2번의 디지털 출력변수
int IW20,IW21;               // 슬레이브 2번의 아날로그 입력변수
int QW20,QW21;               // 슬레이브 2번의 아날로그 출력변수

int i;

int baud_rate1 = 19200;         // 시리얼모듈 통신속도(마스터)
int baud_rate2 = 19200;        // 이더넷모듈 통신속도
int timer000,timer001,timer002;  // 타이머 진행시간 변수
int timer003,timer004,timer005;  // 타이머 진행시간 변수
boolean t000,t001,t002,t003;     // 타이머 동작 변수
boolean t004,t005;     // 타이머 동작 변수

u16 _D[60]; // 데이터 어레이
uint16_t data10[20];    // 슬레이브 1번의 워드읽기 데이터
uint16_t data11[20];    // 슬레이브 1번의 비트쓰기 데이터
uint16_t data12[20];    // 슬레이브 1번의 워드쓰기 데이터
uint16_t data20[20];    // 슬레이브 2번의 워드읽기 데이터
uint16_t data21[20];    // 슬레이브 2번의 비트쓰기 데이터
uint16_t data22[20];    // 슬레이브 2번의 워드쓰기 데이터

void fn();              //타이머 및 기타변수
unsigned long stwait;   //시작시 통신 대기 변수
uint8_t state;
uint8_t format;


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
  master.setTimeOut( 100 ); // if there is no answer in 2000 ms, roll over
  stwait = millis() + 1000;
  state = 0; 
  format = 0;
    
 // 통신속도 및 타임아웃 시간 설정
  master.begin( baud_rate1 ); 
  master.setTimeOut( 100 ); 
  slave_E.begin( &mySerial, baud_rate2 );
  stwait = millis() + 1000;
  state = 0; 
  format = 0;
  
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
    if (master.getState() == COM_IDLE) { state = 1; stwait = millis() + 50; }
    break;
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
    IX10 = data10[0] & 0x0001;     //슬레이브1 디지털입력 00
    IX11 = data10[0] & 0x0002;     //슬레이브1 디지털입력 01
    IX12 = data10[0] & 0x0004;     //슬레이브1 디지털입력 02
    IX13 = data10[0] & 0x0008;     //슬레이브1 디지털입력 03
    IX20 = data20[0] & 0x0001;     //슬레이브2 디지털입력 00
    IX21 = data20[0] & 0x0002;     //슬레이브2 디지털입력 01
    IX22 = data20[0] & 0x0004;     //슬레이브2 디지털입력 02
    IX23 = data20[0] & 0x0008;     //슬레이브2 디지털입력 03

// HMI 로부터 비트쓰기
    MX00 = _D[2] & 0x0001; 
    MX01 = _D[2] & 0x0002; 
    MX02 = _D[2] & 0x0004; 
    MX03 = _D[2] & 0x0008;
    MX10 = _D[22] & 0x0001; 
    MX11 = _D[22] & 0x0002; 
    MX12 = _D[22] & 0x0004; 
    MX13 = _D[22] & 0x0008;      
    MX20 = _D[42] & 0x0001; 
    MX21 = _D[42] & 0x0002; 
    MX22 = _D[42] & 0x0004; 
    MX23 = _D[42] & 0x0008;      

// HMI 로부터 아날로그 쓰기
    MW10 = _D[32];
    MW11 = _D[33];
    MW20 = _D[52];
    MW21 = _D[53];       
    
///////////////////////////////////////////////////////////////////////////////
// 로직 프로그램 시작
///////////////////////////////////////////////////////////////////////////////

    QX00 = MX00; 
    QX01 = MX01;
    QX02 = MX02;
    QX03 = MX03;

//슬레이브 1번 디지털,아날로그 출력
    QX10 = MX10;
    QX11 = MX11;
    QX12 = MX12;
    QX13 = MX13;
    QW10 = MW10;
    QW11 = MW11;    
    
//슬레이브 2번 디지털,아날로그 출력
    QX20 = MX20;
    QX21 = MX21;
    QX22 = MX22;
    QX23 = MX23;
    QW20 = MW20;
    QW21 = MW21;

//////////////////////////////////////////////////////////////////////////////////
// 로직 프로그램 끝
//////////////////////////////////////////////////////////////////////////////////

// 디지털 출력
    digitalWrite(9,!QX00);          // 디지털출력 00
    digitalWrite(8,!QX01);          // 디지털출력 01 
    digitalWrite(7,!QX02);          // 디지털출력 02 
    digitalWrite(6,!QX03);          // 디지털출력 03
    data11[0] = QX10 + (QX11 * 2) + (QX12 * 4) + (QX13 * 8); // 슬레이브 1번 디지털 출력 00 01 02 03
    data21[0] = QX20 + (QX21 * 2) + (QX22 * 4) + (QX23 * 8); // 슬레이브 1번 디지털 출력 00 01 02 03

 // 아날로그 출력
    data12[0] = QW10;                                        // 슬레이브 1번 아날로그출력 00
    data12[1] = QW11;                                        // 슬레이브 1번 아날로그출력 01
    data22[0] = QW20;                                        // 슬레이브 2번 아날로그출력 00
    data22[1] = QW21;                                        // 슬레이브 2번 아날로그출력 01

 // 모드버스 어드레스 mapping  
    _D[0] = IX00 + (IX01*2) + (IX02*4) + (IX03*8) + (IX04*16) + (IX05*32) + (IX06*64) + (IX07*128);
    _D[1] = 0;
    _D[2] = QX00 + (QX01*2) + (QX02*4) + (QX03*8);
    _D[3] = 0;
    
 // 슬레이브 1번 모드버스 어드레스 mapping  
    _D[20] = IX10 + (IX11*2) + (IX12*4) + (IX13*8);
    _D[21] = 0;
    _D[22] = QX10 + (QX11*2) + (QX12*4) + (QX13*8);
    _D[23] = 0;
    _D[24] = IW10;
    _D[25] = IW11;

 // 슬레이브 2번 모드버스 어드레스 mapping  
    _D[40] = IX20 + (IX21*2) + (IX22*4) + (IX23*8);
    _D[41] = 0;
    _D[42] = QX20 + (QX21*2) + (QX22*4) + (QX23*8);
    _D[43] = 0;
    _D[44] = IW20;
    _D[45] = IW21;
    
  slave_E.poll( _D, 60);
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


