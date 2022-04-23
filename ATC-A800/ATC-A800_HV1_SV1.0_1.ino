/*
 * ATC-A800_HV1_SV1.0_1     
 * 
 * Model Name : ATC-A800
 * HardWare Version : 1.0
 * SoftWare Version : 1.0
 * Communication 1: Modbus Slave
 * Creation Date : 2016-10-13
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

#include <ModbusRTUSlave.h>
#include <EEPROM.h>
#include <SimpleTimer.h>

// TIMER OBJECT//
SimpleTimer timer;

//MODBUS REGISTER SIZE//
u16 _D[50];
u8 _M[20];
 
// ID Number //
int id = 1;

// VARIABLES//
bool IX00,IX01,IX02,IX03,IX04,IX05,IX06,IX07,IX08,IX09; /* Digital Input 변수 */
bool QX00,QX01,QX02,QX03,QX04,QX05,QX06,QX07; /* Digital Output 변수 */
bool MX00,MX01,MX02,MX03,MX04,MX05,MX06,MX07; /* Digital Output를 PC에서 동작하기위한 Memory Bit 변수 */
int i;
int baud_rate = 19200; /* 시리얼 통신속도 */
int timer001,timer002,timer003,timer004,timer005; /* 타이머 진행시간 변수 */
boolean t001,t002,t003,t004,t005; /* 타이머 동작 변수 */


// MODBUS ID AND PORT//
ModbusRTUSlave rtu(id, &Serial);

void setup() {

//Assignment for Input & Output Pin//
  for (i = 2; i<=11; i++) {
    pinMode(i, INPUT);
  }
    pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);
    pinMode(A0, OUTPUT);
    pinMode(A1, OUTPUT);
    pinMode(A2, OUTPUT);
    pinMode(A3, OUTPUT);
    pinMode(A4, OUTPUT);
    pinMode(A5, OUTPUT);
       
    rtu.addWordArea(0x0000, _D, 50);
    rtu.addBitArea(0x0000, _M, 20);
    rtu.begin(baud_rate);

// Timer //
   timer.setInterval(100,fn);

}


 
void loop() {

// Digital Input//
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

// Digital Output 변수를 동작하기위한 Memory 변수//
    MX00 = _M[0] & 0x0001;
    MX01 = _M[0] & 0x0002;
    MX02 = _M[0] & 0x0004;
    MX03 = _M[0] & 0x0008; 
    MX04 = _M[0] & 0x0010;
    MX05 = _M[0] & 0x0020;
    MX06 = _M[0] & 0x0040;
    MX07 = _M[0] & 0x0080;  

///////////////////////////////////////////////////////////////////////////////
// Start Application Program Logic
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
//    END Application Program
//////////////////////////////////////////////////////////////////////////////////

  // Digital Output//
    digitalWrite(12,QX00); 
    digitalWrite(13,QX01); 
    digitalWrite(A0,QX02); 
    digitalWrite(A1,QX03);
    digitalWrite(A2,QX04); 
    digitalWrite(A3,QX05);
    digitalWrite(A4,QX06); 
    digitalWrite(A5,QX07);


 // MODBUS ADDRESS//    
    _D[0] = IX00 + (IX01*2) + (IX02*4) + (IX03*8) + (IX04*16) + (IX05*32) + (IX06*64) + (IX07*128) + (IX08*256) + (IX09*512);
    _D[2] = QX00 + (QX01*2) + (QX02*4) + (QX03*8) + (QX04*16) + (QX05*32) + (QX06*64) + (QX07*128);
   
    _D[4] = 0;
    _D[5] = 0;
    _D[6] = 0;
    _D[7] = 0;
    _D[8] = 0;
    _D[9] = 0;
    _D[10] = 0;
    _D[11] = 0; 
    
    rtu.process();
    timer.run();
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




