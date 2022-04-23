/*
 * ATC-C642_HV1_SV1.0      
 * 
 * Model Name : ATC-C642
 * HardWare Version : 1.0
 * SoftWare Version : 1.0
 * Communication 1: Modbus Slave
 * Creation Date : 2016-08-08
 */

/****************************************************************************
 * This program is for ATC-C642 controller.
 * Below is assigned I/O pin number.
 * 
 * Digital Input (12 points) : D22 ~ D33
 * Digital Output (6 points) : D2 ~ D7
 * Analog Input (4 channels) : A0 ~ A3
 * Analog Output (2 channels) : D10 ~ D11 
 * ID select pin (4 pin) : D38(b0), D37(b1), D36(b2), D35(b3) 
 * Communication 1 : RxD, TxD
 * Communication 2 : RxD2, TxD2
 * **************************************************************************
 * 
 * Modbus Address Map
 * 30000 : Digital Input (IX00 ~ IX11)
 * 30001 : Digital Input
 * 
 * 30002 : Digital Output (QX00 ~ QX05)
 * 30003 : Digital Output
 * 
 * 30004 : Analog Input (IW00)
 * 30005 : Analog Input (IW01)
 * 30006 : Analog Input (IW02) 
 * 30007 : Analog Input (IW03)
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
u16 _D[100];
u8 _M[50];

// ID SET//
bool id1 = digitalRead(38);
bool id2 = digitalRead(37);
bool id3 = digitalRead(36);
bool id4 = digitalRead(35);

// ID Number //
int id = 15 - (id1 + (id2 * 2) + (id3 * 4) + (id4 * 8));

// VARIABLES//
bool IX00,IX01,IX02,IX03,IX04,IX05,IX06,IX07,IX08,IX09,IX10,IX11;
bool QX00,QX01,QX02,QX03,QX04,QX05,QX06,QX07;
bool MX00,MX01,MX02,MX03,MX04,MX05,MX06,MX07;
int IW00,IW01,IW02,IW03,QW00,QW01,QW02,QW03;
int i;
int baud_rate = 19200;
int timer001,timer002,timer003,timer004,timer005;
boolean t001,t002,t003,t004,t005;

// MODBUS ID AND PORT//
ModbusRTUSlave rtu(id, &Serial);
ModbusRTUSlave rtu2(id, &Serial);



void setup() {
  
  for (i = 22; i<=33; i++) {
    pinMode(i, INPUT);
  }
  for (i = 2; i<=7; i++) {
    pinMode(i, OUTPUT);
  }
   
    rtu.addWordArea(0x0000, _D, 100);
    rtu.addBitArea(0x0000, _M, 50);
    rtu.begin(baud_rate);

    rtu2.addWordArea(0x0000, _D, 100);
    rtu2.addBitArea(0x0000, _M, 50);
    rtu2.begin(baud_rate);

// Timer //
//   timer.setInterval(100,fn);
}



void loop() {

// Digital Input//
    IX00 = digitalRead(22);
    IX01 = digitalRead(23);
    IX02 = digitalRead(24);
    IX03 = digitalRead(25);
    IX04 = digitalRead(26);
    IX05 = digitalRead(27);
    IX06 = digitalRead(28);
    IX07 = digitalRead(29);
    IX08 = digitalRead(30);
    IX09 = digitalRead(31);
    IX10 = digitalRead(32);
    IX11 = digitalRead(33);

 // Analog Input//
    IW00 = analogRead(A3);
    IW01 = analogRead(A2);
    IW02 = analogRead(A1);
    IW03 = analogRead(A0);

// Digital Intput from HMI//
    MX00 = _M[0] & 0x0001;
    MX01 = _M[0] & 0x0002;
    MX02 = _M[0] & 0x0004;
    MX03 = _M[0] & 0x0008;
    MX04 = _M[0] & 0x0010;
    MX05 = _M[0] & 0x0020;

///////////////////////////////////////////////////////////////////////////////
// Application Program Logic
///////////////////////////////////////////////////////////////////////////////

QX00 = MX00; 
QX01 = MX01;
QX02 = MX02;
QX03 = MX03;
QX04 = MX04; 
QX05 = MX05;

QW00 = _D[12];
QW01 = _D[13];

//////////////////////////////////////////////////////////////////////////////////

// Digital Output//
    digitalWrite(2,QX00); 
    digitalWrite(3,QX01); 
    digitalWrite(4,QX02); 
    digitalWrite(5,QX03);
    digitalWrite(6,QX04); 
    digitalWrite(7,QX05);

 // Analog Output//
    analogWrite(10,QW00);
    analogWrite(11,QW01);


 // MODBUS ADDRESS//    
    _D[0] = IX00 + (IX01*2) + (IX02*4) + (IX03*8) + (IX04*16) + (IX05*32) + (IX06*64) + (IX07*128) + (IX08*256) + (IX09*512) + (IX10*1024) + (IX11*2048);
    _D[1] = 0;
    _D[2] = QX00 + (QX01*2) + (QX02*4) + (QX03*8) + (QX04*16) + (QX05*32);
    _D[3] = 0;   
    _D[4] = IW00;
    _D[5] = IW01;
    _D[6] = IW02;
    _D[7] = IW03;
    _D[8] = 0;
    _D[9] = 0;
    _D[10] = 0;
    _D[11] = 0; 
    _D[12] = QW00;
    _D[13] = QW01;

 //   rtu.process();
    rtu2.process();
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



