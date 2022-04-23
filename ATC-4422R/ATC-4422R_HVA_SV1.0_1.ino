/*
 * ATC-4422R_HVA_SV1.0_1   
 * 
 * Model Name : ATC-4422R
 * HardWare Version : A
 * SoftWare Version : 1.0
 * Communication 1: Modbus Slave
 * Creation Date : 2016-08-21
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

#include <ModbusRTUSlave.h>
#include <EEPROM.h>
#include <SimpleTimer.h>

// TIMER OBJECT//
SimpleTimer timer;

//MODBUS REGISTER SIZE//
u16 _D[40];
u8 _M[30];

// ID SET//
bool id1 = !(digitalRead(A0));
bool id2 = !(digitalRead(A1));
bool id3 = !(digitalRead(A2));
bool id4 = !(digitalRead(A3));

// ID Number //
int id = id1 + (id2 * 2) + (id3 * 4);

// VARIABLES//
bool IX00,IX01,IX02,IX03;
bool QX00,QX01,QX02,QX03;
bool MX00,MX01,MX02,MX03,MX04,MX05,MX06,MX07;
int IW00,IW01,QW00,QW01;
int MW00,MW01,MW02,MW03,MW04,MW05,MW06,MW07;
int i;
int baud_rate = 19200;
int timer001,timer002,timer003,timer004,timer005;
boolean t001,t002,t003,t004,t005;

//VARIABLES FOR Average of Analog //
const int numReadings00 = 20;
const int numReadings01 = 20;
int readings00[numReadings00];      // the readings from the RTD 0 input 
int readings01[numReadings01];      // the readings from the RTD 1 input
int readIndex00 = 0;              // the index of the current RTD 0 reading
int readIndex01 = 0;              // the index of the current RTD 1 reading
int total00 = 0;                  // the running total of RTD 0
int total01 = 0;                  // the running total of RTD 1
int average00 = 0;                // the average of RTD 0
int average01 = 0;                // the average of RTD 1

// MODBUS ID AND PORT//
ModbusRTUSlave rtu(id, &Serial);


void setup() {

//Communication Speed//
if (id4 == 1) {
EEPROM.put(6,19200);
}

EEPROM.get(6,baud_rate);
_D[6] = baud_rate;

//Assignment for Input & Output Pin// 
  for (i = 7; i<=10; i++) {
    pinMode(i, INPUT);
  }
  for (i = 2; i<=3; i++) {
    pinMode(i, OUTPUT);
  }
  for (i = 11; i<=12; i++) {
    pinMode(i, OUTPUT);
  }
   
    rtu.addWordArea(0x0000, _D, 40);
    rtu.addBitArea(0x0000, _M, 30);
    rtu.begin(baud_rate);

// Timer //
   timer.setInterval(100,fn);

// initialize all the readings of RTD input to 0 //
  for (int thisReading = 0; thisReading < numReadings00; thisReading++) {
     readings00[thisReading] = 0;
     readings01[thisReading] = 0;
   }
      
}


 
void loop() {

// Change Communication speed//
if (_D[8] == 1234) {    // Paswoed = 1234
EEPROM.put(6,_D[6]);
_D[8] = 0;
}

// Digital Input//
    IX00 = digitalRead(10);
    IX01 = digitalRead(9);
    IX02 = digitalRead(8);
    IX03 = digitalRead(7);

 // Analog Input//
   total00 = total00 - readings00[readIndex00]; // subtract the last reading:
   total01 = total01 - readings01[readIndex01]; // subtract the last reading:
   readings00[readIndex00] = analogRead(A6); // read from the sensor:
   readings01[readIndex01] = analogRead(A7); // read from the sensor:
   total00 = total00 + readings00[readIndex00]; // add the reading to the total:
   total01 = total01 + readings01[readIndex01]; // add the reading to the total:
   readIndex00 = readIndex00 + 1; // advance to the next position in the array:
   readIndex01 = readIndex01 + 1; // advance to the next position in the array:
   // if we're at the end of the array...
   if (readIndex00 >= numReadings00) {
     // ...wrap around to the beginning:
     readIndex00 = 0;
   }
   if (readIndex01 >= numReadings01) {
     // ...wrap around to the beginning:
     readIndex01 = 0;
   }
   // calculate the average:
   IW00 = total00 / numReadings00;
   IW01 = total01 / numReadings01;




// Digital Intput from HMI//
    MX00 = _M[0] & 0x0001;
    MX01 = _M[0] & 0x0002;
    MX02 = _M[0] & 0x0004;
    MX03 = _M[0] & 0x0008;

// Analog Intput from HMI//
    MW00 = _D[10];
    MW01 = _D[11];

///////////////////////////////////////////////////////////////////////////////
// Application Program Logic
///////////////////////////////////////////////////////////////////////////////

QX00 = MX00;
QX01 = MX01;
QX02 = MX02;
QX03 = MX03;

QW00 = _D[4];
QW01 = _D[5];

//////////////////////////////////////////////////////////////////////////////////

  // Digital Output//
    digitalWrite(12,QX00); 
    digitalWrite(11,QX01); 
    digitalWrite(3,QX02); 
    digitalWrite(2,QX03);


 // Analog Output//
    analogWrite(6,QW00);
    analogWrite(5,QW01);


// MODBUS ADDRESS//    
    _D[0] = IX00 + (IX01*2) + (IX02*4) + (IX03*8);
    _D[1] = QX00 + (QX01*2) + (QX02*4) + (QX03*8);
    
    _D[2] = IW00;
    _D[3] = IW01;
    
    _D[4] = QW00;
    _D[5] = QW01;
    
    _D[6] = baud_rate;
    _D[7] = id;
    
    _D[10] = timer001;
    _D[11] = timer002;
    _D[12] = timer003;    
    _D[13] = timer004;    
    _D[14] = timer005;    
    
    _D[15] = MW05;    
    _D[16] = MW06;    
    _D[17] = MW07;    
  
    _M[0] = MX00 + (MX01*2) + (MX02*4) + (MX03*8) + (MX04*16) + (MX05*32) + (MX06*64) + (MX07*128);
    
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

