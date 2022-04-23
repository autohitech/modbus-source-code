/*
 * ATC-C420_HVA_SV1.0_2     
 * 
 * Model Name : ATC-C420
 * HardWare Version : A
 * SoftWare Version : 1.0
 * Communication 1: Modbus Slave
 * Creation Date : 2016-10-05
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

#include <ModbusRTUSlave.h>
#include <EEPROM.h>
#include <SimpleTimer.h>

// TIMER OBJECT//
SimpleTimer timer;

//MODBUS REGISTER SIZE//
u16 _D[50];
u8 _M[20];

// ID SET//
int id1 = !digitalRead(A5);
int id2 = !digitalRead(A4);   

// ID Number //
int id = id1 + (id2 * 2);

// VARIABLES//
bool IX00,IX01,IX02,IX03,IX04,IX05,IX06,IX07,IX08,IX09,IX10,IX11; /* Digital Input 변수 */
bool QX00,QX01,QX02,QX03,QX04; /* Digital Output 변수 */
bool MX00,MX01,MX02,MX03,MX04; /* Digital Output를 PC에서 동작하기위한 Memory Bit 변수 */
int IW00,IW01; /* Analog Input 변수 */
int i;
int baud_rate; /* 시리얼 통신속도 */
int timer001,timer002,timer003,timer004,timer005; /* 타이머 진행시간 변수 */
boolean t001,t002,t003,t004,t005; /* 타이머 동작 변수 */

//VARIABLES FOR Average of Analog //
const int numReadings00 = 10;
const int numReadings01 = 10;
int readings00[numReadings00];      // the readings from the Analg 0 input 
int readings01[numReadings01];      // the readings from the Analog 1 input
int readIndex00 = 0;              // the index of the current Analog 0 reading
int readIndex01 = 0;              // the index of the current Analog 1 reading
int total00 = 0;                  // the running total of Analog 0
int total01 = 0;                  // the running total of Analog 1
int average00 = 0;                // the average of Analog 0
int average01 = 0;                // the average of Analog 1

// MODBUS ID AND PORT//
ModbusRTUSlave rtu(id, &Serial);


 
void setup() {

//Communication Speed//
if (id2 == 1) {
EEPROM.put(21,19200);
_D[21] = 19200;
}

EEPROM.get(21,baud_rate);
_D[21] = baud_rate;


//Assignment for Input & Output Pin//
  for (i = 2; i<=13; i++) {
    pinMode(i, INPUT);
  }
    pinMode(A0, OUTPUT);
    pinMode(A1, OUTPUT);
    pinMode(A2, OUTPUT);
    pinMode(A3, OUTPUT);
       
    rtu.addWordArea(0x0000, _D, 50);
    rtu.addBitArea(0x0000, _M, 20);
    rtu.begin(baud_rate);

// Timer //
   timer.setInterval(100,fn);


// initialize all the readings of Analog Input to 0 //
  for (int thisReading = 0; thisReading < numReadings00; thisReading++) {
     readings00[thisReading] = 0;
     readings01[thisReading] = 0;
   }
 
void loop() {

// Change Communication speed//
if (_D[22] == 1234) {    // Paswoed = 1234
EEPROM.put(21,_D[21]);
_D[22] = 0;
}

// Digital Input//
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

// Digital Output 변수를 동작하기위한 Memory 변수//
    MX00 = _M[0] & 0x0001;
    MX01 = _M[0] & 0x0002;
    MX02 = _M[0] & 0x0004;
    MX03 = _M[0] & 0x0008; 

///////////////////////////////////////////////////////////////////////////////
// Start Application Program Logic
///////////////////////////////////////////////////////////////////////////////

QX00 = MX00; 
QX01 = MX01;
QX02 = MX02;
QX03 = MX03;

//////////////////////////////////////////////////////////////////////////////////
//    END Application Program
//////////////////////////////////////////////////////////////////////////////////

  // Digital Output//
    digitalWrite(A0,QX00); 
    digitalWrite(A1,QX01); 
    digitalWrite(A2,QX02); 
    digitalWrite(A3,QX03);


 // MODBUS ADDRESS//    
    _D[0] = IX00 + (IX01*2) + (IX02*4) + (IX03*8) + (IX04*16) + (IX05*32) + (IX06*64) + (IX07*128) + (IX08*256) + (IX09*512) + (IX10*1024) + (IX11*2048);
    _D[2] = QX00 + (QX01*2) + (QX02*4) + (QX03*8);
   
    _D[4] = IW00;
    _D[5] = IW01;
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




