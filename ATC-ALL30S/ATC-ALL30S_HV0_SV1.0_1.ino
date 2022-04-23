/*
 * ATC-ALL30S_HV0_SV1.0_1 
 * 
 * Model Name : ATC-ALL30S
 * HardWare Version : ?
 * SoftWare Version : 1.0
 * Communication 1: Modbus Slave
 * Creation Date : 2016-08-18
 */

#include <ModbusRTUSlave.h>
 
u16 _D[20];
u8 _M[50];
int i;
int baud_rate = 9600;
int id = 1;
bool di0,di1,di2,di3,di4,di5,di6,di7,di8,di9,di10,di11,di12,di13,di14,di15;
bool do0,do1,do2,do3,do4,do5,do6,do7;
int ai0,ai1,ai2,ai3,ai4,ai5,ai6,ai7;

ModbusRTUSlave rtu(id, &Serial3);
 
void setup() {
  for (i = 30; i<=45; i++) {
    pinMode(i, INPUT);
  }
//  for (i = 22; i<=29; i++) 
  for (i = 23; i<=28; i++) {
    pinMode(i, OUTPUT);
  } 
   
    rtu.addWordArea(0x0000, _D, 20);
    rtu.addBitArea(0x0000, _M, 50);
    rtu.begin(baud_rate);
    _D[10] = baud_rate;
    _D[11] = id;
}
 
void loop() {

    di0 = digitalRead(30);
    di1 = digitalRead(31);
    di2 = digitalRead(32);
    di3 = digitalRead(33);
    di4 = digitalRead(34);
    di5 = digitalRead(35);
    di6 = digitalRead(36);
    di7 = digitalRead(37);
    di8 = digitalRead(38);
    di9 = digitalRead(39);
    di10 = digitalRead(40);
    di11 = digitalRead(41);
    di12 = digitalRead(42);
    di13 = digitalRead(43);
    di14 = digitalRead(44);
    di15 = digitalRead(45);
        
    ai0 = analogRead(A0);
    ai1 = analogRead(A1);
    ai2 = analogRead(A2);
    ai3 = analogRead(A3);
    ai4 = analogRead(A4);
    ai5 = analogRead(A5);
    ai6 = analogRead(A6);
    ai7 = analogRead(A7);

  do0 = _M[0] & 0x0001;
  do1 = _M[0] & 0x0002;
  do2 = _M[0] & 0x0004;
  do3 = _M[0] & 0x0008;
  do4 = _M[0] & 0x0010;
  do5 = _M[0] & 0x0020;
  do6 = _M[0] & 0x0040;
  do7 = _M[0] & 0x0080;

 //   digitalWrite(22,do0); 
    digitalWrite(23,do0); 
    digitalWrite(24,do1); 
    digitalWrite(25,do2); 
    digitalWrite(26,do3); 
    digitalWrite(27,do4); 
    digitalWrite(28,do5); 
 //   digitalWrite(29,do7); 


    _D[0] = di0 + (di1*2) + (di2*4) + (di3*8) + (di4*16) + (di5*32) + (di6*64) + (di7*128) + (di8*256) + (di9*512) + (di10*1024) + (di11*2048) + (di12*4096) + (di13*8192) + (di14*16384) + (di15*32768);
    _D[1] = do0 + (do1*2) + (do2*4) + (do3*8) + (do4*16) + (do5*32) + (do6*64) + (do7*128);
    _D[10] = ai0;
    _D[11] = ai1;
    _D[12] = ai2;
    _D[13] = ai3;
    _D[14] = ai4;
    _D[15] = ai5;
    _D[16] = ai6;
    _D[17] = ai7;
    
    rtu.process();
    delay(50);


    
}



