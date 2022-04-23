
 /*************
 * ATC-STA_V1.1
 * 
 * Model Name : ATC-SDA (DI4-AI4-DO4)
 * Communication 1: Modbus Slave
 * Communication 2: Modbus Slave
 * Creation Date : 2019-03-06
 ***************************************************************
 * Pin Assignment
 * 
 * Button S/W : D04(SW1), D11(SW2), D12(SW3), D13(SW4) 
 * SM 0 : A00, A01, D07, D08
 * SM 1 : A02, A03, A06, A07
 * SM 2 : D10, D09, D06, D05
 * 
 * **************************************************************************
 * 
 * Modbus Address Map
 * 
 * 30000 : Button S/W 1 
 * 30001 : Button S/W 2  
 * 30002 : Button S/W 3 
 * 30003 : Button S/W 4  
 * 
 * 30005 : SM 0 - 0 
 * 30006 : SM 0 - 1 
 * 30007 : SM 0 - 2 
 * 30008 : SM 0 - 3  
 * 
 * 30010 : SM 1 - 0 
 * 30011 : SM 1 - 1 
 * 30012 : SM 1 - 2 
 * 30013 : SM 1 - 3   
 * 
 * 30015 : SM 2 - 0 
 * 30016 : SM 2 - 1 
 * 30017 : SM 2 - 2 
 * 30018 : SM 2 - 3 
 * 
 * 30020 : MW00
 * 30021 : MW01
 * 30022 : MW02
 * 30023 : MW03
 * 30024 : MW04
 * 30025 : MW05
 */

#include <ModbusRtu.h>
#include <EEPROM.h>
#include <SimpleTimer.h>
#include <SoftwareSerial.h> 
#include "U8glib.h"

const int ledPin1 = 10;
const int ledPin2 = 9;
const int ledPin3 = 6;
const int ledPin4 = 5;
const int tempPin = 7;

#define DEBUG true
String tempValue;
String getRequest;

int id0, id1;
//OLED 선택
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);  // I2C / TWI 

//모드버스 슬레이브의 오브젝트 생성
Modbus slave_0(id0,0,0);
Modbus slave_1(id1);

//소프트웨어 시리얼 열기
SoftwareSerial mySerial = SoftwareSerial(2,3);

// 타이머 오브젝트 생성
SimpleTimer timer;

//변수선언
bool IX00,IX01,IX02,IX03,IX04,IX05,IX06,IX07,IX08,IX09,IX10,IX11;   // 디지털 입력변수
bool QX00,QX01,QX02,QX03;   // 디지털 출력변수
int IW00,IW01,IW02,IW03;    // 아날로그 입력변수
int QW00,QW01,QW02,QW03;    // 아날로그 출력변수
bool MX00,MX01,MX02,MX03;   // 디지털 내부변수
int MW00,MW01,MW02, MW03, MW04, MW05;         // 아날로그 내부변수
int i;

int baud0, baud1, baudrate0, baudrate1;       // 통신속도
int timer000,timer001,timer002;  // 타이머 진행시간 변수
int timer003,timer004,timer005;  // 타이머 진행시간 변수
int timer006,timer007,timer008;  // 타이머 진행시간 변수
boolean t000,t001,t002,t003;     // 타이머 동작 변수
boolean t004,t005,t006,t007;     // 타이머 동작 변수
u16 _D[50];       // 데이터 어레이
void fn();        //타이머 및 기타변수
int flag = 0;
bool sw = 0;
char i0[2], i1[2], i2[2], i3[2];
char i4[2], i5[2], i6[2], i7[2];
char i8[2], i9[2], i10[2], i11[2];
char a0[2], a1[2];

//키 정의

int KeyBack =4;
int KeyPrev = 11;
int KeyNext = 12;
int KeySelect = 13;
int menu = 1;




void setup() {

//    EEPROM.put(0,1);
//    EEPROM.put(2,1);    
//    EEPROM.put(4,1);
//    EEPROM.put(6,1);
  
//id 및 baudrate 불러오기
    EEPROM.get(0,id0);
    EEPROM.get(2,id1);    
    EEPROM.get(4,baud0);
    EEPROM.get(6,baud1);
   _D[10] = id0;
   _D[11] = id1;
   if (baud0 == 0)   baudrate0 = 4800; _D[12] = baudrate0;
   if (baud0 == 1)   baudrate0 = 9600; _D[12] = baudrate0;
   if (baud0 == 2)   baudrate0 = 19200; _D[12] = baudrate0;     
   if (baud0 == 3)   baudrate0 = 38400; _D[12] = baudrate0; 
   
   if (baud1 == 0)   baudrate1 = 4800; _D[13] = baudrate1;
   if (baud1 == 1)   baudrate1 = 9600; _D[13] = baudrate1;
   if (baud1 == 2)   baudrate1 = 19200; _D[13] = baudrate1;     
   if (baud1 == 3)   baudrate1 = 38400; _D[13] = baudrate1; 

//모드버스 슬레이브의 ID변경
slave_0.setID(id0);
slave_1.setID(id1);
   

// 입출력 핀 정의
  pinMode(KeyBack, INPUT);           // set pin to input with pullup  
  pinMode(KeyPrev, INPUT);           // set pin to input with pullup
  pinMode(KeyNext, INPUT);           // set pin to input with pullup
  pinMode(KeySelect, INPUT);           // set pin to input with pullup

// SM0 디지털 입력핀 정의
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);


// SM1 아날로그 입력핀 정의


// SM2 디지털 출력핀 정의
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT); 
  
// 통신속도 및 타임아웃 시간 설정
  slave_0.begin( baudrate0 ); 
  slave_1.begin(&mySerial,  baudrate1 );  


// 타이머 오브젝트 시간 설정
   timer.setInterval(100,fn);




  // 화면 180도 회전
    u8g.setRot180(); 
    delay(100);

}



void loop() {

 // 디지털  입력 읽기
    IX00 = digitalRead(A0);    
    IX01 = digitalRead(A1);
    IX02 = digitalRead(7);
    IX03 = digitalRead(8);

   // 아날로그  입력 읽기
    IW00 = map(analogRead(A2),0,1000,0,100);  
    IW01 = map(analogRead(A3),0,1000,0,100);
    IW02 = map(analogRead(A6),0,1000,0,100);
    IW03 = map(analogRead(A7),0,1000,0,100);

    // 통신 입력 읽기
    MX00 = _D[2] & 0x0001;
    MX01 = _D[2] & 0x0002;
    MX02 = _D[2] & 0x0004;
    MX03 = _D[2] & 0x0008;  
       
    MW00 = _D[20];    
    MW01 = _D[21];
    MW02 = _D[22];
    MW03 = _D[23];

    
//-----------------------------------------------------------------------------------------

QX00 = MX00;
QX01 = MX01;
QX02 = MX02;
QX03 = MX03;


//-----------------------------------------------------------------------------------------

// 디지털 출력
    digitalWrite(10, QX00);    
    digitalWrite(9, QX01);
    digitalWrite(6, QX02);
    digitalWrite(5, QX03);
 
 // 모드버스 어드레스 mapping  
    _D[0] = IX00 + (IX01 * 2) + (IX02 * 4) + (IX03 * 8);
    _D[1] = 0;
    _D[2] = QX00 + (QX01 * 2) + (QX02 * 4) + (QX03 * 8);;
    _D[3] = 0;
    _D[4] = IW00;
    _D[5] = IW01;
    _D[6] = IW02;
    _D[7] = IW03;
    _D[8] = 0;
    _D[9] = 0;
    _D[10] = 0;
    _D[11] = 0;
    _D[12] = 0;
    _D[13] = 0;
    
  
  slave_0.poll( _D, 50 );
  slave_1.poll( _D, 50 );
    timer.run();

    
//-----------------------------------------------------------------------
if (flag == 0) {
  
    u8g.firstPage();
    do  {
      draw();
      flag = 1;
    } while( u8g.nextPage() );
}

//--------------------------------------------------------------------
else {
  
  if (flag == 1) {
    if (sw == 0 && !digitalRead(KeySelect)) {
        flag = 2; sw = 1;  
      }
    }

  if (flag == 2) {
    if (!digitalRead(KeyNext)) {
       menu++;
      }
  if (!digitalRead(KeyPrev)) {
       menu--;
      }
  if (sw == 0 && !digitalRead(KeySelect)) {
      flag = 3; sw = 1;   
      }
  if (!digitalRead(KeyBack)) {
      flag = 0; sw = 1;  
      }     
    }

  if (flag == 3) {   
    if (!digitalRead(KeyBack)) {
      flag = 2; sw = 1;  
      } 
    }
}

//--------------------------------------------------------------------

  if (sw == 0 && flag == 2 ) {   

    u8g.firstPage();
      do  {
       updateMenu(); 
     } while( u8g.nextPage() );
  }

  if (sw == 0 && flag == 3) {
      u8g.firstPage();
      do  {
        executeAction();  
      } while( u8g.nextPage() );
  }

//--------------------------------------------------------------------

//id0 설정
 if (flag == 3 && menu == 2) {
    if (!digitalRead(KeyPrev)) {
       id0--;
   }
    if (!digitalRead(KeyNext)) {
       id0++;
    }
    if (sw == 0 && !digitalRead(KeySelect)) {
     menu++;
     EEPROM.put(0,id0); 
     sw = 1;
    }
    
   u8g.firstPage();
    do  {
      executeAction(); 
    } while( u8g.nextPage() );
    }
    
//baudrate0 설정
 if (flag == 3 && menu == 3) {
   if (!digitalRead(KeyPrev)) {
     if (baudrate0 > 4800) {
       baudrate0 = baudrate0 / 2;
     }
   }

   if (!digitalRead(KeyNext)) {
     if (baudrate0 < 19200) {
       baudrate0 = baudrate0 * 2;
     }
   }
 
 if (sw == 0 && !digitalRead(KeySelect)) {
  menu++;
  sw = 1; 
  if (baudrate0 == 4800) baud0 = 0; EEPROM.put(4,baud0); 
  if (baudrate0 == 9600) baud0 = 1; EEPROM.put(4,baud0); 
  if (baudrate0 == 19200) baud0 = 2; EEPROM.put(4,baud0); 
  if (baudrate0 == 38400) baud0 = 3; EEPROM.put(4,baud0); 
 }
   u8g.firstPage();
    do  {
      executeAction(); 
    } while( u8g.nextPage() );
    }

//id1 설정
 if (flag == 3 && menu == 4) {
    if (!digitalRead(KeyPrev)) {
       id1--;
   }
    if (!digitalRead(KeyNext)) {
       id1++;
    }
    if (sw == 0 && !digitalRead(KeySelect)) {
     menu++;
     EEPROM.put(2,id1); 
     sw = 1;
    }
    
   u8g.firstPage();
    do  {
      executeAction(); 
    } while( u8g.nextPage() );
    }
    
//baudrate1 설정
 if (flag == 3 && menu == 5) {
   if (!digitalRead(KeyPrev)) {
     if (baudrate1 > 4800) {
       baudrate1 = baudrate1 / 2;
     }
   }

   if (!digitalRead(KeyNext)) {
     if (baudrate1 < 19200) {
       baudrate1 = baudrate1 * 2;
     }
   }
 

 if (sw == 0 && !digitalRead(KeySelect)) {
  menu = 2;
  sw = 1; 
  if (baudrate1 == 4800) baud1 = 0; EEPROM.put(6,baud1); 
  if (baudrate1 == 9600) baud1 = 1; EEPROM.put(6,baud1); 
  if (baudrate1 == 19200) baud1 = 2; EEPROM.put(6,baud1); 
  if (baudrate1 == 38400) baud1 = 3; EEPROM.put(6,baud1); 
 }
   u8g.firstPage();
    do  {
      executeAction(); 
    } while( u8g.nextPage() );
    }

//--------------------------------------------------------------------  
if (digitalRead(KeySelect)) {
      sw = 0;    
}
  delay(10);
}
//--------------------------------------------------------------------


void draw() {
  u8g.setFont(u8g_font_10x20);
  u8g.drawStr( 30, 22, "ATC-SDA");
  u8g.drawStr( 0, 42, "[DI4-AI4-DO4]");
  u8g.setFont(u8g_font_6x13);
  u8g.drawStr( 25, 59, "A-Technology");

}


void updateMenu() {
  
  switch (menu) {
    
    case 0:
     menu = 1;
     break;
     
    case 1:
     u8g.setFont(u8g_font_10x20);
     u8g.drawStr( 10, 20, "-> STATUS");
     u8g.drawStr( 10, 40, "   SETUP");
     break;
     
    case 2:
     u8g.setFont(u8g_font_10x20);
     u8g.drawStr( 10, 20, "   STATUS");
     u8g.drawStr( 10, 40, "-> SETUP");
     break;
   
    case 3:
     menu = 2;
     break;  
  }  
}


void executeAction() {
  switch (menu) {
    case 1:
     action1();
     break;
    case 2:
     action2();
     break;     
    case 3:
     action3();
     break;     
    case 4:
     action4();
     break;       
    case 5:
     action5();
     break;  
  }
}

void action1() {
     u8g.setFont(u8g_font_10x20);
     u8g.drawStr( 10, 20, "STATUS");
     u8g.drawHLine(10, 20, 120);
     u8g.setFont(u8g_font_6x13);     
 
     sprintf(i0,"DI %d  %d  %d  %d", IX00, IX01, IX02, IX03);
     u8g.drawStr( 4, 34, i0);
   
      sprintf(i1,"AI %03d %03d %03d %03d", IW00, IW01, IW02, IW03);
     u8g.drawStr( 4, 46, i1);

      sprintf(i2,"DO %d  %d  %d  %d", QX00, QX01, QX02, QX03);
     u8g.drawStr( 4, 58, i2);
 
}

void action2() {
    u8g.setFont(u8g_font_10x20);
     u8g.drawStr( 10, 20, "SETUP");
     u8g.drawHLine(10, 20, 120);
     u8g.setFont(u8g_font_6x13);   
      sprintf(i4,"ID_0 = %d", id0);  
      u8g.drawStr( 4, 34, i4);
}

void action3() {
    u8g.setFont(u8g_font_10x20);
     u8g.drawStr( 10, 20, "SETUP");
     u8g.drawHLine(10, 20, 120);
     u8g.setFont(u8g_font_6x13); 

     sprintf(i5,"BAUDRATE_0 = %d", baudrate0);  
       u8g.drawStr( 4, 34, i5);
}

void action4() {
   u8g.setFont(u8g_font_10x20);
     u8g.drawStr( 10, 20, "SETUP");
     u8g.drawHLine(10, 20, 120);
     u8g.setFont(u8g_font_6x13);   
      sprintf(i6,"ID_1 = %d", id1);  
      u8g.drawStr( 4, 34, i6);
}

void action5() {
     u8g.setFont(u8g_font_10x20);
     u8g.drawStr( 10, 20, "SETUP");
     u8g.drawHLine(10, 20, 120);
     u8g.setFont(u8g_font_6x13); 

     sprintf(i7,"BAUDRATE_1 = %d", baudrate1);  
       u8g.drawStr( 4, 34, i7);
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
