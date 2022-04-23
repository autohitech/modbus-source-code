/*!
LTC2983: Multi-Sensor High Accuracy Digital Temperature Measurement System.

@verbatim

support_functions_LTC2983.cpp:
This file contains all the support functions used in the main program.
@endverbatim

http://www.linear.com/product/LTC2983

http://www.linear.com/product/LTC2983#demoboards

$Revision: 6237 $
$Date: 2016-12-20 15:09:16 -0800 (Tue, 20 Dec 2016) $
Copyright (c) 2014, Linear Technology Corp.(LTC)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of Linear Technology Corp.

The Linear Technology Linduino is not affiliated with the official Arduino team.
However, the Linduino is only possible because of the Arduino team's commitment
to the open-source community.  Please, visit http://www.arduino.cc and
http://store.arduino.cc , and consider a purchase that will help fund their
ongoing work.
*/


//! @ingroup Temperature_Monitors
//! @{
//! @defgroup LTC2983 LTC2983: Multi-Sensor High Accuracy Digital Temperature Measurement System
//! @}

/*! @file
    @ingroup LTC2983
    Library for LTC2983: Multi-Sensor High Accuracy Digital Temperature Measurement System
*/



#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include <SPI.h>
#include "UserInterface.h"
#include "LT_I2C.h"
#include "QuikEval_EEPROM.h"
#include "configuration_constants_LTC2983.h"
#include "table_coeffs_LTC2983.h"
#include "support_functions_LTC2983_di.h"

//! Prints the title block when program first starts.
void print_title()
{
}


// ***********************
// Program the part
// ***********************
void assign_channel(uint8_t chip_select, uint8_t channel_number, uint32_t channel_assignment_data)
{
  uint16_t start_address = get_start_address(CH_ADDRESS_BASE, channel_number);
  transfer_four_bytes(chip_select, WRITE_TO_RAM, start_address, channel_assignment_data);
}


void write_custom_table(uint8_t chip_select, struct table_coeffs coefficients[64], uint16_t start_address, uint8_t table_length)
{
  int8_t i;
  uint32_t coeff;

  output_low(chip_select);

  SPI.transfer(WRITE_TO_RAM);
  SPI.transfer(highByte(start_address));
  SPI.transfer(lowByte(start_address));

  for (i=0; i< table_length; i++)
  {
    coeff = coefficients[i].measurement;
    SPI.transfer((uint8_t)(coeff >> 16));
    SPI.transfer((uint8_t)(coeff >> 8));
    SPI.transfer((uint8_t)coeff);

    coeff = coefficients[i].temperature;
    SPI.transfer((uint8_t)(coeff >> 16));
    SPI.transfer((uint8_t)(coeff >> 8));
    SPI.transfer((uint8_t)coeff);
  }
  output_high(chip_select);
}


void write_custom_steinhart_hart(uint8_t chip_select, uint32_t steinhart_hart_coeffs[6], uint16_t start_address)
{
  int8_t i;
  uint32_t coeff;

  output_low(chip_select);

  SPI.transfer(WRITE_TO_RAM);
  SPI.transfer(highByte(start_address));
  SPI.transfer(lowByte(start_address));

  for (i = 0; i < 6; i++)
  {
    coeff = steinhart_hart_coeffs[i];
    SPI.transfer((uint8_t)(coeff >> 24));
    SPI.transfer((uint8_t)(coeff >> 16));
    SPI.transfer((uint8_t)(coeff >> 8));
    SPI.transfer((uint8_t)coeff);
  }
  output_high(chip_select);
}




// *****************
// Measure channel
// *****************
void measure_channel(uint8_t chip_select, uint8_t channel_number, uint8_t channel_output)
{
  convert_channel(chip_select, channel_number);
  get_result(chip_select, channel_number, channel_output);
}

void measure_channel_c(uint8_t chip_select, uint8_t channel_number, uint8_t channel_output , int8_t change_value)
{
  convert_channel(chip_select, channel_number);
  get_result_c(chip_select, channel_number, channel_output , change_value);
}
////////////////////////// ss0  //////////////////////////////////////////////////////////
void  measure_channel_f(uint8_t chip_select, uint8_t channel_number, uint8_t channel_output , int8_t change_value)
{
  convert_channel(chip_select, channel_number);
  get_result_f(chip_select, channel_number, channel_output , change_value);
}
//////////////////// ss1 ///////////////////////////////////////////////////////////
void  measure_channel_g(uint8_t chip_select, uint8_t channel_number, uint8_t channel_output , int8_t change_value)
{
  convert_channel(chip_select, channel_number);
  get_result_g(chip_select, channel_number, channel_output , change_value);
}
////////////////////////////////////////////////////////////////////

void convert_channel(uint8_t chip_select, uint8_t channel_number)
{
  // Start conversion
  transfer_byte(chip_select, WRITE_TO_RAM, COMMAND_STATUS_REGISTER, CONVERSION_CONTROL_BYTE | channel_number);

  wait_for_process_to_finish(chip_select);
}


void wait_for_process_to_finish(uint8_t chip_select)
{
  uint8_t process_finished = 0;
  uint8_t data;
  while (process_finished == 0)
  {
    data = transfer_byte(chip_select, READ_FROM_RAM, COMMAND_STATUS_REGISTER, 0);
    process_finished  = data & 0x40;
  }
}


// *********************************
// Get results
// *********************************
void get_result(uint8_t chip_select, uint8_t channel_number, uint8_t channel_output)
{
  uint32_t raw_data;
  uint8_t fault_data;
  uint16_t start_address = get_start_address(CONVERSION_RESULT_MEMORY_BASE, channel_number);
  uint32_t raw_conversion_result;

  raw_data = transfer_four_bytes(chip_select, READ_FROM_RAM, start_address, 0);

 // Serial.print(F("\nChannel "));
 // Serial.println(channel_number);

  // 24 LSB's are conversion result
  raw_conversion_result = raw_data & 0xFFFFFF;
  print_conversion_result(raw_conversion_result, channel_output);

  // If you're interested in the raw voltage or resistance, use the following
  if (channel_output != VOLTAGE)
  {
    read_voltage_or_resistance_results(chip_select, channel_number);
  }

  // 8 MSB's show the fault data
  fault_data = raw_data >> 24;
  print_fault_data(fault_data);
}

///
void get_result_c(uint8_t chip_select, uint8_t channel_number, uint8_t channel_output, int8_t change_value)
{
  uint32_t raw_data;
  uint8_t fault_data;
  uint16_t start_address = get_start_address(CONVERSION_RESULT_MEMORY_BASE, channel_number);
  uint32_t raw_conversion_result;

  raw_data = transfer_four_bytes(chip_select, READ_FROM_RAM, start_address, 0);

 // Serial.print(F("\nChannel "));
 // Serial.println(channel_number);

  // 24 LSB's are conversion result
  raw_conversion_result = raw_data & 0xFFFFFF;
  print_conversion_result_c(raw_conversion_result, channel_output, change_value);

  // If you're interested in the raw voltage or resistance, use the following
  if (channel_output != VOLTAGE)
  {
    read_voltage_or_resistance_results(chip_select, channel_number);
  }

  // 8 MSB's show the fault data
  fault_data = raw_data >> 24;
 // print_fault_data(fault_data);
}
//////////////////////////////// 2 /////////////////////////////////////////////////////

 float   fgAve0,fgAve1,fgAve2,fgAve3,fgAve4,fgAve5,fgAve6,fgAve7;
 float   fgAve8,fgAve9,fgAve10,fgAve11,fgAve12,fgAve13,fgAve14,fgAve15;
extern uint16_t fChannel0, fChannel1 , fChannel2, fChannel3, fChannel4, fChannel5, fChannel6, fChannel7;
extern uint16_t fChannel8, fChannel9 , fChannel10, fChannel11, fChannel12, fChannel13, fChannel14, fChannel15;


const uint16_t igNUM = 3; 
const uint16_t igTOTAL = igNUM +1; 
float fCh0[igTOTAL],fCh1[igTOTAL],fCh2[igTOTAL],fCh3[igTOTAL],fCh4[igTOTAL],fCh5[igTOTAL],fCh6[igTOTAL],fCh7[igTOTAL];

float fCh8[igTOTAL],fCh9[igTOTAL],fCh10[igTOTAL],fCh11[igTOTAL],fCh12[igTOTAL],fCh13[igTOTAL],fCh14[igTOTAL],fCh15[igTOTAL];


int igsum0=1, igsum1=1, igsum2=1, igsum3=1, igsum4=1, igsum5=1, igsum6=1, igsum7=1;
int igsum8=1, igsum9=1, igsum10=1, igsum11=1, igsum12=1, igsum13=1, igsum14=1, igsum15=1;

void get_result_f(uint8_t chip_select, uint8_t channel_number, uint8_t channel_output, int8_t change_value)
{

  uint32_t raw_data;
  uint8_t fault_data;
  uint16_t start_address = get_start_address(CONVERSION_RESULT_MEMORY_BASE, channel_number);
  uint32_t raw_conversion_result;
  raw_data = transfer_four_bytes(chip_select, READ_FROM_RAM, start_address, 0);
 raw_conversion_result = raw_data & 0xFFFFFF;

 int32_t signed_data = raw_conversion_result;
 if (signed_data & 0x800000)
    signed_data = signed_data | 0xFF000000;
  if(raw_data & 0x01000000){
    switch(channel_number){  
      case 4:
//Serial.println("aaaa");

      if(fChannel0== SVALUE) {
        fChannel0=SRETURN;  
        fCh0[0] =fCh0[0] -fCh0[igsum0];
        delay(1);
        fCh0[igsum0]=(float(signed_data) / 1024);
        fCh0[0] += fCh0[ igsum0];
        igsum0++;
        if( igsum0 >igNUM){ 
          igsum0=1;
        }
        fgAve0 = (fCh0[0]/igNUM) ;
      }
       //  Serial.print("CHANNEL0:  ");
        // Serial.println(fgAve0);
      break;
      
      case 6:
      if(fChannel1== SVALUE) {
        fChannel1=SRETURN;
        fCh1[0] =fCh1[0] -fCh1[igsum1];
        delay(1);
        fCh1[igsum1]=(float(signed_data) / 1024);
        fCh1[0] += fCh1[ igsum1];
        igsum1++;
        if( igsum1 >igNUM){ 
          igsum1=1;
        }
        fgAve1 = (fCh1[0]/igNUM) ;        
      }
      //Serial.print("CHANNEL1:  ");
     //    Serial.println(fgAve1);
      break;
    
      case 8:
      if(fChannel2== SVALUE) {
        fChannel2=SRETURN;
        fCh2[0] =fCh2[0] -fCh2[igsum2];
        delay(1);
        fCh2[igsum2]=(float(signed_data) / 1024);
        fCh2[0] += fCh2[ igsum2];
        igsum2++;
        if( igsum2 >igNUM){ 
          igsum2=1;
        }
        fgAve2 = (fCh2[0]/igNUM) ;
      }
      break;
      
      case 10:
      if(fChannel3== SVALUE) {
        fChannel3=SRETURN;
        fCh3[0] =fCh3[0] -fCh3[igsum3];
        delay(1);
        fCh3[igsum3]=(float(signed_data) / 1024);
        fCh3[0] += fCh3[ igsum3];
        igsum3++;
        if( igsum3 >igNUM){ 
          igsum3=1;
        }
        fgAve3 = (fCh3[0]/igNUM) ;
      }
      break;
  
      case 12:
      if(fChannel4== SVALUE) {
        fChannel4=SRETURN;
       fCh4[0] =fCh4[0] -fCh4[igsum4];
        delay(1);
        fCh4[igsum4]=(float(signed_data) / 1024);
        fCh4[0] += fCh4[ igsum4];
        igsum4++;
        if( igsum4 >igNUM){ 
          igsum4=1;
        }
        fgAve4 = (fCh4[0]/igNUM) ;
      }
      break;
  
      case 14:
      if(fChannel5== SVALUE) {
        fChannel5=SRETURN;
       fCh5[0] =fCh5[0] -fCh5[igsum5];
        delay(1);
        fCh5[igsum5]=(float(signed_data) / 1024);
        fCh5[0] += fCh5[ igsum5];
        igsum5++;
        if( igsum5 >igNUM){ 
          igsum5=1;
        }
        fgAve5 = (fCh5[0]/igNUM) ;
      }
      break;
      
      case 16:
      if(fChannel6== SVALUE) {
        fChannel6=SRETURN;
        fCh6[0] =fCh6[0] -fCh6[igsum6];
        delay(1);
        fCh6[igsum6]=(float(signed_data) / 1024);
        fCh6[0] += fCh6[ igsum6];
        igsum6++;
        if( igsum6 >igNUM){ 
          igsum6=1;
        }
        fgAve6 = (fCh6[0]/igNUM) ;
      }
      break;
  
      case 18:
      if(fChannel7== SVALUE) {       
        fChannel7=SRETURN;
        fCh7[0] =fCh7[0] -fCh7[igsum7];
        delay(1);
        fCh7[igsum7]=(float(signed_data) / 1024);
        fCh7[0] += fCh7[ igsum7];
        igsum7++;
        if( igsum7 >igNUM){ 
          igsum7=1;
        }
        fgAve7 = (fCh7[0]/igNUM) ;
      }
      break;
    }
  }
}
///////////////////////////// ss1 ///////////////////////////
void get_result_g(uint8_t chip_select, uint8_t channel_number, uint8_t channel_output, int8_t change_value)
{

  uint32_t raw_data;
  uint8_t fault_data;
  uint16_t start_address = get_start_address(CONVERSION_RESULT_MEMORY_BASE, channel_number);
  uint32_t raw_conversion_result;
  raw_data = transfer_four_bytes(chip_select, READ_FROM_RAM, start_address, 0);
 raw_conversion_result = raw_data & 0xFFFFFF;

 int32_t signed_data = raw_conversion_result;
 if (signed_data & 0x800000)
    signed_data = signed_data | 0xFF000000;
  if(raw_data & 0x01000000){
    switch(channel_number){  
      case 4:
//Serial.println("aaaa");

      if(fChannel8== SVALUE) {
        fChannel8=SRETURN;  
        fCh8[0] =fCh8[0] -fCh8[igsum8];
        delay(1);
        fCh8[igsum8]=(float(signed_data) / 1024);
        fCh8[0] += fCh8[ igsum8];
        igsum8++;
        if( igsum8 >igNUM){ 
          igsum8=1;
        }
        fgAve8 = (fCh8[0]/igNUM) ;
      }
      break;
      
      case 6:
      if(fChannel9== SVALUE) {
        fChannel9=SRETURN;
        fCh9[0] =fCh9[0] -fCh9[igsum9];
        delay(1);
        fCh9[igsum9]=(float(signed_data) / 1024);
        fCh9[0] += fCh9[ igsum9];
        igsum9++;
        if( igsum9 >igNUM){ 
          igsum9=1;
        }
        fgAve9 = (fCh9[0]/igNUM) ;
      }
      break;
    
      case 8:
      if(fChannel10== SVALUE) {
        fChannel10=SRETURN;
        fCh10[0] =fCh10[0] -fCh10[igsum10];
        delay(1);
        fCh10[igsum10]=(float(signed_data) / 1024);
        fCh10[0] += fCh10[ igsum10];
        igsum10++;
        if( igsum10 >igNUM){ 
          igsum10=1;
        }
        fgAve10 = (fCh10[0]/igNUM) ;
      }
      break;
      
      case 10:
      if(fChannel11== SVALUE) {
        fChannel11=SRETURN;
        fCh11[0] =fCh11[0] -fCh11[igsum11];
        delay(1);
        fCh11[igsum11]=(float(signed_data) / 1024);
        fCh11[0] += fCh11[ igsum11];
        igsum11++;
        if( igsum11 >igNUM){ 
          igsum11=1;
        }
        fgAve11 = (fCh11[0]/igNUM) ;
      }
      break;
  
      case 12:
      if(fChannel12== SVALUE) {
        fChannel12=SRETURN;
       fCh12[0] =fCh12[0] -fCh12[igsum12];
        delay(1);
        fCh12[igsum12]=(float(signed_data) / 1024);
        fCh12[0] += fCh12[ igsum12];
        igsum12++;
        if( igsum12 >igNUM){ 
          igsum12=1;
        }
        fgAve12 = (fCh12[0]/igNUM) ;
      }
      break;
  
      case 14:
      if(fChannel13== SVALUE) {
        fChannel13=SRETURN;
       fCh13[0] =fCh13[0] -fCh13[igsum13];
        delay(1);
        fCh13[igsum13]=(float(signed_data) / 1024);
        fCh13[0] += fCh13[ igsum13];
        igsum13++;
        if( igsum13 >igNUM){ 
          igsum13=1;
        }
        fgAve13 = (fCh13[0]/igNUM) ;
      }
      break;
      
      case 16:
      if(fChannel14== SVALUE) {
        fChannel14=SRETURN;
        fCh14[0] =fCh14[0] -fCh14[igsum14];
        delay(1);
        fCh14[igsum14]=(float(signed_data) / 1024);
        fCh14[0] += fCh14[ igsum14];
        igsum14++;
        if( igsum14 >igNUM){ 
          igsum14=1;
        }
        fgAve14 = (fCh14[0]/igNUM) ;
      }
      break;
  
      case 18:
      if(fChannel15== SVALUE) {       
        fChannel15=SRETURN;
        fCh15[0] =fCh15[0] -fCh15[igsum15];
        delay(1);
        fCh15[igsum15]=(float(signed_data) / 1024);
        fCh15[0] += fCh15[ igsum15];
        igsum15++;
        if( igsum15 >igNUM){ 
          igsum15=1;
        }
        fgAve15 = (fCh15[0]/igNUM) ;
      }
      break;
    }
  }
}


/////////////////////////// /////////////////////////////
void print_conversion_result(uint32_t raw_conversion_result, uint8_t channel_output)
{
  int32_t signed_data = raw_conversion_result;
  float scaled_result;

  // Convert the 24 LSB's into a signed 32-bit integer
  if (signed_data & 0x800000)
    signed_data = signed_data | 0xFF000000;

  // Translate and print result
  if (channel_output == TEMPERATURE)
  {
    scaled_result = float(signed_data) / 1024;
   // Serial.print(F("  Temperature = "));
  //  Serial.println(scaled_result );
  }
  else if (channel_output == VOLTAGE)
  {
    scaled_result = float(signed_data) / 2097152;
    //(F("  Direct ADC reading in V = "));
   // Serial.println(scaled_result);
  }

}


void print_conversion_result_c(uint32_t raw_conversion_result, uint8_t channel_output, int8_t change_value)
{
  int32_t signed_data = raw_conversion_result;
  float scaled_result;

  // Convert the 24 LSB's into a signed 32-bit integer
  if (signed_data & 0x800000)
    signed_data = signed_data | 0xFF000000;

  // Translate and print result
  if (channel_output == TEMPERATURE)
  {
    scaled_result = float(signed_data) / 1024;
   // Serial.print(F("  Temperature = "));
    //Serial.println(scaled_result-   change_value                             );//////
  }
  else if (channel_output == VOLTAGE)
  {
    scaled_result = float(signed_data) / 2097152;
    //(F("  Direct ADC reading in V = "));
   // Serial.println(scaled_result);
  }

}




void read_voltage_or_resistance_results(uint8_t chip_select, uint8_t channel_number)
{
  int32_t raw_data;
  float voltage_or_resistance_result;
  uint16_t start_address = get_start_address(VOUT_CH_BASE, channel_number);

  raw_data = transfer_four_bytes(chip_select, READ_FROM_RAM, start_address, 0);
  voltage_or_resistance_result = (float)raw_data/1024;
 ////// Serial.print(F("  Voltage or resistance = "));
 ////// Serial.println(voltage_or_resistance_result);
}


// Translate the fault byte into usable fault data and print it out
void print_fault_data(uint8_t fault_byte)
{
  
}

// *********************
// SPI RAM data transfer
// *********************
// To write to the RAM, set ram_read_or_write = WRITE_TO_RAM.
// To read from the RAM, set ram_read_or_write = READ_FROM_RAM.
// input_data is the data to send into the RAM. If you are reading from the part, set input_data = 0.

uint32_t transfer_four_bytes(uint8_t chip_select, uint8_t ram_read_or_write, uint16_t start_address, uint32_t input_data)
{
  uint32_t output_data;
  uint8_t tx[7], rx[7];

  tx[6] = ram_read_or_write;
  tx[5] = highByte(start_address);
  tx[4] = lowByte(start_address);
  tx[3] = (uint8_t)(input_data >> 24);
  tx[2] = (uint8_t)(input_data >> 16);
  tx[1] = (uint8_t)(input_data >> 8);
  tx[0] = (uint8_t) input_data;

  spi_transfer_block(chip_select, tx, rx, 7);

  output_data = (uint32_t) rx[3] << 24 |
                (uint32_t) rx[2] << 16 |
                (uint32_t) rx[1] << 8  |
                (uint32_t) rx[0];

  return output_data;
}


uint8_t transfer_byte(uint8_t chip_select, uint8_t ram_read_or_write, uint16_t start_address, uint8_t input_data)
{
  uint8_t tx[4], rx[4];

  tx[3] = ram_read_or_write;
  tx[2] = (uint8_t)(start_address >> 8);
  tx[1] = (uint8_t)start_address;
  tx[0] = input_data;
  spi_transfer_block(chip_select, tx, rx, 4);
  return rx[0];
}


// ******************************
// Misc support functions
// ******************************
uint16_t get_start_address(uint16_t base_address, uint8_t channel_number)
{
  return base_address + 4 * (channel_number-1);
}


bool is_number_in_array(uint8_t number, uint8_t *array, uint8_t array_length)
// Find out if a number is an element in an array
{
  bool found = false;
  for (uint8_t i=0; i< array_length; i++)
  {
    if (number == array[i])
    {
      found = true;
    }
  }
  return found;
}








