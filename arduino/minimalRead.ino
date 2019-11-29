/*

Arduino UNO sketch to show minimal coding required to read distance from tinyLiDAR 
This program will continually print the measured distance from tinyLiDAR which
is operating in its default single step mode.
Last Edit: Oct 23, 2017
Copyright (c) 2017 by Dinesh Bhatia 

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>

Notes:
This code requires the "Arduino I2C Master Library rev5" library to allow the 
standard I2C stretch feature to work properly on the UNO.
** Please install it before running this sketch. ** 
The library can be downloaded from here:
http://dsscircuits.com/articles/arduino-i2c-master-library

*/


#define SCL_PORT PORTC
#define SDA_PORT PORTC
#define SCL_PIN 5        //std SCL pin
#define SDA_PIN 4        //std SDA pin
#include <I2C.h>
 
void setup() {

     Serial.begin(115200);     //setup the serial port
     I2c.begin(); 

} //setup

void loop() {

  uint16_t i;

  while(1)
  {
   
    I2c.write(0x10,'D');  //take single measurement
    I2c.read(0x10,2);     // request 2 bytes from tinyLiDAR
    i = I2c.receive();        // receive MSB byte
    i = i<<8 | I2c.receive(); // receive LSB byte and put them together
    Serial.println(i);        // print distance in mm 
    delay(100);               // delay as required (13ms or higher in default single step mode)
 
  } //while
 

} //loop
 