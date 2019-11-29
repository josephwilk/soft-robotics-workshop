/*

Arduino UNO sketch to check response rate for tinyLiDAR 
It will run a number of readings as fast as possible and give the effective response rate in Hz.
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

#define samples 1000     //number of samples for each loop
#define SCL_PORT PORTC
#define SDA_PORT PORTC
#define SCL_PIN 5        //std SCL pin
#define SDA_PIN 4        //std SDA pin
#include <I2C.h>
 
void setup() {

     Serial.begin(115200);     //setup the serial port
     I2c.begin(); 
     I2c.write(0x10,'M', 'C'); //send MC command for continuous mode 
     delay(1000);              //give time to reboot

} //setup

void loop() {

     uint16_t oldTime = 0;
     uint16_t timeNow = 0;
     uint16_t duration = 0.0;
     uint16_t i,j;

       while(1)
       {
       
          for (j = 0; j<samples;j++)
          {

               I2c.write(0x10,'D');  //take single reading
               I2c.read(0x10,2);     // request 2 bytes from tinyLiDAR
               i = I2c.receive();        // receive 1st byte
               i = i<<8 | I2c.receive(); // receive 2nd byte and put them together
          }

          timeNow = millis();           //read the current time in milliseconds
          duration = timeNow - oldTime;
          oldTime = timeNow;
          Serial.print(F("Effective Response Rate is ") ); 
          Serial.print( samples/(duration*.001) );  
          Serial.println(F("Hz ") );  

       } //while

} //loop
