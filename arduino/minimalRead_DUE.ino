/*

Arduino DUE sketch to show minimal coding required to read distance from tinyLiDAR 
This program will continually print the measured distance from tinyLiDAR which
is operating in its default single step mode.

Last Edit: Jan 26, 2018
Copyright (c) 2018 by Dinesh Bhatia 

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
	This minimal sketch uses the Wire.h library for the Arduino DUE

	Grove connector wiring: 
		Black = GND pin
		Red = +3.3V out from DUE
		White = pin 20 (SDA)
		Yellow = pin 21 (SCL)
*/

#include <Wire.h>
 
void setup() {

    Serial.begin(115200);     //setup the serial port
    Wire.begin();  

} //setup

void loop() {

  uint16_t i;

  while(1)
  {
   
    Wire.beginTransmission(0x10);  // take single measurement
    Wire.write('D');
    Wire.endTransmission();

	  Wire.requestFrom(0x10, 2);    // request 2 bytes from tinyLiDAR

    i = Wire.read();        // receive MSB byte
    i = i<<8 | Wire.read(); // receive LSB byte and put them together

    Serial.println(i);        // print distance in mm 
    delay(100);               // delay as required (14ms or higher in default single step mode)
 
  } //while
 

} //loop
 