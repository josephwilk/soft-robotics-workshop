/*
 
Arduino UNO sketch for tinyLiDAR 'Follow Me 2', Rev 1.1
This code uses 3x tinyLiDARs mounted on a pan-tilt platform with 2x analog servos to track an object within range

Last Edit: Oct 30, 2017
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

>> Change the I2C addresses for the tinyLiDARs using the AR command from the Terminal GUI Sketch before using this sketch

>> This code requires the "Arduino I2C Master Library rev5" library to allow the 
standard I2C stretch feature to work properly on the UNO.
** Please install it before running this sketch. ** 
The library can be downloaded from here:
http://dsscircuits.com/articles/arduino-i2c-master-library

>> You can run the tinyLiDARs on 3.3v or 5v for this sketch


Servos used: 
   Two of Hitec HS-55 servos and a Pan/Tilt bracket like #ROB-10335 from Sparkfun

Servo Pinout:
  Yellow pin = PWM input 
  Red pin = +5v 
  Black pin = GND

*/


// #define showDebugOutput   // comment out this line to hide the serial terminal debug output 


#define rebootTime  500 //in ms for tinyLiDAR reboot 

// change as needed
#define Left_tinyLiDAR 0x12     //I2C address for sensor on left side (to your left when facing the sensors)
#define Right_tinyLiDAR 0x13    //I2C address for sensor on right side (to your right when facing the sensors)
#define Bottom_tinyLiDAR 0x11   //I2C address for bottom sensor

#define tiltServo 10  // hardware pin for servo connection to tilt servo (yellow wire)
#define panServo 9    // hardware pin for servo connection to pan servo (yellow wire)

#define servoDirection -1   //is -1 for HS55 and 1 for SG90
#define tiltMinDeg  30  
#define tiltMaxDeg  120 
#define panMinDeg 20  
#define panMaxDeg  160 
#define step_res 1 				//min step size in degrees 
#define start_pulse_width 530 	//pulse width in microseconds for 0deg on servo
#define microsecondsPerDegree 10  //10us increments = 1deg steps on analog servo

#define servoDelayTime 1        // in ms
#define doubleCheckDelayTime 5  //in ms
#define sensingRangeMax 250     //in mm ** change as desired ** 
#define sensingRangeMin 20      //in mm ** change as desired ** 
#define movementThreshold_Limit_min 5    //in mm 
#define movementThreshold_Limit_max 100  //in mm 

#define SingleSensorAccelFactor 0.75  //for tilt
#define SingleSensorAccelFactorH 0.3  //for pan

#define SCL_PORT PORTC
#define SDA_PORT PORTC
#define SCL_PIN 5 //std SCL pin
#define SDA_PIN 4 //std SDA pin
#include <I2C.h>

int Left_Distance = 0;
int Right_Distance = 0;
int Top_Distance = 0;
int Bottom_Distance = 0;

float panAngle = 90.0;  // starting PAN angle, global variable
float tiltAngle = 90.0;  // starting TILT angle, global variable

int movementThreshold = movementThreshold_Limit_min;    //in mm to start - will be dynamically updated
int movementThreshold_Tilt = movementThreshold_Limit_min;    //in mm to start - will be dynamically updated

void setup() 
{
	int i;
	Serial.begin(115200); //for terminal used in debug only

	I2c.begin(); 
	I2c.setSpeed(0);  //0 is the 100KHz 1 is 400KHz
	I2c.pullup(0);    //disable internal pullup resistors
	I2c.timeOut(35);  //after 35ms I2C will release itself and reinitialize 

	pinMode(tiltServo, OUTPUT);		//set pin to output mode
	pinMode(panServo, OUTPUT);  	//set pin to output mode

	setupPWM16_2();   	//setup the TC1 timer for 16bit PWM - 2 outputs 
	updatePanServo(); 	//initial PAN angle 
	updateTiltServo(); 	//initial TILT angle 

	writeCommand(Bottom_tinyLiDAR,"PT");  //set to tinyLiDAR preset 
	writeCommand(Left_tinyLiDAR,"PT");    //set to tinyLiDAR preset 
	writeCommand(Right_tinyLiDAR,"PT");   //set to tinyLiDAR preset 
	delay(rebootTime);                    //to give some time for tinyLiDAR's reboot

	writeCommand(Bottom_tinyLiDAR,"MC");  //set to continuous mode for quickest response time
	writeCommand(Left_tinyLiDAR,"MC");    //set to continuous mode for quickest response time
	writeCommand(Right_tinyLiDAR,"MC");   //set to continuous mode for quickest response time
	delay(rebootTime);                    //to give some time for tinyLiDAR's reboot 

    Serial.println(F("\n\r ***********************************  "));
    Serial.println(F(" tinyLiDAR Follow Me Demo 2, ver 1.1 "));
    Serial.println(F(" ***********************************  \n\r"));

} //setup()


void loop() 
{

	while(1)
	{

		//Measure distance on the 3x tinyLiDARs
		Left_Distance = Read_Distance(Left_tinyLiDAR); 
		Right_Distance = Read_Distance(Right_tinyLiDAR); 
		Bottom_Distance = Read_Distance(Bottom_tinyLiDAR); 
		Top_Distance = (Left_Distance + Right_Distance)/2; //"top" is avg of the left and and right 'eyes'

		if (Left_Distance && Right_Distance) 	// only if both sensor show an object in range 
		{
		    //adjust threshold to midway of the two measured distances 
		    movementThreshold = abs(Left_Distance - Right_Distance)/2;
		    
		    //limit the threshold to min 
		    if (movementThreshold< movementThreshold_Limit_min)
		    {

		      movementThreshold = movementThreshold_Limit_min;

		    } //if

		    //and also limit the threshold to max
		    if (movementThreshold> movementThreshold_Limit_max)
		    {

		      movementThreshold = movementThreshold_Limit_max;

		    } //if

		    //if movement is to left then rotate left (i.e. if the distance is less on left side)
		    if (Left_Distance + movementThreshold < Right_Distance) 
		    {

				panAngle = panAngle - servoDirection*(step_res * movementThreshold/movementThreshold_Limit_min);

		    } //if

		    //if movement is to right then rotate right (i.e. if the distance is less on right side)
		    if (Right_Distance + movementThreshold < Left_Distance)
		    {

				panAngle = panAngle + servoDirection*(step_res * movementThreshold/movementThreshold_Limit_min);

		    } //if
		 
		    updatePanServo();
		   
		} //if

		if (Left_Distance && Right_Distance && Bottom_Distance)  // only if all sensors show an object in range
		{

			//adjust threshold to midway of the two measured distances 
			movementThreshold_Tilt = abs(Top_Distance - Bottom_Distance)/2;

			//limit the threshold to min 
			if (movementThreshold_Tilt< movementThreshold_Limit_min)
			{

			  movementThreshold_Tilt = movementThreshold_Limit_min;

			} //if

			//and also limit the threshold to max
			if (movementThreshold_Tilt> movementThreshold_Limit_max)
			{

			  movementThreshold_Tilt = movementThreshold_Limit_max;

			} //if

			//if movement is to TOP then rotate TOP (i.e. if the distance is less on top side)
			if (Top_Distance + movementThreshold_Tilt < Bottom_Distance) 
			{

			  tiltAngle = tiltAngle - servoDirection*(step_res * movementThreshold_Tilt/movementThreshold_Limit_min);

			} //if

			//if movement is to BOTTOM then rotate BOTTOM (i.e. if the distance is less on bottom side)
			if (Bottom_Distance + movementThreshold_Tilt < Top_Distance)
			{

			  tiltAngle = tiltAngle + servoDirection*(step_res * movementThreshold_Tilt/movementThreshold_Limit_min);

			} //if

			updateTiltServo();
		   
		} //if

		if (Left_Distance && !Right_Distance) // if left "eye" is showing an object in range only - ignore bottom
		{
		  
			delay(doubleCheckDelayTime); 

			//take another measurement to be sure before turning
			Left_Distance = Read_Distance(Left_tinyLiDAR); 
			Right_Distance = Read_Distance(Right_tinyLiDAR); 
			Bottom_Distance = Read_Distance(Bottom_tinyLiDAR); 

			if (Left_Distance && !Right_Distance) 
			{

				panAngle = panAngle - servoDirection*(step_res * SingleSensorAccelFactorH*Left_Distance/movementThreshold_Limit_min);          
				updatePanServo();

			} //if 

		} //if 

		if (!Left_Distance && Right_Distance ) // if right "eye" is showing an object in range only - ignore bottom
		{

			delay(doubleCheckDelayTime); 

			//take another measurement to be sure before turning
			Left_Distance = Read_Distance(Left_tinyLiDAR); 
			Right_Distance = Read_Distance(Right_tinyLiDAR); 
			Bottom_Distance = Read_Distance(Bottom_tinyLiDAR); 

			if (!Left_Distance && Right_Distance )  
			{

				panAngle = panAngle + servoDirection*(step_res * SingleSensorAccelFactorH*Right_Distance/movementThreshold_Limit_min);           
				updatePanServo();

			} //if 

		} //if 

		if (Left_Distance && Right_Distance && !Bottom_Distance)  // if Top "eyes" are showing an object in range only
		{
		    
			delay(doubleCheckDelayTime); 

			//take another measurement to be sure before turning
			Left_Distance = Read_Distance(Left_tinyLiDAR); 
			Right_Distance = Read_Distance(Right_tinyLiDAR); 
			Bottom_Distance = Read_Distance(Bottom_tinyLiDAR); 
			Top_Distance = (Left_Distance + Right_Distance)/2; //avg of the 2 readings above

			if (Left_Distance && Right_Distance && !Bottom_Distance)  // if Top "eye" is showing an object in range only
			{

				tiltAngle = tiltAngle - servoDirection*(step_res * SingleSensorAccelFactor * Top_Distance/movementThreshold_Limit_min);                                                
				updateTiltServo();

			} //if

		} //if

		if (!Left_Distance && !Right_Distance && Bottom_Distance) // if Bottom "eye" is showing an object in range only
		{

			delay(doubleCheckDelayTime); 

			//take another measurement to be sure before turning
			Left_Distance = Read_Distance(Left_tinyLiDAR); 
			Right_Distance = Read_Distance(Right_tinyLiDAR); 
			Bottom_Distance = Read_Distance(Bottom_tinyLiDAR); 
			Top_Distance = (Left_Distance + Right_Distance)/2; //avg of the 2 readings above

			if (!Left_Distance && !Right_Distance && Bottom_Distance) 
			{

				tiltAngle = tiltAngle + servoDirection*(step_res * SingleSensorAccelFactor * Bottom_Distance/movementThreshold_Limit_min);                        
				updateTiltServo();

			} //if 

		} //if

	} //while

} // loop()


void setupPWM16_2() 
{

    DDRB |= _BV(PB1) | _BV(PB2) ;       // pin 9 OC1A as output and pin 10 OC1B as output

    TCCR1A = _BV(COM1A1)                // Clear OC1A on Compare Match, set OC1A at BOTTOM (non-inverting mode)
			| _BV(COM1B1)		        // Clear OC1B on Compare Match, set OC1B at BOTTOM (non-inverting mode)
			| _BV(WGM11);     	       	// mode 14: fast PWM, TOP=ICR1 
    TCCR1B = _BV(WGM13) | _BV(WGM12)  	// mode 14: fast PWM, TOP=ICR1 cont'd
			| _BV(CS11) ;               // div by 8 prescaling 
    ICR1 = 0xffff;                      // TOP counter value  

} //setupPWM16

 
void updateTiltServo(void)   //work on the global tiltAngle instead to limit it here as needed 
{
  
   if (tiltAngle<tiltMinDeg)
      tiltAngle = tiltMinDeg;

   if (tiltAngle>tiltMaxDeg)
      tiltAngle = tiltMaxDeg;
   
	int j = (start_pulse_width+tiltAngle*microsecondsPerDegree)*2;

    OCR1B = j;    //write the 16bit value to TC1 timer [OCR1A = pin 9, OCR1B = pin 10]

#ifdef showDebugOutput

    Serial.print(F("tinyLiDAR Debug >> Top: "));
    Serial.print(Top_Distance);

    Serial.print(F(", Bottom: "));
    Serial.print(Bottom_Distance);

    Serial.print(F(", TiltAngle: "));
    Serial.print(tiltAngle);

    Serial.print(F(", TILT_threshold: "));
    Serial.print(movementThreshold_Tilt);

    Serial.print(F(", Left: "));
    Serial.print(Left_Distance);

    Serial.print(F(", Right: "));
    Serial.print(Right_Distance);

    Serial.print(F(", PanAngle: "));
    Serial.print(panAngle);

    Serial.print(F(", PAN_threshold: "));
    Serial.println(movementThreshold);

#endif

} //writeToServo
  

void updatePanServo(void)  //work on the global panAngle instead to limit it here as needed 
{
   
   if (panAngle<panMinDeg)
      panAngle = panMinDeg;

   if (panAngle>panMaxDeg)
      panAngle = panMaxDeg;
   
    int j = (start_pulse_width+panAngle*microsecondsPerDegree)*2;

    OCR1A = j;    //write the 16bit value to TC1 timer [OCR1A = pin 9, OCR1B = pin 10]
 

#ifdef showDebugOutput

    Serial.print(F("tinyLiDAR Debug >> Top: "));
    Serial.print(Top_Distance);

    Serial.print(F(", Bottom: "));
    Serial.print(Bottom_Distance);

    Serial.print(F(", TiltAngle: "));
    Serial.print(tiltAngle);

    Serial.print(F(", TILT_threshold: "));
    Serial.print(movementThreshold_Tilt);

    Serial.print(F(", Left: "));
    Serial.print(Left_Distance);

    Serial.print(F(", Right: "));
    Serial.print(Right_Distance);

    Serial.print(F(", PanAngle: "));
    Serial.print(panAngle);

    Serial.print(F(", PAN_threshold: "));
    Serial.println(movementThreshold);

#endif

} //writeToServo

uint16_t Read_Distance(uint8_t targetAddr) 
{

    uint16_t i;
    if( !I2c.write(targetAddr,'D') ) 
    {     

        i = I2c.read(targetAddr,2); //request 2 bytes for distance

        while ( I2c.available() ) 
        { 

            i = I2c.receive();        // receive 1st byte
            i = i<<8 | I2c.receive(); // receive 2nd byte

        } //while

    }
    else
    {

        Serial.print(F(" Error talking to device at address 0x")); 
        Serial.println(targetAddr, HEX);
        return 0; //exit

    } //if else
    

	if (i >= sensingRangeMin && i <= sensingRangeMax)
	{

		return i; //send back the distance only if within range

	} 
	else 
	{

		return 0; //else return zero

	} //if else

    return i;

} //Read_Distance() 

void writeCommand(uint8_t targetAddr, String command)
{

  if (command.length() == 2)
  {

	I2c.write(targetAddr,command.charAt(0), command.charAt(1)); 

  }
  else
  {

    I2c.write(targetAddr,command.charAt(0) ); 

  } //if else

} //writeCommand()
