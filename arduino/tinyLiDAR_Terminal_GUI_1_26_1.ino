/*

tinyLiDAR Terminal GUI sketch for Arduino UNO, LEONARDO and MEGA 
ver 1.26.1 to align with Firmware 1.4.1
- updated to remove use of strtok() & fix compiler warnings

Provides a simple user interface for all supported commands in tinyLiDAR. 

Last Edit: March 27, 2018 
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

    This code requires the "Arduino I2C Master Library rev5" library to allow the 
    standard I2C stretch feature to work properly on the UNO and LEONARDO.
    ** Please install it before running this sketch. ** 
    The library can be downloaded from here:
    http://dsscircuits.com/articles/arduino-i2c-master-library

    The Arduino terminal must be set to 115200 baud rate with 'Both NL & CR' selected.
    
    Compatible with: 

        Arduino LEONARDO(ATMEGA32U4-XUMU) 
        Arudino UNO (ATMEGA328P-PU)
        Arduino MEGA (ATMEGA2560-16AU)
   
*/
// ***************************************** 
// Please uncomment for your board type: 
// 
// #define ArduinoLEONARDO
 #define ArduinoUNO
 //#define ArduinoMEGA 
// ***************************************** 
 

#define QresponseSize 23    // number of bytes to read for each Q command
#define rebootTime  1000     // in ms 

// default tinyLiDAR module address used in this terminal to talk to tinyLiDAR
// can change for session using the I command
#define tinyLiDAR_Address 0x10  

//default config values for the W command
#define SignalRateLimit 10 //default value for SignalRateLimit which can be 0.00 to 65.00 MCPS, is entered here as 100x the required value
#define SigmalLimit 60  //default value for SigmaLimit in mm 
#define TimingBudget 20 //default value for TimingBudget in milliseconds (ms), value can be 20 to 2000ms
#define VCELperiod 18 //default value for VCELperiod selection, can choose either 18/14 or 14/10. 18 = 18/14, 14 = 14/10
//#define VCELperiod 14 //default value for VCELperiod selection, can choose either 18/14 or 14/10. 18 = 18/14, 14 = 14/10

//default config values for the calibration commands
#define OffsetCalDistanceToTarget 100   //exact distance to target in mm used for the CD command
#define XtalkCalDistanceToTarget 400    //exact distance to target in mm used for the CX command

#define minTimeRT  20 // wait for at least this much time + timingBudget before attempting reading
#define minTimeBtwnCmnd 10	//in ms
#define deltaDelay 5  // incremental time to wait for data to be ready for real time mode

#define PHtime 200
#define PLtime 33
#define PStime 20
#define PTtime 18


#ifdef ArduinoLEONARDO 

	#include <I2C.h>
    #define SCL_PORT PORTD
    #define SDA_PORT PORTD
    #define SCL_PIN 0 //std SCL pin for LEONARDO
    #define SDA_PIN 1 //std SDA pin for LEONARDO
    #define I2Cpins B00000011

#elif defined ArduinoMEGA

	#include <Wire.h>
    #define SCL_PORT PORTD
    #define SDA_PORT PORTD
    #define SCL_PIN 0 //std SCL pin for DUE/MEGA 
    #define SDA_PIN 1 //std SDA pin for DUE/MEGA
    #define I2Cpins B00000011

#elif defined ArduinoUNO

	#include <I2C.h>
    #define SCL_PORT PORTC
    #define SDA_PORT PORTC
    #define SCL_PIN 5 //std SCL pin for UNO
    #define SDA_PIN 4 //std SDA pin for UNO
    #define I2Cpins B00110000

#endif


#define WriteSetupDelay delayMicroseconds(1000)  

bool tinyLiDAR_Default = false;

// Command line variables
String command; // String input from command prompt

char inByte; // Byte input from command prompt
uint8_t I2C_Address = tinyLiDAR_Address;    //global variable
uint8_t Topi2cAdr  = tinyLiDAR_Address;     //set to single board default to start off - used for auto I2C loop
uint8_t Boti2cAdr = tinyLiDAR_Address;
bool RTmode = false;	// flag to indicate if Real Time mode is active 
uint16_t _timingBudget = PLtime;  //measurement time for Real Time mode - updated based on mode
uint8_t I2c_write(byte address = 0x10, int data1 = 0x44, int data2 = -1 ); //default values used if not provided in function call 
uint8_t I2c_read(uint8_t address = 0x10, uint8_t numBytes = 1);
uint8_t I2c_receive(void);
uint8_t I2c_available(void);
uint8_t I2c_writeArray(uint8_t address, uint8_t command, uint8_t paramArray[], uint8_t size);
void tinyLiDAR(String _command,uint16_t _delay,uint8_t _targetAddr);
void Write_I2C_Long16(uint8_t address, uint8_t command ,uint8_t data0, uint16_t dataByte16); 
void Qcommand(uint8_t I2C_Address);
void Dcommand(byte j, byte delayBetween, uint8_t addr);  // j num of bytes, up to 255ms delay between and from addr
uint16_t Read_Distance(uint8_t addr); 
bool confirm(void); 


void setup() {

#if (defined ArduinoLEONARDO || defined ArduinoUNO)

    I2c.begin(); 
    I2c.setSpeed(0);  //0 is the 100KHz 1 is 400KHz
    I2c.pullup(0);    //disable internal pullup resistors
    I2c.timeOut(10);  //after 35ms I2C will release itself and re-initialize if SDA is stuck low

#elif defined ArduinoMEGA

	Wire.begin(); 
	Wire.setClock(100000); //set to 100KHz

#endif

    Serial.begin(115200); // start serial communication at 115200bps

    Serial.println(F("\
    ---------------------------------  \n\
  :: tinyLiDAR Command Terminal v1.26 ::\n\
    ---------------------------------       "));
    Serial.print(F("\n  Default I2C target address: 0x"));
    Serial.println(I2C_Address, HEX);
    printMenu();

} // setup()

void loop() 
{

    uint8_t i, m;
    uint16_t cal_offset_distance = OffsetCalDistanceToTarget;
    uint16_t cal_xtalk_distance = XtalkCalDistanceToTarget;
    uint8_t i2cAdr = tinyLiDAR_Address;
    uint8_t tL_addr[0x68];
    uint8_t StartI2cAdr = 0;
    uint8_t param[6]; 
    uint16_t InputValues[6];


    if (Serial.available() > 0) 
    {

        inByte = Serial.read();

        // only accept input if a letter, number comma or space is typed
        if ((inByte >= 65 && inByte <= 90) || (inByte >= 97 && inByte <= 122) || (inByte >= 48 && inByte <= 57) || inByte == 44 || inByte == 32) 
        {

            command.concat(inByte);

        } // if

    } // if


    if (inByte == 13) // Process command only after the ENTER key is pressed
    {

        command.toUpperCase();

        inByte = 0;


        if (command.equals("")) 
        {
            //make D as default command
            Serial.print(F("[default] Command 'D' to 0x")); //new dec 1 12:33pm****
            Serial.print(I2C_Address, HEX);

            if(RTmode)
            {
                Dcommand(1,(_timingBudget+minTimeRT), I2C_Address);   
            }else 
            {
                Dcommand(1, 0, I2C_Address); 
            }


        } // Perform a function because of a command:
        else if (command.equals("T0")) 
        {

            Serial.println(F(": Disable WatchDog Timer. "));
            tinyLiDAR(command,rebootTime,I2C_Address);
           
        } 
        else if (command.equals("T1")) 
        {

            Serial.println(F(": Enable WatchDog Timer. "));
            tinyLiDAR(command,rebootTime,I2C_Address);
          
        } 
        else if (command.equals("D")) 
        {

            Serial.print(F("\n\r Command 'D' to 0x"));
            Serial.println(I2C_Address, HEX);

            if(RTmode)
            {
                Dcommand(1,(_timingBudget+minTimeRT), I2C_Address);   
            }else 
            {
                Dcommand(1, 0, I2C_Address); 
            }

        } 
        else if (command.equals("E")) 
        {

            Serial.println(F(": LED indicator disabled. "));
            tinyLiDAR(command,0,I2C_Address);

        } 
        else if (command.equals("F")) 
        {

            Serial.println(F(": LED indicator enabled. "));
            tinyLiDAR(command,0,I2C_Address);

        } 
        else if (command.equals("G")) 
        {

            Serial.println(F(": LED on. "));
            tinyLiDAR(command,0,I2C_Address);

        } 
        else if (command.equals("DC")) 
        {

            Serial.println(F("\n\r Command 'D C' to continually read distance "));
            Serial.println(F("\n *** Press the ENTER key to exit this loop *** "));
            Cont_Read_Distance(I2C_Address); 
          
        } 
        else if (command.equals("MR")) 
        {

            RTmode = true;    
            Serial.println(F(": tinyLiDAR set to Real Time mode. "));
            tinyLiDAR(command,rebootTime,I2C_Address);
          
        } 

        else if (command.equals("MC")) 
        {

            RTmode = false;    
            Serial.println(F(": tinyLiDAR set to Continuous mode. "));
            tinyLiDAR(command,rebootTime,I2C_Address);
          
        } 
        else if (command.equals("MS")) 
        {
            RTmode = false;    
            Serial.println(F(": tinyLiDAR set to SingleStep/UltraLowPower mode. "));
            tinyLiDAR(command,rebootTime,I2C_Address);
        }         

             
        else if (command.equals("PL")) 
        {

            _timingBudget = PLtime;     
            Serial.println(F(": tinyLiDAR preset to Long Range configuration.  "));
            tinyLiDAR(command,rebootTime+_timingBudget,I2C_Address);

        } 
        else if (command.equals("PH")) 
        {

            _timingBudget = PHtime;     
            Serial.println(F(": tinyLiDAR preset to High Accuracy configuration.  "));
            tinyLiDAR(command,rebootTime+_timingBudget,I2C_Address);

        } 
        else if (command.equals("PS")) 
        {

            _timingBudget = PStime;     
            Serial.println(F(": tinyLiDAR preset to High Speed configuration.  "));
            tinyLiDAR(command,rebootTime+_timingBudget,I2C_Address);
        } 
        else if (command.equals("PT")) 
        {

            _timingBudget = PTtime;     
            Serial.println(F(": tinyLiDAR preset to default Long Range+Speed configuration.  "));
            tinyLiDAR(command,rebootTime+_timingBudget,I2C_Address);

        } 

		else if (command.equals("U")) 
        {  

            Serial.println(F(": start Ultrasonic Emulation mode "));
            tinyLiDAR(command,rebootTime,I2C_Address);

        }

        else if (command.equals("X")) 
        {  

            Serial.println(F(": reboot tinyLiDAR.  "));
            tinyLiDAR(command,rebootTime,I2C_Address);

        }
        else if (command.equals("Y")) 
        {

            Serial.println(F(": save LED mode and reboot tinyLiDAR.  "));
            tinyLiDAR(command,rebootTime,I2C_Address);

        }
        else if (command.equals("Q")) 
        {

            Serial.print(F("\n\r Command 'Q' to 0x"));
            Serial.print(I2C_Address, HEX);
            Qcommand(I2C_Address);

        } 
        else if (command.equals("CD")) 
        {

            RTmode = false; //to select the proper D command later

            Serial.print(F("\n\r Command 'C D' to 0x"));
            Serial.print(I2C_Address, HEX);
            Serial.print(F(": Perform Offset Cal at a distance of = "));
            Serial.print(cal_offset_distance);  
            Serial.println(F(" mm"));

            //turn off WDT
            Serial.println(F(": Disable WatchDog Timer. "));
            tinyLiDAR("T0",rebootTime,I2C_Address);

            //set to high accuracy mode (200ms per measurement)
            _timingBudget = PHtime;     
            Serial.println(F(": tinyLiDAR preset to High Accuracy configuration.  "));
            tinyLiDAR("PH",rebootTime+_timingBudget,I2C_Address);

            // set to continuous mode
            Serial.println(F(": tinyLiDAR set to Continuous mode. "));
            tinyLiDAR("MC",rebootTime,I2C_Address);

            Serial.println(F(" Distance Before Cal: "));

            Dcommand(10,215, I2C_Address);      //need 200ms at least so give a bit extra

            Write_I2C_Long16(I2C_Address, 0x43, 0x44, cal_offset_distance); //CD command

            Serial.println(F("\n\r **** Calibrating Now - Please wait approx 10sec **** "));

            delay(11000);     //give it over 10sec to calibrate now

            // reboot tinyLiDAR
            Serial.println(F(": reboot tinyLiDAR.  "));
            tinyLiDAR("X",rebootTime,I2C_Address);

            Serial.println(F(" Distance After Cal: "));

            Dcommand(10,215, I2C_Address);     //need 200ms at least so give a bit extra

            //set to single step mode again
            Serial.println(F(": tinyLiDAR set to SingleStep/UltraLowPower mode. "));
            tinyLiDAR("MS",rebootTime,I2C_Address);


            Serial.println(F("\n\r **** Cal Complete **** "));

        } 
        else if (command.equals("CX")) 
        {

            RTmode = false; //to select the proper D command later

            Serial.print(F("\n Command 'C X' to 0x"));
            Serial.print(I2C_Address, HEX);
            Serial.print(F(": Perform Crosstalk Cal at a distance of = "));
            Serial.print(cal_xtalk_distance); 
            Serial.println(F(" mm"));


            //turn off WDT
            Serial.println(F(": Disable WatchDog Timer. "));
            tinyLiDAR("T0",rebootTime,I2C_Address);

            //set to high accuracy mode (200ms per measurement)
            _timingBudget = PHtime;     
            Serial.println(F(": tinyLiDAR preset to High Accuracy configuration.  "));
            tinyLiDAR("PH",rebootTime,I2C_Address);

            
            //set to continuous mode
            Serial.println(F(": tinyLiDAR set to Continuous mode. "));
            tinyLiDAR("MC",rebootTime,I2C_Address);

            Serial.println(F(" Distance Before Cal: "));

            Dcommand(10,215, I2C_Address);      //need 200ms at least so give a bit extra

            Write_I2C_Long16(I2C_Address, 0x43, 0x58, cal_xtalk_distance); //CX command

            Serial.println(F("\n\r **** Calibrating Now - Please wait approx 10sec **** "));
            
            delay(11000);                   //give it over 10sec to calibrate now

            // reboot tinyLiDAR
            Serial.println(F(": reboot tinyLiDAR.  "));
            tinyLiDAR("X",rebootTime,I2C_Address);

            Serial.println(F(" Distance After Cal: "));
           
            Dcommand(10,215, I2C_Address);      //need 200ms at least so give a bit extra

            //set to single step mode again
            Serial.println(F(": tinyLiDAR set to SingleStep/UltraLowPower mode. "));
            tinyLiDAR("MS",rebootTime,I2C_Address);

            Serial.println(F("\n\r **** Cal Complete **** "));

        } 
        else if (command.equals("RESET")) 
        {

            Serial.println(F("\n\r  Command 'RESET' to General Call Address 0x00: "));
            Serial.println(F("  tinyLiDAR(s) will now be reset to factory defaults. \n"));    
            
            I2c_write(0,0x06);  
            delay(rebootTime);    //wait for reboot 
            asm volatile("jmp 0");  //restart Arduino  

        } 
        else if (command.startsWith("I")) 
        {

            String hexstring = command.substring(0);

            if (hexstring == "I") 
            {
                
                I2C_Address = tinyLiDAR_Address; //set to default I2C addr if no value entered 

            } else {

                I2C_Address = strtol( & hexstring[1], NULL, 16);

            } // if else

            Serial.print(F("\n\rCommand 'I' "));
            Serial.println(F(": change the Terminal's I2C address."));
            Serial.print(F(" --> New I2C Address for Terminal = 0x"));
            Serial.println(I2C_Address, HEX);
            Serial.println("");

        } 
        else if (command.startsWith("R")) 
        {

            String hexstring = command.substring(0);

            if (hexstring == "R") 
            {

                I2C_Address = tinyLiDAR_Address; //set to default if no value entered 

            } else 
            {

                i2cAdr = strtol( & hexstring[1], NULL, 16);

            } // if else

            Serial.print(F("\n\rCommand 'R' to 0x"));
            Serial.print(I2C_Address, HEX);
            Serial.println(F(": change the I2C target address for tinyLiDAR."));
            Serial.print(F(" --> New I2C Address for tinyLiDAR = 0x"));
            Serial.println(i2cAdr, HEX);
            Serial.print(F("\n\r"));

            //change the actual address in tinyLiDAR   
            I2c_write(I2C_Address,0x52, i2cAdr);  //R is 0x52, the new desired address 
            delay(60); //wait for the new address to take effect in tinyLiDAR (EEPROM write delay)

        } 
       

        else if (command.equals("AZ"))   //for autonomous mode exit only
        {

            Serial.print(F("\n\r Command 'A Z'"));
            Serial.println(F(": Autonomous Mode Exit. "));

            #ifdef ArduinoUNO
                
                I2c.end();      // disable Arduino I2C pins so we can set to low
                DDRC = DDRC | I2Cpins;  //1 is out, 0 is in
                PORTC &= ~I2Cpins;      // all low out

            #elif defined ArduinoLEONARDO

                I2c.end();      // disable Arduino I2C pins so we can set to low
                DDRD = DDRD | I2Cpins;  //1 is out, 0 is in
                PORTD &= ~I2Cpins;      // all low out

            #elif defined ArduinoMEGA

                Wire.end();  
                DDRD = DDRD | I2Cpins;  //1 is out, 0 is in
                PORTD &= ~I2Cpins;      // all low out

            #endif

            delay(2*rebootTime); //req for reboot
	        //reset Arduino to ensure I2C works
	        asm volatile("jmp 0");  //restart Arduino  

        } 

		else if (command.equals("UZ"))   //for ultrasonic emulation mode exit only
        {

            Serial.print(F("\n\r Command 'U Z'"));
            Serial.println(F(": Ultrasonic Emulation Mode Exit. "));
            Serial.println(F(" >> Press RESET on tinyLiDAR now << "));
         

            #ifdef ArduinoUNO
                
                I2c.end();      // disable Arduino I2C pins so we can set to low
                DDRC = DDRC | I2Cpins;  //1 is out, 0 is in
                PORTC &= ~I2Cpins;      // all low out

            #elif defined ArduinoLEONARDO

                I2c.end();      // disable Arduino I2C pins so we can set to low
                DDRD = DDRD | I2Cpins;  //1 is out, 0 is in
                PORTD &= ~I2Cpins;      // all low out

            #elif defined ArduinoMEGA

                Wire.end(); 
                DDRD = DDRD | I2Cpins;  //1 is out, 0 is in
                PORTD &= ~I2Cpins;      // all low out

            #endif
                
                Serial.print(F("\n   "));

                for(i = 11; i > 0 ; i--)
                {
                    Serial.print(i-1);
                    Serial.print(F(" "));
                     delay(200);    // give it time to reboot and sense the lines being held low                  
                }

            delay(rebootTime);
	        //reset Arduino to ensure I2C works
	        asm volatile("jmp 0");  //restart Arduino  

        } 
        else if (command.equals("AR")) 
        {

            String hexstring = command.substring(0);

            if (hexstring == "AR") 
            {

              StartI2cAdr = tinyLiDAR_Address; //set to default if no value entered 

            } else 
            {

              StartI2cAdr = strtol( & hexstring[2], NULL, 16); //start at this new range as required for programming the units found                  

            } // if else

            i2cAdr = StartI2cAdr; //start loop at this new range as required for programming the units found              

            Serial.print(F("\n\r Command 'A R', using starting Adr of 0x"));
            Serial.print(i2cAdr, HEX);          //ensure default I2C is used here only so we can write to it
            Serial.println(F(": I2C Auto-Configure loop. "));
            Serial.println(F(" ** NOTE: Only the RESET BUTTON can ABORT the AR command ** "));
            Serial.println(F(" >> Place finger in front of a blinking tinyLiDAR sensor now << "));

            //is X command without post delay
            Serial.println(F(": reboot tinyLiDAR.  "));
            tinyLiDAR("X",0,I2C_Address);

            #ifdef ArduinoUNO

                I2c.end();      // disable Arduino I2C pins so we can set to low
                DDRC = DDRC | I2Cpins;  //1 is out, 0 is in
                PORTC = SCL_PIN;      // all low out except SCL pin
                delay(1000);    // give it time to reboot and sense the lines being held low                  
                //start up again
                I2c.begin(); 
                I2c.setSpeed(0);  //0 is the 100KHz 1 is 400KHz
                I2c.pullup(0);    //disable internal pullup resistors

            #elif defined ArduinoLEONARDO

                I2c.end();      // disable Arduino I2C pins so we can set to low
                DDRD = DDRD | I2Cpins;  //1 is out, 0 is in
                PORTD = SCL_PIN;      // all low out except SCL pin

                delay(1000);    // give it time to reboot and sense the lines being held low                  
                
                //start up again
                I2c.begin(); 
                I2c.setSpeed(0);  //0 is the 100KHz 1 is 400KHz
                I2c.pullup(0);    //disable internal pullup resistors

            #elif defined ArduinoMEGA

                Wire.end(); 
                DDRD = DDRD | I2Cpins;  //1 is out, 0 is in
                PORTD = SCL_PIN;      // all low out except SCL pin
                delay(1000);    // give it time to reboot and sense the lines being held low                  

                //start up again
                Wire.begin(); 
                Wire.setClock(100000); //set to 100KHz

            #endif
        
            do
            {  

                inByte = 0;
                while(1)
                {

                    i = 1; 
                    do
                    {

                        i = I2c_write(tinyLiDAR_Address,'E');  //use E command for LED disable 
                        delay(100);                            //space out reads a bit

                        if (Serial.available() > 0) 
                        {

                            inByte = (char)Serial.read(); // store the char
                            if (inByte == '\r')
                                 goto LoopExit;

                        } // if 

                    } while(i); // code is 0 when it finds a board

                    if(i2cAdr==tinyLiDAR_Address)
                    {

                        i2cAdr++;

                    } // if 

                    Serial.print(F(" -->> Found a new tinyLiDAR sensor, now setting address to 0x"));
                    Serial.print(i2cAdr, HEX);
                    Serial.print(F("... "));

                    //change the actual address in tinyLiDAR   
                    I2c_write(tinyLiDAR_Address,0x52, i2cAdr); //0x52 is 'R' command 

                    delay(100);  //short delay for eeprom write

                    // reboot this one 
                    Serial.println(F(": reboot tinyLiDAR.  "));
                    tinyLiDAR("X",rebootTime,I2C_Address);

                    i2cAdr++;

                    Serial.println(F("done. Point to next one or hit ENTER key to quit. " ));                    

                    if (Serial.available() > 0) 
                    {

                        inByte = (char)Serial.read(); // store the char
                        if (inByte == '\r')
                             // break;
                           goto LoopExit;

                    } // if

                } // while 

LoopExit:
                Serial.println(F(" Okay, now exiting I2C Auto-Configure loop. "));

            }while (inByte != '\r'); // only the ENTER key will exit loop

        } 
        else if (command.equals("V"))  
        {

            Serial.println(F("\n\r Command 'V'. Scan and sequentially blink each tinyLiDAR on bus."));
            Serial.println(F("\n\r Scanning..."));

            m = 0;
            Topi2cAdr = 0;

            memset(tL_addr,0,sizeof(tL_addr)); //clear out the array first

            for ( i = tinyLiDAR_Address; i < tinyLiDAR_Address+sizeof(tL_addr); i++) 
            {

                if ( I2c_write(i,0x45) == 0 ) 
                {

                    tL_addr[i] = 0xa5;  //use '0xa5' to say it's found a board at this addr

                    if (i == tinyLiDAR_Address)
                         tinyLiDAR_Default = true; //check if a module is preset on bus at default addr and set flag

                    Serial.print(F(" Found tinyLiDAR at: 0x"));
                    Serial.println(i, HEX);

                    if (i>Topi2cAdr)
                         Topi2cAdr = i;  

                    m++;

                } //if

            } // for i

            Serial.print (" Scan complete. Found ");
            Serial.print (m, DEC);
            Serial.println (" sensor(s). ");
  
            i =  tinyLiDAR_Address-1; 

            do
            {

                i++;

                if (i > ( tinyLiDAR_Address + sizeof(tL_addr) ) ) 
                {

                      i = tinyLiDAR_Address;

                } //if 

                 if (tL_addr[i] == 0xa5)
                 {

                    Serial.print(F(" Now blinking tinyLiDAR at 0x"));
                    Serial.print(i, HEX);
                    Serial.println(F(". Hit 'ENTER' to quit. "));

                    for (m=0; m<10 ; m++)
                    {

                        tinyLiDAR("G",100,i);
                        tinyLiDAR("E",100,i);

                        if (Serial.available() > 0) {

                            inByte = (char)Serial.read(); // store the char
                            if (inByte == '\r')
                                 break;
                            
                        } //if 

                    }//for

                 } //if 

            }while(inByte != '\r'); //keep looping until ENTER key is hit

            //to enable the LED indicator again
            tinyLiDAR("F",0,i);            

            Serial.println("");

        }
        else if (command.startsWith("B"))
        {

            uint8_t i = 0;
            Serial.println(F("\n Command 'B': Autonomous IoT Mode."));

            String hexstring = command.substring(0);

            if (hexstring == "B")  
            {

                Serial.println(F("\n\r> Nothing entered - using default values: "));

                InputValues[0] = 0;   //default Object Mode False, 1 is True, 0 = False, 1 = True
                InputValues[1] = 50;  //default Lower threshold in mm, 10 to 2000mm 
                InputValues[2] = 150;  //default Upper threshold in mm, 10 to 2000mm 
                InputValues[3] = 100; //default delay between measurements in ms, 1 to 65535ms 
                InputValues[4] = 1;   //default is LED indicator ON when triggered, 0 = Off, 1 = On

            }   
            else 
            {

				String pch = "";
				i = 0;
				do {

					pch = getValue(hexstring,' ',i+1); //skip command char 
					InputValues[i] = strtoul(pch.c_str(), NULL, 10);
					i++;

				} while(pch != "");
				
                Serial.println(F(" The values you entered were - "));

            }
 
        	if ( InputValues[0] == 0 )
        	{
            	Serial.println(F("   Operation Mode = Trigger/Release for State Tracking "));                    	
            	param[0] = 0;  
        	}
        	else
        	{
            	Serial.println(F("   Operation Mode = Lower/Upper thresholds for Object Detection "));                    	
            	param[0] = 0xff;                  	
        	}

           	
            Serial.print(F("   Low side limit = "));                    
            Serial.print( InputValues[1] );  
            Serial.println(F(" mm"));
            param[1] = ((uint16_t)InputValues[1] & 0xff00) >> 8; //MSB
            param[2] = (uint8_t)InputValues[1]; //LSB

            Serial.print(F("   High side limit = "));                    
            Serial.print( InputValues[2] );  
            Serial.println(F(" mm"));
            param[3] = ((uint16_t)InputValues[2] & 0xff00) >> 8; //MSB
            param[4] = (uint8_t)InputValues[2]; //LSB

            Serial.print(F("   Delay between measurements = "));                    
            Serial.print( InputValues[3] );  
            Serial.println(F(" ms"));
            param[5] = ((uint16_t)InputValues[3] & 0xff00) >> 8; //MSB
            param[6] = (uint8_t)InputValues[3]; //LSB
           
            Serial.print(F("   LED indicator = "));

            if((uint8_t)InputValues[4] == 0)  //0 means OFF else ON
            {
                param[7] = 0;
                Serial.println(F(" OFF"));

            } else 
            {
                param[7] = 0xff;
                Serial.println(F(" ON"));
            }


            Serial.println();
            delay(100); 

			Serial.println(F("okay to write? [y/n] "));

			if(!confirm())
			{
				Serial.println(F(": command aborted.  "));				
                goto TerminateCommand2; 
			}
			
            Serial.println(F(": reboot tinyLiDAR.  "));
            tinyLiDAR("X",(rebootTime),I2C_Address);

            Serial.println(F(": tinyLiDAR set to SingleStep/UltraLowPower mode. "));
            tinyLiDAR("MS",(rebootTime),I2C_Address);

            Serial.println(F(": Now writing parameters to tinyLiDAR. "));

			if( !I2c_writeArray(I2C_Address,0x42, param, 8) )  //total of 8 bytes           	
            {  

               WriteSetupDelay; 

            } //if mc
            else
            {
                if (!command.equals("")) 
                {

                    Serial.println(F("Invalid argument.")); 

                } // if

            } //if else 

            delay(rebootTime);
            //now reset Arduino to ensure I2C works
            asm volatile("jmp 0");  //restart Arduino  

TerminateCommand2:
            i = 0; //nop

        }

        else if (command.startsWith("A"))
        {

            uint8_t i = 0;
            Serial.println(F("\n Command 'A': Autonomous Mode."));

            String hexstring = command.substring(0);

            if (hexstring == "A")  
            {

                Serial.println(F("\n\r> Nothing entered - using default values: "));
                InputValues[0] = 10;  //default Low side limit in mm
                InputValues[1] = 100; //default High side limit in mm
                InputValues[2] = 20;  //default Repetition interval in 0.1sec incr
                InputValues[3] = 5;   //default PW is 0 for 1ms else in 0.1sec incr 
                InputValues[4] = 1;   //default is LED indicator ON
                // i = 5;

            }   
            else
            {

				String pch = "";
				i = 0;
				do {

					pch = getValue(hexstring,' ',i+1); //skip command char 
					InputValues[i] = strtoul(pch.c_str(), NULL, 10);
					i++;

				} while(pch != "");

                Serial.println(F(" The values you entered were - "));

            }
        
            Serial.print(F("   Low side limit = "));                    
            Serial.print( InputValues[0] );  
            Serial.println(F(" mm"));

            param[0] = ((uint16_t)InputValues[0] & 0xff00) >> 8; //MSB
            param[1] = (uint8_t)InputValues[0]; //LSB

            Serial.print(F("   High side limit = "));                    
            Serial.print( InputValues[1] );  
            Serial.println(F(" mm"));

            param[2] = ((uint16_t)InputValues[1] & 0xff00) >> 8; //MSB
            param[3] = (uint8_t)InputValues[1]; //LSB

            Serial.print(F("   Repetition interval = "));                    
            Serial.print( InputValues[2]   );    
            Serial.print(F(" for "));
            Serial.print( InputValues[2] *100 );  
            Serial.println(F(" ms"));

            param[4] = ((uint16_t)InputValues[2] & 0xff00) >> 8; //MSB
            param[5] = (uint8_t)InputValues[2]; //LSB


            Serial.print(F("   Pulse Width = "));
            Serial.print( InputValues[3]   );    
            Serial.print(F(" for "));
            Serial.print( InputValues[3] *100 );  
            Serial.println(F(" ms"));

            param[6] = ((uint16_t)InputValues[3] & 0xff00) >> 8; //MSB
            param[7] = (uint8_t)InputValues[3]; //LSB
       
            Serial.print(F("   LED indicator = "));

            if((uint8_t)InputValues[4] == 0x01)  //only use 1 to turn it ON
            {
                param[8] = 0xff;
                Serial.println(F(" ON"));

            } else 
            {
                param[8] = 0;
                Serial.println(F(" OFF"));
            }


            Serial.println();
            delay(100); 

			Serial.println(F("okay to write? [y/n] "));

			if(!confirm())
			{
				Serial.println(F(": command aborted.  "));				
                goto TerminateCommand; 
			}

            Serial.println(F(": reboot tinyLiDAR.  "));
            tinyLiDAR("X",(rebootTime),I2C_Address);
            
            Serial.println(F(": tinyLiDAR set to SingleStep/UltraLowPower mode. "));
            tinyLiDAR("MS",(rebootTime),I2C_Address);

            Serial.println(F(": Now writing parameters to tinyLiDAR. "));

			if( !I2c_writeArray(I2C_Address,0x41, param, 9) )             	
            {  

               WriteSetupDelay; 

            } //if mc
            else
            {
                if (!command.equals("")) 
                {

                    Serial.println(F("Invalid argument."));

                } // if

            } //if else 


            delay(rebootTime);
	        //now reset Arduino to ensure I2C works
	        asm volatile("jmp 0");  //restart Arduino  

TerminateCommand:
			i = 0; //nop

        }
        else if (command.startsWith("W"))
        {

			uint8_t i = 0;
            String hexstring = command.substring(0);
	        Serial.println(F("\n Command 'W': change VL53L0 parameters."));            

            if (hexstring == "W") 
            {

                Serial.println(F("\n\r> Nothing entered - using default values: "));
                //set to default if no value is entered
				InputValues[0]  = SignalRateLimit; //note this is sent to tinyLiDAR as 100x actual req value
				InputValues[1]  = SigmalLimit; 
				InputValues[2]  = TimingBudget; 
				InputValues[3]  = VCELperiod; 

            }
            else 
            {

				String pch = "";
				i = 0;
				do {

					pch = getValue(hexstring,' ',i+1); //skip command char 
					InputValues[i] = strtoul(pch.c_str(), NULL, 10);
					i++;

				} while(pch != "");
				
                Serial.println(F(" The values you entered were - "));

			}

            Serial.print(F("   SignalLimit = "));
            Serial.print(  (float)InputValues[0]/100, 2 ); //show to 2 dec places
            Serial.println(F(" Mcps"));

            param[0] = ((uint16_t)InputValues[0] & 0xff00) >> 8; //MSB
            param[1] = (uint8_t)InputValues[0]; //LSB

            Serial.print(F("   SigmalLimit = "));
            Serial.print(InputValues[1]);
            Serial.println(F(" mm"));

            param[2] = (uint8_t)InputValues[1];                             

            Serial.print(F("   timeBudget = "));
            Serial.print( InputValues[2] );
            Serial.println(F(" ms"));

            param[3] = ((uint16_t)InputValues[2] & 0xff00) >> 8; //MSB
            param[4] = (uint8_t)InputValues[2]; //LSB

            if (InputValues[3] == 14) 
            {                                

                Serial.println(F("   preRangeVcselPeriod = 14 "));
                Serial.println(F("   finalRangeVcselPeriod = 10 "));

            } else if (InputValues[3] == 18) 
            {                                

                Serial.println(F("   preRangeVcselPeriod = 18 "));
                Serial.println(F("   finalRangeVcselPeriod = 14 "));

            } //if elseif

            param[5] = (uint8_t)InputValues[3]; 


            Serial.println();
            delay(100); 

			Serial.println(F("okay to write? [y/n] "));

			if(!confirm())
			{
				Serial.println(F(": command aborted.  "));				
                goto TerminateCommand3; 
			}

            Serial.println(F(": reboot tinyLiDAR.  "));
            tinyLiDAR("X",rebootTime,I2C_Address);

            Serial.println(F(": tinyLiDAR set to SingleStep/UltraLowPower mode. "));
            tinyLiDAR("MS",rebootTime,I2C_Address);

            Serial.println(F(": Now writing parameters to tinyLiDAR. "));

			if( !I2c_writeArray(I2C_Address,0x57, param, 6) )            	
            {  

               WriteSetupDelay; 

            } //if 
            else
            {
                if (!command.equals("")) 
                {

                    Serial.println(F("Invalid argument."));

                } // if

            } //if else 
			delay(rebootTime); //allow a bit of time for the above command 

TerminateCommand3:
            Serial.println(F("> Done. "));        

        }

        if (command != "") 
        {

            printMenu();
            command = "";

        }// if

    } //if elseif 

}//loop


// ---- functions ----

void printMenu() 
{

   Serial.println(F("\n Please enter a command [XX = hex I2C addr]: \n\
   d - read distance                   ::   dc - continuous read (from terminal) \n\
   q - query settings                  ::   w - write custom VL53L0X config \n\
   mc - continuous mode                ::   mr - real time mode  \n\
   ms - single step/ULP mode           ::   reset - reset to factory defaults \n\
   pl - long range preset              ::   ps - high speed preset \n\
   ph - high accuracy preset           ::   pt - tinyLiDAR preset \n\
   e - disable LED indicator           ::   f - enable LED indicator \n\
   g - LED on                          ::   t0/t1 - Disable/Enable WatchDog Timer \n\
   cd - cal offset distance            ::   cx - cal crosstalk \n\
   x - reboot tinyLiDAR                ::   y - save LED mode and reboot tinyLiDAR \n\
   arXX - auto I2C addr config loop    ::   v - scan & verify I2C addr loop \n\
   rXX - change tinyLiDAR's I2C addr   ::   iXX - change Terminal's I2C addr \n\
   a - start autonomous mode           ::   b - start autonomous IoT mode \n\
   az - end autonomous mode \n\
   u - start ultrasonic emulation mode ::   uz - end ultrasonic emulation mode \n\
    \
   "));

} //printMenu


void Write_I2C_Long16(uint8_t address, uint8_t command ,uint8_t data0, uint16_t dataByte16) 
{

    uint8_t dataByteH, dataByteL;

    dataByteH = (uint8_t)(dataByte16>>8 & 0x00ff);
    dataByteL = (uint8_t)(dataByte16 & 0x00ff);

    uint8_t param[3] = {data0,dataByteH,dataByteL};

    if( !I2c_writeArray(address,command, param, 3) )
    {  

        WriteSetupDelay;  
        I2c_read(address,1); 

    } //if 
  
} //Write_I2C_Long16()


void Qcommand(uint8_t I2C_Address)
{

    uint8_t i;
    uint8_t j = 0;
    long ilong;
    uint16_t k;
    uint8_t inputStream[30]; 

    Serial.println(F(": Query tinyLiDAR Configuration Parameters "));

    if( !I2c_write(I2C_Address,0x51) )
    {     

        k = I2c_read(I2C_Address,QresponseSize);   //request full set of data

        while ( I2c_available() ) 
        { 

            inputStream[j++] = I2c_receive();      // receive a byte

        } // while

    }
    else
    {
        
      Serial.println(F(" Device not ready. ")); 
      return; //exit

    } // if else 

    Serial.print(F("\n  >> Current Operation Mode is: "));

    if (inputStream[0] == 0x43) 
    {

        Serial.println(F("Continuous"));

    } 
     else if (inputStream[0] == 0x4c) 
    {

        Serial.println(F("SingleStep/UltraLowPower"));

    } 
     else if (inputStream[0] == 0x52) 
    {

        Serial.println(F("RealTime"));

    } // if elseif 


    if ( (inputStream[14] & 0x01) == 0) 
    {

        Serial.println(F("  >> WatchDog Timer: OFF"));

    } else 
    {

        Serial.println(F("  >> WatchDog Timer: ON"));

    } // if else

    i = (inputStream[14] & 0x06) >> 1; //take the 2 bits for LED Indicator

    if ( i == 0) 
    {

        Serial.println(F("  >> LED Indicator: OFF"));

    } 
    else if (i == 1)
    {

        Serial.println(F("  >> LED Indicator: ON"));

    } 
    else if ( i == 2)
    {

        Serial.println(F("  >> LED Indicator: Measurement"));

    } // if elseif                

    Serial.print(F("  >> Current Preset Configuration is: "));

    switch (inputStream[1]) 
    {

        case 0x53:
            Serial.println(F("High Speed"));
            break;
        case 0x52:
            Serial.println(F("Long Range"));
            break;
        case 0x41:
            Serial.println(F("High Accuracy"));
            break;
        case 0x43:
            Serial.println(F("Custom"));
            break;
        case 0x54:
            Serial.println(F("tinyLiDAR"));
            break;
        default:
            Serial.println(F("** UNKNOWN **"));
            break;

    } // switch

    Serial.print(F("  >> Signal Rate Limit = "));
    k = (uint16_t)((uint8_t)(inputStream[2]) << 8 | (uint8_t) inputStream[3]);
  
    Serial.print((float) k / 65536.0 + .005, 2);
    Serial.println(F(" MCPS"));

    Serial.print(F("  >> Sigma Estimate Limit = "));
    Serial.print((inputStream[4]), DEC);
    Serial.println(F(" mm"));

    Serial.print(F("  >> Timing Budget = "));
    k = (uint16_t)((uint8_t)(inputStream[5]) << 8 | (uint8_t) inputStream[6]);
    Serial.print(k, DEC);
    Serial.println(F(" ms"));

    if (inputStream[7] == 14) 
    {

        Serial.println(F("  >> Pre Range VCSEL Period = 14 "));
        Serial.println(F("  >> Final Range VCSEL Period = 10 "));

    } else {

        Serial.println(F("  >> Pre Range VCSEL Period = 18 "));
        Serial.println(F("  >> Final Range VCSEL Period = 14 "));

    } // if else

    Serial.print(F("  >> tinyLiDAR Firmware Version = "));
    Serial.print(inputStream[8], DEC); // print the character
    Serial.print(F("."));
    Serial.print(inputStream[9], DEC); // print the character
    Serial.print(F("."));
    Serial.println(inputStream[10], DEC); // print the character
    //Serial.print(F("\n\r"));
    Serial.print(F("  >> ST PAL API Version = "));
    Serial.print(inputStream[11], DEC); // print the character
    Serial.print(F("."));
    Serial.print(inputStream[12], DEC); // print the character
    Serial.print(F("."));
    Serial.println(inputStream[13], DEC); // print the character
    //Serial.print(F("\n\r"));

    if(inputStream[14] & 0x08)
    {  //true means custom

        Serial.println(F("  >> Offset Cal: Custom"));

    } else 
    {

        Serial.println(F("  >> Offset Cal: Default"));

    }

    Serial.print(F("  >> Cal Offset = "));
    ilong = (uint32_t)( (uint32_t)inputStream[15]<<24 | (uint32_t)inputStream[16]<<16 | (uint32_t)inputStream[17]<<8 | (uint32_t)inputStream[18]);
    Serial.print( (float)(ilong/1000.0) , 1);  // for offset only 
    Serial.println(F("mm"));
    Serial.print(F("  >> Xtalk = "));
    ilong = (uint32_t)( (uint32_t)inputStream[19]<<24 | (uint32_t)inputStream[20]<<16 | (uint32_t)inputStream[21]<<8 | (uint32_t)inputStream[22]);
    Serial.print( (float)(ilong/65536.0) , 3);  
    Serial.println(F("MCPS "));
        
} //Qcommand()


void Dcommand(byte j, byte delayBetween, uint8_t targetAddr)  // j num of bytes, up to 255ms delay between and from targetAddr
{

	int Measured_Distance = 0;
    uint16_t k = 10; //max number of delay iterations before giving up 

	if (RTmode)
	{ //is Real Time mode

            //start the measurement
            I2c_write(targetAddr,'D'); 
    
            delay(delayBetween);  //wait for measurement to finish
           
            do {

                delay(deltaDelay);  //incremental waits
                Measured_Distance = Read_Distance(targetAddr); 
                k--;

            }while( k && (  Measured_Distance == 1 ) );            

            if (!k){

                Serial.println(F(" Error talking to device "));
                return; 
            }

            if (Measured_Distance > 9){
             
                Serial.print(F("     Distance = "));
                Serial.print(Measured_Distance, DEC);     // print the Measured_Distance 
                Serial.println(F("mm"));

            } 
            else if (Measured_Distance == 1)
            {

                Serial.println(F(" VL53L0 Status Code: Sigma Fail "));
            
            }
            else if (Measured_Distance == 2)
            {

                Serial.println(F(" VL53L0 Status Code: Signal Fail "));
            
            }
            else if (Measured_Distance == 3)
            {

                Serial.println(F(" VL53L0 Status Code: Min Range Fail "));
            
            }
            else if (Measured_Distance == 4)
            {

                Serial.println(F(" VL53L0 Status Code: Phase Fail [Out of Range] "));
            
            }
            else 
            {

                Serial.println(F(" - "));  //invalid distance data will show as '-'
            
            } //if else



    } 
    else
    { //is for normal reads (not Real Time)

        for (byte k =0;k<j;k++)
        {
            Measured_Distance = Read_Distance(targetAddr); 

            if (Measured_Distance > 9){
             
                Serial.print(F("     Distance = "));
                Serial.print(Measured_Distance, DEC);     // print the Measured_Distance 
                Serial.println(F("mm"));

            } 
            else if (Measured_Distance == 1)
            {

                Serial.println(F(" VL53L0 Status Code: Sigma Fail "));
            
            }
            else if (Measured_Distance == 2)
            {

                Serial.println(F(" VL53L0 Status Code: Signal Fail "));
            
            }
            else if (Measured_Distance == 3)
            {

                Serial.println(F(" VL53L0 Status Code: Min Range Fail "));
            
            }
            else if (Measured_Distance == 4)
            {

                Serial.println(F(" VL53L0 Status Code: Phase Fail [Out of Range] "));
            
            }
            else 
            {

                Serial.println(F(" - "));  //invalid distance data will show as '-'
            
            } //if else

            delay(delayBetween); 

        } // for i  

    } // if else 
  
} //Dcommand()


uint16_t Read_Distance(uint8_t targetAddr) 
{

    uint16_t i;
 

    if( !I2c_write(targetAddr,'D') ) 
    {     

        i = I2c_read(targetAddr,2); //request 2 bytes for distance

        while ( I2c_available() ) 
        { 

            i = I2c_receive();        // receive 1st byte
            i = i<<8 | I2c_receive(); // receive 2nd byte

        } 

    }
    else
    {

        Serial.print(F(" Device not ready ")); 
        return 0; //exit

    } //if else
    
    return i;

} //Read_Distance() 
 
 
//
// This function will continually read the distance until the ENTER key is hit
// Note this is a terminal based function which means the Arduino is sending the read 
// command in a continuous loop. 
// 
void Cont_Read_Distance(uint8_t targetAddr) 
{

    RTmode = false; //to select the proper D command later

    //set to the continuous mode so we can read without delays
    Serial.println(F(": tinyLiDAR set to Continuous mode. "));
    tinyLiDAR("MC",rebootTime,I2C_Address);

    do
    {  

        inByte = 0;
        while(1)
        {

            Dcommand(1, 0, targetAddr); //read distance

            if (Serial.available() > 0) 
            {

                inByte = (char)Serial.read(); // store the char
                if (inByte == '\r')
                   goto LoopExit;

            } // if

        } // while 

LoopExit:
        Serial.println(F("\n Okay, now exiting continuous read loop. "));

    }while (inByte != '\r'); // only the ENTER key will exit loop

    //set back to single step mode
    Serial.println(F(": tinyLiDAR set to SingleStep/UltraLowPower mode. "));
    tinyLiDAR("MS",rebootTime,targetAddr);
 
} //Cont_Read_Distance()


// generic I2C Write function 
uint8_t I2c_write(uint8_t address, int data1, int data2)  
{

	uint8_t i = 0;

	#if (defined ArduinoLEONARDO || defined ArduinoUNO)
	 	 
	    if (data2 < 0)
		 	i = I2c.write(address,(uint8_t)data1);  
		else
			i = I2c.write(address, (uint8_t)data1, (uint8_t)data2);  
		 

	#elif defined ArduinoMEGA

		byte dataArray[2];

		Wire.beginTransmission(address);  
	    
		    if (data2 > -1) // send 2 bytes if there is a second argument
		    {
				dataArray[0] = (uint8_t)data1;
				dataArray[1] = (uint8_t)data2;
				i = Wire.write(dataArray, 2);
		    }
		    else 
		    {
			    i = Wire.write(data1);	
		    }

	    Wire.endTransmission();

	#endif


	return i;  //0 means success else error

} //I2c_write


// generic I2C request data for read function
uint8_t I2c_read(uint8_t address, uint8_t numBytes)
{

	uint8_t i = 0;

	#if (defined ArduinoLEONARDO || defined ArduinoUNO)
	 	
		i = I2c.read(address,numBytes);  

	#elif defined ArduinoMEGA

	 Wire.requestFrom(address, numBytes);
 
	#endif

	return i;

} //I2c_read


//generic I2C function for receive
uint8_t I2c_receive(void)
{

	uint8_t i = 0;

	#if (defined ArduinoLEONARDO || defined ArduinoUNO)

        i = I2c.receive();  

	#elif defined ArduinoMEGA

	i = Wire.read();

	#endif

	return i;

} //I2c_receive

//generic function for I2C available
uint8_t I2c_available(void)
{
	uint8_t i = 0;

	#if (defined ArduinoLEONARDO || defined ArduinoUNO)

        i = I2c.available();

	#elif defined ArduinoMEGA

    	i = Wire.available();
 
	#endif

	
	return i;
}  

//generic I2C write with array 
uint8_t I2c_writeArray(uint8_t address, uint8_t command, uint8_t paramArray[], uint8_t size)
{

	uint8_t i = 0;

	#if (defined ArduinoLEONARDO || defined ArduinoUNO)

		i = I2c.write(address,command, paramArray, size); 

	#elif defined ArduinoMEGA

		uint8_t newParamArray[11];  
		newParamArray[0] = command;

		for (uint8_t j = 1; j < (size+1); j++ )
		{
			newParamArray[j] =  paramArray[j-1];
		}

		Wire.beginTransmission(address);  
		i = Wire.write(newParamArray, size+1); 
	    Wire.endTransmission();

	#endif

 	return i;

}
 

void tinyLiDAR(String _command,uint16_t _delay,uint8_t _targetAddr)
{

    if (_command.length() == 2)
    {

         I2c_write(_targetAddr,_command.charAt(0), _command.charAt(1)); 

    }else{

        I2c_write(_targetAddr,_command.charAt(0) ); 

    } // if else

    delay(_delay); // delay time after command

}


String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;
	for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}


bool confirm()
{
	uint8_t inByte = 0;

	while(1)
	{
		    if (Serial.available() > 0) 
		    {

		        inByte = Serial.read();
		        // only accept input if a letter, number comma or space is typed
		        if ((inByte >= 65 && inByte <= 90) || (inByte >= 97 && inByte <= 122) || (inByte >= 48 && inByte <= 57) || inByte == 44 || inByte == 32) 
		        {

		            if (inByte == 121)
		            {
			    		return true;
		            } 
		            else if (inByte == 110)
		            {
			    		return false;
		            }

		        } // if

		    } // if
	 
	} //while

} //confirm()
 
// ---- end of code ----