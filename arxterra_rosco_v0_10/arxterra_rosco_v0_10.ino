


/***********************************
 * Reminder: Serial Monitor Baud Rate set to 57600
 * to do list
 * pingers code to stop rover included but not tested/debugged
 * code for pan/tilt offset angle
 * continue to clean up code
 *
 * Servo Note:
 * the min pulse width (default is 544 microseconds) corresponds to the minimum angle (0-degree), shaft rotates fully left. 
 * the max pulse width, (defaults to 2400 microseconds) corresponds to the maximum angle (180-degree), shaft rotates fully right.  
 * rotation is left-to-right clockwise
 ***********************************/

/* The following two libraries are currently not 
 * implemented on the Pathfinder Rover
 * #include <Wire.h>      // I2C support
 * #include <L3G4200D.h>  // 3-axis Gyro
 */
/* The following two libraries are included with the Arduino IDE  */  
#include <SPI.h>


/* The Adb library has the following modifications made to bring
 * it into compliance Arduino version 1.04 (Compiles without errors)
 *  1. Adb.h, usb.cpp, and max3421e.cpp
 *  2. Change the line: #include "wiring.h" to #include "Arduino.h"
 * Runing on ADK generates a "OSCOKIRQ failed to assert" error.
 * Problem and possible solution provided here.
 *  http://forum.arduino.cc/index.php?topic=68205.0
 */
 

#include <AFMotor.h>

#define FALSE 0
#define TRUE  1

// Mapping pins 
// ATmega2560-Arduino http://arduino.cc/en/Hacking/PinMapping2560
// ATmega328-Arduino  http://arduino.cc/en/Hacking/PinMapping168
#include "pinouts_pathfinder.h"

// Commands to Numeric Value Mapping
//               Data[0] =   CMD TYPE | Qual
//                        bit  7654321   0      
#define MOVE         0x01   // 0000000   1        
#define CAMERA_MOVE  0x02   // 0000001   0                  
#define CAMERA_HOME  0x04   // 0000010   0

/* future
 * #define ATMEGA_TEMP    8
 * ATmega internal temperature sensor ... Need to research how to read
 *   http://forum.arduino.cc/index.php/topic,38043.0.html
 *   http://forum.arduino.cc/index.php/topic,26299.0.html
 *   Problem is time required when you switch between voltage reference sources
 *   for ADC to stabilize. This would limit how often we check this internal sensor.
 * cell phone temperature...     ArxRover needs to add
 */

// Adb connection.
//Connection * connection;   // the connection variable holds a pointer (i.e. an address) to a variable of data type Connection

/* Saved in UserState File on Android Phone
 * public var cameraAdjustForMotion:Boolean = false;
 * public var cameraCanPan:Boolean = true;
 * public var cameraCanTilt:Boolean = true;
 * public var cameraConfigDefault:CameraConfig;
 * public var cameraConfigMotion:CameraConfig;
 * public var cameraIndex:int;
 * public var roverName:String;
 */
 
/* Defaults set in the Control Panel's Control Options pop-up Window 
 * Duty Cycle Steps      6 (4 -12)
 * Polling msec          500 (300 - 800)
 * Minimum Duty Cycle    100 (10 - 140)   *** ArxRover *** 
 * Top Duty Cycle        212 (180 - 255)
 * Motion State Display  ON  (Motion Text Display)
 * Control Tips Display  ON  (Tool Tips)  
 * Range Sensor Display  ON  (Numeric and Graphical Icon of Ultrasonic Data)
 */

// Timers and Sensor Variables and Default Values
unsigned long timer1;
unsigned long timer2;

AF_DCMotor motorL(1, MOTOR12_64KHZ);
AF_DCMotor motorR(2, MOTOR12_64KHZ);

boolean collisionDetection = FALSE;

void move(uint8_t * motordata) {
  motorL.setSpeed(motordata[2]);   
  motorR.setSpeed(motordata[4]);
  motorL.run(motordata[1]);
  motorR.run(motordata[3]);
}

// Event handler for the shell connection. 
void commandHandler()    // declared void in version 1.00
{
  byte data[5];
  int i = 0;
  
  // we want to leave the loop if the command
  // is CAMERA_HOME because the length is 1. Otherwise
  // the command array is of length 5
  boolean cameraHome = false;
  
  if( Serial.peek() == CAMERA_HOME ) cameraHome = true;
  while(!cameraHome && Serial.available() < 5) {}
  
  while(Serial.available()) {
    data[i] = Serial.read();
    i++;
    if( cameraHome ) break;
  }
   
    byte cmd = data[0];
    
    if (cmd == MOVE) {
      /***********************************
      * motion command
      * motordata[1]    left run    (FORWARD, BACKWARD, BRAKE, RELEASE)
      * motordata[2]    left speed  0 - 255
      * motordata[3];   right run   (FORWARD, BACKWARD, BRAKE, RELEASE) 
      * motordata[4];   right speed 0 - 255
      ***********************************/
      move(data);
      //flashled();
      //collisionDetection = (data[1] == 1) || (data[2] == 1);  // set to true if any motor is moving forward
    }
    else if (cmd == CAMERA_MOVE){
      /***********************************
       * pan and tilt command 
       * data[1]    0
       * data[2]    pan degrees (0 to 180)
       * data[3]    0
       * data[4]    tilt degrees (0 to 180)
       ***********************************/
      move_camera(data[2],data[4]);
      //Serial.print("Pan Servo To Position: ");
      //Serial.print(data[2]);
      //Serial.print(", ")
      //Serial.println(data[4]);
    }
    else if (cmd == CAMERA_HOME){
      /***********************************
       * camera home command 
       * pan position = 90 (default), tilt position = 90 (default)
       ***********************************/ 
      home_camera();
      // Serial.println("Camera Home");
   }
 
}
void flashled() {
  digitalWrite(13, HIGH);
  delay(250);
  digitalWrite(13, LOW); 
}


void setup()
{
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  //Serial.print("\r\nStart\n");
  
  init_servos();
  
 // ADB::init();     // source of OSCOKIRQ error

  // Open an ADB stream to the phone's shell. Auto-reconnect
  //connection = ADB::addConnection("tcp:4567", true, adbEventHandler); 
  timer1 = millis();
  timer2 = millis();
  

}

void loop()
{ 
  // Serial.println("I am alive");  
  sendData();
 
  if( Serial.available() ) {
    commandHandler();
  }
}

int cleanBat = 1024;   //************************* REMOVE ************************

void sendData(){
   if(millis() - timer2 > 250) {            // Has it been over 300ms since last send? **** convert to interrupt? ****
    int motor1 = analogRead(MOTOR1CURRENT);
    int motor2 = analogRead(MOTOR2CURRENT);
    int tempSensor = analogRead(TEMPERATURE);
    cleanBat -= 100;
    if (cleanBat < 0) cleanBat = 1024;
    // Serial.println(cleanBat);
    //int cleanBat = analogRead(CLEAN_BAT);
    int dirtyBat = analogRead(DIRTY_BAT);
    /*
    float cleanBat_tp = cleanBat*5.0*2.4/1023.0;  // test point
    float dirtyBat_tp = dirtyBat*5.0*1.6/1023.0;
    Serial.print("Clean Voltage= ");
    Serial.println(cleanBat_tp);
    Serial.print("Dirty Voltage= ");
    Serial.println(dirtyBat_tp);
    */
    byte sendData[3];
    /*
    sendData[0]= 0x1;              // motor 1 current data package 
    sendData[1]= motor1 >> 8;
    sendData[2]= motor1 & 0xff;
    
    sendData[3]= 0x2;              // motor 2 current data package
    sendData[4]= motor2 >> 8;      
    sendData[5]= motor2 & 0xff;
    
    sendData[6]= 0x3;             // temperature sensor data package
    sendData[7]= tempSensor >> 8;
    sendData[8]= tempSensor & 0xff;
   
    uint16_t range;              // local variable for range data  
    sendData[9]= 0x4;            // distance sensor data package
    range = getRangeLeft();
    sendData[10]= highByte(range);   // C++ rangeLeft >> 8
    sendData[11]= lowByte(range);    // C++ rangeLeft & 0xff
    
    sendData[12]= 0x5;            // distance sensor data package
    range = getRangeRight();
    sendData[13]= highByte(range);   // C++ rangeLeft >> 8
    sendData[14]= lowByte(range);    // C++ rangeLeft & 0xff
    */
    sendData[0]= 0x6;            // clean battery data package
    sendData[1]= cleanBat >> 8;
    sendData[2]= cleanBat & 0xff;
    /*
    sendData[18]= 0x7;            // dirty battery data package
    sendData[19]= dirtyBat >> 8;
    sendData[20]= dirtyBat & 0xff;
    
    sendData[21]= 0x8;            // pan and tilt angle data package
    sendData[22]= getPanPosition();
    sendData[23]= getTiltPosition();
    */
    //connection->write(24, sendData);   // see adb header file
    Serial.write(sendData, 3);
    timer2 = millis(); //reset the timer
  }
  


}

