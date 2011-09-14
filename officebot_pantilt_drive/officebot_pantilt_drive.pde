
// This code controls the Create
// Created by Zhengqin Fan
// Modified 2011-09-09 by Alaina Hardie
// Modified 2011-09-14 by Dan Barry

// Requires Roomba library, available at:
// http://www.open.com.au/mikem/arduino/Roomba/

#include <Roomba.h>
 
#define DRIVING_NONE 0
#define DRIVING_FORWARD 1
#define DRIVING_BACKWARD 2
#define DRIVING_TURNLEFT 3
#define DRIVING_TURNRIGHT 4

#define MAX_FORWARD_SPEED 500  // speeds are in mm/sec
#define MAX_BACKWARD_SPEED -500
#define MAX_TURN_SPEED 500
#define MIN_FORWARD_SPEED 50
#define MIN_BACKWARD_SPEED -50
#define MIN_TURN_SPEED 50
#define DELTA_FORWARD_SPEED 50
#define DELTA_TURN_SPEED 50
#define DEFAULT_FORWARD_SPEED 300
#define DEFAULT_BACKWARD_SPEED -300
#define DEFAULT_TURN_SPEED 200
#define RAMPUP_SPEED_DELAY 300  // controls how long between speed steps when ramping up from a stop to default speed

#define TIME_OUT 3000  // number of milliseconds to wait for command before you consider it a timeout and stop the base
#define TURN_OFF_CREATE 900000  // number of milliseconds of inactivity before powering down create to save battery power (15 min = 900 seconds = 900000 msec)

/*
typedef struct 
{
  uint8_t bumpsAndWheelDrops ; // packet id 7
  uint8_t wall; // packet id 8
  uint8_t cliffLeft; // packet id 9
  uint8_t cliffFrontLeft; // packet id 10
  uint8_t cliffFrontRight; // packet id 11
  uint8_t cliffRight; // packet id 12
  uint8_t virtualWall; // packet id 13
  uint8_t lsdWheelCurrents; // packet id 14
  uint8_t unused1; // packet id 15
  uint8_t unused2; // packet id 16
  uint8_t irByte; // packet id 17
  uint8_t buttons; // packet id 18
  uint16_t distance; // packet id 19
  uint16_t angle; // packet id 20
  uint8_t chargingState; // packet id 21
  int16_t voltage; // packet id 22
  int16_t current; // packet id 23
  int8_t batteryTemp; // packet id 24
  uint16_t batteryCharge; // packet id 25
  uint16_t batteryCapacity; // packet id 26
} sensorData726;


sensorData726 sensors;

*/

Roomba myBase(&Serial2); // instance for the Create on Serial2 (pins 16 and 17)


const int powerPin = 4;  // this pin is connected to the Create's Device Detect (DD) pin and is used to toggle power
const int chargingIndicatorPin = 5;  // placeholder for monitoring if the create battery is being charged, not implemented yet
const int powerMonitor = 6; // high if the power from the power box is on, meaning that the Create is on and so is the power box's switch
const int powerIndicatorPin = 7;  // output to drive an LED that the user can see.

// drive inputs, put them on pins that can trigger interrupts-- NOTE THAT INTERRUPT NUMBERS ARE DIFFERENT THAN THE PIN NUMBERS!!
const int forwardCmd = 5;   // interrupt 5 is on pin 18
const int backwardCmd = 4;  // interrupt 4 is on pin 19
const int rightCmd = 3;    // interrupt 3 is on pin 20
const int leftCmd = 2;    // interrupt 2 is on pin 21

const int emergencyShutdownCmd = 0;  // interrupt 0 is on pin 2  shuts down the create in response to a button push


int driving = DRIVING_NONE; // true if it's in the driving state
unsigned long lastCmdMs; // The number of milliseconds since the program started that the last command was received
int baseFwdSpeed, baseTurnSpeed;

void powerOnCreate()
// power the Create on
{
        if (!digitalRead(powerMonitor))  // if the power is off, turn the Create on.
        {
            digitalWrite(powerPin, LOW);
            delay (10);
            digitalWrite(powerPin, HIGH);
            delay(500);
            digitalWrite(powerPin, LOW);
            
            while (!digitalRead(powerMonitor))
            {
               if (digitalRead(powerIndicatorPin)) pinMode(powerIndicatorPin, LOW);
               else pinMode(powerIndicatorPin, LOW);
               delay(500);  // wait for the create to power up
            }
            pinMode(powerIndicatorPin, LOW);
            delay(2000); // takes a couple seconds to be ready to drive
            myBase.start();
            myBase.safeMode();
            pinMode(powerIndicatorPin, HIGH);           
        }  
}

void powerOffCreate()  // power the Create off
{
  myBase.drive(0, myBase.DriveStraight);  // stop moving
  if (digitalRead(powerMonitor))  // if the power is on, turn the Create off.
        {
            digitalWrite(powerPin, LOW);
            delay (10);
            digitalWrite(powerPin, HIGH);
            delay(500);
            digitalWrite(powerPin, LOW);
        } 
        delay(500);
        if (!digitalRead(powerMonitor)) pinMode(powerIndicatorPin, LOW); 
        lastCmdMs = millis();  // remember when the command was done
  driving = DRIVING_NONE;
  baseTurnSpeed = 0;
  baseFwdSpeed = 0;
  lastCmdMs = millis();
}

void emergencyShutdown()
{
  noInterrupts();
  powerOffCreate();
}

void moveForward()
{
  powerOnCreate();  // if the create is not powered on, turn it on.
  if (baseFwdSpeed == 0) // if starting from a stop, begin with default speed
  {
    while (baseFwdSpeed < DEFAULT_FORWARD_SPEED)  // ramp up to speed from a stop
    {
      baseFwdSpeed += DELTA_FORWARD_SPEED;
      myBase.drive(baseFwdSpeed, myBase.DriveStraight);
      delay(RAMPUP_SPEED_DELAY);
    }
    driving = DRIVING_FORWARD;
  }
  else
  {  
    if (driving == DRIVING_BACKWARD && baseFwdSpeed + DELTA_FORWARD_SPEED > MIN_BACKWARD_SPEED) stop();  // if we are slowing down and get real slow, stop.
    else  // we are either moving forward or we are moving backward with sufficient speed to be OK to slow down without stopping
    {
      if (baseFwdSpeed <= MAX_FORWARD_SPEED - DELTA_FORWARD_SPEED) baseFwdSpeed += DELTA_FORWARD_SPEED;  // moving foward or slowing down backward means adding DELTA_FORWARD_SPEED
      myBase.drive(baseFwdSpeed, myBase.DriveStraight);
      if (baseFwdSpeed > 0) driving = DRIVING_FORWARD;
      else driving = DRIVING_BACKWARD;
    }
  }
  baseTurnSpeed = 0;
  lastCmdMs = millis();  // remember when the command was done
}

void moveBackward()
{
  powerOnCreate();
  if (baseFwdSpeed == 0) // if starting from a stop, begin with default speed
  {
    while (baseFwdSpeed > DEFAULT_BACKWARD_SPEED)  // ramp up to speed from a stop
    {
      baseFwdSpeed -= DELTA_FORWARD_SPEED;
      myBase.drive(baseFwdSpeed, myBase.DriveStraight);
      delay(RAMPUP_SPEED_DELAY);
    }
    driving = DRIVING_BACKWARD;
  }
  else 
  {
    if (driving == DRIVING_FORWARD && baseFwdSpeed - DELTA_FORWARD_SPEED < MIN_FORWARD_SPEED) stop();  // if we are slowing down and get real slow, stop.
    else
    {
      if (baseFwdSpeed >= MAX_BACKWARD_SPEED + DELTA_FORWARD_SPEED) baseFwdSpeed -= DELTA_FORWARD_SPEED;
      myBase.drive(baseFwdSpeed, myBase.DriveStraight);
      if (baseFwdSpeed < 0) driving = DRIVING_BACKWARD;
      else driving = DRIVING_FORWARD;
    }
  }
  baseTurnSpeed = 0;
  lastCmdMs = millis();  // remember when the command was done
}

void turnRight()
{
  powerOnCreate();
  if (baseTurnSpeed == 0) // if starting from a stop, begin with default speed
  {
    while (baseTurnSpeed < DEFAULT_TURN_SPEED)  // ramp up to speed from a stop
    {
      baseFwdSpeed += DELTA_TURN_SPEED;
      myBase.drive(baseTurnSpeed, myBase.DriveInPlaceCounterClockwise); // this is wrong, for some reason; left should be counter and right should be clockwise
      delay(RAMPUP_SPEED_DELAY);
    }
    driving = DRIVING_TURNRIGHT;  
  }
  else
  {
    if (driving == DRIVING_TURNLEFT && baseTurnSpeed - DELTA_TURN_SPEED < MIN_TURN_SPEED) stop();  // if we are slowing down and get real slow, stop
    else
    {
      if (driving == DRIVING_TURNLEFT)
      {
        baseTurnSpeed -= DELTA_TURN_SPEED;  // got a turn right command while turning left, means slow down the turn, but still turn left
        myBase.drive(baseTurnSpeed, myBase.DriveInPlaceClockwise); // this is wrong, for some reason; left should be counter and right should be clockwise
      }
      else
      {
        if (baseTurnSpeed <= MAX_TURN_SPEED - DELTA_TURN_SPEED) baseTurnSpeed += DELTA_TURN_SPEED;  // got a turn right command while turning right, means speed up the turn
        myBase.drive(baseTurnSpeed, myBase.DriveInPlaceCounterClockwise); // this is wrong, for some reason; left should be counter and right should be clockwise
      }
    }
  } 
  baseFwdSpeed = 0;
  lastCmdMs = millis();  // remember when the command was done
}

void turnLeft()
{
  powerOnCreate();
  if (baseTurnSpeed == 0) // if starting from a stop, begin with default speed
  {
    while (baseTurnSpeed < DEFAULT_TURN_SPEED)  // ramp up to speed from a stop
    {
      baseFwdSpeed += DELTA_TURN_SPEED;
      myBase.drive(baseTurnSpeed, myBase.DriveInPlaceClockwise); // this is wrong, for some reason; left should be counter and right should be clockwise
      delay(RAMPUP_SPEED_DELAY);
    }
    driving = DRIVING_TURNLEFT;  
  }
  else
  {
    if (driving == DRIVING_TURNRIGHT && baseTurnSpeed - DELTA_TURN_SPEED < MIN_TURN_SPEED) stop();  // if we are slowing down and get real slow, stop
    else
    {
      if (driving == DRIVING_TURNRIGHT)
      {
        baseTurnSpeed -= DELTA_TURN_SPEED;  // got a turn left command while turning right, means slow down the turn, but still turn right
        myBase.drive(baseTurnSpeed, myBase.DriveInPlaceCounterClockwise); // this is wrong, for some reason; left should be counter and right should be clockwise
      }
      else
      {
        if (baseTurnSpeed <= MAX_TURN_SPEED - DELTA_TURN_SPEED) baseTurnSpeed += DELTA_TURN_SPEED;  // got a turn left command while turning left, means speed up the turn
        myBase.drive(baseTurnSpeed, myBase.DriveInPlaceClockwise); // this is wrong, for some reason; left should be counter and right should be clockwise
      }
    }
  } 
  baseFwdSpeed = 0;
  lastCmdMs = millis();  // remember when the command was done
}


void stop()
{
  powerOnCreate();
  myBase.drive(0, myBase.DriveStraight);
  driving = DRIVING_NONE;
  baseTurnSpeed = 0;
  baseFwdSpeed = 0;
  lastCmdMs = millis();
}
  

void setup() 
{ 
  pinMode(powerPin, OUTPUT); // toggle DD to toggle power on the Create
  pinMode(powerMonitor, INPUT); // high if power from Create via power board (and power board's switch) is on
  pinMode(powerIndicatorPin, OUTPUT); // output to drive an LED that the user can see.
  
  //Attach an interrupt to the input pins and monitor for low to high transitions
  attachInterrupt(forwardCmd, moveForward, RISING);
  attachInterrupt(backwardCmd, moveBackward, RISING);
  attachInterrupt(rightCmd, turnRight, RISING);
  attachInterrupt(leftCmd, turnLeft, RISING);
  attachInterrupt(emergencyShutdownCmd, emergencyShutdown, CHANGE);
  
  // start serial port at 57600 bps for the create
  Serial2.begin(57600); 
  stop();  // this will power up the create and initialize the state
}

void loop() 
{   
  if ((millis() > lastCmdMs + TIME_OUT) && (driving != DRIVING_NONE) ) stop();
  if (millis() > lastCmdMs + TURN_OFF_CREATE) powerOffCreate();
  delay(10);
}
 
