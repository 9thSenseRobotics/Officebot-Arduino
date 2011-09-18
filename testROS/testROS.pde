// rosRoomba receives ROS messages for movement commands, also monitors interrupt pins for movement commands
// publishes the roomba sensors to "robot_state"

// Requires Roomba, ROS library folders, and rosRoomba folder with header file, to be located in sketchbook/libraries

#include <ros.h>
#include <std_msgs/String.h>
#include <Roomba.h>
#include <rosRoomba/rosRoomba.h>


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

#define PUBLISH_RATE 100  // only publish once every PUBLISH_RATE times through the main loop

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

void skypeCallback( const std_msgs::String& msgSkype);

Roomba myBase(&Serial2); // instance for the Create on Serial2 (pins 16 and 17)

ros::NodeHandle  nh;

ros::Subscriber<std_msgs::String> subscriberSkype("SkypeChat", &skypeCallback );
  
std_msgs::String ReceivedCommands;
rosRoomba::rosRoomba RobotStateMsg;
ros::Publisher robot_state("robot_state",&RobotStateMsg);
ros::Publisher robot_commands("robot_commands", &ReceivedCommands);

const int powerPin = 4;  // this pin is connected to the Create's Device Detect (DD) pin and is used to toggle power
const int chargingIndicatorPin = 5;  // placeholder for monitoring if the create battery is being charged, not implemented yet
const int powerMonitor = 6; // high if the power from the power box is on, meaning that the Create is on and so is the power box's switch
const int powerIndicatorPin = 7;  // output to drive an LED that the user can see.

// drive inputs, put them on pins that can trigger interrupts-- NOTE THAT INTERRUPT NUMBERS ARE DIFFERENT THAN THE PIN NUMBERS!!
const int forwardCmd = 2;   // interrupt 2 is on pin 21
const int backwardCmd = 3;  // interrupt 3 is on pin 20
const int leftCmd = 4;   // interrupt 4 is on pin 19 
const int rightCmd = 5;  // interrupt 5 is on pin 18

const int emergencyShutdownCmd = 0;  // interrupt 0 is on pin 2  shuts down the create in response to a button push

int driving = DRIVING_NONE; // true if it's in the driving state
unsigned long lastCmdMs; // The number of milliseconds since the program started that the last command was received
int baseFwdSpeed, baseTurnSpeed, speedCmd;
int counter;  // inside the loop we will only publish once every PUBLISH_RATE cycles
int emergencyShutdownReceived = 0;

void skypeCallback( const std_msgs::String& msgSkype)
{
    ReceivedCommands = msgSkype;
    robot_commands.publish(&ReceivedCommands);
    if (strlen( (const char* ) msgSkype.data) > 2 ) return;  // invalid format, more than 2 characters
    char cmd = msgSkype.data[0];
    if (strlen( (const char* ) msgSkype.data) > 1 )
    {
      speedCmd = (int)msgSkype.data[1];
      if (speedCmd < 49  || speedCmd > 57) return;  // invalid format, not a number between 1 and 9
      speedCmd = (speedCmd - 48) * MIN_FORWARD_SPEED;
    }
    else speedCmd = 0;  // speed not specified, so we will just increase or decrease by DELTA
    
    switch(cmd)
    {
      case 's':    // stop
        stop();
        break;
      
      case 'f':  // move forward
        if (speedCmd == 0) moveForward();
        else moveCommandedSpeed();
        break;
        
      case 'b':  //move backward
        if (speedCmd == 0) moveBackward();
        else 
        {
          speedCmd *= -1;
          moveCommandedSpeed();
        }
        break;
        
      case 'r':  //turn right
        if (speedCmd == 0) turnRight();
        else turnCommandedSpeed();
        break;
        
      case 'l':  // turn left
        if (speedCmd == 0) turnLeft();
        else 
        {
          speedCmd *= -1;
          turnCommandedSpeed();
        }
        break;
      
      default:  // unknown command
        //stop();  w// we need to ignore, not stop because otherwise we will stop when people are just saying stuff like "hi"
        break;
    }

}


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
  speedCmd = 0;
  lastCmdMs = millis();
}

void emergencyShutdown()
{
  noInterrupts();
  emergencyShutdownReceived = 1;
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

void moveCommandedSpeed()
{
    powerOnCreate();
    if (speedCmd > baseFwdSpeed) 
    {
      while (speedCmd > baseFwdSpeed)
      {
        baseFwdSpeed += DELTA_FORWARD_SPEED;
        myBase.drive(baseFwdSpeed, myBase.DriveStraight);
        delay(RAMPUP_SPEED_DELAY);
      }
    }
    else
    {
      while (speedCmd < baseFwdSpeed)
      {
        baseFwdSpeed -= DELTA_FORWARD_SPEED;
        myBase.drive(baseFwdSpeed, myBase.DriveStraight);
        delay(RAMPUP_SPEED_DELAY);
      }
    }
    if (baseFwdSpeed > 0) driving = DRIVING_FORWARD;
    else driving = DRIVING_BACKWARD;
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

void turnCommandedSpeed()
{
    powerOnCreate();
    bool turnRight = true;
    driving = DRIVING_TURNRIGHT;
    if (speedCmd < 0)    // a negative speedCmd means to turn left
    {
      turnRight = false;
      speedCmd *= -1;
      driving = DRIVING_TURNLEFT;
    }
    if (speedCmd > baseTurnSpeed) 
    {
      while (speedCmd > baseFwdSpeed)  // if we are changing direction of the turn, then this will happen without a rampup, but that is OK, since it suggests a need to turn fast.
      {
        baseTurnSpeed += DELTA_TURN_SPEED;
        if (turnRight) myBase.drive(baseTurnSpeed, myBase.DriveInPlaceCounterClockwise); // this is wrong, for some reason; left should be counter and right should be clockwise
        else myBase.drive(baseTurnSpeed, myBase.DriveInPlaceClockwise); // this is wrong, for some reason; left should be counter and right should be clockwise
        delay(RAMPUP_SPEED_DELAY);
      }
    }
    else
    {
      while (speedCmd < baseFwdSpeed)
      {
        baseTurnSpeed -= DELTA_TURN_SPEED;
        if (turnRight) myBase.drive(baseTurnSpeed, myBase.DriveInPlaceCounterClockwise); // this is wrong, for some reason; left should be counter and right should be clockwise
        else myBase.drive(baseTurnSpeed, myBase.DriveInPlaceClockwise); // this is wrong, for some reason; left should be counter and right should be clockwise
        delay(RAMPUP_SPEED_DELAY);
      }
    }
    baseFwdSpeed = 0;
    lastCmdMs = millis();  // remember when the command was done
}


void stop()
{
 // powerOnCreate();
  myBase.drive(0, myBase.DriveStraight);
  driving = DRIVING_NONE;
  baseTurnSpeed = 0;
  baseFwdSpeed = 0;
  speedCmd = 0;
  lastCmdMs = millis();
}
  

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  pinMode(powerPin, OUTPUT); // toggle DD to toggle power on the Create
  pinMode(powerMonitor, INPUT); // high if power from Create via power board (and power board's switch) is on
  pinMode(powerIndicatorPin, OUTPUT); // output to drive an LED that the user can see.
  
    //Attach an interrupt to the input pins and monitor for low to high transitions
 /* attachInterrupt(forwardCmd, moveForward, RISING);
  attachInterrupt(backwardCmd, moveBackward, RISING);
  attachInterrupt(rightCmd, turnRight, RISING);
  attachInterrupt(leftCmd, turnLeft, RISING);
  attachInterrupt(emergencyShutdownCmd, emergencyShutdown, LOW);
  */
  
  counter = 0;  // report robot state on first pass through the loop
  // start serial port at 57600 bps for the create
  Serial2.begin(57600); 
   myBase.start();
   myBase.safeMode();
   stop();  // this will power up the create and initialize the state
  
  nh.initNode();
  nh.advertise(chatter);
 // nh.advertise(robot_state);
  nh.advertise(robot_commands);
  nh.subscribe(subscriberSkype);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
