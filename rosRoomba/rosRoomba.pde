// rosRoomba receives ROS messages for movement commands, also monitors interrupt pins for movement commands
// publishes the roomba sensors to "robot_state"

// Requires Roomba, ROS library folders, and rosRoomba folder with header file, to be located in sketchbook/libraries

#include <Roomba.h>
#include <ros.h>
#include <rosRoomba/rosRoomba.h>
#include <std_msgs/String.h>

/* notes from Roomba.h
    /// Starts the Roomba driving with a specified wheel velocity and radius of turn
    /// \param[in] velocity Speed in mm/s (-500 to 500 mm/s)
    /// \param[in] radius Radius of the turn in mm. (-2000 to 2000 mm). 
    /// Any of the special values in enum Roomba::Drive may be used instead of a radius value
    void drive(int16_t velocity, int16_t radius);

    /// Starts the Roomba driving with a specified velocity for each wheel
    /// Create only. No equivalent on Roomba.
    /// \param[in] leftVelocity Left wheel velocity in mm/s (-500 to 500 mm/s)
    /// \param[in] rightVelocity Right wheel velocity in mm/s (-500 to 500 mm/s)
    void driveDirect(int16_t leftVelocity, int16_t rightVelocity);
    
    /// \enum Drive
    /// Special values for radius in Roomba::drive()
    typedef enum
    {
	DriveStraight                = 0x8000,
	DriveInPlaceClockwise        = 0xFFFF,
	DriveInPlaceCounterClockwise = 0x0001,
    } Drive;
    
To play a song or tone, define the notes, as in the example below,
then assign the song a number, in this example, the number is 0:
roomba.song(0, sweetChildOfMine, sizeof(sweetChildOfMine));
Then play the song with
roomba.playSong(0); where you put the song number in place of 0

To light the leds, you specify which of the two green leds should
light with the first entry, 0x00 for none, 0x02 for middle one,
0x08 for the end one, 0x0A for both
For the color led, you select the color (0=green, 255 = red) 
and the intensity (0 = off, 255 = brightest)
roomba.leds(0x0A,128,255); gets you both green lights on and
the colored light to amber, full intensity
roomba.leds(0x02,0,128); gets you the end green light off,
the middle green light on, and the colored one to green, half intensity

  
*/

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
#define DEFAULT_FORWARD_SPEED 500
#define DEFAULT_BACKWARD_SPEED -300
#define DEFAULT_TURN_SPEED 300
#define RAMPUP_SPEED_DELAY 300  // controls how long between speed steps when ramping up from a stop to default speed


#define TURN_TIME 200
#define MOVE_TIME 2000
#define TIME_OUT 5000  // number of milliseconds to wait for command before you consider it a timeout and stop the base
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


Roomba myBase(&Serial2); // instance for the Create on Serial2 (pins 16 and 17)

ros::NodeHandle  nh;
  
std_msgs::String ReceivedCommands;
rosRoomba::rosRoomba RobotStateMsg;
ros::Publisher robot_state("robot_state",&RobotStateMsg);
ros::Publisher robot_commands("robot_commands", &ReceivedCommands);
void skypeCallback( const std_msgs::String& msgSkype);
ros::Subscriber<std_msgs::String> subscriberSkype("SkypeChat", &skypeCallback );

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "hello world!";

const int powerPin = 4;  // this pin is connected to the Create's Device Detect (DD) pin and is used to toggle power
const int chargingIndicatorPin = 5;  // placeholder for monitoring if the create battery is being charged, not implemented yet
const int powerMonitor = 6; // high if the power from the power box is on, meaning that the Create is on and so is the power box's switch
const int powerIndicatorPin = 7;  // output to drive an LED that the user can see.
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
        slowStop();
        break;
        
      case 'S':    // stop
        slowStop();
        break;
        
      case 'q':    // stop
        stop();
        break;
        
      case 'Q':    // stop
        stop();
        break;
      
      case 'f':  // move forward
        if (speedCmd == 0) moveForwardNow();
        else moveCommandedSpeed();
        break;
        
      case 'F':  // move forward
        if (speedCmd == 0) moveForwardNow();
        else moveCommandedSpeed();
        break;
        
      case 'b':  //move backward
        if (speedCmd == 0) moveBackwardNow();
        else 
        {
          speedCmd *= -1;
          moveCommandedSpeed();
        }
        break;
        
      case 'B':  //move backward
        if (speedCmd == 0) moveBackwardNow();
        else 
        {
          speedCmd *= -1;
          moveCommandedSpeed();
        }
        break;
        
      case 'r':  //turn right
        if (speedCmd == 0) turnRightNow();
        else turnCommandedSpeed();
        break;
        
      case 'R':  //turn right
        if (speedCmd == 0) turnRightNow();
        else turnCommandedSpeed();
        break;
        
      case 'l':  // turn left
        if (speedCmd == 0) turnLeftNow();
        else 
        {
          speedCmd *= -1;
          turnCommandedSpeed();
        }
        break;
        
      case 'L':  // turn left
        if (speedCmd == 0) turnLeftNow();
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
 /*       if (!digitalRead(powerMonitor))  // if the power is off, turn the Create on.
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
*/
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
  ReceivedCommands.data = "command issued to move more forward";
  robot_commands.publish(&ReceivedCommands);
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
    
    baseFwdSpeed = DEFAULT_BACKWARD_SPEED;
    //myBase.drive(baseFwdSpeed, myBase.DriveStraight);
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
  ReceivedCommands.data = "command issued to move more backward";
  robot_commands.publish(&ReceivedCommands);
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
    
    if (baseFwdSpeed > 0)
    {
       driving = DRIVING_FORWARD;
       ReceivedCommands.data = "command issued to move forward";
    }     
    else
    {
      driving = DRIVING_BACKWARD;
      ReceivedCommands.data = "command issued to move backward";
    }
    robot_commands.publish(&ReceivedCommands);
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
      baseTurnSpeed += DELTA_TURN_SPEED;
      myBase.drive(baseTurnSpeed, myBase.DriveInPlaceClockwise); 
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
        myBase.drive(baseTurnSpeed, myBase.DriveInPlaceCounterClockwise); 
      }
      else
      {
        if (baseTurnSpeed <= MAX_TURN_SPEED - DELTA_TURN_SPEED) baseTurnSpeed += DELTA_TURN_SPEED;  // got a turn right command while turning right, means speed up the turn
        myBase.drive(baseTurnSpeed, myBase.DriveInPlaceClockwise); 
      }
    }
  } 
  baseFwdSpeed = 0;
  ReceivedCommands.data = "command issued to move more right";
  robot_commands.publish(&ReceivedCommands);
  lastCmdMs = millis();  // remember when the command was done
}

void moveForwardNow()
{
  lastCmdMs = millis();  // remember when the command was done
  myBase.drive(DEFAULT_FORWARD_SPEED, myBase.DriveStraight);
  delay(MOVE_TIME);
  slowStop();
  driving = DRIVING_NONE;
}

void moveBackwardNow()
{
  lastCmdMs = millis();  // remember when the command was done
  myBase.drive(DEFAULT_BACKWARD_SPEED, myBase.DriveStraight);
  delay(MOVE_TIME);
  myBase.drive(0, myBase.DriveStraight);
  driving = DRIVING_NONE;
}

void turnRightNow()
{
  lastCmdMs = millis();  // remember when the command was done
  myBase.drive(DEFAULT_TURN_SPEED, myBase.DriveInPlaceClockwise);
  delay(TURN_TIME);
  myBase.drive(0, myBase.DriveInPlaceCounterClockwise);
  driving = DRIVING_NONE;
}

void turnLeftNow()
{
  lastCmdMs = millis();  // remember when the command was done
  myBase.drive(DEFAULT_TURN_SPEED, myBase.DriveInPlaceCounterClockwise);
  delay(TURN_TIME);
  myBase.drive(0, myBase.DriveInPlaceCounterClockwise);
  driving = DRIVING_NONE;
}

  
void turnLeft()
{
  powerOnCreate();
  if (baseTurnSpeed == 0) // if starting from a stop, begin with default speed
  {
    while (baseTurnSpeed < DEFAULT_TURN_SPEED)  // ramp up to speed from a stop
    {
      baseTurnSpeed += DELTA_TURN_SPEED;
      myBase.drive(baseTurnSpeed, myBase.DriveInPlaceCounterClockwise); 
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
        myBase.drive(baseTurnSpeed, myBase.DriveInPlaceClockwise); 
      }
      else
      {
        if (baseTurnSpeed <= MAX_TURN_SPEED - DELTA_TURN_SPEED) baseTurnSpeed += DELTA_TURN_SPEED;  // got a turn left command while turning left, means speed up the turn
        myBase.drive(baseTurnSpeed, myBase.DriveInPlaceCounterClockwise); 
      }
    }
  } 
  baseFwdSpeed = 0;
  ReceivedCommands.data = "command issued to move more left";
  robot_commands.publish(&ReceivedCommands);
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
      while (speedCmd > baseTurnSpeed)  // if we are changing direction of the turn, then this will happen without a rampup, but that is OK, since it suggests a need to turn fast.
      {
        baseTurnSpeed += DELTA_TURN_SPEED;
        if (turnRight) myBase.drive(baseTurnSpeed, myBase.DriveInPlaceClockwise); 
        else myBase.drive(baseTurnSpeed, myBase.DriveInPlaceCounterClockwise); 
        delay(RAMPUP_SPEED_DELAY);
      }
    }
    else
    {
      while (speedCmd < baseTurnSpeed)
      {
        baseTurnSpeed -= DELTA_TURN_SPEED;
        if (turnRight) myBase.drive(baseTurnSpeed, myBase.DriveInPlaceClockwise); 
        else myBase.drive(baseTurnSpeed, myBase.DriveInPlaceCounterClockwise); 
        delay(RAMPUP_SPEED_DELAY);
      }
    }
    baseFwdSpeed = 0;
    ReceivedCommands.data = "command issued to turn";
    robot_commands.publish(&ReceivedCommands);
    lastCmdMs = millis();  // remember when the command was done
}

void slowStop()
{
  powerOnCreate();
  if (driving < 3) // not turning
  {
    while (baseFwdSpeed > MIN_FORWARD_SPEED) 
    {
        baseFwdSpeed -= DELTA_FORWARD_SPEED;
        myBase.drive(baseFwdSpeed, myBase.DriveStraight);
        delay(RAMPUP_SPEED_DELAY);
    }
    while (baseFwdSpeed < MIN_BACKWARD_SPEED)
    {
        baseFwdSpeed += DELTA_FORWARD_SPEED;
        myBase.drive(baseFwdSpeed, myBase.DriveStraight);
        delay(RAMPUP_SPEED_DELAY);
    }
  }
  else  // turning
  {
    while (baseTurnSpeed > MIN_TURN_SPEED)
    {
      baseTurnSpeed -= DELTA_TURN_SPEED;
      if (driving == DRIVING_TURNRIGHT) myBase.drive(baseTurnSpeed, myBase.DriveInPlaceClockwise); 
      else myBase.drive(baseTurnSpeed, myBase.DriveInPlaceCounterClockwise); 
    }
  }
  stop();
}
void stop()
{
  powerOnCreate();
  myBase.drive(0, myBase.DriveStraight);
  driving = DRIVING_NONE;
  baseTurnSpeed = 0;
  baseFwdSpeed = 0;
  speedCmd = 0;
  ReceivedCommands.data = "command issued to stop";
  robot_commands.publish(&ReceivedCommands);
  lastCmdMs = millis();
}
  
bool checkConnectionToRoomba()
{
  uint8_t buf[2];
  return myBase.getData(buf, 2);
}

void flashLED(int pinNum = 13, int delayTime = 500, int numFlashes = 5)
{
  pinMode(pinNum, OUTPUT);
  for (int i = 0; i < numFlashes; i++)
  {
    digitalWrite(pinNum, HIGH);
    delay(delayTime);
    digitalWrite(pinNum, LOW);
    delay(delayTime);
  }
}

void getSensorData()
{
    myBase.getSensors (myBase.Sensors7to26, (uint8_t *)&sensors, sizeof(sensors));
    
    if (sensors.bumpsAndWheelDrops || sensors.wall || sensors.cliffLeft || sensors.cliffFrontLeft ||
         sensors.cliffFrontRight || sensors.cliffRight || sensors.virtualWall) stop();
       
    int deltaT = (int) ((millis() - lastCmdMs) / 1000); 
    
    RobotStateMsg.bumpsAndWheelDrops = sensors.bumpsAndWheelDrops;
    RobotStateMsg.wall = sensors.wall;
    RobotStateMsg.cliffLeft = sensors.cliffLeft;
    RobotStateMsg.cliffFrontLeft = sensors.cliffFrontLeft;
    RobotStateMsg.cliffFrontRight = sensors.cliffFrontRight;
    RobotStateMsg.cliffRight = sensors.cliffRight;
    RobotStateMsg.virtualWall = sensors.virtualWall;
    RobotStateMsg.lsdWheelCurrents = sensors.lsdWheelCurrents;
    RobotStateMsg.irByte = sensors.irByte;
    RobotStateMsg.buttons = sensors.buttons;
    RobotStateMsg.distance = sensors.distance;
    RobotStateMsg.angle = sensors.angle;
    RobotStateMsg.chargingState = sensors.chargingState;
    RobotStateMsg.voltage = sensors.voltage;
    RobotStateMsg.current = sensors.current;
    RobotStateMsg.batteryTemp = sensors.batteryTemp;
    RobotStateMsg.batteryCharge = sensors.batteryCharge;
    RobotStateMsg.batteryCapacity = sensors.batteryCapacity;
    
    RobotStateMsg.drivingMode = driving;
    RobotStateMsg.FwdCommandSpeed = baseFwdSpeed;
    RobotStateMsg.TurnCommandSpeed = baseTurnSpeed;
    RobotStateMsg.SecondsSinceLastCommand = deltaT;
    RobotStateMsg.PowerBoxOn = powerMonitor;
    RobotStateMsg.EmergencyShutdown = emergencyShutdownReceived;
  
    robot_state.publish( &RobotStateMsg );
}

void goForStroll()
{
   moveForward();
   delay(400);
   slowStop();
   delay(100);
   turnRight();
   delay(300);
   slowStop();
   delay(100);
   turnLeft();
   delay(300);
   slowStop();
   delay(100);
   moveBackward();
   slowStop(); 
   /*
   delay(100);
   speedCmd = 300;
   turnCommandedSpeed();
   delay(300);
   stop();
   delay(100);
   speedCmd = -300;
   turnCommandedSpeed();
   delay(300);
   stop();
   delay(100);
   speedCmd = -300;
   moveCommandedSpeed();
   delay(500);
   stop(); 
   */
}

void setup() 
{ 
  pinMode(powerPin, OUTPUT); // toggle DD to toggle power on the Create
  pinMode(powerMonitor, INPUT); // high if power from Create via power board (and power board's switch) is on
  pinMode(powerIndicatorPin, OUTPUT); // output to drive an LED that the user can see.
  
  myBase.start();
  myBase.fullMode();
  //flashLED(); // this provides an indicator that we are at this place and also provides some delay time
             // for the roomba to settle down  
  for (int i = 0; i < 3; i++)
  {
    myBase.leds(0x00,255,128);  // visual indicator that we have connected to the create
    delay(300);
    myBase.leds(0x00,255,0);
    delay(300);
  }
             
  nh.initNode();
  //nh.advertise(robot_state);
  nh.advertise(robot_commands);
  nh.subscribe(subscriberSkype);
  
  nh.advertise(chatter);
  /*
  while (!nh.connected())
  {
    myBase.leds(0x02,255,128);  // visual indicator that we are stuck
    delay(300);
    myBase.leds(0x00,255,0);  // visual indicator that we are stuck
    delay(300);
  }
  */

  stop();   
  counter = 0; 
}

void loop() 
{   
  myBase.leds(0x02,0,32);  // visual indicator that we are at the top of the loop, all is well
  if ((millis() > lastCmdMs + TIME_OUT) && (driving != DRIVING_NONE) ) stop();
  
  //if (millis() > lastCmdMs + TURN_OFF_CREATE) powerOffCreate();

  //if (!counter % PUBLISH_RATE) getSensorData(); // only publish once every PUBLISH_RATE times through the loop
  //if (counter < 1) goForStroll();
  
  //if (counter > 10000) counter = PUBLISH_RATE;
  //counter++;

  //myBase.leds(0x08,255,32);  // visual indicator that we are stuck inside the loop
 // str_msg.data = hello;
  //chatter.publish( &str_msg );
  
  /*
  if (nh.connected()) nh.spinOnce();  // spinOnce() is a blocking call if comm is lost to ROS
  else
  {
    stop();
    while (!nh.connected())
    {
      myBase.leds(0x02,255,128);  // visual indicator that we are stuck
      delay(300);
      myBase.leds(0x00,255,0);  // visual indicator that we are stuck
      delay(300);
    }
  }
  */
  
  nh.spinOnce();
  delay(500);
}

