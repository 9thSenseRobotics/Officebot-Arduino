// rosRoomba receives ROS messages for movement commands, also monitors interrupt pins for movement commands
// publishes the roomba sensors to "robot_state"

// Requires Roomba, ROS library folders, and rosRoomba folder with header file, to be located in sketchbook/libraries

#include <Servo.h>
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
#define MIN_TURN_SPEED 100
#define DELTA_FORWARD_SPEED 50
#define DELTA_BACKWARD_SPEED -50
#define DELTA_TURN_SPEED 100
#define DEFAULT_FORWARD_SPEED 300
#define DEFAULT_BACKWARD_SPEED -300
#define DEFAULT_TURN_SPEED 300
#define RAMPUP_SPEED_DELAY 200  // controls how long between speed steps when ramping up from a stop to default speed


#define MIN_TURN_TIME 100
#define MIN_MOVE_TIME 500
#define TIME_OUT 8000  // number of milliseconds to wait for command before you consider it a timeout and stop the base
#define TURN_OFF_CREATE 900000  // number of milliseconds of inactivity before powering down create to save battery power (15 min = 900 seconds = 900000 msec)

#define MIN_PAN 10
#define MAX_PAN 140
#define CENTER_PAN 75
#define DELTA_PAN 10

#define MIN_TILT 40
#define MAX_TILT 130
#define CENTER_TILT 85
#define DELTA_TILT 10

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
Servo panServo, tiltServo;  // create servo objects
ros::NodeHandle  nh;
  
std_msgs::String ReceivedCommands;
rosRoomba::rosRoomba RobotStateMsg;
ros::Publisher robot_state("robot_state",&RobotStateMsg);
ros::Publisher robot_commands("robot_commands", &ReceivedCommands);
void skypeCallback( const std_msgs::String& msgSkype);
ros::Subscriber<std_msgs::String> subscriberSkype("SkypeChat", &skypeCallback );

const int powerPin = 4;  // this pin is connected to the Create's Device Detect (DD) pin and is used to toggle power
const int chargingIndicatorPin = 5;  // placeholder for monitoring if the create battery is being charged, not implemented yet
const int powerMonitor = 6; // high if the power from the power box is on, meaning that the Create is on and so is the power box's switch
const int powerIndicatorPin = 7;  // output to drive an LED that the user can see.
const int emergencyShutdownCmd = 0;  // interrupt 0 is on pin 2  shuts down the create in response to a button push

int driving = DRIVING_NONE; // true if it's in the driving state
unsigned long lastCmdMs; // The number of milliseconds since the program started that the last command was received
int baseStraightSpeed, baseTurnSpeed;
int counter;  // inside the loop we will only publish once every PUBLISH_RATE cycles
int emergencyShutdownReceived = 0;
int numSteps; // number of incremental movements to do
int tiltPosition, panPosition;

void skypeCallback( const std_msgs::String& msgSkype)
{
    ReceivedCommands = msgSkype;
    robot_commands.publish(&ReceivedCommands);
    numSteps = strlen( (const char* ) msgSkype.data);
    if ( numSteps > 5 ) return;  // invalid format, more than 4 characters
    for (int i = 1; i < numSteps; i++) if ( (msgSkype.data[i] != msgSkype.data[0]) && msgSkype.data[i] != msgSkype.data[0] + 32 ) return; 
          // if string is not all identical characters, allowing for first character to be a capital, return    
    char cmd = msgSkype.data[0];  
    switch(cmd)
    {
      case 'z':    // stop
        ReceivedCommands.data = "command issued to slowly stop";
        robot_commands.publish(&ReceivedCommands);
        slowStop();
        break;
        
      case 'Z':    // stop
        ReceivedCommands.data = "command issued to slowly stop";
        robot_commands.publish(&ReceivedCommands);
        slowStop();
        break;
        
      case 'x':    // stop
        stop();
        ReceivedCommands.data = "command issued to stop";
        robot_commands.publish(&ReceivedCommands);
        break;
        
      case 'X':    // stop
        stop();
        ReceivedCommands.data = "command issued to stop";
        robot_commands.publish(&ReceivedCommands);
        break;
      
      case 'w':  // move forward
        ReceivedCommands.data = "command issued to move forward";
        robot_commands.publish(&ReceivedCommands);
        driving = DRIVING_FORWARD;
        moveForward();
        break;
        
      case 'W':  // move forward
        ReceivedCommands.data = "command issued to move forward";
        robot_commands.publish(&ReceivedCommands);
        driving = DRIVING_FORWARD;
        moveForward();
        break;
        
      case 's':  //move backward
        ReceivedCommands.data = "command issued to move backward";
        robot_commands.publish(&ReceivedCommands);
        driving = DRIVING_BACKWARD;
        moveBackward();
        break;
        
      case 'S':  //move backward
        ReceivedCommands.data = "command issued to move backward";
        robot_commands.publish(&ReceivedCommands);
        driving = DRIVING_BACKWARD;
        moveBackward();
        break;
        
      case 'd':  //turn right
        driving = DRIVING_TURNRIGHT;
        ReceivedCommands.data = "command issued to turn right";
        robot_commands.publish(&ReceivedCommands);
        turn();
        break;
        
      case 'D':  //turn right
        driving = DRIVING_TURNRIGHT;
        ReceivedCommands.data = "command issued to turn right";
        robot_commands.publish(&ReceivedCommands);
        turn();
        break;
        
      case 'a':  // turn left
        driving = DRIVING_TURNLEFT;
        ReceivedCommands.data = "command issued to turn left";
        robot_commands.publish(&ReceivedCommands);
        turn();
        break;
        
      case 'A':  // turn left
        driving = DRIVING_TURNLEFT;
        ReceivedCommands.data = "command issued to turn left";
        robot_commands.publish(&ReceivedCommands);
        turn();
        break;
        
     case 'u':  // tilt up
        tiltServoUp();
        break;

     case 'U': // tilt up
        tiltServoUp();
        break;  
  
      case 'n':  // tilt down
        tiltServoDown();
        break;

      case 'N':  // tilt down
        tiltServoDown();
        break; 
   
      case 'k': // pan right
        panServoRight();
        break;
     
      case 'K': // pan right
        panServoRight();
        break; 
 
      case 'h': // pan left
        panServoLeft();
        break;
        
      case 'H': // pan left
        panServoLeft();
        break; 
        
      case 'j': //center servos
        centerServos();
        break;
  
      case 'J': //center servos
        centerServos();
        break;
  
      case 'c':  // center servos
        centerServos();
        break;
   
      case 'C':  // center servos
        centerServos();
        break;  
  
      case 'm':  //max down tilt servo  
        maxDownServoTilt();
        break;
    
      case 'M':  //max down tilt servo  
        maxDownServoTilt();
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
            myBase.fullMode();
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
  baseStraightSpeed = 0;
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
  lastCmdMs = millis();  // prevents a timeout in loop()
  if (numSteps == 1)     // just get it done
  {
    myBase.drive(DEFAULT_FORWARD_SPEED, myBase.DriveStraight);
    delay(MIN_MOVE_TIME);
    stop();
  }
  else  // use ramp up and ramp down
  {
    myBase.drive(MIN_FORWARD_SPEED, myBase.DriveStraight);
    baseStraightSpeed = MIN_FORWARD_SPEED;
    while (baseStraightSpeed < DEFAULT_FORWARD_SPEED)  // ramp up to speed from a stop
    {
        baseStraightSpeed += DELTA_FORWARD_SPEED;
        myBase.drive(baseStraightSpeed, myBase.DriveStraight);
        delay(RAMPUP_SPEED_DELAY);
    }
    for (int i = 1; i < numSteps; i++)
    {
      baseStraightSpeed += DELTA_FORWARD_SPEED;
      if (baseStraightSpeed <= MAX_FORWARD_SPEED) myBase.drive(baseStraightSpeed, myBase.DriveStraight);
      delay(RAMPUP_SPEED_DELAY);
    }
    delay(MIN_MOVE_TIME * ( (3 * numSteps) - 6 ));  // numSteps here can range from 2 to 5, so we can get 0, 3MMT, 6MMT, and 9MMT 
    slowStop();
  }
}

void moveBackward()
{
  lastCmdMs = millis();  // prevents a timeout in loop()
  if (numSteps == 1) 
  {
    myBase.drive(DEFAULT_BACKWARD_SPEED, myBase.DriveStraight);
    delay(MIN_MOVE_TIME);
    stop();
  }
  else
  {
    myBase.drive(MIN_BACKWARD_SPEED, myBase.DriveStraight);
    baseStraightSpeed = MIN_BACKWARD_SPEED;
    while (baseStraightSpeed > DEFAULT_BACKWARD_SPEED)  // ramp up to speed from a stop, note that these are negative numbers
    {
        baseStraightSpeed += DELTA_BACKWARD_SPEED;
        myBase.drive(baseStraightSpeed, myBase.DriveStraight);
        delay(RAMPUP_SPEED_DELAY);
    }
    for (int i = 1; i < numSteps; i++)
    {
      baseStraightSpeed += DELTA_BACKWARD_SPEED;
      if (baseStraightSpeed >= MAX_BACKWARD_SPEED) myBase.drive(baseStraightSpeed, myBase.DriveStraight);
      delay(RAMPUP_SPEED_DELAY);
    }
    delay(MIN_MOVE_TIME * ( (3 * numSteps) - 6 ));  // numSteps here can range from 2 to 5, so we can get 0, 3MMT, 6MMT, and 9MMT 
    if (numSteps > 3) delay(1000);  // for longer runs, add some extra time
    slowStop();
  }
}

void turn()
{
  lastCmdMs = millis();  // prevents a timeout in loop()
  if (driving == DRIVING_TURNRIGHT) myBase.drive(DEFAULT_TURN_SPEED, myBase.DriveInPlaceClockwise);
  else myBase.drive(DEFAULT_TURN_SPEED, myBase.DriveInPlaceCounterClockwise);
  delay(MIN_TURN_TIME * numSteps);  // numSteps here can range from 1 to 5, so we can get 1, 3MMT, 9MMT, 13MMT, and 17MT numSteps);
  if (numSteps > 3) delay(500);  // for longer runs, add some extra time
  
  stop();
 }


void slowStop()
{
  powerOnCreate();
  if (driving < 3) // not turning
  {
    while (baseStraightSpeed > MIN_FORWARD_SPEED) 
    {
        baseStraightSpeed -= DELTA_FORWARD_SPEED;
        myBase.drive(baseStraightSpeed, myBase.DriveStraight);
        delay(RAMPUP_SPEED_DELAY);
    }
    while (baseStraightSpeed < MIN_BACKWARD_SPEED)  // negative numbers here
    {
        baseStraightSpeed -= DELTA_BACKWARD_SPEED;
        myBase.drive(baseStraightSpeed, myBase.DriveStraight);
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
  baseStraightSpeed = 0;
  lastCmdMs = millis();
}
 
void panServoRight()
{
  panPosition += DELTA_PAN * numSteps;
  if (panPosition > MAX_PAN) panPosition = MAX_PAN;
  panServo.write(panPosition); 
}

void panServoLeft()
{
  panPosition -= DELTA_PAN * numSteps;
  if (panPosition < MIN_PAN) panPosition = MIN_PAN;
  panServo.write(panPosition); 
}

void tiltServoUp()
{
  tiltPosition -= DELTA_TILT * numSteps;
  if (tiltPosition < MIN_TILT) tiltPosition = MIN_TILT;
  tiltServo.write(tiltPosition); 
}

void tiltServoDown()
{
  tiltPosition += DELTA_TILT * numSteps;
  if (tiltPosition > MAX_TILT) tiltPosition = MAX_TILT;
  tiltServo.write(tiltPosition); 
}

void centerServos()
{
  panPosition = CENTER_PAN;
  panServo.write(panPosition);
  delay(300);
  tiltPosition = CENTER_TILT;
  tiltServo.write(tiltPosition);
}

void maxDownServoTilt()
{
  tiltPosition = MAX_TILT;
  tiltServo.write(tiltPosition);
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
    RobotStateMsg.FwdCommandSpeed = baseStraightSpeed;
    RobotStateMsg.TurnCommandSpeed = baseTurnSpeed;
    RobotStateMsg.SecondsSinceLastCommand = deltaT;
    RobotStateMsg.PowerBoxOn = powerMonitor;
    RobotStateMsg.EmergencyShutdown = emergencyShutdownReceived;
  
    robot_state.publish( &RobotStateMsg );
}

void goForStroll()
{
  numSteps = 2;
  ReceivedCommands.data = "command issued to move forward";
  robot_commands.publish(&ReceivedCommands);
  driving = DRIVING_FORWARD;
  moveForward();
  delay(100);
  
  driving = DRIVING_TURNRIGHT;
  ReceivedCommands.data = "command issued to turn right";
  robot_commands.publish(&ReceivedCommands);
  turn();
  delay(100);
   
  driving = DRIVING_TURNLEFT;
  ReceivedCommands.data = "command issued to turn left";
  robot_commands.publish(&ReceivedCommands);
  turn();
  delay(100);
   
  ReceivedCommands.data = "command issued to move backward";
  robot_commands.publish(&ReceivedCommands);
  driving = DRIVING_BACKWARD;
  moveBackward();
}

void setup() 
{ 
  pinMode(powerPin, OUTPUT); // toggle DD to toggle power on the Create
  pinMode(powerMonitor, INPUT); // high if power from Create via power board (and power board's switch) is on
  pinMode(powerIndicatorPin, OUTPUT); // output to drive an LED that the user can see.
  
  panServo.attach(5);  // attaches the servos to pins 5 and 7
  tiltServo.attach(7); 
  
  centerServos();
  
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

  stop();   
  counter = 0; 
}

void loop() 
{   
  myBase.leds(0x02,0,32);  // visual indicator that we are at the top of the loop
  if ((millis() > lastCmdMs + TIME_OUT) && (driving != DRIVING_NONE) ) stop();
  
  //if (millis() > lastCmdMs + TURN_OFF_CREATE) powerOffCreate();

  //if (!counter % PUBLISH_RATE) getSensorData(); // only publish once every PUBLISH_RATE times through the loop
  //if (counter < 1) goForStroll();
  
  //if (counter > 10000) counter = PUBLISH_RATE;
  //counter++;

  //myBase.leds(0x08,255,32);  // visual indicator that we are stuck inside the loop
  
  nh.spinOnce();
  delay(500);
}

