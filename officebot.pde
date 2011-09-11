// This code controls the pan and tilt and Create
// Created by Zhengqin Fan
// Modified 2011-09-09 by Alaina Hardie

// Requires Roomba library, available at:
// http://www.open.com.au/mikem/arduino/Roomba/

#include <Servo.h> 
#include <Roomba.h>
 
 
#define DRIVING_NONE 0
#define DRIVING_FORWARD 1
#define DRIVING_BACKWARD 2
#define DRIVING_TURNLEFT 3
#define DRIVING_TURNRIGHT 4

#define N_G1     31
#define N_Ab1	 32
#define N_A1	 33
#define N_Bb1    34
#define N_B1	 35
#define N_C2	 36
#define N_Cs2	 37
#define N_D2	 38
#define N_Eb2	 39
#define N_E2	 40
#define N_F2	 41
#define N_Fs2	 42
#define N_G2	 43
#define N_Ab2	 44
#define N_A2	 45
#define N_Bb2	 46
#define N_B2	 47
#define N_C3	 48
#define N_Cs3	 49
#define N_D3	 50
#define N_Eb3	 51
#define N_E3	 52
#define N_F3	 53
#define N_Fs3	 54
#define N_G3	 55
#define N_Ab3	 56
#define N_A3	 57
#define N_Bb3	 58
#define N_B3	 59
#define N_C4	 60
#define N_Cs4	 61
#define N_D4	 62
#define N_Eb4	 63
#define N_E4	 64
#define N_F4	 65
#define N_Fs4	 66
#define N_G4	 67
#define N_Ab4	 68
#define N_A4	 69
#define N_Bb4	 70
#define N_B4	 71
#define N_C5	 72
#define N_Cs5	 73
#define N_D5	 74
#define N_Eb5	 75
#define N_E5	 76
#define N_F5	 77
#define N_Fs5	 78
#define N_G5	 79
#define N_Ab5	 80
#define N_A5	 81
#define N_Bb5	 82
#define N_B5	 83
#define N_C6	 84
#define N_Cs6	 85
#define N_D6	 86
#define N_Eb6	 87
#define N_E6	 88
#define N_F6	 89
#define N_Fs6	 90
#define N_G6	 91
#define N_Ab6	 92

uint8_t sweetChildOfMine[] = 
{
  74, 14,  86, 14,  69, 14,  79, 14,  91, 14,  69, 14,  90, 14,  69, 14, 74, 14,  86, 14,  69, 14,  79, 14,  91, 14,  69, 14,  90, 14,  69, 14,
  76, 14,  86, 14,  69, 14,  79, 14,  91, 14,  69, 14,  90, 14,  69, 14, 76, 14,  86, 14,  69, 14,  79, 14,  91, 14,  69, 14,  90, 14,  69, 14,
  79, 14,  86, 14,  69, 14,  79, 14,  91, 14,  69, 14,  90, 14,  69, 14, 79, 14,  86, 14,  69, 14,  79, 14,  91, 14,  69, 14,  90, 14,  69, 14,
  74, 14,  86, 14,  69, 14,  79, 14,  91, 14,  69, 14,  90, 14,  69, 14, 74, 14,  86, 14,  69, 14,  79, 14,  91, 14,  69, 14,  90, 14,  69, 14,
  74, 14,  86, 14,  69, 14,  79, 14,  91, 14,  69, 14,  90, 14,  69, 14, 74, 14,  86, 14,  69, 14,  79, 14,  91, 14,  69, 14,  90, 14,  69, 14,
  76, 14,  86, 14,  69, 14,  79, 14,  91, 14,  69, 14,  90, 14,  69, 14, 76, 14,  86, 14,  69, 14,  79, 14,  91, 14,  69, 14,  90, 14,  69, 14,
  79, 14,  86, 14,  69, 14,  79, 14,  91, 14,  69, 14,  90, 14,  69, 14, 79, 14,  86, 14,  69, 14,  79, 14,  91, 14,  69, 14,  90, 14,  69, 14,
  74, 14,  86, 14,  69, 14,  79, 14,  91, 14,  69, 14,  90, 14,  69, 14, 74, 14,  86, 14,  69, 14,  79, 14,  91, 14,  69, 14,  90, 14,  69, 14
};

//uint8_t ironMan[] = {
//                    N_B1, 64, N_D2, 64, N_D2, 32, N_E2, 32, N_E2, 64, 
//                    N_G2, 16, N_Fs2, 16, N_G2, 16, N_Fs2, 16, N_G2, 32, N_D2, 32, N_D2, 32, N_E2, 32, N_E2, 64  
//                  };
//
//uint8_t buzz[] = {31, 32};

Servo panservo;  // instance for control of the pan (left-right) servo 
Servo tiltservo; // instance for control of the tilt (up-down) servo

Roomba myBase(&Serial1); // instance for the Create on Serial1

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

const int powerPin = 2;  // this pin is connected to the Create's Device Detect (DD) pin and is used to toggle power
const int chargingPin = 3; // high if it's charging
const int powerMonitor = 4; // high if the power from the power box is on, meaning that the Create is on and so is the power box's switch

const int panServoPin = 9; // pin for control of the pan (left-right) servo
const int tiltServoPin = 10; // pin for control of the tilt (up-down) servo

const int panPosDefault = 90; // default pan position
const int tiltPosDefault = 100; // default tilt position

sensorData726 sensors;

int PmonitorValue = 0; // value of power
int inByte = 0; // byte read from serial in
int panpos = panPosDefault; // current position of pan servo (default set here)     
int tiltpos = tiltPosDefault; // current position of tilt servo (default set here)
int driving = DRIVING_NONE; // true if it's in the driving state

const unsigned long lastCmdTimeoutMs = 3000; // number of milliseconds to wait for command before you consider it a timeout and stop the base
unsigned long lastCmdMs = 0; // The number of milliseconds since the program started that the last command was received
int timedOut = 0; // true if there hasn't been a command received within the timeout period; false if a command has been received and there's no timeout

const int baseFwdSpeedDefault = 500; // default forward speed (in mm/sec)
const int baseBwdSpeedDefault = -500; // default backward speed (in mm/sec)
const int baseTurnSpeedDefault = 200; // default turning speed (in mm/sec)
int baseFwdSpeed = baseFwdSpeedDefault; // forward speed in mm/sec 
int baseBwdSpeed = baseBwdSpeedDefault; // backward speed in mm/sec
int baseTurnSpeed = baseTurnSpeedDefault; // backward speed in mm/sec

void centerPanTilt()
// center the pan & tilt mechanism
{
  panservo.write(panPosDefault);
  tiltservo.write(tiltPosDefault);
  panpos = panPosDefault;
  tiltpos = tiltPosDefault;
  
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
        }  
}

void powerOffCreate()
// power the Create off
{
        if (digitalRead(powerMonitor))  // if the power is on, turn the Create off.
        {
            digitalWrite(powerPin, LOW);
            delay (10);
            digitalWrite(powerPin, HIGH);
            delay(500);
            digitalWrite(powerPin, LOW);
        }  
}


void setup() 
{ 
  pinMode(powerPin, OUTPUT); // toggle DD to toggle power on the Create
  pinMode(powerMonitor, INPUT); // high if power from Create via power board (and power board's switch) is on
  panservo.attach(panServoPin);  // configure the pan servo
  tiltservo.attach(tiltServoPin); // configure the tilt servo

  // start serial port at 9600 bps:
  Serial.begin(9600);
  Serial1.begin(57600);
  
  establishContact();  // send a byte to establish contact until receiver responds 
  
  myBase.start();
  myBase.safeMode();
  centerPanTilt();

  myBase.song(0, sweetChildOfMine, sizeof(sweetChildOfMine));
//  myBase.song(2, buzz, sizeof(buzz));
//  myBase.song(3, ironMan, sizeof(ironMan));
  //myBase.song(1, song1, sizeof(song1));
  //myBase.song(2, song2, sizeof(song2));
  //myBase.song(3, song3, sizeof(song3));

} 

void loop() 
{ 
//  if (
//      (lastCmdMs + lastCmdTimeoutMs) >= millis() &&
//      driving != DRIVING_NONE
//     ) // if timeout has been reached and the base is moving then send a stop command so it doesn't go out of control and crash into something
//  {
//    driving = DRIVING_NONE;
//    myBase.drive(0, myBase.DriveStraight);
//  }
  
  if (Serial.available() > 0) // if there's feedback from the commander, take it 
  { 
    inByte = Serial.read(); // read the next available character
    lastCmdMs = millis();
    
    switch (inByte)
    {
       /* 
        * navigation speed settings - forward, backward and turning
        */
      case '1': // forward speed: 50 mm/sec
        baseFwdSpeed = 50;
        break;
      case '2': // forward speed: 100 mm/sec
        baseFwdSpeed = 100;
        break;
      case '3': // forward speed: 150 mm/sec
        baseFwdSpeed = 150;
        break;
      case '4': // forward speed: 200 mm/sec
        baseFwdSpeed = 200;
        break;
      case '5': // forward speed: 250 mm/sec
        baseFwdSpeed = 250;
        break;
      case '6': // forward speed: 300 mm/sec
        baseFwdSpeed = 300;
        break;
      case '7': // forward speed: 350 mm/sec
        baseFwdSpeed = 350;
        break;
      case '8': // forward speed: 400 mm/sec
        baseFwdSpeed = 400;
        break;
      case '9': // forward speed: 450 mm/sec
        baseFwdSpeed = 450;
        break;
      case '0': // forward speed: 500 mm/sec
        baseFwdSpeed = 500;
        break;
      case '!': // backward speed: 50 mm/sec
        baseBwdSpeed = -50;
        break;
      case '@': // backward speed: 100 mm/sec
        baseBwdSpeed = -100;
        break;
      case '#': // backward speed: 150 mm/sec
        baseBwdSpeed = -150;
        break;
      case '$': // backward speed: 200 mm/sec
        baseBwdSpeed = -200;
        break;
      case '%': // backward speed: 250 mm/sec
        baseBwdSpeed = -250;
        break;
      case '^': // backward speed: 300 mm/sec
        baseBwdSpeed = -300;
        break;
      case '&': // backward speed: 350 mm/sec
        baseBwdSpeed = -350;
        break;
      case '*': // backward speed: 400 mm/sec
        baseBwdSpeed = -400;
        break;
      case '(': // backward speed: 450 mm/sec
        baseBwdSpeed = -450;
        break;
      case ')': // backward speed: 500 mm/sec
        baseBwdSpeed = -500;
        break;
       case 'Q': // turning speed: 50 mm/sec
        baseTurnSpeed = 50;
        break;
      case 'W': // turning speed: 100 mm/sec
        baseTurnSpeed = 100;
        break;
      case 'E': // turning speed: 150 mm/sec
        baseTurnSpeed = 150;
        break;
      case 'R': // turning speed: 200 mm/sec
        baseTurnSpeed = 200;
        break;
      case 'T': // turning speed: 250 mm/sec
        baseTurnSpeed = 250;
        break;
      case 'Y': // turning speed: 300 mm/sec
        baseTurnSpeed = 300;
        break;
      case 'U': // turning speed: 350 mm/sec
        baseTurnSpeed = 350;
        break;
      case 'I': // turning speed: 400 mm/sec
        baseTurnSpeed = 400;
        break;
      case 'O': // turning speed: 450 mm/sec
        baseTurnSpeed = 450;
        break;
      case 'P': // turning speed: 500 mm/sec
        baseTurnSpeed = 500;
        break;

     /*
      * pan/tilt commands
      */

     case 'c': // center pan/tilt
        centerPanTilt();
        break;
      case 'd': // pan right
            if (panpos < 180)
                panpos++;
            panservo.write(panpos);
            delay(15);
            break;
        case 'a': // pan left
            if (panpos > 0)
                panpos--;
            panservo.write(panpos);
            delay(15);
            break;
        case 's': // tilt down
            if (tiltpos < 160)
                tiltpos++;
            tiltservo.write(tiltpos);
            delay(15);
            break;
        case 'w': // tilt up
            if (tiltpos > 30)
                tiltpos--;
            tiltservo.write(tiltpos);
            delay(15);
            break;

        /* 
         * driving action commands - forward, backward, turn, stop
         */

        case 'f': // drive create forward
          driving = DRIVING_FORWARD;
          myBase.drive(baseFwdSpeed, myBase.DriveStraight);
          break;
        case 'b': // drive create backward
          driving = DRIVING_BACKWARD;
          myBase.drive(baseBwdSpeed, myBase.DriveStraight);
          break;
        case 'l': // turn create left
          driving = DRIVING_TURNLEFT;
          myBase.drive(baseTurnSpeed, myBase.DriveInPlaceCounterClockwise); // this is wrong, for some reason; left should be counter and right should be clockwise
          break;
        case 'r': // turn create right
          driving = DRIVING_TURNRIGHT;
          myBase.drive(baseTurnSpeed, myBase.DriveInPlaceClockwise); // this is wrong, for some reason; left should be counter and right should be clockwise
          break;
        case 'e': // end create's movement
          driving = DRIVING_NONE;
          myBase.drive(0, myBase.DriveStraight);
          break;
          
        /* 
         * Other Create actions and status commands 
         */
         
        case 'h': // set DD high
            digitalWrite(powerPin, HIGH);  
            break;
        case 'm': // return power state from power board (and thus Create, because the Create is powered from the power board)
            Serial.print(digitalRead(powerMonitor));  
            break;
        case 'y': // power on the Create
          powerOnCreate();
          break;
        case 'n': // power off the Create
          powerOffCreate();
          break;

        /* 
         * Other Create actions and status commands 
         */

        case 'p': // play Sweet Child of Mine
          myBase.playSong(0); 
          break;
        case 'i': // play Iron Man
       //   myBase.playSong(3); 
          break;
        case 'k': // play buzz
         // myBase.playSong(2); 
          break;
        case 'S': // get sensors
          myBase.getSensors (myBase.Sensors7to26, (uint8_t *)&sensors, sizeof(sensors));
          
          Serial.print ("Battery capcity: ");
          Serial.println (sensors.batteryCapacity);
          Serial.print ("Voltage: ");
          Serial.println (sensors.voltage);
          break;
      }           
      myBase.getSensors (myBase.Sensors7to26, (uint8_t *)&sensors, sizeof(sensors));
          Serial.print ("bumps and wheel drops: ");
          Serial.println (sensors.bumpsAndWheelDrops);
      
      // should be unnecessary in safe mode; here for testing
      if (sensors.bumpsAndWheelDrops || sensors.wall || sensors.cliffLeft || sensors.cliffFrontLeft ||
          sensors.cliffFrontRight || sensors.cliffRight || sensors.virtualWall)
      {
          driving = DRIVING_NONE;
          myBase.drive(0, myBase.DriveStraight);
          myBase.playSong(3);
      }
    }
} 


void establishContact() {
  while (Serial.available() <= 0) {
    Serial.print('A');   // send a capital A
    delay(300);
    }
}
 
