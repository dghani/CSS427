// CSS 427
// 12/8/2017
// Molly Watson and Daniel Mamaghani

#include <SPI.h>
#include <WiFi101.h>
#include <BlynkSimpleWiFiShield101.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <SoftwareSerial.h>
#include <NewPing.h>

const int LEFT_TRIG = 23;
const int LEFT_ECHO = 22;
const int RIGHT_TRIG = 45;
const int RIGHT_ECHO = 44;
const int FORWARD_TRIG = 51;
const int FORWARD_ECHO = 50;

const int LED_PIN = 53;

const int MAX_DISTANCE = 50;

NewPing left_sonar(LEFT_TRIG, LEFT_ECHO, MAX_DISTANCE);
NewPing right_sonar(RIGHT_TRIG, RIGHT_ECHO, MAX_DISTANCE);
NewPing forward_sonar(FORWARD_TRIG, FORWARD_ECHO, MAX_DISTANCE);

volatile int left_distance;
volatile int right_distance;
volatile int forward_distance;



//#include <SD.h>
//#include "RTClib.h"
//
//// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
//#define LOG_INTERVAL  1000 // mills between entries (reduce to take more/faster data)
//
//// how many milliseconds before writing the logged data permanently to disk
//// set it to the LOG_INTERVAL to write each time (safest)
//// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
//// the last 10 reads if power is lost but it uses less power and is much faster!
//#define SYNC_INTERVAL 1000 // mills between calls to flush() - to write data to the card
//uint32_t syncTime = 0; // time of last sync()
//
//RTC_DS1307 RTC;
//
//const int chipSelect = 10;
//
//File logfile;




//void logCurrentTime() {
//  DateTime now;
//
//  // log milliseconds since starting
//  uint32_t m = millis();
//  logfile.print(m);           // milliseconds since start
//  logfile.print(", ");    
//
//    // fetch the time
//  now = RTC.now();
//  // log time
//  logfile.print(now.unixtime()); // seconds since 1/1/1970
//  logfile.print(", ");
//  logfile.print('"');
//  logfile.print(now.year(), DEC);
//  logfile.print("/");
//  logfile.print(now.month(), DEC);
//  logfile.print("/");
//  logfile.print(now.day(), DEC);
//  logfile.print(" ");
//  logfile.print(now.hour(), DEC);
//  logfile.print(":");
//  logfile.print(now.minute(), DEC);
//  logfile.print(":");
//  logfile.print(now.second(), DEC);
//  logfile.print('"');
//  logfile.print(", ");
//
//  if ((millis() - syncTime) >= SYNC_INTERVAL) {
//    syncTime = millis();
//    logfile.flush(); 
//  }
//}

int status = WL_IDLE_STATUS;

Adafruit_MotorShield AFMSright(0x61); // Right-side wheels of rover
Adafruit_MotorShield AFMSleft(0x60); // Left-side wheels of rover

// Select which 'port' (motor) to use for each side: M1 (1), M2 (2), M3 (3), or M4 (4)
Adafruit_DCMotor *rightM1 = AFMSright.getMotor(1);  // assign right Motor 1 to ptr
Adafruit_DCMotor *rightM2 = AFMSright.getMotor(2);  // assign right Motor 2 to ptr 
Adafruit_DCMotor *rightM3 = AFMSright.getMotor(3);  // assign right Motor 3 to ptr

Adafruit_DCMotor *leftM1 = AFMSleft.getMotor(1);    // assign left Motor 1 to ptr
Adafruit_DCMotor *leftM2 = AFMSleft.getMotor(2);    // assign left Motor 2 to ptr
Adafruit_DCMotor *leftM3 = AFMSleft.getMotor(3);    // assign left Motor 3 to ptr

short OTSpeed = 255;      // Turn speed: outside wheels
short adjOTSpeed = 235;   // Turn speed: outside wheels (adjusted to match turn radiuses)
short ITSpeed = 75;       // Turn speed: inside wheels
short adjITSpeed = 69;    // Turn speed: inside wheels (adjusted to match turn radiuses)
short rollSpeed = 150;    // Rolling speed (for forward/backward motion)

// Daniel's Samsung Hotspot
char ssid[] = "Galaxy_S_Dan"; // network ssid
char pass[] = "xraq0835";     // network password (use for WPA, or use as key for WEP)
int keyIndex = 0;             // network key Index number (needed only for WEP)

//// WS House network
//char ssid[] = "PorkChopSandwiches-Slow";  // network ssid
//char pass[] = "";                         // network password (use for WPA, or use as key for WEP)

// Blynk Settings:
// Authorization token for Blynk app
char auth[] = "8ba86a335d474d61a1650b24f03ef64a"; // device: Wifi Card


const int BAUD_RATE = 9600;

//SoftwareSerial mySerial(6, 7); // RX, TX

volatile bool allowSelfDriving = false;






void setup()
{

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  //Initialize serial and wait for port to open:
  Serial.begin(BAUD_RATE);
//  mySerial.begin(BAUD_RATE);
  AFMSright.begin();              // initialize motor shield for right-side wheels
  AFMSleft.begin();               // initialize motor shield for left-side wheels
  halt(); // make sure the robot isn't moving right now.
  
  while(!Serial) {
    // wait for Serial to connect.
  }

  Serial.println("Blink.begin()...");
  Blynk.begin(auth, ssid, pass);  // initialize Blynk setup on system-side
  Serial.println("Blink.begin()...done");

  

//  // initialize logging
//  
//  Serial.println("Setting up logging...");
//  
//  // make sure that the default chip select pin is set to
//  // output, even if you don't use it:
//  pinMode(chipSelect, OUTPUT);
//  
//  // see if the card is present and can be initialized:
//  if (!SD.begin(10, 11, 12, 13)) {
//    // error
//  }
//  
//  // create a new file
//  char filename[] = "LOGGER00.CSV";
//  for (uint8_t i = 0; i < 100; i++) {
//    filename[6] = i/10 + '0';
//    filename[7] = i%10 + '0';
//    if (! SD.exists(filename)) {
//      // only open a new file if it doesn't exist
//      logfile = SD.open(filename, FILE_WRITE); 
//      break;  // leave the loop!
//    }
//  }
//  
//  // connect to RTC
//  Wire.begin();  
//  if (!RTC.begin()) {
//    logfile.println("RTC failed");
//  }
//  
//  logfile.println("millis,stamp,datetime");
//  
}

volatile bool done = false;
void loop()
{
  Blynk.run();
  
//  // begin self driving.
//  allowSelfDriving = true;

//  // stop self driving.
//  allowSelfDriving = false;
  
  while (allowSelfDriving && !done) {
//    done = true;
    //forwardReverseTest();
    ultrasonicDrivingTest_v1(rollSpeed, millis(), 10000); 
  }
}

// sets the speed of the rover, and the current angle it is turning.
void setSpeedAngle(int16_t rover_speed, int16_t theta) {

  Serial.println("start of setSpeedAngle");
  
  // 0 <= rover_speed <= 255
  // -180 <= theta <= 180
  theta = ((theta + 180) % 360) - 180;
  rover_speed = min(max(0, rover_speed), 255);

  int left_speed = rover_speed * (45 - theta % 90) / 45 ;
  int right_speed = min(255, 2 * rover_speed + left_speed);
  right_speed = min(right_speed, 2 * rover_speed - left_speed);

  if (theta < -90) {
    left_speed = -1*left_speed;
    right_speed = -1*right_speed;
  }
  else if (theta < 0) {
    left_speed = -1*left_speed;
    right_speed = right_speed;
  }
  else if (theta < 90) {
    left_speed = left_speed;
    right_speed = right_speed;
  }
  else {
    left_speed = left_speed;
    right_speed = -1*right_speed;
  }
  
  setLRSpeeds(left_speed, right_speed);
  Serial.println("end of setSpeedAngle");
}


void setLRSpeeds(int left_speed, int right_speed) {

  Serial.println("start of setLRSpeeds");
  
  // -255 <= left_speed <= 255
  left_speed = max(-255, left_speed);
  left_speed = min(left_speed, 255);
  
  // -255 <= right_speed <= 255
  right_speed = max(-255, right_speed);
  right_speed = min(right_speed, 255);

  if (left_speed == 0) {
    leftM1->run(RELEASE);
    leftM2->run(RELEASE);
    leftM3->run(RELEASE);
  }
  else if (left_speed < 0) {
    left_speed = -1 * left_speed;
    leftM1->setSpeed(left_speed);
    leftM2->setSpeed(left_speed);
    leftM3->setSpeed(left_speed);
    leftM1->run(BACKWARD);
    leftM2->run(BACKWARD);
    leftM3->run(BACKWARD);
  }
  else {
    leftM1->setSpeed(left_speed);
    leftM2->setSpeed(left_speed);
    leftM3->setSpeed(left_speed);
    leftM1->run(FORWARD);
    leftM2->run(FORWARD);
    leftM3->run(FORWARD);
  }

  if (right_speed == 0) {
    rightM1->run(RELEASE);
    rightM2->run(RELEASE);
    rightM3->run(RELEASE);
  }
  else if (right_speed < 0) {
    right_speed = -1 * right_speed;
    rightM1->setSpeed(right_speed);
    rightM2->setSpeed(right_speed);
    rightM3->setSpeed(right_speed);
    rightM1->run(BACKWARD);
    rightM2->run(BACKWARD);
    rightM3->run(BACKWARD);
  }
  else {
    rightM1->setSpeed(right_speed);
    rightM2->setSpeed(right_speed);
    rightM3->setSpeed(right_speed);
    rightM1->run(FORWARD);
    rightM2->run(FORWARD);
    rightM3->run(FORWARD);
  }

  Serial.println("in setLRSpeeds: callin Blynk.run()");
  Blynk.run();
  Serial.println("in setLRSpeeds: finished calling Blynk.run()");
  
  Serial.println("end of setLRSpeeds");
}

void halt()
{
  // "release" motors from running (i.e. stop powering them)
  rightM1->run(RELEASE);
  rightM2->run(RELEASE);
  rightM3->run(RELEASE);

  leftM1->run(RELEASE);
  leftM2->run(RELEASE);
  leftM3->run(RELEASE);
}

void forward()
{
  rightM1->setSpeed(rollSpeed);
  rightM2->setSpeed(rollSpeed);
  rightM3->setSpeed(rollSpeed);
  leftM1->setSpeed(rollSpeed);
  leftM2->setSpeed(rollSpeed);
  leftM3->setSpeed(rollSpeed);
  
  rightM1->run(FORWARD);
  rightM2->run(FORWARD);
  rightM3->run(FORWARD);
  leftM1->run(FORWARD);
  leftM2->run(FORWARD);
  leftM3->run(FORWARD);
}

void backward()
{
  rightM1->setSpeed(rollSpeed);
  rightM2->setSpeed(rollSpeed);
  rightM3->setSpeed(rollSpeed);
  leftM1->setSpeed(rollSpeed);
  leftM2->setSpeed(rollSpeed);
  leftM3->setSpeed(rollSpeed);
  
  rightM1->run(BACKWARD);
  rightM2->run(BACKWARD);
  rightM3->run(BACKWARD);
  leftM1->run(BACKWARD);
  leftM2->run(BACKWARD);
  leftM3->run(BACKWARD);
}

void left()
{
  leftM1->setSpeed(adjITSpeed);
  leftM2->setSpeed(adjITSpeed);
  leftM3->setSpeed(adjITSpeed);
  rightM1->setSpeed(adjOTSpeed);
  rightM2->setSpeed(adjOTSpeed);
  rightM3->setSpeed(adjOTSpeed);
  
  rightM1->run(FORWARD);
  rightM2->run(FORWARD);
  rightM3->run(FORWARD);
  leftM1->run(FORWARD);
  leftM2->run(FORWARD);
  leftM3->run(FORWARD);
}

void right()
{
  rightM1->setSpeed(ITSpeed);
  rightM2->setSpeed(ITSpeed);
  rightM3->setSpeed(ITSpeed);
  leftM1->setSpeed(OTSpeed);
  leftM2->setSpeed(OTSpeed);
  leftM3->setSpeed(OTSpeed);
  
  rightM1->run(FORWARD);
  rightM2->run(FORWARD);
  rightM3->run(FORWARD);
  leftM1->run(FORWARD);
  leftM2->run(FORWARD);
  leftM3->run(FORWARD);
}

/*
void right_90()
{
  right();
  delay(2000);  // turn right for 2 seconds
  halt();
}
*/
void forwardReverseTest() {
  // Forward-Reverse test
  int16_t rover_speed = 255;
  int16_t theta = 0;

  setSpeedAngle(rover_speed, theta);
  delay(1000);
  
  halt();
  delay(2000);
  
  theta = 180;
  setSpeedAngle(rover_speed, theta);
  delay(1000);
  
  halt();
}


void ultrasonicDrivingTest_v1(int16_t rover_speed, uint32_t start_time, uint32_t RUN_TIME_LIMIT_MILLIS) {
  Serial.println("STARTED THE LOOP");
//  while (((millis() - start_time) < RUN_TIME_LIMIT_MILLIS) && allowSelfDriving) {
  while (allowSelfDriving) {

    digitalWrite(LED_PIN, HIGH);
    Serial.println("I'm in the loop!");
    
    static uint64_t last_sonar_update = 0;
    static const int sonar_wait_millis = 29; // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    
//    left_distance = left_sonar.convert_cm(left_sonar.ping_median());
//    right_distance = right_sonar.convert_cm(right_sonar.ping_median());
//    forward_distance = forward_sonar.convert_cm(forward_sonar.ping_median());
    
    if ((millis() - last_sonar_update) > sonar_wait_millis) {
      last_sonar_update = millis();
      left_distance = left_sonar.convert_cm(left_sonar.ping_median());
      right_distance = right_sonar.convert_cm(right_sonar.ping_median());
      forward_distance = forward_sonar.convert_cm(forward_sonar.ping_median());
  
      // record everything.
      if (left_distance > 0) {
//        logCurrentTime();
//        logfile.print("L: ");
//        logfile.println(left_distance);
        
        Serial.print("L: ");
        Serial.println(left_distance);
      }
      if (right_distance > 0) {
//        logCurrentTime();
//        logfile.print("R: ");
//        logfile.println(right_distance);
        
        Serial.print("R: ");
        Serial.println(right_distance);
      }
      if (forward_distance > 0) {
//        logCurrentTime();
//        logfile.print("F: ");
//        logfile.println(forward_distance);
        
        Serial.print("F: ");
        Serial.println(forward_distance);
      }
  
      
      // make a driving decision
  
      // UPPER_TURN_THRESHOLD should not exceed the max distance reportable by sensors.
      int UPPER_TURN_THRESHOLD = 50;
      UPPER_TURN_THRESHOLD = min(UPPER_TURN_THRESHOLD, MAX_DISTANCE);
      
      // For LOWER_TURN_THRESHOLD, about 17cm should be theoretical min. 
      // based on rover size and sensor positions.
      int LOWER_TURN_THRESHOLD = 45; 
  
      // find the minimum non-zero distance reported by our sensors.
      // if all sensors return zero, then set it to a large distance.
      int16_t min_dist = MAX_DISTANCE + 1;
      if (left_distance > 0) {
        min_dist = min(min_dist, left_distance);
      }
      if (right_distance > 0) {
        min_dist = min(min_dist, right_distance);
      }
      if (forward_distance > 0) {
        min_dist = min(min_dist, forward_distance);
      }

      // set angle of turn based upon the minimum reported distance.
      int16_t theta = 0;
      if (min_dist < LOWER_TURN_THRESHOLD) {
        theta = 90;
      }
      else if ((LOWER_TURN_THRESHOLD < min_dist) && (min_dist < UPPER_TURN_THRESHOLD)) {
        // move theta linear from 0 to 90 as
        // min_dist decreases from UPPER_TURN_THRESHOLD to LOWER_TURN_THRESHOLD.

        // point-slope form
        int slope = (0 - 90)/(UPPER_TURN_THRESHOLD - LOWER_TURN_THRESHOLD);
        theta = slope * (min_dist - LOWER_TURN_THRESHOLD) + 90;
      }
      else {
        theta = 0;
      }
      
      Serial.print("rover_speed: ");
      Serial.print(rover_speed);
      Serial.print(", theta: ");
      Serial.println(theta);
      setSpeedAngle(rover_speed, theta);
    }
  }
  halt();
  digitalWrite(LED_PIN, LOW);
  Serial.println("ENDED THE LOOP");
}



BLYNK_WRITE(V0) // move forward
{
  if (param[0])
     forward();
  else
    halt();
}

BLYNK_WRITE(V1) // move backward
{
  if (param[0])
     backward();
  else
    halt();
}

BLYNK_WRITE(V2) // turn right
{
  if (param[0])
     right();
  else
    halt();
}

BLYNK_WRITE(V3) // turn left
{
  if (param[0])
     left();
  else
    halt();
}

BLYNK_WRITE(V4) 
{
  if (param[0]) {
    halt();
    allowSelfDriving = true;
  }
  else {
    halt();
    allowSelfDriving = false;
    halt();
//    done = false;
    halt(); // just a little paranoid
  }
}

BLYNK_WRITE(V5) // turn right 90 degrees
{
  if (param[0]) {
    halt();
    allowSelfDriving = false;
    halt(); 
  }
  else {
    halt();
  }
}

