#include "common/mavlink.h"
#include "sbus.h"
#include <Servo.h>
//#define SERIAL_GCS Serial2
#define DEBUG_SERIAL 1 // change to 0 if do not want to print anything out
#define SERIAL_ARDUPILOT Serial1 // Serial1 of teensy is used for Ardupilot 
#define SERIAL_ARDUPILOT_BAUD 115200 // Baudrate
#define MAX_MAVLINK_MESSAGE 1000 // maximum mavlink message
#define MAX_SBUS 1811 //SBUS read max pulse
#define MIN_SBUS 172 //SBUS read min pulse
#define MAX_RC 2000 //Max RC signal want to send to motor, esc
#define MIN_RC 1000 //Min RC signal want to send to motor, esc
#define MOTOR_LEFT_PIN 2 // Connect ESC pulse to pin 2 of teensy
#define MOTOR_RIGHT_PIN 3  // Connect ESC pulse to pin 3 of teensy
#define SERVO_LEFT_PIN 4 // Left servo pin - pin 4 of teensy
#define SERVO_RIGHT_PIN 5 // Right servo pin - pin 4 of teensy
#define CHANNEL_MODE 4 // channel 4 of RC controller is used for mode change: skid or normal
#define CHANNEL_MODE_RC 1500 // threshold whether being in mode skid or normal
int sX = 0; //mapped from SBUS read to X (skid docs file)
int sY = 0; //mapped from SBUS read to Y (skid docs file)
int sYm = 0; //mapped from SBUS read to inverted X (skid docs file)
int sXm = 0; //mapped from SBUS read to inverted Y (skid docs file)
int RPL = 0; //RPL is "right plus left": R+L
int RML = 0; //RML is "right minuc left": R-L
int leftMotor = 0; //Left motor: -100 -> 100
int rightMotor = 0; //Right motor: -100 -> 100
int leftMotor_RC = 0; //Left motor to ESC: from MIN_RC ->  MAX_RC
int rightMotor_RC = 0; //Right motor to ESC: from MIN_RC ->  MAX_RC
int leftServo_RC = 0; //Left servo: from MIN_RC ->  MAX_RC
int rightServo_RC = 0; //Right servo: from MIN_RC ->  MAX_RC
std::array<uint16_t, 16> channels; // channels read from SBUS
const int LED = 13;
SbusRx sbus_rx(&Serial2); //SBUS received from the RC receiver (X8R)
// variables used to output RC pulse
Servo motorL; 
Servo motorR;
Servo servoL;
Servo servoR;

void setup() {
  
  pinMode(LED, OUTPUT);

  //attach the teensy pin to ESC funtionality
  motorL.attach(MOTOR_LEFT_PIN, MIN_RC, MAX_RC);
  motorR.attach(MOTOR_RIGHT_PIN, MIN_RC, MAX_RC);
  servoL.attach(SERVO_LEFT_PIN, MIN_RC, MAX_RC);
  servoR.attach(SERVO_RIGHT_PIN, MIN_RC, MAX_RC);
  delay(1000);
  //begin the serial
  Serial.begin(115200);
  sbus_rx.Begin();

  //initialize the array of channels
  for (int i = 0; i < 16; i++){
    channels[i] = 1000;
  }
}

void loop() {

  if (sbus_rx.Read()) {
    // if channel 4 read from the transmitter is larger than the threshold
    if (sbus_rx.rx_channels()[CHANNEL_MODE] >= CHANNEL_MODE_RC){ 
    
      //sX = map(sbus_rx.rx_channels()[1],MIN_SBUS,MAX_SBUS,100,-100);
      sXm = map(sbus_rx.rx_channels()[1],MIN_SBUS,MAX_SBUS,-100,100);  // X inverted
      sY = map(sbus_rx.rx_channels()[0],MIN_SBUS,MAX_SBUS,-100,100); // Y
      //sYm = map(sbus_rx.rx_channels()[0],MAX_SBUS,MIN_SBUS,-100,100);
  
      RPL = (100 - abs(sXm))*(sY/100) + sY; // R+L
      RML = (100 - abs(sY))*(sXm/100) + sXm; //R-L
      leftMotor = (RPL - RML)/2; //Left motor : -100 -> 100
      rightMotor = (RPL + RML)/2; //Right motor : -100 -> 100
      leftMotor_RC = map(leftMotor,-100,100,MIN_RC,MAX_RC); // mapped left motor from MIN_RC ->  MAX_RC 
      rightMotor_RC = map(rightMotor,-100,100,MIN_RC,MAX_RC);  // mapped left motor from MIN_RC ->  MAX_RC 


      //out put these pin to escs and servos
      motorL.writeMicroseconds(leftMotor_RC);
      motorR.writeMicroseconds(rightMotor_RC);
      servoL.writeMicroseconds((MAX_RC + MIN_RC)/2);
      servoR.writeMicroseconds((MAX_RC + MIN_RC)/2);

      
      //printout for debugging
      if (DEBUG_SERIAL){
        Serial.println("____Skid mode____");
        Serial.print("Left motor: ");
        Serial.println(leftMotor_RC);
        Serial.print("Right motor: ");
        Serial.println(rightMotor_RC);
      }
      
      
    }
    // if not, just mapped the SBUS pulse read to servo pins
    else
    {
      
      leftMotor_RC = map(sbus_rx.rx_channels()[0],MIN_SBUS,MAX_SBUS,MIN_RC,MAX_RC);
      rightMotor_RC = map(sbus_rx.rx_channels()[1],MIN_SBUS,MAX_SBUS,MIN_RC,MAX_RC);
      leftServo_RC = map(sbus_rx.rx_channels()[2],MIN_SBUS,MAX_SBUS,MIN_RC,MAX_RC);
      rightServo_RC = map(sbus_rx.rx_channels()[3],MIN_SBUS,MAX_SBUS,MIN_RC,MAX_RC);
      
      
      //out put these pin to escs and servos
      motorL.writeMicroseconds(leftMotor_RC);
      motorR.writeMicroseconds(rightMotor_RC);
      servoL.writeMicroseconds(leftServo_RC);
      servoR.writeMicroseconds(rightServo_RC);

      //printout for debugging
      if (DEBUG_SERIAL){
        Serial.println("____Normal mode____");
        Serial.print("Channel 1: ");
        Serial.print(sbus_rx.rx_channels()[0]);
        Serial.print(",");
        Serial.print(leftMotor_RC);
        Serial.println();

        Serial.print("Channel 2: ");
        Serial.print(sbus_rx.rx_channels()[1]);
        Serial.print(",");
        Serial.print(rightMotor_RC);
        Serial.println();

        Serial.print("Channel 3: ");
        Serial.print(sbus_rx.rx_channels()[2]);
        Serial.print(",");
        Serial.print(leftServo_RC);
        Serial.println();

        Serial.print("Channel 4: ");
        Serial.print(sbus_rx.rx_channels()[3]);
        Serial.print(",");
        Serial.print(rightServo_RC);
        Serial.println();
      }
    }
    
    delay(10);
  }
  
}
