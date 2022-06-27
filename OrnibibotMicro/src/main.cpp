#include <Arduino.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Thread.h>
#include <ThreadController.h>
#include "SBUS.h"
#include "OrnibibBot.h"

#include <iostream>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <string.h>

// #include <ESP_SBUS.h>  -> have not finished

#define M_PI  3.14159265358979323846f  /* pi */
#define stopStroke 0
#define downStroke 1
#define upStroke 2
#define glide 3

//Left Wing Configuration
#define upStrokeL 1900
#define downStrokeL 1400
// #define midLeft 1600
#define midLeft 60

//Right Wing Configuration
#define upStrokeR 1100
#define downStrokeR 1600
// #define midRight 1400
#define midRight 60

//SBUS
#define SBUS_SPEED  100000
#define Peri1       10


// ThreadController threadController = ThreadController();
// Thread* servoThread = new Thread();

TaskHandle_t Task1;
TaskHandle_t Task2;
SBUS sbus;
OrnibiBot robot;

Servo leftWing;
Servo rightWing;
Servo pitchTail;
Servo rollTail;


const int leftPin=21;
const int rightPin=19;
const int rollPin=18;
const int PitchPin=5;

bool flapMode;

const char* ssid     = "ntlab1802";
const char* password = "hoge1802";
// Set the rosserial socket server IP address
IPAddress server(192,168,30,243);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
// Make a chatter publisher
std_msgs::Int16 str_msg;
std_msgs::Int16 cnt_msg;

volatile double freqFlap=4;
volatile double pTail;
volatile double rTail;
volatile uint16_t counter=0;
bool failSafe;
bool lostFrame;

int _Servo[5];
int* _targetServo = _Servo ;


void sendData(){
    short  sbus_servo_id[16]; 
    char sbus_data[25] = {
      0x0f, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00
    };

    // for (int i = 0; i <= 15; i++) {
    //   sbus_servo_id[i] = (int)(  10.667 * (double)(target_angle[i] + 90.0) + 64);
    // }
    sbus_servo_id[0] = (int)(10.667 * (double)(_targetServo[0] + 90) + 64);
    sbus_servo_id[1] = (int)(10.667 * (double)(_targetServo[1] + 90) + 64);
    sbus_servo_id[2] = (int)(10.667 * (double)(_targetServo[2] + 90) + 64);
    sbus_servo_id[3] = (int)(10.667 * (double)(_targetServo[3] + 90) + 64);
    
    /* sbus送信用に変換した目標角度をS.BUS送信用バッファに詰め込む  */
    sbus_data[0] = 0x0f;
    sbus_data[1] =  sbus_servo_id[0] & 0xff;
    sbus_data[2] = ((sbus_servo_id[0] >> 8) & 0x07 ) | ((sbus_servo_id[1] << 3 ) );
    sbus_data[3] = ((sbus_servo_id[1] >> 5) & 0x3f ) | (sbus_servo_id[2]  << 6);
    sbus_data[4] = ((sbus_servo_id[2] >> 2) & 0xff ) ;
    sbus_data[5] = ((sbus_servo_id[2] >> 10) & 0x01 ) | (sbus_servo_id[3] << 1 )   ;
    sbus_data[6] = ((sbus_servo_id[3] >> 7) & 0x0f ) | (sbus_servo_id[4]  << 4 )   ;
    sbus_data[7] = ((sbus_servo_id[4] >> 4) & 0x7f ) | (sbus_servo_id[5]  << 7 )   ;
    sbus_data[8] = ((sbus_servo_id[5] >> 1) & 0xff ) ;
    sbus_data[9] = ((sbus_servo_id[5] >> 9) & 0x03 ) ;

    /* sbus_dataを25bit分送信 */  
    Serial2.write(sbus_data, 25); 
}

void freq_cb(const geometry_msgs::Twist& flap){
    robot._flapFreq = flap.linear.x;
}

ros::Publisher chatter("chatter", &str_msg);
ros::Publisher cnt("cntr", &cnt_msg);
ros::Subscriber<geometry_msgs::Twist> flap("flapFreq", freq_cb);

volatile double getFlapMs(double freq){
  return (volatile double)(1000/freq);
}

volatile int16_t interpolateFlap(int amplitude, volatile double freq, int time){
    
    return (volatile int16_t) (amplitude * sin(((2*M_PI)/getFlapMs(robot._flapFreq) * time)));

}

void paramUpdate( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    if (nh.connected()) {
    digitalWrite(2, HIGH);
  
    if(flapMode==true && robot._flapFreq>0.01f){

      robot._amplitude = 35;
    
      _targetServo[0] = midLeft+robot.sineFlap();
      _targetServo[1] = midRight-robot.sineFlap();
      _targetServo[2] = 0;
      _targetServo[3] = 0;

      if(robot._time<robot._periode)robot._time++;
      // if(counter<robot._periode)counter++;
      else robot._time=0;  
      



      /*Problem Here
      it causes delay during interpolation*/
      // str_msg.data = robot.flapping;
      // chatter.publish( &str_msg );
    }
  }
  
  else {
    Serial.println("Not Connected");
      _targetServo[0] = midLeft;
      _targetServo[1] = midRight;
      _targetServo[2] = 0;
      _targetServo[3] = 0;
    digitalWrite(2, LOW);

  }
  sbus.setPosition(_targetServo);
  nh.spinOnce();
  delay(1);
  } 
}

void motorUpdate( void * pvParameters ){
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
      // sendData();
      sbus.sendPosition();
      delay(20);
  }
}

void setup()
{
  pinMode(2, OUTPUT);
  Serial.begin(115200);
  leftWing.attach(leftPin, 800, 2200);
  rightWing.attach(rightPin, 800, 2200);
  rollTail.attach(rollPin, 800, 2200);
  pitchTail.attach(PitchPin, 800, 2200);

  sbus.init();

  // Serial2.begin(SBUS_SPEED, SERIAL_8E2, 16, 17, true); 

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());

  // Start to be polite
  nh.advertise(chatter);
  nh.subscribe(flap);
  flapMode=true;
  
  xTaskCreatePinnedToCore(
                    paramUpdate,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */
  delay(500);
  xTaskCreatePinnedToCore(
                    motorUpdate,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 0 */
  delay(500);

  _targetServo[0] = midLeft;
  _targetServo[1] = midRight;
  _targetServo[2] = 0;
  _targetServo[3] = 0;

}

void loop()
{ 
 
}