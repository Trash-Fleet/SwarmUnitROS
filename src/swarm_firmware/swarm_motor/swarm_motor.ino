#include <ros.h>
#include <std_msgs/Int32.h>
//#include <std_msgs/Float32.h>
#include <SoftwareSerial.h>

#define rxPin1 12  // pin 3 connects to motor controller TX  (not used in this example)
#define txPin1 11  // pin 4 connects to motor controller RX
#define rxPin2 9  // pin 3 connects to motor controller TX  (not used in this example)
#define txPin2 8  // pin 4 connects to motor controller RX
#define chAPin1 2 // chA pin on motor encoder
#define chBPin1 4 // chB pin on motor encoder1
#define chAPin2 3 // chA pin on motor encoder
#define chBPin2 5 // chB pin on motor encoder1

#define TICKS_PER_REV 134.4 // ppr of motor encoder
#define WHEEL_RADIUS 0.04 // radius of attached wheel (m)

// MOTOR CONTROLLER HELPER FUNCTIONS
SoftwareSerial smcSerial1 = SoftwareSerial(rxPin1, txPin1);
SoftwareSerial smcSerial2 = SoftwareSerial(rxPin2, txPin2);

// required to allow motors to move
// must be called when millis()controller restarts and after any error
void exitSafeStart()
{
  smcSerial1.write(0x83);
  smcSerial2.write(0x83);
}
 
// speed should be a number from -3200 to 3200
void setMotorSpeed(int speed, int motor)
{
  if (motor == 1){
    if (speed < 0)
    {
      smcSerial1.write(0x86);  // motor reverse command
      speed = -speed;  // make speed positive
    }
    else
    {
      smcSerial1.write(0x85);  // motor forward command
    }
    smcSerial1.write(speed & 0x1F);
    smcSerial1.write(speed >> 5 & 0x7F);
  } 
  else {
    if (speed < 0)
    {
      smcSerial2.write(0x86);  // motor reverse command
      speed = -speed;  // make speed positive
    }
    else
    {
      smcSerial2.write(0x85);  // motor forward command
    }
    smcSerial2.write(speed & 0x1F);
    smcSerial2.write(speed >> 5 & 0x7F);
  }
}

// ROS HELPER FUNCTIONS
ros::NodeHandle nh;
int cur_ticks = 0;
int prev_ticks = 0;
int desired_speed = 0;
int cur_speed = 0;
unsigned long prev_time;

void speed1_callback( const std_msgs::Int32& msg){
  setMotorSpeed(msg.data, 1);
}

void speed2_callback( const std_msgs::Int32& msg){
  setMotorSpeed(msg.data, 2);
}

void checkEncoder1(){
    if (digitalRead(chBPin1) == HIGH) {
      cur_ticks += 1;
    } else {
      cur_ticks -= 1;
    }
}

void checkEncoder2(){
    if (digitalRead(chBPin2) == HIGH) {
      cur_ticks += 1;
    } else {
      cur_ticks -= 1;
    }
}

std_msgs::Int32 enc_msg;
ros::Subscriber<std_msgs::Int32> sub1("motor1_vel", &speed1_callback);
ros::Subscriber<std_msgs::Int32> sub2("motor2_vel", &speed2_callback);
ros::Publisher pub("enc1", &enc_msg);
 
void setup()
{
  pinMode(chAPin1, INPUT);
  pinMode(chBPin1, INPUT);
  pinMode(chAPin2, INPUT);
  pinMode(chBPin2, INPUT);
  
  // Initialize software serial object with baud rate of 19.2 kbps.
  smcSerial1.begin(19200);
  smcSerial2.begin(19200);
 
  // The Simple Motor Controller must be running for at least 1 ms
  // before we try to send serial data, so we delay here for 5 ms.
  delay(5);
 
  // If the Simple Motor Controller has automatic baud detection
  // enabled, we first need to send it the byte 0xAA (170 in decimal)
  // so that it can learn the baud rate.
  smcSerial1.write(0xAA);
  smcSerial2.write(0xAA);
 
  // Next we need to send the Exit Safe Start command, which
  // clears the safe-start violation and lets the motor run.
  exitSafeStart();

  attachInterrupt(digitalPinToInterrupt(chAPin1), checkEncoder1, RISING); 
  attachInterrupt(digitalPinToInterrupt(chAPin2), checkEncoder2, RISING); 
  prev_time = millis();

  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.advertise(pub);
}
 
void loop()
{
//  if (millis() - prev_time > 100){
//    unsigned long cur_time = millis();
//    int tmp_ticks = cur_ticks;
//    float cur_ang_vel = (tmp_ticks - prev_ticks)/TICKS_PER_REV/(cur_time - prev_time)*1000;
//    float cur_vel = cur_ang_vel * WHEEL_RADIUS;  
//    
//    prev_time = cur_time;
//    prev_ticks = tmp_ticks;
//
//    if ((cur_vel - de
//    
//    enc_msg.data = cur_vel;
//    pub.publish(&enc_msg);
//  }
  
  nh.spinOnce();
  delay(1);
}
