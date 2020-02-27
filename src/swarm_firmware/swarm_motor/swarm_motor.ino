#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
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

// PID Constants
float motor1_p = 500;
float motor1_i = 0;
float motor1_d = 0;
float motor1_punch = 0;
float motor1_deadzone = 0;
float motor1_i_clamp = 0;
float motor1_out_clamp = 3200;
float motor1_last, motor1_accumulate;

float motor2_p = 500;
float motor2_i = 0;
float motor2_d = 0;
float motor2_punch = 0;
float motor2_deadzone = 0;
float motor2_i_clamp = 0;
float motor2_out_clamp = 3200;
float motor2_last, motor2_accumulate;

// Motor Variables
int enc1_cur_ticks = 0;
int enc1_prev_ticks = 0;
int motor1_command = 0;
float motor1_desired_speed = 0;
float motor1_cur_speed = 0;

int enc2_cur_ticks = 0;
int enc2_prev_ticks = 0;
int motor2_command = 0;
float motor2_desired_speed = 0;
float motor2_cur_speed = 0;

// Timer variables
int timer1_freq = 10; 
int timer1_counter;

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

void checkEncoder1(){
    if (digitalRead(chBPin1) == HIGH) {
      enc1_cur_ticks += 1;
    } else {
      enc1_cur_ticks -= 1;
    }
}

void checkEncoder2(){
    if (digitalRead(chBPin2) == HIGH) {
      enc2_cur_ticks -= 1;
    } else {
      enc2_cur_ticks += 1;
    }
}
 
// ISR for timer1
// calculate velocity and perform PID update @ 10 Hz
ISR(TIMER1_OVF_vect)
{
  TCNT1 = timer1_counter;   // reset timer

  int enc1_diff = enc1_cur_ticks - enc1_prev_ticks;
  int enc2_diff = enc2_cur_ticks - enc2_prev_ticks;

  motor1_cur_speed = 2 * PI * WHEEL_RADIUS * (enc1_diff / TICKS_PER_REV) / 0.1;
  motor2_cur_speed = 2 * PI * WHEEL_RADIUS * (enc2_diff / TICKS_PER_REV) / 0.1;

  enc1_prev_ticks = enc1_cur_ticks;
  enc2_prev_ticks = enc2_cur_ticks;

  if (motor1_desired_speed == 0){
    motor1_command = 0;
  }
  else {
    int motor1_pid =  pid(motor1_desired_speed, motor1_cur_speed, 
                    motor1_p, motor1_i, motor1_d, 
                    motor1_i_clamp, motor1_out_clamp, motor1_punch, motor1_deadzone, 
                    &motor1_last, &motor1_accumulate);  
                    
    motor1_command += motor1_pid;
  }

  if (motor2_desired_speed == 0){
    motor2_command = 0;
  }
  else{
    int motor2_pid =  pid(motor2_desired_speed, motor2_cur_speed, 
                    motor2_p, motor2_i, motor2_d, 
                    motor2_i_clamp, motor2_out_clamp, motor2_punch, motor2_deadzone, 
                    &motor2_last, &motor2_accumulate);
                    
    motor2_command += motor2_pid;
  }

//  setMotorSpeed(motor1_command, 1);
//  setMotorSpeed(motor2_command, 2);

  Serial.print(enc1_cur_ticks);
  Serial.print(",");
  Serial.print(enc2_cur_ticks);
  Serial.print(",");
  Serial.print(motor1_cur_speed);
  Serial.print(",");
  Serial.print(motor2_cur_speed);
  Serial.print("\n");
}

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

  // setup timers for velocity calculation
  cli();//stop interrupts

  TCCR1A = 0;
  TCCR1B = 0;
  timer1_counter = 65536 - (16000000 / 256 / timer1_freq);   // preload timer 65536 - 16MHz/256/2Hz (34286 for 0.5sec) (59286 for 0.1sec)
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt

  sei();//allow interrupts

  // initialize Serial w/ baud 9600
  Serial.begin(115200);
}
 
void loop()
{    
  while(Serial.available() > 0){
    // read velocities
    motor1_desired_speed = Serial.parseFloat();
    motor2_desired_speed = Serial.parseFloat();
    
    char r = Serial.read();
    if(r == '\n'){}

    setMotorSpeed(motor1_desired_speed, 1);
    setMotorSpeed(motor2_desired_speed, 2);
  }  
}
