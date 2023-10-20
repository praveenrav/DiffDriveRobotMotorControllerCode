#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>

#define PI 3.14159265359
#define GEAR_RATIO 2248.46
#define WHEEL_SEPARATION 0.2395 // Wheel separation distance in meters
#define WHEEL_RADIUS 0.04 // Wheel radius in meters

/*
#define in1 17
#define in2 16
#define PWM1 9
#define enc1_A 21
#define enc1_B 19

#define in3 15
#define in4 14
#define PWM2 10
#define enc2_A 2
#define enc2_B 3
*/

bool toggle = false;

// Motor 1 Pins:
const int in1 = 17;
const int in2 = 16;
const int PWM1 = 9;
const int enc1_A = 21;
const int enc1_B = 19;

// Motor 2 Pins:
const int in3 = 15;
const int in4 = 14;
const int PWM2 = 10;
const int enc2_A = 2;
const int enc2_B = 3;

static int8_t rot_enc_table_CCW[] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0}; // Denotes CCW as positive
static int8_t rot_enc_table_CW[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0}; // Denotes CW as positive
int8_t old_AB1 = 0;
int8_t old_AB2 = 0;

long enc1_count = 0;
long enc2_count = 0;

long enc1_count_prev = 0;
long enc2_count_prev = 0;
long enc1_count_cur = 0;
long enc2_count_cur = 0;

double Kp1 = 250;
double Ki1 = 7.5;
double Kd1 = 7.5;

double Kp2 = 275;
double Ki2 = 7.5;
double Kd2 = 7.5;


int sample_time = 1000; // 1 second sample time

double time_cur;
double time_prev = 0;

double time_cur_PID;
double time_prev_PID = 0;

double vel1_des = 0;
double vel2_des = 0;

double vel1_calc_PID;
double vel1_calc_prev;
double vel2_calc_PID;
double vel2_calc_prev;

// State variables calculated during each loop execution:
double vel1_calc_gen;
double vel2_calc_gen;
double lin_vel_calc_gen;
double ang_vel_calc_gen;
double x_vel_calc_gen;
double y_vel_calc_gen;
double x_calc_gen;
double y_calc_gen;
double ang_calc_gen;

double vel_calc;
double ang_calc;
double x_calc;
double y_calc;

double err1_cur;
double integ1_term;
double deriv1_term;
int PWM1_calc = 0;

double err2_cur;
double integ2_term;
double deriv2_term;
int PWM2_calc = 0;

bool isPIDActive = false;
bool isPIDActive_prev = false;

double time_start;
double time_current;

bool vel_sel = false;


rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__String msg;
std_msgs__msg__String msg2;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

//twist message cb
void calcDesVel(const void *msgin) {
  // Subscription callback method to use input Twist messages and differential-drive kinematics to calculate the desired robot wheel velocities:
  
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;

  //char msg_str[] = {*msg->data.data};
  //const char s[3] = "; ";
  //char* lin_vel = strtok(msg_str, s); 
  //char* ang_vel = strtok(NULL, s);

  //double lin_vel_des = atof(lin_vel);
  //double ang_vel_des = atof(ang_vel);
  //double ICR_rad = (lin_vel_des)/(ang_vel_des);
  //double ICR_rad = (msg->linear.x)/(msg->angular.z);

  //vel1_des = ang_vel_des * (ICR_rad - (0.5 * WHEEL_SEPARATION));
  //vel2_des = ang_vel_des * (ICR_rad + (0.5 * WHEEL_SEPARATION));

  sprintf(msg2.data.data, "%.5f; %.5f", vel1_des, vel2_des);
  msg2.data.size = strlen(msg2.data.data);
  
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

}


void setup(){
  
  set_microros_transports();

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(enc1_A, INPUT);
  pinMode(enc1_B, INPUT);

  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(enc2_A, INPUT);
  pinMode(enc2_B, INPUT);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  

  attachInterrupt(digitalPinToInterrupt(enc1_A), enc1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc1_B), enc1_ISR, CHANGE);

  attachInterrupt(digitalPinToInterrupt(enc2_A), enc2_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2_B), enc2_ISR, CHANGE);


  time_start = millis();

  delay(2000);
  

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "motor_controller", "", &support));


  // Create subscriber
  RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "motor_vels"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "motor_wheel_vels"));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &calcDesVel, ON_NEW_DATA));

  msg2.data.data = (char * ) malloc(200 * sizeof(char));
  msg2.data.size = 0;
  msg2.data.capacity = 200;

}

void loop() {
  
  time_current = millis();

  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  // Calculating current motor velocity at beginning of loop:
  calcCurVel();

  // Computing PID Output values and performing PID control on motors if activated:
  computePID();

  togglePID();

  /*
  if((time_current - time_start) > 2000)
  {
    if(!isPIDActive)
    {

    }

  }
  */


}

void motor1PWM(int amp)
{
  // amp - amplitude of PWM signal

  if(amp > 0)
  {
    // CCW Motion of Motor 1:
    analogWrite(PWM1, amp);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);    
  }
  else if(amp < 0)
  {
    // CW Motion of Motor 1:
    amp = -1 * amp; // Causing the amplitude to become positive
    analogWrite(PWM1, amp);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);  
  }

}

void motor2PWM(int amp)
{
  // amp - amplitude of PWM signal

  if(amp > 0)
  {
    // CW Motion of Motor 2:
    analogWrite(PWM2, amp);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);    
  }
  else if(amp < 0)
  {
    // CCW Motion of Motor 2:
    amp = -1 * amp; // Causing the amplitude to become positive
    analogWrite(PWM2, amp);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);  
  }

}



void computePID()
{

  // Updating current data:
  time_cur_PID = millis();
  

  // Ensuring the sampling rate is 
  if((((int) time_cur_PID - time_prev_PID) < 200) || !isPIDActive)
  {
    return;
  }

  // Updating current velocity in rev/min:
  vel1_calc_PID = vel1_calc_gen;
  vel2_calc_PID = vel2_calc_gen;

  // Updating current velocity and calculating errors:
  err1_cur = vel1_des - vel1_calc_PID;
  err2_cur = vel2_des - vel2_calc_PID;


  deriv1_term = 5 * Kd1 * (vel1_calc_prev - vel1_calc_PID); // Calculating derivative term for motor 1
  deriv2_term = 5 * Kd2 * (vel2_calc_prev - vel2_calc_PID); // Calculating derivative term for motor 2

  // Calculating integral term for motor 1, accounting for integral windup:
  if(integ1_term > 255)
  {
    integ1_term = 255;
  }
  else if(integ1_term < -255)
  {
    integ1_term = -255;
  }
  
  // Calculating integral term for motor 2, accounting for integral windup:
  if(integ2_term > 255)
  {
    integ2_term = 255;
  }
  else if(integ2_term < -255)
  {
    integ2_term = -255;
  }

  // Calculating control effort:
  PWM1_calc += (int) ((Kp1 * err1_cur) + integ1_term + deriv1_term);
  PWM2_calc += (int) ((Kp2 * err2_cur) + integ2_term + deriv2_term);


  // Saturating PWM signals, if necessary:
  PWM1_calc = satPWMsig(PWM1_calc);
  PWM2_calc = satPWMsig(PWM2_calc);

  motor1PWM(PWM1_calc);
  motor2PWM(PWM2_calc);

  // Calculating integral terms for Motors 1 and 2:
  integ1_term += (Ki1 * err1_cur) * 0.2;
  integ2_term += (Ki2 * err2_cur) * 0.2;

  // Updating previous data:
  vel1_calc_prev = vel1_calc_PID;
  vel2_calc_prev = vel2_calc_PID;
  time_prev_PID = time_cur_PID;

}

void togglePID()
{

  isPIDActive = !isPIDActive;

  // Switching from manual mode to auto mode:
  if(isPIDActive && !isPIDActive_prev)
  {
    Initialize();
  }
  isPIDActive_prev = isPIDActive;


}

void Initialize()
{
  vel1_calc_prev = vel1_calc_gen;
  vel2_calc_prev = vel2_calc_gen;

  // Calculating integral terms for motor 1 and 2:
  integ1_term = 0;
  integ2_term = 0;


}

void calcCurVel()
{
  time_cur = millis();

  if((time_cur - time_prev) > 100)
  {
    // Wheel velocities in m/s:
    vel1_calc_gen = (double) ((enc1_count - enc1_count_prev) * (2 * PI * WHEEL_RADIUS))/(GEAR_RATIO * 0.001 * (time_cur - time_prev));
    vel2_calc_gen = (double) ((enc2_count - enc2_count_prev) * (2 * PI * WHEEL_RADIUS))/(GEAR_RATIO * 0.001 * (time_cur - time_prev));

    // Linear and angular velocities in m/s and rad/s, respectively:
    lin_vel_calc_gen = 0.5 * (vel1_calc_gen + vel2_calc_gen);
    ang_vel_calc_gen = (vel2_calc_gen - vel1_calc_gen)/WHEEL_SEPARATION;

    // Calculating yaw angle of robot:
    ang_calc_gen += ang_vel_calc_gen * 0.001 * (time_cur - time_prev);

    // Calculating x- and y- velocities:
    x_vel_calc_gen = lin_vel_calc_gen * cos(ang_calc_gen);
    y_vel_calc_gen = lin_vel_calc_gen * sin(ang_calc_gen);
    
    // Calculating robot's x- and y- positions:
    x_calc_gen += x_vel_calc_gen * 0.001 * (time_cur - time_prev);
    y_calc_gen += y_vel_calc_gen * 0.001 * (time_cur - time_prev);


  
    /*
    Serial.print("Motor 1 Velocity: ");
    Serial.println(vel1_calc_gen);
    Serial.print("Motor 2 Velocity: ");
    Serial.println(vel2_calc_gen);
    Serial.println();
    */

    // Updating previous values:
    enc1_count_prev = enc1_count;
    enc2_count_prev = enc2_count;
    time_prev = time_cur;
  }



}


int satPWMsig(int PWM_sig)
{

  if(PWM_sig > 255)
  {
    return 255;
  }
  else if(PWM_sig < -255)
  {
    return -255;
  }
  else
  {
    return PWM_sig;
  }

}

void enc1_ISR()
{

  old_AB1 <<= 2;

  if(digitalRead(enc1_A)) // Determines if A is high
  {
    old_AB1 |= 0x02;
  }

  if(digitalRead(enc1_B)) // Determines if B is high
  {
    old_AB1 |= 0x01;
  }

  enc1_count = enc1_count + rot_enc_table_CCW[old_AB1 & 0x0f];

}


void enc2_ISR()
{
  old_AB2 <<= 2;

  if(digitalRead(enc2_A)) // Determines if A is high
  {
    old_AB2 |= 0x02;
  }

  if(digitalRead(enc2_B)) // Determines if B is high
  {
    old_AB2 |= 0x01;
  }
  enc2_count = enc2_count + rot_enc_table_CW[old_AB2 & 0x0f];

}

