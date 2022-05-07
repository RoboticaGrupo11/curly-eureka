//AGV Machine - Vinay Lanka

//Import Motor - Cytron SPG30E-30K
#include "Motor.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <PID_v1.h>
#include <std_msgs/Int16.h>
#include <ros/time.h>

#include <Servo.h>
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

Servo myservo;
Servo myservo2;

int pos = 0;
int pos2 = 0;

ros::NodeHandle  nh;

int updatenh=0;

#define LOOPTIME 10

Motor right(11,10,7,2);
Motor left(5,6,3,4);

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2

double left_kp = 3.8 , left_ki = 0 , left_kd = 0.0;             // modify for optimal performance
double right_kp = 3.2 , right_ki = 0 , right_kd = 0.0;

float demandx=0;
float demandz=0;

double demand_speed_left;
double demand_speed_right;

double right_input = 0, right_output = 0, right_setpoint = 0;
PID rightPID(&right_input, &right_output, &right_setpoint, right_kp, right_ki, right_kd, DIRECT);  

double left_input = 0, left_output = 0, left_setpoint = 0;
PID leftPID(&left_input, &left_output, &left_setpoint, left_kp, left_ki, left_kd, DIRECT);  

unsigned long currentMillis;
unsigned long prevMillis;

float encoder0Diff; 
float encoder1Diff;

float encoder0Error;
float encoder1Error;

float encoder0Prev;
float encoder1Prev;

void cmd_vel_cb( const geometry_msgs::Twist& twist){
  demandx = twist.linear.x;
  demandz = twist.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb ); 
std_msgs::Int16 left_wheel_msg;
ros::Publisher left_wheel_pub("lwheel", &left_wheel_msg);

std_msgs::Int16 right_wheel_msg;
ros::Publisher right_wheel_pub("rwheel", &right_wheel_msg);

double pos_act_left = 0;                    //Actual position for wheel in encoder ticks
double pos_act_right = 0;                    


/** SERVOOS**/

ros::NodeHandle nh3;
std_msgs::Int16 str_msg;
ros::Publisher angle("angle", &str_msg);

void messageCb( const std_msgs::String& string){
  nh3.loginfo(string.data);
  if(String(string.data).equals("x")){
    nh3.loginfo("Pinzón es gay");
    while(pos <= 150) { // goes from 0 degrees to 180 degrees
    pos = pos+1;
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    str_msg.data = pos;
    angle.publish(&str_msg);
    delay(15);                       // waits 15ms for the servo to reach the position
    }
   }else if(String(string.data).equals("z")){
    while (pos >=0) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
    pos = pos-1;
    str_msg.data = pos;
    angle.publish(&str_msg);
    }
  }else if(String(string.data).equals("i")){
    if (pos2<180){
      pos2=pos2+30;
      myservo2.write(pos2);
      str_msg.data = pos;
      angle.publish(&str_msg);
    }
  }else if(String(string.data).equals("k")){
    if (pos2>=0){
      pos2=pos2-30;
      myservo2.write(pos2);
      str_msg.data = pos;
      angle.publish(&str_msg);
    }
  }
  }


ros::Subscriber<std_msgs::String> sub("/manipulador", &messageCb );


// ******************SERVO CONTROL ***************++

ros::NodeHandle  nh2;
int d_max = 5;
int grados = 0;
double d = 0;

Servo servo;

void servo_cb( const std_msgs::Float32& cmd_msg){
  d = cmd_msg.data-10;
  if (d >= 0 and d <= 5){
      grados = round((d/d_max)*180);
      servo.write(grados);
  }
}


ros::Subscriber<std_msgs::Float32> sub("servo", servo_cb);

/****************** SETUP ****************/

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(left_wheel_pub);   //prepare to publish speed in ROS topic
  nh.advertise(right_wheel_pub);
//  Serial.begin(115200);
  
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(1);
  rightPID.SetOutputLimits(-100, 100);

  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(1);
  leftPID.SetOutputLimits(-100, 100);
  
//  Serial.println("Basic Encoder Test:");
  attachInterrupt(digitalPinToInterrupt(left.en_a), change_left_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left.en_b), change_left_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right.en_a), change_right_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right.en_b), change_right_b, CHANGE);

  myservo.attach(8);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(9);
  myservo.write(0);
  myservo2.write(0);
  nh3.initNode();
  nh3.subscribe(sub);
  nh2.initNode();
  nh2.subscribe(sub);
  servo.attach(9); //attach it to pin 9
}

void loop() {
  
  nh3.spinOnce();
  nh2.spinOnce();
  currentMillis = millis();
  if (currentMillis - prevMillis >= LOOPTIME){
    prevMillis = currentMillis;

    demand_speed_left = demandx - (demandz*0.099);
    demand_speed_right = demandx + (demandz*0.099);
  
    /*PID controller for speed control
      Base speed being 1 ms and the demand_speed variables controlling it at fractions of the base.
      The PID controller keeps trying to match the difference 
      in encoder counts to match with the required amount, hence controlling the speed. */
    encoder0Diff = encoder0Pos - encoder0Prev; // Get difference between ticks to compute speed
    encoder1Diff = encoder1Pos - encoder1Prev;
    
    pos_act_left = encoder0Pos;
    pos_act_right = encoder1Pos;
  
    encoder0Error = (demand_speed_left*7.12634074)-encoder0Diff; // 3965 ticks in 1m = 39.65 ticks in 10ms, due to the 10 millis loop
    encoder1Error = (demand_speed_right*7.12634074)-encoder1Diff;
  
    encoder0Prev = encoder0Pos; // Saving values
    encoder1Prev = encoder1Pos;
  
    left_setpoint = demand_speed_left*7.12634074;  //Setting required speed as a mul/frac of 1 m/s
    right_setpoint = demand_speed_right*7.12634074;
  
    left_input = encoder0Diff;  //Input to PID controller is the current difference
    right_input = encoder1Diff;
    
    leftPID.Compute();
    left.rotate(left_output);
    rightPID.Compute();
    right.rotate(right_output);
//    Serial.print(encoder0Pos);
//    Serial.print(",");
//    Serial.println(encoder1Pos);
    publishPos(LOOPTIME);
    if(updatenh>10){
      nh.spinOnce();
      updatenh=0;
    }else{
      updatenh++;
    }
  }
}


//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishPos(double time) {
  left_wheel_msg.data = pos_act_left;
  right_wheel_msg.data = pos_act_right;
  left_wheel_pub.publish(&left_wheel_msg);
  right_wheel_pub.publish(&right_wheel_msg);
}


// ************** encoders interrupts **************

// ************** encoder 1 *********************


void change_left_a(){  

  // look for a low-to-high on channel A
  if (digitalRead(left.en_a) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(left.en_b) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(left.en_b) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
 
}

void change_left_b(){  

  // look for a low-to-high on channel B
  if (digitalRead(left.en_b) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(left.en_a) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(left.en_a) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  

}

// ************** encoder 2 *********************

void change_right_a(){  

  // look for a low-to-high on channel A
  if (digitalRead(right.en_a) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(right.en_b) == LOW) {  
      encoder1Pos = encoder1Pos - 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(right.en_b) == HIGH) {   
      encoder1Pos = encoder1Pos - 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;          // CCW
    }
  }
 
}

void change_right_b(){  

  // look for a low-to-high on channel B
  if (digitalRead(right.en_b) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(right.en_a) == HIGH) {  
      encoder1Pos = encoder1Pos - 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(right.en_a) == LOW) {   
      encoder1Pos = encoder1Pos - 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;          // CCW
    }
  }
  

}
