
#include <ros.h>
#include <std_msgs/Int16.h>
 
// Handles startup and shutdown of ROS
ros::NodeHandle nh;


 //Configuración de los motores. 
//Motor 1
#define motorAPin1  5  // Pin 14 de L293
#define motorAPin2 6  // Pin 10 de L293
//Motor 2
#define motorBPin3  10 // Pin  7 de L293
#define motorBPin4  9  // Pin  2 de L293

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 2
#define ENC_IN_RIGHT_A 3
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 7
#define ENC_IN_RIGHT_B 4


//Estado motor 
int estado='c';


// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;
 
// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
 
// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
 
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);
 
// 100ms interval for measurements
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;
 
// Increment the number of ticks
void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
     
    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    }
    else {
      right_wheel_tick_count.data++;  
    }    
  }
  else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    }
    else {
      right_wheel_tick_count.data--;  
    }   
  }
}
 
// Increment the number of ticks
void left_wheel_tick() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);
 
  if(val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }
   
  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    }
    else {
      left_wheel_tick_count.data++;  
    }  
  }
  else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    }
    else {
      left_wheel_tick_count.data--;  
    }   
  }
}
 
void setup() {
  Serial.begin(57600);
  pinMode(motorAPin1, OUTPUT); 
  pinMode(motorAPin2, OUTPUT); 
  pinMode(motorBPin3, OUTPUT);
  pinMode(motorBPin4, OUTPUT);

 
  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
 
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);
 
  // ROS Setup
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
}
 
void loop() {

  /**
  if(Serial.available()>0){        // lee el serial 
    estado = Serial.read();
  }
*/
estado='w';
  if(estado=='w'){           // Boton desplazar al Frente
    go_ahead(0);
  }
  else if(estado=='s'){          // Boton atras
    go_back(0);
  }
  else if(estado=='q'){         // Boton IZQ
    turn_left(0);
  }
  else if(estado=='e'){          // Boton DER 
    turn_right(0);
  } 
  else if(estado=='x'){
  go_stop();
  }
   
  // Record the time
  currentMillis = millis();
 
  // If 100ms have passed, print the number of ticks
  if (currentMillis - previousMillis > interval) {
     
    previousMillis = currentMillis;
     
    rightPub.publish( &right_wheel_tick_count );
    leftPub.publish( &left_wheel_tick_count );
    nh.spinOnce();
  }
}


/**************************** MOTOR CONTROL ****************************/
void go_ahead(int t)  //motor rotate clockwise -->robot go ahead
{
  digitalWrite(motorAPin1, HIGH);
  digitalWrite(motorAPin2, LOW);
  digitalWrite(motorBPin3, HIGH);
  digitalWrite(motorBPin4,LOW);
  if(t!=0){ delay(t); }
}
void go_back(int t) //motor rotate counterclockwise -->robot go back
{
  digitalWrite(motorAPin1, LOW);
  digitalWrite(motorAPin2, HIGH);
  digitalWrite(motorBPin3, LOW);
  digitalWrite(motorBPin4,HIGH); 
  if(t!=0){ delay(t); }
}
void go_stop() //motor brake -->robot stop
{
  digitalWrite(motorAPin1, LOW);
  digitalWrite(motorAPin2, LOW);
  digitalWrite(motorBPin3, LOW);
  digitalWrite(motorBPin4,LOW); 
}
void turn_right(int t)  //left motor rotate clockwise and right motor rotate counterclockwise -->robot turn right
{
  digitalWrite(motorAPin1, LOW);
  digitalWrite(motorAPin2, HIGH);
  digitalWrite(motorBPin3, HIGH);
  digitalWrite(motorBPin4, LOW);
  if(t!=0){ delay(t); }
}
void turn_left(int t) //left motor rotate counterclockwise and right motor rotate clockwise -->robot turn left
{
  digitalWrite(motorAPin1, HIGH);
  digitalWrite(motorAPin2, LOW);
  digitalWrite(motorBPin3, LOW);
  digitalWrite(motorBPin4, HIGH);
  if(t!=0){ delay(t); }
}
