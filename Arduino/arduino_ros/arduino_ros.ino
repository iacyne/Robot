
#include <ros.h>
#include <geometry_msgs/Twist.h>



const byte pin_fwdG = 5; //for H-bridge: run motor forward
const byte pin_bwdG = 6; //for H-bridge: run motor backward

const byte pin_fwdD = 9; //for H-bridge: run motor forward
const byte pin_bwdD = 10; //for H-bridge: run motor backward

double w_r=0, w_l=0;
//wheel_rad is the wheel radius ,wheel_sep is dist between wheels
double wheel_rad = 0.0325, wheel_sep = 0.295;

ros::NodeHandle nh;
//makes this program as a node in ROS . It’s a node object.

int lowSpeed = 200;
int highSpeed = 50;
double speed_ang=0, speed_lin=0;


void messageCb( const geometry_msgs::Twist& msg){ //Define the method that will be called when the topic message is received
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
//Creates a subscriber to the “cmd_vel” topic with the callback function “messageCb” to be called on receiving a message.
void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);


void setup(){
 Serial.begin(57600);
 Motors_init();
 nh.initNode();
 nh.subscribe(sub); //subscribe/start listening
}

void loop(){
 MotorL(w_l*10);
 MotorR(w_r*10);
 nh.spinOnce();//checks if there are any callbacks/services to control the update rate of ROS.
}

void Motors_init(){
 pinMode(pin_fwdD,OUTPUT);
  pinMode(pin_bwdD,OUTPUT);
  pinMode(pin_fwdG,OUTPUT);
  pinMode(pin_bwdG,OUTPUT);
  
  digitalWrite(pin_fwdG,0);  //stop motor
  digitalWrite(pin_bwdG,0);  //stop motor
  digitalWrite(pin_fwdD,0);  //stop motor
  digitalWrite(pin_bwdD,0);  //stop motor
}

void MotorL(int Pulse_Width1){
 if (Pulse_Width1 > 0){
     analogWrite(pin_fwdG, Pulse_Width1);
 }
 if (Pulse_Width1 < 0){
     Pulse_Width1=abs(Pulse_Width1);
     analogWrite(pin_bwdG, Pulse_Width1);
 }
 if (Pulse_Width1 == 0){
     analogWrite(pin_fwdG, Pulse_Width1);
     analogWrite(pin_bwdG, Pulse_Width1);
 }
}
void MotorR(int Pulse_Width2){
 if (Pulse_Width2 > 0){
     analogWrite(pin_fwdD, Pulse_Width2);
 
 }
 if (Pulse_Width2 < 0){
     Pulse_Width2=abs(Pulse_Width2);
     analogWrite(pin_bwdD, Pulse_Width2);
   
 }
 if (Pulse_Width2 == 0){
    analogWrite(pin_fwdD, Pulse_Width2);
     analogWrite(pin_bwdD, Pulse_Width2);

 }
}
