/* 
 * rosserial Time and TF Example
 * Publishes a transform at current time
 */

#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <PinChangeInt.h>



ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

const byte pin_aG = 3;   //for encoder pulse A
const byte pin_bG = 8;   //for encoder pulse B
const byte pin_aD = 2;   //for encoder pulse A
const byte pin_bD = 7;   //for encoder pulse B

const byte pin_fwdG = 5; //for H-bridge: run motor forward - IN1
const byte pin_bwdG = 6; //for H-bridge: run motor backward -IN2
const byte pin_fwdD = 9; //for H-bridge: run motor forward - IN 1
const byte pin_bwdD = 10; //for H-bridge: run motor backward- IN 2


double encoderAG = 0;
double encoderBG = 0;
double encoderAD = 0;
double encoderBD = 0;

unsigned long previousMillis=0;

double w_r=0, w_l=0;
double x=0.0;
double y=0.0;
double theta =0.0;

double distanceD=0;
double distanceG=0;
double distanceC=0;

char base_link[] = "/base_link";
char odom[] = "/odom";

int lowSpeed = 200;
int highSpeed = 50;
double speed_ang=0, speed_lin=0;
double wheel_rad = 0.035, wheel_sep = 0.24;


void messageCb( const geometry_msgs::Twist& msg){ //Define the method that will be called when the topic message is received
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
//Creates a subscriber to the “cmd_vel” topic with the callback function “messageCb” to be called on receiving a message.

void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);


void setup()
{
 pinMode(pin_aG,INPUT_PULLUP);
  pinMode(pin_bG,INPUT_PULLUP);
  pinMode(pin_aD,INPUT_PULLUP);
  pinMode(pin_bD,INPUT_PULLUP);
  
    digitalWrite(pin_fwdG,0);  //stop motor
  digitalWrite(pin_bwdG,0);  //stop motor
  digitalWrite(pin_fwdD,0);  //stop motor
  digitalWrite(pin_bwdD,0);  //stop motor
  
    attachInterrupt(digitalPinToInterrupt(pin_aG), detect_aG,CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_aD), detect_aD, CHANGE);
  attachPinChangeInterrupt(pin_bG, detect_bG,CHANGE);
  attachPinChangeInterrupt(pin_bD, detect_bD, CHANGE);

  
  
  nh.initNode();
  broadcaster.init(nh);
  nh.subscribe(sub); //subscribe/start listening
}

void loop()
{  
   unsigned long currentMillis=millis();
  if ( currentMillis-previousMillis>= 10){ // boucle appellé toutes les 100ms
       previousMillis=currentMillis;
       
  distanceG = ((encoderAG+encoderBG)*2*3.14*wheel_rad)/1920;  // 0.035m=Radius of the wheels
  encoderAG=0;
  encoderBG=0;
  
  distanceD = ((encoderAD+encoderBD)*2*3.14*wheel_rad)/1920;  //
  encoderAD=0;
  encoderBD=0;

  distanceC= (distanceG+distanceD)/2;
  

  
  theta += (distanceD-distanceG)/wheel_sep;// 0.24m =distance between wheels
  x += cos(theta)*distanceC;
  y += sin(theta)*distanceC;
  
 
 // tf odom->base_link
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  
  t.transform.rotation = tf::createQuaternionFromYaw(theta);
  t.header.stamp = nh.now();
  
  broadcaster.sendTransform(t);
  
  MotorL(w_l*10);
 MotorR(w_r*10);
 
  nh.spinOnce();
  
  }
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

void detect_aG() {
  encoderAG+=1; //increasing encoder at new pulse
  //m_direction = digitalRead(pin_b); //read direction of motor
}
void detect_bG() {
  encoderBG+=1; //increasing encoder at new pulse
  //m_direction = digitalRead(pin_b); //read direction of motor
}
void detect_aD() {
  encoderAD+=1; //increasing encoder at new pulse
  //m_direction = digitalRead(pin_b); //read direction of motor
}
void detect_bD() {
  encoderBD+=1; //increasing encoder at new pulse
  //m_direction = digitalRead(pin_b); //read direction of motor
}
