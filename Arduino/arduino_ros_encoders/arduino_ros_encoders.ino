/* 
 * rosserial Time and TF Example
 * Publishes a transform at current time
 */

#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <PinChangeInt.h>



ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

const byte pin_aG = 3;   //for encoder pulse A
const byte pin_bG = 8;   //for encoder pulse B
const byte pin_aD = 2;   //for encoder pulse A
const byte pin_bD = 7;   //for encoder pulse B


double encoderAG = 0;
double encoderBG = 0;
double encoderAD = 0;
double encoderBD = 0;

unsigned long previousMillis=0;

double x=0.0;
double y=0.0;
double theta =0.0;

double distanceD=0;
double distanceG=0;
double distanceC=0;

char base_link[] = "/base_link";
char odom[] = "/odom";

void setup()
{
 pinMode(pin_aG,INPUT_PULLUP);
  pinMode(pin_bG,INPUT_PULLUP);
  pinMode(pin_aD,INPUT_PULLUP);
  pinMode(pin_bD,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pin_aG), detect_aG,CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_aD), detect_aD, CHANGE);
  attachPinChangeInterrupt(pin_bG, detect_bG,CHANGE);
  attachPinChangeInterrupt(pin_bD, detect_bD, CHANGE);
  
  nh.initNode();
  broadcaster.init(nh);
}

void loop()
{  
   unsigned long currentMillis=millis();
  if ( currentMillis-previousMillis>= 10){ // boucle appellé toutes les 100ms
       previousMillis=currentMillis;
       
  distanceG = ((encoderAG+encoderBG)*2*3.14*0.035)/1920;  // 0.035m=Radius of the wheels
  encoderAG=0;
  encoderBG=0;
  
  distanceD = ((encoderAD+encoderBD)*2*3.14*0.035)/1920;  //
  encoderAD=0;
  encoderBD=0;

  distanceC= (distanceG+distanceD)/2;
  

  
  theta += (distanceD-distanceG)/0.24;// 0.24m =distance between wheels
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
  nh.spinOnce();
  
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
