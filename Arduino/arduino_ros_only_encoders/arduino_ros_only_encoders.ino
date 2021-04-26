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


#define encodPinAG      2                       // Quadrature encoder A pin
#define encodPinAD      3                      //for encoder pulse A



volatile long encoderPosG = 0;
volatile long encoderLastG=0;
volatile long encoderLastD=0;
volatile long encoderPosD = 0;
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
  pinMode(encodPinAG, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encodPinAD, INPUT_PULLUP);                  // quadrature encoder input A

  attachInterrupt(digitalPinToInterrupt(encodPinAG), encoderG, FALLING);               // update encoder position
  attachInterrupt(digitalPinToInterrupt(encodPinAD), encoderD, FALLING);
  
  nh.initNode();
  broadcaster.init(nh);
}

void loop()
{  
   unsigned long currentMillis=millis();
  if ( currentMillis-previousMillis>= 10){ // boucle appellé toutes les 100ms
       previousMillis=currentMillis;
       
  distanceG = ((encoderPosG-encoderLastG)*2*3.14*0.035)/1920;  // 0.035m=Radius of the wheels
  encoderLastG=encoderPosG;
  
  distanceD = ((encoderPosD-encoderLastD)*2*3.14*0.035)/1920;  //
  encoderLastD=encoderPosD;

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

void encoderG()  {                                     // pulse and direction, direct port reading to save cycles
 if(PIND & B10000000)   encoderPosG ++;
 else  encoderPosG --;
}
void encoderD()  {                                     // pulse and direction, direct port reading to save cycles
 if(PINB & B00000001)   encoderPosD --;
 else  encoderPosD ++;
}
