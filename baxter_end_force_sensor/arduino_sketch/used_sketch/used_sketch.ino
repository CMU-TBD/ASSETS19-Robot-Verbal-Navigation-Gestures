/*
 * rosserial Temperature Sensor Example
 * 
 * This tutorial demonstrates the usage of the
 * Sparkfun TMP102 Digital Temperature Breakout board
 * http://www.sparkfun.com/products/9418
 * 
 * Source Code Based off of:
 * http://wiring.org.co/learning/libraries/tmp102sparkfun.html
 */
/* FSR simple testing sketch. 
 
Connect one end of FSR to power, the other end to Analog 0.
Then connect one end of a 10K resistor from Analog 0 to ground 
 
For more information see www.ladyada.net/learn/sensors/fsr.html */
 
#include <ros.h>
#include <std_msgs/Float32.h>


ros::NodeHandle  nh;

std_msgs::Float32 force_msg;
ros::Publisher pub_force("baxter_force_sensor_left", &force_msg);
int fsrPin = 0;     // the FSR and 10K pulldown are connected to a0
int fsrReading;     // the analog reading from the FSR resistor divider
 
void setup(void) {

  nh.initNode();
  nh.advertise(pub_force);
  // We'll send debugging information via the Serial monitor
  //Serial.begin(9600);   
}

long publisher_timer;
long curr_time;
   
void loop(void) {

  curr_time = millis();
  if (curr_time > publisher_timer) {

    fsrReading = analogRead(fsrPin);  
    force_msg.data = fsrReading;
    pub_force.publish(&force_msg);

    //Debugging
//    Serial.print("Analog reading = ");
//    Serial.print(fsrReading);     // the raw analog reading

    // wait for certain time to make sure its 50Hz
    publisher_timer = curr_time + 20;
  }
  nh.spinOnce();
} 
