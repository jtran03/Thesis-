#include <ros.h>
#include <std_msgs/UInt32.h>

// Initalise ROS
ros::NodeHandle nh;

// Declare message types
std_msgs::UInt32 eStopMsg;

// Declare global variables
const int eStopPin = 12;

// Callback Functions ------------------------------------------------------
// Return true if emergency stop pressed
bool isEStopPressed(){
  digitalWrite(LED_BUILTIN, digitalRead(eStopPin));
  return digitalRead(eStopPin);
}

// Declare publishers or subscribers
ros::Publisher eStopPub("amr/status/eStopPressed", &eStopMsg);

// SETUP-------------------------------------------------------------

void setup() {
  nh.initNode();

  // Advertise the topics
  nh.advertise(eStopPub);

  // Iniitalise Pins
  pinMode(eStopPin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

// LOOP-------------------------------------------------------------
void loop()
{
    eStopMsg.data = isEStopPressed();
    eStopPub.publish( &eStopMsg );
    nh.spinOnce();
}
