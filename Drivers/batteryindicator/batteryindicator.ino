#include <ros.h>
#include <std_msgs/Float32.h>

// Initalise ROS
ros::NodeHandle nh;

// Declare message types
std_msgs::Float32 voltage_msg;
std_msgs::Float32 current_msg;
std_msgs::Float32 status_msg;

// Declare global variables
int v_analogPin = A4; // potentiometer wiper (middle terminal) connected to analog pin 3
float SYSTEM_VOLTAGE = 5;
float ADC_RESOLUTION = 1023;
float WIPER_RATIO = 10000/2000;
float VOLTAGE_SYS_ERROR = 0.06;
float CURRENT_SYS_ERROR = 0.08;
float rawVoltage = 0;
float trueVoltage = 0;
float publisherRate = 1000; // 1Hz
unsigned long timeNow = 0;

// Callback Functions ------------------------------------------------------
void rate_msg_cb( const std_msgs::Float32& status_msg){

    // Convert Hz to mulliseconds
    publisherRate = (1/status_msg.data) * 1000;
}

// Declare publishers or subscribers
ros::Publisher voltage_pub("amr/status/voltage", &voltage_msg);
ros::Publisher current_pub("amr/status/current", &current_msg);
ros::Subscriber<std_msgs::Float32> rate_sub("amr/status/rate", &rate_msg_cb );

// SETUP-------------------------------------------------------------
void setup() {
  nh.initNode();

  // Advertise the topics
  nh.advertise(voltage_pub);
  nh.advertise(current_pub);

  // Subscribe to topics
  nh.subscribe(rate_sub);
}

// LOOP-------------------------------------------------------------
void loop()
{
    // Read data from sensor
    rawVoltage = (analogRead(v_analogPin)*SYSTEM_VOLTAGE)/ADC_RESOLUTION + VOLTAGE_SYS_ERROR;// read the input pin
    rawVoltage = (analogRead(v_analogPin)*SYSTEM_VOLTAGE)/ADC_RESOLUTION;// read the input pin
    trueVoltage = rawVoltage*WIPER_RATIO;

    // Calculate voltage and current
    float voltage = analogRead(A0)*5/1023.0;
    float current = (voltage-2.5)/0.185 + CURRENT_SYS_ERROR;

    // Publish at a specific rate
    if(millis() >= timeNow + publisherRate){

        // Update time
        timeNow = millis();

        // Load message into publishers
        voltage_msg.data = voltage;
        voltage_pub.publish( &voltage_msg );
        current_msg.data = current;
        current_pub.publish( &current_msg );
    }

    nh.spinOnce();
}
