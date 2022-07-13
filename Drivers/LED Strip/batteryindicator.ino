#include <ros.h>
#include <std_msgs/float.h>


ros::NodeHandle  nh;


int v_analogPin = A4; // potentiometer wiper (middle terminal) connected to analog pin 3
                    // outside leads to ground and +5V
float SYSTEM_VOLTAGE = 5;
float ADC_RESOLUTION = 1023;
float WIPER_RATIO = 10000/2000;
float VOLTAGE_SYS_ERROR = 0.06;
float CURRENT_SYS_ERROR = 0.08;
float rawVoltage = 0;
float trueVoltage = 0;


void setup() {
  Serial.begin(9600);           //  setup serial
}

void loop() {
  // VOLTAGE STUFF
  rawVoltage = (analogRead(v_analogPin)*SYSTEM_VOLTAGE)/ADC_RESOLUTION + VOLTAGE_SYS_ERROR;// read the input pin
  trueVoltage = rawVoltage*WIPER_RATIO;


  // CURRENT STUFF
  // put your main code here, to run repeatedly:
  int adc = analogRead(A0);
  float voltage = adc*5/1023.0;
  float current = (voltage-2.5)/0.185 + CURRENT_SYS_ERROR;

  // Debug
  Serial.print("Voltage = ");
  Serial.println(trueVoltage);          // debug value
  Serial.print("Current : ");
  Serial.println(current);
  delay(50);
}
