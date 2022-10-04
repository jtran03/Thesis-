///////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <FastLED.h>
#include <ros.h> 
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define NUM_LEDS 60
#define DATA_PIN 4
#define CLOCK_PIN 13
#define BRIGHTNESS 50
#define LED_TYPE WS2812
#define COLOUR_ORDER GRB
#define MAX_DELAY 750
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define RED CRGB::Red
#define CYAN CRGB::Cyan
#define BLACK CRGB::Black
#define GREEN CRGB::Green 
#define ORANGERED CRGB::OrangeRed
#define BLUE CRGB::Blue
#define WHITE CRGB::White
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define INITIALISATION_STATE 0
#define EMERGENCY_STATE 10
#define WAITING_STATE 20
#define AUTONOMOUS_STATE 30
#define MANUAL_STATE 40
#define REVERSE_STATE 50 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
int LEFT_START = 0; 
int LEFT_END = 10; 
int FRONT_START = 11; 
int FRONT_END = 20; 
int RIGHT_START = 21; 
int RIGHT_END = 30; 
int BACK_START = 31; 
int BACK_END = 40; 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
CRGB leds[NUM_LEDS];
ros::NodeHandle nh;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
int STATE = 0; 
int PREV_STATE = 0; 
float angVel = 0; 
unsigned long timeNow = 0; 
int ledBrightness = 0; 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* CMD Call back Function */
void cmd_CB( const std_msgs::Float32& cmdMsg){
  
  /* Capture Subscribed Message */
  angVel = cmdMsg.data; 
}

/* STATE Call back Function */
void state_CB( const std_msgs::Int32& stateMsg){
  
  /* Capture Subscribed Message */
  STATE = stateMsg.data; 
}

/* STATE Call back Function */
void led_CB( const std_msgs::Int32& ledMsg){
  
  /* Capture Subscribed Message */
  ledBrightness = ledMsg.data; 
  FastLED.setBrightness(ledBrightness); 
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Subscriber nodes */
ros::Subscriber<std_msgs::Float32> cmd_SUB("/amr/cmdInput", &cmd_CB );
ros::Subscriber<std_msgs::Int32> state_SUB("/amr/State", &state_CB );
ros::Subscriber<std_msgs::Int32> led_SUB("/amr/led/brightness", &led_CB );
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {

  // ROS
  nh.initNode();
  nh.subscribe(cmd_SUB);  
  nh.subscribe(state_SUB); 
  nh.subscribe(led_SUB); 

  
  // FAST LED
  delay(2000);
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOUR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  // Left Indicator State
  if (angVel > 0){
      goodDelay(MAX_DELAY - abs(angVel*500)); 
      turnOnLEDs(LEFT_START, LEFT_END, ORANGERED, 20); 
      goodDelay(MAX_DELAY - abs(angVel*500));  
      turnOffLEDS(LEFT_START, LEFT_END); 
  }

  // Right Indicator State
  if (angVel < 0){
      goodDelay(MAX_DELAY - abs(angVel*500)); 
      turnOnLEDs(RIGHT_START, RIGHT_END, ORANGERED, 20); 
      goodDelay(MAX_DELAY - abs(angVel*500));  
      turnOffLEDS(RIGHT_START, RIGHT_END); 
  }

  // Check if are WAITING_STATE 
  if (STATE == WAITING_STATE)
  {
      if (STATE != PREV_STATE){
        turnOnLEDs(0, NUM_LEDS, CYAN, 0); 
      }
      
      PREV_STATE = STATE; 
  }

  // Check if are AUTONOMOUS_STATE 
  if (STATE > AUTONOMOUS_STATE && STATE < AUTONOMOUS_STATE + 7 && STATE != PREV_STATE)
  {
      turnOnLEDs(0, NUM_LEDS, GREEN, 10); 
      goodDelay(MAX_DELAY);
      turnOffLEDS(0, NUM_LEDS); 
      goodDelay(MAX_DELAY);
      PREV_STATE = STATE; 
  }

  // Check if are MANUAL_STATE 
  if (STATE == MANUAL_STATE && STATE != PREV_STATE)
  {
      turnOnLEDs(0, NUM_LEDS, WHITE, 10); 
      goodDelay(MAX_DELAY);
      turnOffLEDS(0, NUM_LEDS); 
      goodDelay(MAX_DELAY);
      PREV_STATE = STATE; 
  }

  // Check if are EMERGENCY_STATE 
  if (STATE == EMERGENCY_STATE)
  {
      
      if (STATE != PREV_STATE){
        turnOnLEDs(0, NUM_LEDS, RED, 0); 
      }
      
      PREV_STATE = STATE; 
  }
  
  nh.spinOnce(); 
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void turnOnLEDs(int startIndex, int endIndex, CRGB::HTMLColorCode Colour, int delay)
{

   // Move a single white led 
   for(int whiteLed = startIndex; whiteLed < endIndex; whiteLed = whiteLed + 1) {

      // Turn our current led on to white, then show the leds
      leds[whiteLed] = Colour;
      
      // Show the leds (only one of which is set to white, from above)
      FastLED.show();

      // Delay
      goodDelay(delay);  
      
   };
};
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void goodDelay(int delayAmount)
{
    unsigned long currentTime = millis(); 
    while(millis() - currentTime < delayAmount); 
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void turnOffLEDS(int startIndex, int endIndex)
{

   // Move a single white led 
   for(int index = startIndex; index < endIndex; index = index + 1) {

      // Turn our current led on to white, then show the leds
      leds[index] = CRGB::Black;
      
      // Show the leds (only one of which is set to white, from above)
      FastLED.show();  

      goodDelay(1); 
   };
};
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
