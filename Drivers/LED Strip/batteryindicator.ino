/*
 * rosserial LED Strip
 * Blinks an LED on callback
 * Blinks LED Strip
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <FastLED.h>

#define LED_PIN     4
#define NUM_LEDS    20
#define BRIGHTNESS  64
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];
ros::NodeHandle  nh;
char LED_STRIP_STATE[5];

/* Call back Function */
void messageCb( const std_msgs::String& toggle_msg){

  /* Blink the LED*/
  digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));

  /* Capture Subscribed Message */
  char state;
  state = toggle_msg.data[0];
  ledstrip(state);
}

/* Subscriber node */
ros::Subscriber<std_msgs::String> sub("toggle_led", &messageCb );


/*########################################################################*/
/* Setup */
/*########################################################################*/
void setup()
{

  delay( 3000 ); // power-up safety delay

  /* ROS Serial Code */
  pinMode(LED_BUILTIN, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);

  /* LED Strip Code */
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  BRIGHTNESS );
}
/*########################################################################*/
/* Loop */
/*########################################################################*/
void loop()
{
  nh.spinOnce();
  delay(1);
}
