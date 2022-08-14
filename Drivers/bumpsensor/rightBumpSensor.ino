// 1 = Front Middle
// 2 = Front Right
// 3 = Front Right Corner
// 4 = Right Side 1
// 5 = Right Side 2
// 6 = Right Side 3
// 7 = Right Side 4
// 8 = Back Right Corner
// 9 = Back Right Left

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

// Declare global variables
int NO_BUMP_SEGMENT = 9;

// Initalise ROS
ros::NodeHandle nh;

// Declare message types
std_msgs::Int16MultiArray bump_msg;

// Declare publishers or subscribers
ros::Publisher bump_pub("/bumpr", &bump_msg);

// SETUP-------------------------------------------------------------
void setup() {

  // Initialise Node
  nh.initNode();

  // Advertise the topics
  nh.advertise(bump_pub);

  // Initialise pins
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);

}

// LOOP-------------------------------------------------------------
void loop()
{

    // Declare empty array
    int state[NO_BUMP_SEGMENT];

    // Read each pin state
    for (int i = 0; i < NO_BUMP_SEGMENT; i++){
      state[i] = digitalRead(i+2);
    }

    bump_msg.data_length = NO_BUMP_SEGMENT;
    bump_msg.data = state;
    bump_pub.publish(&bump_msg);

    nh.spinOnce();
}
