#include "ros/ros.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <signal.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <dirent.h>
#include <string.h>
#include <time.h>


#include "high_level_commands.h"
#include "communications.h"
 #include "end_effector_servos/Num.h"


void chatterCallback(const end_effector_servos::Num::ConstPtr &num)
{
  SetPosition(2, num->position_one);
  SetVelocity(3, num->velocity_one);
  SetLED(2,num->color_one);
  SetPosition(3, num->position_two);
  SetVelocity(3, num->velocity_two);
  SetLED(3,num->color_two);
  SetPosition(4, num->position_three);
  SetVelocity(4, num->velocity_three);
  SetLED(4,num->color_three);
}


int main(int argc, char **argv)
{
   
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("end_effector_servos", 1000, chatterCallback);

  int device_num = 0; // Default to device ID
  InitDXL(2,3);
  InitDXL(3,3);
  InitDXL(4,3);
  SetLED(2,7);
  SetLED(3,7);
  SetLED(4,7);

  ros::spin();

  return 0;
}