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
  SetPosition(3, num->position_one);
  SetPosition(4, num->position_two);
}


int main(int argc, char **argv)
{
   
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("end_effector_servos", 1000, chatterCallback);

  int device_num = 0; // Default to device ID
  InitDXL(3,3);
  InitDXL(4,3);

  SetLED(3,7);
  SetLED(4,7);
  SetVelocity(3, 700);
  SetVelocity(4, 700);


  ros::spin();

  return 0;
}