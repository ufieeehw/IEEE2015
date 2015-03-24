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
#include "ieee2015_end_effector_servos/Num.h"


void chatterCallback(const ieee2015_end_effector_servos::Num::ConstPtr &num)
{
  SetControl(3, num->control_one);
  SetControl(4, num->control_two);
  SetPosition(1, num->position_one);
  SetPosition(4, num->position_two);
}


int main(int argc, char **argv)
{
   
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("ieee2015_end_effector_servos", 1000, chatterCallback);

  InitDXL(1,3);
  TorqueDisable(1);
  SetLED(1,7);
  SetControl(1,1);
  TorqueEnable(1);
  SetTorque(1,1000);
  for (int i = 0; i < 1000; i++)
  {
      SetTorque(1,1000);

      SetVelocity(1,i);
      sleep(1);
  }


  


  ros::spin();

  return 0;
}