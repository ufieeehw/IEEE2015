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
  SetPosition(3, num->position_one);
  SetPosition(4, num->position_two);
}


int main(int argc, char **argv)
{
   
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("ieee2015_end_effector_servos", 1000, chatterCallback);


  uint16_t position;
  uint8_t mode;

  InitDXL(3,3);
  InitDXL(4,3);

  TorqueDisable(3);
  TorqueDisable(4);

  TorqueEnable(3);
  TorqueEnable(4);

  SetMaxTorque(3,1023);
  ReadTorque(3, &position);
  ReadControl(3, &mode);
  std::cout << position << mode << "\n";
  SetTorque(3,1023);
  ReadTorque(3, &position);
  ReadControl(3, &mode);
  std::cout << position << mode << "\n";
  SetMaxTorque(4,1023);
  ReadTorque(4, &position);
  ReadControl(4, &mode);
  std::cout << position << mode << "\n";
  SetTorque(4,1023);
  ReadTorque(4, &position);
  ReadControl(4, &mode);
  std::cout << position << mode << "\n";

  SetVelocity(3,700);
  SetVelocity(4,700);

  SetLED(3,7);
  SetLED(4,7);

  /*
  SetLED(1,7);
  SetMaxTorque(1,1023);

  for (int i = 0; i < 1000; i++)
  {
    SetTorque(1,1023);
    SetVelocity(1,1000);
  }*/
  

  ros::spin();

  return 0;
}