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

void read_from_servos(int dxl_id){

  uint16_t position, angle, c_load;
  uint8_t mode, alarm, error_return, u_volt, l_volt, c_volt, temp;

  ReadTorque(dxl_id, &position);
  std::cout << "Torque is: " << (int)position << " on servo id " << dxl_id << "\n";
  ReadPosition(dxl_id, &position);
  std::cout << "Position is: " << (int)position << " on servo id " << dxl_id << "\n";
  ReadControl(dxl_id, &mode);
  std::cout << "Control is: " << (int)mode << " on servo id " << dxl_id << "\n";
  ReadCWAngle(dxl_id, &angle);
  std::cout << "CW angle limit is: " << (int)angle << " on servo id " << dxl_id << "\n";
  ReadCWWAngle(dxl_id, &angle);
  std::cout << "CWW angle limit is: " << (int)angle << " on servo id " << dxl_id << "\n";
  ReadAlarm(dxl_id, &alarm);
  std::cout << "Alarm is: " << (int)alarm << " on servo id " << dxl_id << "\n";
  ReadError(dxl_id, &error_return);
  std::cout << "Error is: " << (int)error_return << " on servo id " << dxl_id << "\n";
  ReadUpperVoltage(dxl_id, &u_volt);
  std::cout << "Upper Voltage is: " << (int)u_volt << " on servo id " << dxl_id << "\n";
  ReadLowerVoltage(dxl_id, &l_volt);
  std::cout << "Lower Voltage is: " << (int)l_volt << " on servo id " << dxl_id << "\n";
  ReadCurrentVoltage(dxl_id, &c_volt);
  std::cout << "Curernt Voltage is: " << (int)c_volt << " on servo id " << dxl_id << "\n";
  ReadCurrentLoad(dxl_id, &c_load);
  std::cout << "Current Load is: " << (int)c_load << " on servo id " << dxl_id << "\n";
  ReadCurrentTemp(dxl_id, &temp);
  std::cout << "Current Temp is: " << (int)temp << " on servo id " << dxl_id << "\n";
  std::cout << "--------------------------------------------------------------------------";

}


void chatterCallback(const ieee2015_end_effector_servos::Num::ConstPtr &num)
{
  SetPosition(3, num->position_one);
  SetPosition(4, num->position_two);
  read_from_servos(3);
  read_from_servos(4);

}


int main(int argc, char **argv)
{
   
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("ieee2015_end_effector_servos", 1000, chatterCallback);

  // Enable Servos
  InitDXL(5,3);
  InitDXL(6,3);
  InitDXL(3,3);
  InitDXL(4,3);

  // Turn Torque off to set Control
  TorqueDisable(3);
  TorqueDisable(4);
  TorqueDisable(5);
  TorqueDisable(6);

  // Set Control
  SetControl(3,2);
  SetControl(4,2);
  SetControl(5,1);
  SetControl(6,1);
  
  // Done changing control
  TorqueEnable(3);
  TorqueEnable(4);
  TorqueEnable(5);
  TorqueEnable(6);

  // Set max torques
  SetMaxTorque(3,1023);
  SetMaxTorque(4,1023);
  SetMaxTorque(5, 1023);
  SetMaxTorque(6, 1023);
  
  // Set Torque for Angle mode servos
  SetTorque(3,1023);
  SetTorque(4,1023);

  // Set Velocity
  SetVelocity(3,700);
  SetVelocity(4,700);

  // Setup Complete
  SetLED(3,7);
  SetLED(4,7);
  SetLED(5,7);
  SetLED(6,7);

  /*
  for (int i = 0; i < 1023; i++)
  {
    SetTorque(5,1000);
    SetTorque(6,1000);
    SetVelocity(6,1000);
    SetVelocity(5,1000);
  }
  */

  ros::spin();

  return 0;
}