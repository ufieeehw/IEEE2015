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

bool is_testing = false;

uint16_t m_pos_l = 0;
uint16_t m_pos_r = 0;
uint16_t t_pos_l = 0;
uint16_t t_pos_r = 0;


void ToWheelMode(int dxl_id)
{
    SetCWAngleLimit(dxl_id,0);
    SetCW_WAngleLimit(dxl_id,0);
    SetControl(dxl_id,1);
}


void ToAngleMode(int dxl_id)
{
    SetCWAngleLimit(dxl_id,0);
    SetCW_WAngleLimit(dxl_id,1023);
    SetControl(dxl_id,2);
}

void PAYLOAD(int dxl_id, int wheel_or_angle){

    if (wheel_or_angle == 1)
    {
      TorqueDisable(dxl_id);
      ToAngleMode(dxl_id);
      TorqueEnable(dxl_id);
      SetMaxTorque(dxl_id,1023);
      SetTorque(dxl_id,1023);
      SetVelocity(dxl_id,700);
    }
    if (wheel_or_angle == 2)
    {
      TorqueDisable(dxl_id);
      ToWheelMode(dxl_id);
      TorqueEnable(dxl_id);
      SetMaxTorque(dxl_id,1023);
      SetTorque(dxl_id,1023);
    }
    

}

void servo_up(){

  SetVelocity(5,785);
  SetVelocity(6,785);

  sleep(3);

  SetVelocity(5,0);
  SetVelocity(6,0);

  PAYLOAD(5,1);
  PAYLOAD(6,1);

  SetPosition(5, m_pos_r);
  SetPosition(6, m_pos_l);

  PAYLOAD(5,2);
  PAYLOAD(6,2);

}

void servo_down(){

  SetVelocity(5,1860);
  SetVelocity(6,1860);

  sleep(3);

  SetVelocity(5,0);
  SetVelocity(6,0);

  PAYLOAD(5,1);
  PAYLOAD(6,1);

  SetPosition(5, m_pos_r);
  SetPosition(6, m_pos_l);

  PAYLOAD(5,2);
  PAYLOAD(6,2);

}


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

int last_control = 1;

void chatterCallback(const ieee2015_end_effector_servos::Num::ConstPtr &num)
{

  if(last_control != num->control){
    if (num->control == 1) // Set to angle mode
    {
      PAYLOAD(3,1);
      PAYLOAD(4,1);
      SetLED(3,7);
      SetLED(4,7);
      servo_down();

    }
    if (num->control == 2) // Set to continous mode
    {
      servo_up();
      PAYLOAD(3,2);
      SetVelocity(3,500);
      SetLED(3,5);

      usleep(500);

      PAYLOAD(4,2);
      SetVelocity(4,500);
      SetLED(4,5);

    }
  }

  SetPosition(3, num->position_one);
  SetPosition(4, num->position_two);

  last_control = num->control;

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("ieee2015_end_effector_servos", 1000, chatterCallback);

  uint16_t position;

  if (is_testing == true)
  {
    
    InitDXL(1,3);
    SetID(1,6);
    SetLED(6,1);

  }

  if (is_testing == false)
  {

    InitDXL(3,3);
    PAYLOAD(3,1);

    usleep(1000);

    InitDXL(4,3);
    PAYLOAD(4,1);

    usleep(1000);

    InitDXL(5,3);
    PAYLOAD(5,2);

    usleep(1000);

    InitDXL(6,3);
    PAYLOAD(6,2);

    SetVelocity(5,0);
    SetVelocity(6,0);

    usleep(1000);

    ReadPosition(5, &m_pos_r);
    ReadPosition(6, &m_pos_l);

    // SetTemp(dxl_id,150);


    // Setup complete buffoonery
    for (int i = 4; i < 7; i++)
    {
      SetLED(3,0);
      SetLED(4,0);
      SetLED(5,0);
      SetLED(6,0);

      usleep(100000);

      SetLED(3,i);
      SetLED(4,i);
      SetLED(5,i);
      SetLED(6,i);

      usleep(100000);
    }

    ros::spin();

    return 0;

  }

  return 0;

}