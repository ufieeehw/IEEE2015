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

#define LARGE_SERVO 3
#define SMALL_SERVO 4
#define SIDE_ONE    5
#define SIDE_TWO    6



bool is_testing = false;
int last_control = 1;

uint16_t m_pos_l = 0;
uint16_t m_pos_r = 0;
uint16_t t_pos_l = 0;
uint16_t t_pos_r = 0;

void init(int dxl_id){
      InitDXL(dxl_id,3);
}

void read_from_servos(int dxl_id){

  uint16_t position, angle, c_load;
  uint8_t mode, alarm, error_return, u_volt, l_volt, c_volt, temp, m_temp;

  std::cout << "--------------------------------------------------------------------------";

  std::cout << "\n\nServo: " << dxl_id << "\n\n";

  ReadTorque(dxl_id, &position);
  std::cout << "Torque:          " << (int)position << "\n";
  ReadPosition(dxl_id, &position);
  std::cout << "Position:        " << (int)position << "\n";
  ReadControl(dxl_id, &mode);
  std::cout << "Control:         " << (int)mode << "\n";
  ReadCWAngle(dxl_id, &angle);
  std::cout << "CW Limit:        " << (int)angle << "\n";
  ReadCWWAngle(dxl_id, &angle);
  std::cout << "CWW Limit:       " << (int)angle << "\n";
  ReadAlarm(dxl_id, &alarm);
  std::cout << "Alarm:           " << (int)alarm << "\n";
  ReadError(dxl_id, &error_return);
  std::cout << "Error:           " << (int)error_return << "\n";
  ReadUpperVoltage(dxl_id, &u_volt);
  std::cout << "High Volt Limit: " << (int)u_volt << "\n";
  ReadLowerVoltage(dxl_id, &l_volt);
  std::cout << "Low Volt limit:  " << (int)l_volt << "\n";
  ReadCurrentVoltage(dxl_id, &c_volt);
  std::cout << "Current Voltage: " << (int)c_volt << "\n";
  ReadCurrentLoad(dxl_id, &c_load);
  std::cout << "Current Load:    " << (int)c_load << "\n";
  ReadCurrentTemp(dxl_id, &temp);
  std::cout << "Current Temp:    " << (int)temp << "\n";
  ReadCurrentTemp(dxl_id, &temp);
  std::cout << "Current Temp:    " << (int)temp << "\n\n";
  
}

void ToAngleMode(int dxl_id){

  TorqueDisable(dxl_id);
  SetCWAngleLimit(dxl_id,0);
  SetCW_WAngleLimit(dxl_id,1023);
  SetControl(dxl_id,2);
  read_from_servos(dxl_id);
  SetTemp(dxl_id,150);
  TorqueEnable(dxl_id);
  SetMaxTorque(dxl_id,1023);
  SetTorque(dxl_id,1023);

}


void ToWheelMode(int dxl_id){

  TorqueDisable(dxl_id);
  SetCWAngleLimit(dxl_id,0);
  SetCW_WAngleLimit(dxl_id,0);
  SetControl(dxl_id,1);
  read_from_servos(dxl_id);
  SetTemp(dxl_id,150);
  TorqueEnable(dxl_id);
  SetMaxTorque(dxl_id,1023);
  SetTorque(dxl_id,1023);

}

// Function that will be used for verification of setting switch
void PAYLOAD(int dxl_id, int wheel_or_angle){

  uint8_t error_return;

  if (wheel_or_angle == 1)
  {
    ToAngleMode(dxl_id);
    SetVelocity(dxl_id,700);
    /*ReadError(dxl_id, &error_return);
    if (error_return != 0)
    {
      PAYLOAD(dxl_id, 1);
    }
  */
   }
  if (wheel_or_angle == 2)
  {
    ToWheelMode(dxl_id);
    /*
    ReadError(dxl_id, &error_return);
    if (error_return != 0)
    {
      PAYLOAD(dxl_id, 2);
    }*/
  }
}

// For right now this is angle mode
void servo_down(){

  SetVelocity(SMALL_SERVO, 0);
  SetVelocity(LARGE_SERVO, 0);

  SetLED(SIDE_ONE,7);
  SetLED(SIDE_TWO,7);
  SetPosition(SIDE_ONE,0);
  SetPosition(SIDE_TWO,0);

  sleep(1);

  PAYLOAD(LARGE_SERVO, 1);
  PAYLOAD(SMALL_SERVO, 1);

  SetLED(LARGE_SERVO,4);
  SetLED(SMALL_SERVO,4);
  SetLED(SIDE_ONE,4);

}

// Wheel mode
void servo_up(){

  SetVelocity(SMALL_SERVO, 0);
  SetVelocity(LARGE_SERVO, 0);
  
  SetLED(SIDE_ONE,7);
  SetLED(SIDE_TWO,7);
  SetPosition(SIDE_ONE,1023);
  SetPosition(SIDE_TWO,1023);

  sleep(1);

  PAYLOAD(LARGE_SERVO, 2);
  PAYLOAD(SMALL_SERVO, 2);

  SetVelocity(SMALL_SERVO,500);
  SetVelocity(LARGE_SERVO, 500);

  SetLED(LARGE_SERVO, 5);
  SetLED(SMALL_SERVO, 5);
  SetLED(SIDE_ONE,5);
  SetLED(SIDE_TWO,5);
      
}

void blink_LED(){

  for (int i = 4; i < 8; i++){

    SetLED(LARGE_SERVO,0);
    SetLED(SMALL_SERVO,0);
    SetLED(SIDE_ONE,0);
    SetLED(SIDE_TWO,0);

    usleep(60000);

    SetLED(LARGE_SERVO,i);
    SetLED(SMALL_SERVO,i);
    SetLED(SIDE_ONE,i);
    SetLED(SIDE_TWO,i);

    usleep(60000);

  }
}

void chatterCallback(const ieee2015_end_effector_servos::Num::ConstPtr &num)
{
  int contol_mode = num->control;

  if(last_control != contol_mode){

    if (contol_mode == 1) // Set to angle mode
    {
      servo_down();
    }

    if (contol_mode == 2) // Set to continous mode
    {
      servo_up();
    }

  }
  SetPosition(LARGE_SERVO, num->position_one);
  SetPosition(SMALL_SERVO, num->position_two);

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

    init(LARGE_SERVO);
    PAYLOAD(LARGE_SERVO, 1);

    init(SMALL_SERVO);
    PAYLOAD(SMALL_SERVO, 1);

    init(SIDE_ONE);
    PAYLOAD(SIDE_ONE, 1);

    init(SIDE_TWO);
    PAYLOAD(6, 1);

    SetPosition(SIDE_ONE,0);
    SetPosition(SIDE_TWO,0);
    sleep(1);

    read_from_servos(LARGE_SERVO);
    read_from_servos(SMALL_SERVO);
    read_from_servos(SIDE_ONE);
    read_from_servos(SIDE_TWO);

    blink_LED();

    ros::spin();

    return 0;

  }

  return 0;

}