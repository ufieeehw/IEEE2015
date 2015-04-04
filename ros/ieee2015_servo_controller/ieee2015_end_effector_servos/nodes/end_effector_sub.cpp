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

#define LARGE_SERVO 4
#define SMALL_SERVO 3
#define SIDE_ONE    5
#define SIDE_TWO    6

bool is_testing = false;
int last_control = 2;
bool moving = 0;
uint16_t position;
uint16_t m_pos_l = 0;
uint16_t m_pos_r = 0;
uint16_t t_pos_l = 0;
uint16_t t_pos_r = 0;

void init(int dxl_id){

      InitDXL(dxl_id,3);
}

void blink_LED(){

  for (int i = 4; i < 8; i++){

    SetLED(LARGE_SERVO,0);
    SetLED(SMALL_SERVO,0);
    SetLED(SIDE_ONE,0);
    SetLED(SIDE_TWO,0);

    usleep(40000);

    SetLED(LARGE_SERVO,i);
    SetLED(SMALL_SERVO,i);
    SetLED(SIDE_ONE,i);
    SetLED(SIDE_TWO,i);

    usleep(40000);

  }
}

void read_from_servos(int dxl_id){

  uint16_t position, angle, c_load, c_gain;
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
  std::cout << "Current Temp:    " << (int)temp << "\n";
  ReadCurrentGain(dxl_id, &c_gain);
  std::cout << "Current Gain:    " << (int)c_gain << "\n\n";
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
   }
  if (wheel_or_angle == 2)
  {
    ToWheelMode(dxl_id);
  }
}

void calibrate_large_servo(){

  uint16_t c_load;

  PAYLOAD(LARGE_SERVO,2);
  SetVelocity(LARGE_SERVO, 500);
  SetLED(LARGE_SERVO,1);

  ReadCurrentLoad(LARGE_SERVO, &c_load);
  usleep(1000000);
  c_load = 1000;

  while(c_load != 1554){
    ReadCurrentLoad(LARGE_SERVO, &c_load);
  }

  SetVelocity(LARGE_SERVO, 0);
  SetLED(LARGE_SERVO, 5);
  sleep(1);
  SetVelocity(LARGE_SERVO, 2047);
  usleep(3500000);
  SetVelocity(LARGE_SERVO, 0);
}

void calibrate_sides_DOWN(){

  uint16_t c_load_r;
  uint16_t c_load_l;

  SetLED(SIDE_TWO,3);
  SetLED(SIDE_ONE,3);

  SetVelocity(SIDE_ONE, 500);
  SetVelocity(SIDE_TWO, 500);
  

  ReadCurrentLoad(SIDE_ONE, &c_load_r);
  usleep(500);
  ReadCurrentLoad(SIDE_TWO, &c_load_l);
  usleep(500);
  c_load_l = 1000;
  c_load_r = 1000;
  usleep(500000);

  while((c_load_r > 450) ){
    ReadCurrentLoad(SIDE_ONE, &c_load_r);
    ReadCurrentLoad(SIDE_ONE, &c_load_l);
  }

  SetVelocity(SIDE_ONE, 0);
  SetVelocity(SIDE_TWO, 0);

  ReadPosition(SIDE_ONE, &position);
  m_pos_r = position;
  ReadPosition(SIDE_TWO, &position);
  m_pos_l = position;
}

void sides_UP(){

  int count = 0;

  while(count <= 6)
  {
    SetVelocity(SIDE_ONE, 1520);
    ReadMovingStatus(SIDE_ONE, &moving);
    usleep(200);

    if (moving == true)
    {
      SetVelocity(SIDE_TWO, 1520);
      ReadMovingStatus(SIDE_TWO, &moving);
      usleep(200);
      if (moving == true)
      {
        count++;
      }else{
        SetVelocity(SIDE_TWO,0);
        SetVelocity(SIDE_ONE,0);
      }
    }
    else{
      SetVelocity(SIDE_TWO,0);
      SetVelocity(SIDE_ONE,0);
    }
    sleep(1);
    SetVelocity(SIDE_ONE, 0);
    SetVelocity(SIDE_TWO, 0);
    moving = false;
    PAYLOAD(SIDE_ONE,1);
    PAYLOAD(SIDE_TWO,1);
    SetPosition(SIDE_ONE, m_pos_r);
    SetPosition(SIDE_TWO, m_pos_l);
    sleep(1);
    PAYLOAD(SIDE_ONE, 2);
    PAYLOAD(SIDE_TWO, 2);
  } 
}

void sides_DOWN(){

  int count = 0;

  while (count <= 4)
  {
    SetVelocity(SIDE_ONE, 470);
    ReadMovingStatus(SIDE_ONE, &moving);
    usleep(200);
    if (moving == true)
    {
      SetVelocity(SIDE_TWO, 480);
      ReadMovingStatus(SIDE_TWO, &moving);
      usleep(200);
      if (moving == true)
      {
        count++;
      }
      else{
        SetVelocity(SIDE_TWO,0);
        SetVelocity(SIDE_ONE,0);
      }
    }
    else{
      SetVelocity(SIDE_TWO,0);
      SetVelocity(SIDE_ONE,0);
    }
    sleep(1);
    SetVelocity(SIDE_ONE, 0);
    SetVelocity(SIDE_TWO, 0);
    moving = false;
    PAYLOAD(SIDE_ONE,1);
    PAYLOAD(SIDE_TWO,1);
    SetPosition(SIDE_ONE, m_pos_r);
    SetPosition(SIDE_TWO, m_pos_l);
    sleep(1);
    PAYLOAD(SIDE_ONE, 2);
    PAYLOAD(SIDE_TWO, 2);
  }
  calibrate_sides_DOWN();
}

void mode_DOWN(){

  SetVelocity(SMALL_SERVO, 0);
  SetVelocity(LARGE_SERVO, 0);

  SetLED(SIDE_ONE,7);
  SetLED(SIDE_TWO,7);
  calibrate_sides_DOWN();

  SetVelocity(SIDE_ONE, 0);
  SetVelocity(SIDE_TWO, 0);

  PAYLOAD(LARGE_SERVO, 1);
  PAYLOAD(SMALL_SERVO, 1);

  SetLED(LARGE_SERVO,4);
  SetLED(SMALL_SERVO,4);
  SetLED(SIDE_ONE,4);
}

void mode_UP(){

  SetVelocity(SIDE_ONE, 0);
  SetVelocity(SIDE_TWO, 0);

  PAYLOAD(LARGE_SERVO, 2);
  PAYLOAD(SMALL_SERVO, 2);

  //SetVelocity(SMALL_SERVO, 1000);
  //SetVelocity(LARGE_SERVO, 1000);
  sleep(2);
  SetVelocity(SMALL_SERVO, 000);
  SetVelocity(LARGE_SERVO, 000);

  SetLED(LARGE_SERVO, 5);
  SetLED(SMALL_SERVO, 5);
  SetLED(SIDE_ONE,5);
  SetLED(SIDE_TWO,5);   
}

void chatterCallback(const ieee2015_end_effector_servos::Num::ConstPtr &num){
  int contol_mode = num->control;

  if(last_control != contol_mode){

    if (contol_mode == 1) // Set to angle mode
    {
      mode_DOWN();
    }

    if (contol_mode == 2) // Set to continous mode
    {
      mode_UP();
    }

  }
  SetPosition(LARGE_SERVO, num->position_one);
  SetPosition(SMALL_SERVO, num->position_two);

  last_control = num->control;
}


int main(int argc, char **argv){

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("ieee2015_end_effector_servos", 1000, chatterCallback);

  if (is_testing == true)
  {
    /*
    InitDXL(1,3);
    SetID(1,4);
    SetLED(4,1);
    */

  }

  if (is_testing == false)
  {

    init(SIDE_ONE);
    PAYLOAD(SIDE_ONE, 2);

    init(SIDE_TWO);
    PAYLOAD(SIDE_TWO, 2);

    calibrate_sides_DOWN();
    calibrate_large_servo();

    PAYLOAD(LARGE_SERVO, 1);

    init(SMALL_SERVO);
    PAYLOAD(SMALL_SERVO, 1);

    SetVelocity(SIDE_ONE, 0);
    SetVelocity(SIDE_TWO, 0);

    blink_LED();

    ros::spin();

    return 0;

  }

  return 0;
}