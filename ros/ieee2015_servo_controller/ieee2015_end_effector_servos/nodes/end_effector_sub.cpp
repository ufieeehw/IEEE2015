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
#include "ieee2015_end_effector_servos/EE.h"
#include <math.h> 

#define SMALL_SERVO 3
#define LARGE_SERVO 4
#define SIDE_ONE    5
#define SIDE_TWO    6

bool is_testing = false;
int side_control = 1;
int large_control = 1;
int small_control = 1;
bool moving = 0;
uint16_t position;

uint16_t m_pos_l = 0;
uint16_t m_pos_r = 0;
uint16_t t_pos_l = 0;
uint16_t t_pos_r = 0;

float past_location_one = 0;
float past_location_two = 0;

void init(int dxl_id){

      InitDXL(dxl_id,3);
}

void blink_LED(){

  for (int i = 1; i < 8; i++){

    SetLED(LARGE_SERVO,0);
    SetLED(SMALL_SERVO,0);
    SetLED(SIDE_ONE,0);
    SetLED(SIDE_TWO,0);

    usleep(50000);

    SetLED(LARGE_SERVO,i);
    SetLED(SMALL_SERVO,i);
    SetLED(SIDE_ONE,i);
    SetLED(SIDE_TWO,i);

    usleep(50000);
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
    SetVelocity(dxl_id,500);
   }
  if (wheel_or_angle == 2)
  {
    ToWheelMode(dxl_id);
  }
}

void calibrate_large_servo(){

  uint16_t c_load;
  bool moving;

  PAYLOAD(LARGE_SERVO,2);
  SetVelocity(LARGE_SERVO, 500);
  SetLED(LARGE_SERVO,1);

  ReadCurrentLoad(LARGE_SERVO, &c_load);
  c_load = 1000;

  while(c_load != 1554){
    ReadCurrentLoad(LARGE_SERVO, &c_load);
  }

  
  SetVelocity(LARGE_SERVO, 0);
  SetLED(LARGE_SERVO, 5);
  PAYLOAD(LARGE_SERVO,2);
  SetVelocity(LARGE_SERVO, 2047);
  usleep(500000);
  ReadMovingStatus(LARGE_SERVO, &moving);

  while(moving == false){
    PAYLOAD(LARGE_SERVO,2);
    SetVelocity(LARGE_SERVO, 2047);
    ReadMovingStatus(LARGE_SERVO, &moving);
  }
  usleep(3000000);
  SetVelocity(LARGE_SERVO,0);
  SetLED(LARGE_SERVO,1);
  PAYLOAD(LARGE_SERVO,2);
}

void calibrate_small_servo(){

  uint16_t c_load;
  bool moving;

  PAYLOAD(SMALL_SERVO,2);
  SetVelocity(SMALL_SERVO, 500);
  SetLED(SMALL_SERVO,1);

  ReadCurrentLoad(SMALL_SERVO, &c_load);
  c_load = 1000;

  while(c_load != 1554){
    ReadCurrentLoad(SMALL_SERVO, &c_load);
  }

  SetVelocity(SMALL_SERVO, 0);

  SetLED(SMALL_SERVO, 5);
  PAYLOAD(SMALL_SERVO,2);
  SetVelocity(SMALL_SERVO, 1500);
  usleep(500000);
  ReadMovingStatus(SMALL_SERVO, &moving);

  while(moving == false){
    PAYLOAD(SMALL_SERVO,2);
    SetVelocity(SMALL_SERVO, 2047);
    ReadMovingStatus(SMALL_SERVO, &moving);
  }
  sleep(2);
  SetVelocity(SMALL_SERVO,0);
  SetLED(SMALL_SERVO,1); 
}

bool open_large_servo(ieee2015_end_effector_servos::EE::Request  &req,
                      ieee2015_end_effector_servos::EE::Response &res)
{
  PAYLOAD(LARGE_SERVO,2);
  SetLED(LARGE_SERVO,1);
  SetVelocity(LARGE_SERVO, 2047);
  usleep(500000);
  SetVelocity(LARGE_SERVO, 0);
  SetLED(LARGE_SERVO, 5);
  PAYLOAD(LARGE_SERVO,2);
  sleep(1);
  calibrate_large_servo();
  return true;
}

bool open_small_servo(ieee2015_end_effector_servos::EE::Request  &req,
                      ieee2015_end_effector_servos::EE::Response &res)
{
  PAYLOAD(SMALL_SERVO,2);
  SetLED(SMALL_SERVO,1);
  SetVelocity(SMALL_SERVO, 2047);
  usleep(500000);
  SetVelocity(SMALL_SERVO, 0);
  SetLED(SMALL_SERVO, 5);
  PAYLOAD(SMALL_SERVO,2);
  sleep(1);
  calibrate_small_servo();
  return true;
}

bool close_large_servo(ieee2015_end_effector_servos::EE::Request  &req,
                       ieee2015_end_effector_servos::EE::Response &res)
{
  uint16_t c_load;

  PAYLOAD(LARGE_SERVO,2);
  SetVelocity(LARGE_SERVO, 500);
  SetLED(LARGE_SERVO,1);

  ReadCurrentLoad(LARGE_SERVO, &c_load);
  c_load = 1000;

  while(c_load != 1554){
    ReadCurrentLoad(LARGE_SERVO, &c_load);
  }

  
  SetVelocity(LARGE_SERVO, 0);
  SetLED(LARGE_SERVO, 5);
  PAYLOAD(LARGE_SERVO,2);
  return true;
}

bool close_small_servo(ieee2015_end_effector_servos::EE::Request  &req,
                       ieee2015_end_effector_servos::EE::Response &res)
{
  uint16_t c_load;

  PAYLOAD(SMALL_SERVO,2);
  SetVelocity(SMALL_SERVO, 500);
  SetLED(SMALL_SERVO,1);

  ReadCurrentLoad(SMALL_SERVO, &c_load);
  c_load = 1000;

  while(c_load != 1554){
    ReadCurrentLoad(SMALL_SERVO, &c_load);
  }

  SetVelocity(SMALL_SERVO, 0);

  SetLED(SMALL_SERVO, 5);
  PAYLOAD(SMALL_SERVO,2);
  return true;
}

void calibrate_sides_DOWN()
{

  uint16_t c_load_r;
  uint16_t c_load_l;
  bool moving;

  SetLED(SIDE_TWO,3);
  SetLED(SIDE_ONE,3);

  SetVelocity(SIDE_ONE, 600);
  ReadMovingStatus(SIDE_ONE, &moving);
  usleep(200);

  if (moving == true)
    {
      SetVelocity(SIDE_TWO, 600);
      ReadMovingStatus(SIDE_TWO, &moving);
      usleep(200);
      if (moving != true)
      {
        SetVelocity(SIDE_TWO,0);
        SetVelocity(SIDE_ONE,0);
        calibrate_sides_DOWN();
      }
    }
  

  ReadCurrentLoad(SIDE_ONE, &c_load_r);
  usleep(500);
  ReadCurrentLoad(SIDE_TWO, &c_load_l);
  usleep(500);
  c_load_l = 1000;
  c_load_r = 1000;
  usleep(500000);

  while((c_load_r > 750) && (c_load_l >750)){
    ReadCurrentLoad(SIDE_ONE, &c_load_r);
    ReadCurrentLoad(SIDE_TWO, &c_load_l);
  }

  SetVelocity(SIDE_ONE, 0);
  SetVelocity(SIDE_TWO, 0);

  ReadPosition(SIDE_ONE, &position);
  m_pos_r = position;
  ReadPosition(SIDE_TWO, &position);
  m_pos_l = position;
}

void calibrate_sides_UP(){

  uint16_t c_load_r;
  uint16_t c_load_l;
  bool moving;

  SetLED(SIDE_TWO,3);
  SetLED(SIDE_ONE,3);

  SetVelocity(SIDE_ONE, 1400);
  ReadMovingStatus(SIDE_ONE, &moving);
  usleep(200);

  if (moving == true)
    {
      SetVelocity(SIDE_TWO, 1400);
      ReadMovingStatus(SIDE_TWO, &moving);
      usleep(200);
      if (moving != true)
      {
        SetVelocity(SIDE_TWO,0);
        SetVelocity(SIDE_ONE,0);
        calibrate_sides_UP();
      }
    }
  

  ReadCurrentLoad(SIDE_ONE, &c_load_r);
  usleep(500);
  ReadCurrentLoad(SIDE_TWO, &c_load_l);
  usleep(500);
  c_load_l = 1000;
  c_load_r = 1000;
  usleep(500000);

  while((c_load_r > 1100) && (c_load_l > 1100)){
    ReadCurrentLoad(SIDE_ONE, &c_load_r);
    ReadCurrentLoad(SIDE_TWO, &c_load_l);
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

  while(count <= 5)
  {
    if (count == 0)
    {
      SetVelocity(SIDE_ONE, 1600);
      SetVelocity(SIDE_TWO, 1600);
    }
    else{
      SetVelocity(SIDE_ONE, 1520);
      SetVelocity(SIDE_TWO, 1520);
    }
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
    usleep(400000);
    PAYLOAD(SIDE_ONE, 2);
    PAYLOAD(SIDE_TWO, 2);
  }
  calibrate_sides_UP(); 
}

void sides_DOWN(){

  int count = 0;

  while (count <= 3)
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
    usleep(500000);
    PAYLOAD(SIDE_ONE, 2);
    PAYLOAD(SIDE_TWO, 2);
  }
  calibrate_sides_DOWN();
}

bool mode_DOWN(ieee2015_end_effector_servos::EE::Request  &req,
               ieee2015_end_effector_servos::EE::Response &res)
{

  SetVelocity(SMALL_SERVO, 0);
  SetVelocity(LARGE_SERVO, 0);

  SetLED(SIDE_ONE,7);
  SetLED(SIDE_TWO,7);
  sides_DOWN();

  SetVelocity(SIDE_ONE, 0);
  SetVelocity(SIDE_TWO, 0);

  PAYLOAD(LARGE_SERVO, 1);
  PAYLOAD(SMALL_SERVO, 1);

  SetLED(LARGE_SERVO,4);
  SetLED(SMALL_SERVO,4);
  SetLED(SIDE_ONE,4);
  return true;
}

bool mode_UP(ieee2015_end_effector_servos::EE::Request  &req,
             ieee2015_end_effector_servos::EE::Response &res)
{
  
  SetVelocity(SIDE_ONE, 0);
  SetVelocity(SIDE_TWO, 0);

  PAYLOAD(LARGE_SERVO, 2);
  PAYLOAD(SMALL_SERVO, 2);
  sides_UP();


  //SetVelocity(SMALL_SERVO, 1000);
  //SetVelocity(LARGE_SERVO, 1000);
  sleep(2);
  SetVelocity(SMALL_SERVO, 000);
  SetVelocity(LARGE_SERVO, 000);

  SetLED(LARGE_SERVO, 5);
  SetLED(SIDE_TWO,5);  
  return true;
  
}



void chatterCallback(const ieee2015_end_effector_servos::Num::ConstPtr &num){

  SetPosition(LARGE_SERVO, num->position_one);
  SetPosition(SMALL_SERVO, num->position_two);

}

int main(int argc, char **argv){

  ros::init(argc, argv, "end_effector_service");

  ros::NodeHandle n;

  ros::ServiceServer mode_up = n.advertiseService("mode_up", mode_UP);
  ros::ServiceServer mode_down = n.advertiseService("mode_down", mode_DOWN);
  ros::ServiceServer close_small = n.advertiseService("close_small_servo", close_small_servo);
  ros::ServiceServer open_small = n.advertiseService("open_small_servo", open_small_servo);
  ros::ServiceServer close_large = n.advertiseService("close_large_servo", close_large_servo);
  ros::ServiceServer open_large = n.advertiseService("open_large_servo", open_large_servo);

  ros::Subscriber sub = n.subscribe("ieee2015_end_effector_servos", 1000, chatterCallback);
  
  if (is_testing == true)
  {
    ros::spin();
    /*
    InitDXL(5,3);
    SetID(6,5);
    SetLED(5,1);
    */

  }

  if (is_testing == false)
  {

    init(SIDE_ONE);
    PAYLOAD(SIDE_ONE, 2);

    init(SIDE_TWO);
    PAYLOAD(SIDE_TWO, 2);

    init(LARGE_SERVO);
    PAYLOAD(LARGE_SERVO, 1);

    init(SMALL_SERVO);
    PAYLOAD(SMALL_SERVO, 1);

    calibrate_sides_DOWN();
    sleep(1);
    calibrate_sides_DOWN();

    sides_UP();
    
    calibrate_large_servo();

    calibrate_small_servo(); 
    

    blink_LED(); 

    ros::spin();

    return 0;

  }

  return 0;
}