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


class XL320_servo {

  public:

    void init(int dxl_id){
      InitDXL(dxl_id,3);
    }

    void ToAngleMode(int dxl_id){
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
        SetTemp(dxl_id,150);
        SetMaxTorque(dxl_id,1023);
        SetTorque(dxl_id,1023);
        SetVelocity(dxl_id,700);
       }
      if (wheel_or_angle == 2)
      {
        TorqueDisable(dxl_id);
        ToWheelMode(dxl_id);
        TorqueEnable(dxl_id);
        SetTemp(dxl_id,150);
        SetMaxTorque(dxl_id,1023);
        SetTorque(dxl_id,1023);
      }
   }

    void read_from_servos(int dxl_id){

      uint16_t position, angle, c_load;
      uint8_t mode, alarm, error_return, u_volt, l_volt, c_volt, temp;

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
      std::cout << "Current Temp:    " << (int)temp << "\n\n";
      

    }

    void ToWheelMode(int dxl_id){
      SetCWAngleLimit(dxl_id,0);
      SetCW_WAngleLimit(dxl_id,0);
      SetControl(dxl_id,1);
    }

    void servo_down(){

      uint16_t position;

      SetVelocity(SIDE_ONE, 1850);
      SetVelocity(SIDE_TWO, 1850);

      sleep(3);

      SetVelocity(SIDE_ONE,0);
      SetVelocity(SIDE_TWO,0);

      PAYLOAD(SIDE_ONE,1);
      PAYLOAD(SIDE_TWO,1);

      SetVelocity(SIDE_ONE,1000);
      SetVelocity(SIDE_TWO,1000);

      SetPosition(SIDE_ONE, 0);
      SetPosition(SIDE_TWO, 0);

      sleep(1);

      PAYLOAD(SIDE_ONE,2);
      PAYLOAD(SIDE_TWO,2);
      SetLED(SIDE_ONE,SIDE_ONE);
      SetLED(SIDE_TWO,SIDE_ONE);

      ReadPosition(SIDE_ONE, &position);
      std::cout << position << "\n";
      ReadPosition(SIDE_TWO, &position);
      std::cout << position << "\n";

    }

    void servo_up(){

      uint16_t position;

      SetVelocity(SIDE_ONE,785);
      SetVelocity(SIDE_TWO,785);

      sleep(3);

      SetVelocity(SIDE_ONE,0);
      SetVelocity(SIDE_TWO,0);

      PAYLOAD(SIDE_ONE,1);
      PAYLOAD(SIDE_TWO,1);

      SetVelocity(SIDE_ONE,1000);
      SetVelocity(SIDE_TWO,1000);

      SetPosition(SIDE_ONE, 0);
      SetPosition(SIDE_TWO, 0);

      sleep(1);

      PAYLOAD(SIDE_ONE,2);
      PAYLOAD(SIDE_TWO,2);

      SetLED(SIDE_ONE,SIDE_ONE);
      SetLED(SIDE_TWO,SIDE_ONE);

      ReadPosition(SIDE_ONE, &position);
      std::cout << position << "\n";
      ReadPosition(SIDE_TWO, &position);
      std::cout << position << "\n";

    }
};



bool is_testing = false;
int last_control = 1;

uint16_t m_pos_l = 0;
uint16_t m_pos_r = 0;
uint16_t t_pos_l = 0;
uint16_t t_pos_r = 0;

XL320_servo EE_LARGE;
XL320_servo EE_SMALL;
XL320_servo EE_SIDE_ONE;
XL320_servo EE_SIDE_TWO;



void blink_LED(){

  for (int i = 4; i < 8; i++){

    SetLED(LARGE_SERVO,0);
    SetLED(SMALL_SERVO,0);
    SetLED(SIDE_ONE,0);
    SetLED(SIDE_TWO,0);

    usleep(100000);

    SetLED(LARGE_SERVO,i);
    SetLED(SMALL_SERVO,i);
    SetLED(SIDE_ONE,i);
    SetLED(SIDE_TWO,i);

    usleep(100000);

  }
}


void chatterCallback(const ieee2015_end_effector_servos::Num::ConstPtr &num)
{

  if(last_control != num->control){
    if (num->control == 1) // Set to angle mode
    {
      EE_SIDE_ONE.servo_down();
      EE_LARGE.PAYLOAD(LARGE_SERVO,1);
      EE_SMALL.PAYLOAD(SMALL_SERVO,1);
      SetLED(LARGE_SERVO,7);
      SetLED(SMALL_SERVO,7);
      

    }
    if (num->control == 2) // Set to continous mode
    {
      EE_SIDE_ONE.servo_up();
      EE_LARGE.PAYLOAD(LARGE_SERVO,2);
      SetVelocity(LARGE_SERVO, 500);
      SetLED(LARGE_SERVO,2);


      EE_SMALL.PAYLOAD(SMALL_SERVO,2);
      SetVelocity(SMALL_SERVO,500);
      SetLED(SMALL_SERVO,2);

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
    
    InitDXL(1,LARGE_SERVO);
    SetID(1,SIDE_TWO);
    SetLED(SIDE_TWO,1);

  }

  if (is_testing == false)
  {


    EE_LARGE.init(LARGE_SERVO);
    EE_LARGE.PAYLOAD(LARGE_SERVO, 1);

    EE_SMALL.init(SMALL_SERVO);
    EE_SMALL.PAYLOAD(SMALL_SERVO, 1);

    EE_SIDE_ONE.init(SIDE_ONE);
    EE_SIDE_ONE.PAYLOAD(SIDE_ONE, 2);

    EE_SIDE_TWO.init(SIDE_TWO);
    EE_SIDE_TWO.PAYLOAD(SIDE_TWO, 2);

    SetVelocity(SIDE_ONE,0);
    SetVelocity(SIDE_TWO,0);

    EE_LARGE.read_from_servos(LARGE_SERVO);
    EE_SMALL.read_from_servos(SMALL_SERVO);
    EE_SIDE_ONE.read_from_servos(SIDE_ONE);
    EE_SIDE_TWO.read_from_servos(SIDE_TWO);

    ReadPosition(SIDE_ONE, &m_pos_r);
    ReadPosition(SIDE_TWO, &m_pos_l);

    std::cout << "Base Position Setting" << m_pos_r << "\n";
    std::cout << "Base Position Setting" << m_pos_l << "\n";

    blink_LED();

    ros::spin();

    return 0;

  }

  return 0;

}