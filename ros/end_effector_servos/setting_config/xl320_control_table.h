#ifndef XL320_CONTROL_TABLE_H
#define XL320_CONTROL_TABLE_H


/* EEPROM */
// This persists through powerdowns.

static const uint8_t MODEL_NUMBER_REG = 0;
static const uint8_t MODEL_NUMBER_SIZE = 2;

static const uint8_t FIRMWARE_VERSION_REG = 2;
static const uint8_t FIRMWARE_VERSION_SIZE = 1;

static const uint8_t ID_REG = 3;
static const uint8_t ID_SIZE = 1;
static const uint8_t ID_MIN = 0;
static const uint8_t ID_MAX = 252;

static const uint8_t BAUD_RATE_REG = 4;
static const uint8_t BAUD_RATE_SIZE = 1;
static const uint8_t BAUD_RATE_MIN = 0;
static const uint8_t BAUD_RATE_MAX = 3;

static const uint8_t RETURN_DELAY_REG = 5;
static const uint8_t RETURN_DELAY_SIZE = 1;
static const uint8_t RETURN_DELAY_MIN = 0;
static const uint8_t RETURN_DELAY_MAX = 254;

static const uint8_t CW_ANGLE_LIMIT_REG = 6;
static const uint8_t CW_ANGLE_LIMIT_SIZE = 2;
static const uint16_t CW_ANGLE_LIMIT_MIN = 0;
static const uint16_t CW_ANGLE_LIMIT_MAX = 1023;

static const uint8_t CCW_ANGLE_LIMIT_REG = 8;
static const uint8_t CCW_ANGLE_LIMIT_SIZE = 2;
static const uint16_t CCW_ANGLE_LIMIT_MIN = 0;
static const uint16_t CCW_ANGLE_LIMIT_MAX = 1023;

static const uint8_t CONTROL_MODE_REG = 11;
static const uint8_t CONTROL_MODE_SIZE = 1;
static const uint8_t CONTROL_MODE_MIN = 1;
static const uint8_t CONTROL_MODE_MAX = 2;

static const uint8_t LIMIT_TEMPERATURE_REG = 12;
static const uint8_t LIMIT_TEMPERATURE_SIZE = 1;
static const uint8_t LIMIT_TEMPERATURE_MIN = 0;
static const uint8_t LIMIT_TEMPERATURE_MAX = 150;

static const uint8_t LOWER_LIMIT_VOLTAGE_REG = 13;
static const uint8_t LOWER_LIMIT_VOLTAGE_SIZE = 1;
static const uint8_t LOWER_LIMIT_VOLTAGE_MIN = 50;
static const uint8_t LOWER_LIMIT_VOLTAGE_MAX = 250;

static const uint8_t UPPER_LIMIT_VOLTAGE_REG = 14;
static const uint8_t UPPER_LIMIT_VOLTAGE_SIZE = 1;
static const uint8_t UPPER_LIMIT_VOLTAGE_MIN = 50;
static const uint8_t UPPER_LIMIT_VOLTAGE_MAX = 250;

static const uint8_t MAX_TORQUE_REG = 15;
static const uint8_t MAX_TORQUE_SIZE = 2;
static const uint16_t MAX_TORQUE_MIN = 0;
static const uint16_t MAX_TORQUE_MAX = 1023;

static const uint8_t RETURN_LEVEL_REG = 17;
static const uint8_t RETURN_LEVEL_SIZE = 1;
static const uint8_t RETURN_LEVEL_MIN = 0;
static const uint8_t RETURN_LEVEL_MAX = 2;

static const uint8_t ALARM_SHUTDOWN_REG = 18;
static const uint8_t ALARM_SHUTDOWN_SIZE = 1;
static const uint8_t ALARM_SHUTDOWN_MIN = 0;
static const uint8_t ALARM_SHUTDOWN_MAX = 127;

/* RAM */
// This is reset after every powerdown.

static const uint8_t TORQUE_ENABLE_REG = 24;
static const uint8_t TORQUE_ENABLE_SIZE = 1;
static const uint8_t TORQUE_ENABLE_MIN = 0;
static const uint8_t TORQUE_ENABLE_MAX = 1;

static const uint8_t LED_REG = 25;
static const uint8_t LED_SIZE = 1;
static const uint8_t LED_MIN = 0;
static const uint8_t LED_MAX = 7;

static const uint8_t D_GAIN_REG = 27;
static const uint8_t D_GAIN_SIZE = 1;
static const uint8_t D_GAIN_MIN = 0;
static const uint8_t D_GAIN_MAX = 254;

static const uint8_t I_GAIN_REG = 28;
static const uint8_t I_GAIN_SIZE = 1;
static const uint8_t I_GAIN_MIN = 0;
static const uint8_t I_GAIN_MAX = 254;

static const uint8_t P_GAIN_REG = 29;
static const uint8_t P_GAIN_SIZE = 1;
static const uint8_t P_GAIN_MIN = 0;
static const uint8_t P_GAIN_MAX = 254;

static const uint8_t GOAL_POSITION_REG = 30;
static const uint8_t GOAL_POSITION_SIZE = 2;
static const uint16_t GOAL_POSITION_MIN = 0;
static const uint16_t GOAL_POSITION_MAX = 1023;

static const uint8_t GOAL_VELOCITY_REG = 32;
static const uint8_t GOAL_VELOCITY_SIZE = 2;
static const uint16_t GOAL_VELOCITY_MIN = 0;
static const uint16_t GOAL_VELOCITY_MAX = 1023;

static const uint8_t GOAL_TORQUE_REG = 35;
static const uint8_t GOAL_TORQUE_SIZE = 2;
static const uint16_t GOAL_TORQUE_MIN = 0;
static const uint16_t GOAL_TORQUE_MAX = 1023;

static const uint8_t PRESENT_POSITION_REG = 37;
static const uint8_t PRESENT_POSITION_SIZE = 2;

static const uint8_t PRESENT_VELOCITY_REG = 39;
static const uint8_t PRESENT_VELOCITY_SIZE = 2;

static const uint8_t PRESENT_LOAD_REG = 41;
static const uint8_t PRESENT_LOAD_SIZE = 2;

static const uint8_t PRESENT_VOLTAGE_REG = 45;
static const uint8_t PRESENT_VOLTAGE_SIZE = 1;

static const uint8_t PRESENT_TEMPERATURE_REG = 46;
static const uint8_t PRESENT_TEMPERATURE_SIZE = 1;

static const uint8_t REGISTERED_INSTRUCTION_REG = 47;
static const uint8_t REGISTERED_INSTRUCTION_SIZE = 1;

static const uint8_t MOVING_REG = 49;
static const uint8_t MOVING_SIZE = 1;

static const uint8_t HARDWARE_ERROR_REG = 50;
static const uint8_t HARDWARE_ERROR_SIZE = 1;

static const uint8_t PUNCH_REG = 51;
static const uint8_t PUNCH_SIZE = 2;
static const uint16_t PUNCH_MIN = 0;
static const uint16_t PUNCH_MAX = 1023;

#endif // XL320_CONTROL_TABLE_H
