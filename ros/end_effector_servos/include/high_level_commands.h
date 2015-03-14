#include <stdint.h>
#include <stdbool.h>

/*
  These should be common use cases that require knowledge of the registry layout.
  So these will be specific to XL-320

  In general the return value is the "success" of the tx/rx, while the actual queried
  values are returned through the arguments as appropriate.
*/

/*
  Send a ping. Sets the modelnum and firmware_version to what is received from
  the device.
*/
bool SendPing(uint8_t dxl_id, int* modelnum, int* firmware_version);

// Query the current position of the servo in native units. (0 to 1024)
bool ReadPosition(uint8_t dxl_id, uint16_t* position);
bool SetPosition(uint8_t dxl_id, uint16_t position);

/*
  Set the return status level. This settings changes the verbosity of response
  from the dynamixels after each command. Can be used to optimize for transmission size, etc.
*/
bool ReadStatusReturnLevel(uint8_t dxl_id, uint8_t* return_level);

// Check whether the dynamixel is still moving.
bool ReadMovingStatus(uint8_t dxl_id, bool* moving);

/*
  Set the LED color/on/off.
  The color should be a number 0-8.
  The first 3 bits correspond to R G and B,
  So, 0 (0b000) is off and 8 (0b111) is white.
*/

bool SetID(uint8_t dxl_id, uint8_t new_id);
bool SetLED(uint8_t dxl_id, uint8_t color);

// "Torque enable" is whether the device is actuating for not.
bool SetTorqueEnable(uint8_t dxl_id, uint8_t is_enabled);
bool TorqueEnable(uint8_t dxl_id);
bool TorqueDisable(uint8_t dxl_id);


bool SetVelocity(uint8_t dxl_id, uint16_t velocity);
bool SetPositionAndVelocity(uint8_t dxl_id, uint16_t position, uint16_t velocity);
