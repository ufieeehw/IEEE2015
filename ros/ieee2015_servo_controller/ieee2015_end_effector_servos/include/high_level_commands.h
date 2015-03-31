#include <stdint.h>
#include <stdbool.h>

/*
  These should be common use cases that require knowledge of the registry layout.
  So these will be specific to XL-320

  In general the return value is the "success" of the tx/rx, while the actual queried
  values are returned through the arguments as appropriate.
*/

bool SendPing(uint8_t dxl_id, int* modelnum, int* firmware_version);
bool SetID(uint8_t dxl_id, uint8_t new_id);
bool SetBaud(uint8_t dxl_id, uint8_t new_baud);
bool SetCWAngleLimit(uint8_t dxl_id, uint16_t limit);
bool SetCW_WAngleLimit(uint8_t dxl_id, uint16_t limit);
bool SetControl(uint8_t dxl_id, uint8_t value);
bool SetAlarm(uint8_t dxl_id, uint8_t value);
bool SetTorqueEnable(uint8_t dxl_id, uint8_t is_enabled);
bool TorqueEnable(uint8_t dxl_id);
bool TorqueDisable(uint8_t dxl_id);
bool SetLED(uint8_t dxl_id, uint8_t color);
bool SetMaxTemp(uint8_t dxl_id, uint8_t temp);
bool SetTorque(uint8_t dxl_id, uint16_t value);
bool SetVelocity(uint8_t dxl_id, uint16_t velocity);
bool SetPosition(uint8_t dxl_id, uint16_t position);
bool SetDGain(uint8_t dxl_id, uint8_t value);
bool SetTemp(uint8_t dxl_id, uint8_t color);
bool SetIGain(uint8_t dxl_id, uint8_t value);
bool SetPGain(uint8_t dxl_id, uint8_t value);
bool ReadPosition(uint8_t dxl_id, uint16_t* position);
bool ReadCWAngle(uint8_t dxl_id, uint16_t* angle);
bool ReadCWWAngle(uint8_t dxl_id, uint16_t* angle);
bool ReadLoad(uint8_t dxl_id, uint16_t* load);
bool ReadStatusReturnLevel(uint8_t dxl_id, uint8_t* return_level);
bool SetMaxTorque(uint8_t dxl_id, uint16_t position);
bool ReadTorque(uint8_t dxl_id, uint16_t* position);
bool ReadMovingStatus(uint8_t dxl_id, bool* moving);
bool ReadControl(uint8_t dxl_id, uint8_t* mode);
bool ReadAlarm(uint8_t dxl_id, uint8_t* alarm);
bool ReadError(uint8_t dxl_id, uint8_t* error_return);
bool ReadUpperVoltage(uint8_t dxl_id, uint8_t* u_volt);
bool ReadLowerVoltage(uint8_t dxl_id, uint8_t* l_volt);
bool ReadCurrentVoltage(uint8_t dxl_id, uint8_t* c_volt);
bool ReadCurrentLoad(uint8_t dxl_id, uint16_t* c_load);
bool ReadCurrentTemp(uint8_t dxl_id, uint8_t* temp);
bool ReadMaxTemp(uint8_t dxl_id, uint8_t* m_temp);
