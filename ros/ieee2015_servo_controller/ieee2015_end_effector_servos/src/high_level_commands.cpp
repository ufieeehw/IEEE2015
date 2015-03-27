
#include <stdio.h>
#include <assert.h>

#include "high_level_commands.h"
#include "packet_utils.h"
#include "communications.h"
#include "xl320_control_table.h"


/* PING ---------------------------------------------------------------------------------------  */


bool SendPing(uint8_t dxl_id, int* modelnum, int* firmware_version) {
  uint8_t data[MAX_PACKET_BYTES];

  // Make packet.
  uint16_t num_parameters_tx = MakePingPacket(data, dxl_id);

  uint16_t num_parameters_rx = 4; // Error, model low, model high, firmware
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    *modelnum = GetWordParam(data, 1);
    *firmware_version = GetByteParam(data, 3);
    return true;
  } else {
    return false;
  }
}



/* WRITING COMMANDS ---------------------------------------------------------------------------- */ 

bool SetID(uint8_t dxl_id, uint8_t new_id) {
  uint8_t data[MAX_PACKET_BYTES];

  int led_register = 3;
  int num_parameters_tx = MakeWriteBytePacket(data, dxl_id, led_register, new_id);

  uint16_t num_parameters_rx = 1; // Error
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    return true;
  } else {
    return false;
  }
}

bool SetBaud(uint8_t dxl_id, uint8_t new_baud){
  uint8_t data[MAX_PACKET_BYTES];

  int baud_register = 4;
  int num_parameters_tx = MakeWriteBytePacket(data, dxl_id, baud_register, new_baud);

  uint16_t num_parameters_rx = 1; // Error
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    return true;
  } else {
    return false;
  }
}

bool SetCWAngleLimit(uint8_t dxl_id, uint16_t limit) {
  uint8_t data[MAX_PACKET_BYTES];

  int limit_register = 6;
  int num_parameters_tx = MakeWriteWordPacket(data, dxl_id, limit_register, limit);

  uint16_t num_parameters_rx = 2; // Error
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    return true;
  } else {
    return false;
  }
}

bool SetCW_WAngleLimit(uint8_t dxl_id, uint16_t limit) {
  uint8_t data[MAX_PACKET_BYTES];

  int limit_register = 8;
  int num_parameters_tx = MakeWriteWordPacket(data, dxl_id, limit_register, limit);

  uint16_t num_parameters_rx = 2; // Error
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    return true;
  } else {
    return false;
  }
}

bool SetControl(uint8_t dxl_id, uint8_t value) {

  uint8_t data[MAX_PACKET_BYTES];
  int position_register = 11;
  int num_parameters_tx = MakeWriteWordPacket(data, dxl_id, position_register, value);

  uint16_t num_parameters_rx = 1; // Error
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  
  if (result && error == 0) {
    return true;
  } else {
    return false;
  }
}

bool SetMaxTorque(uint8_t dxl_id, uint16_t position) {
  uint8_t data[MAX_PACKET_BYTES];

  int position_register = 15;
  int num_parameters_tx = MakeWriteWordPacket(data, dxl_id, position_register, position);

  uint16_t num_parameters_rx = 1; // Error
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    return true;
  } else {
    return false;
  }
}

bool SetAlarm(uint8_t dxl_id, uint8_t value) {
  uint8_t data[MAX_PACKET_BYTES];

  int gain_register = 18;
  int num_parameters_tx = MakeWriteWordPacket(data, dxl_id, gain_register, value);

  uint16_t num_parameters_rx = 1; // Error
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    return true;
  } else {
    return false;
  }
}

bool SetTorqueEnable(uint8_t dxl_id, uint8_t is_enabled) {
  assert(is_enabled == 0 || is_enabled == 1);
  uint8_t data[MAX_PACKET_BYTES];
  int torque_register = 24;
  int num_parameters_tx = MakeWriteBytePacket(data, dxl_id, torque_register, is_enabled);
  uint16_t num_parameters_rx = 1; // Error
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);
  if (dxl_id == BROADCAST_ID) {
    return result;
  }

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    return true;
  } else {
    return false;
  }
}

bool TorqueEnable(uint8_t dxl_id) {
  return SetTorqueEnable(dxl_id, 1);
}

bool TorqueDisable(uint8_t dxl_id) {
  return SetTorqueEnable(dxl_id, 0);
}


bool SetLED(uint8_t dxl_id, uint8_t color) {
  uint8_t data[MAX_PACKET_BYTES];

  int led_register = 25;
  int num_parameters_tx = MakeWriteBytePacket(data, dxl_id, led_register, color);

  uint16_t num_parameters_rx = 1; // Error
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    return true;
  } else {
    return false;
  }
}

bool SetDGain(uint8_t dxl_id, uint8_t value) {
  uint8_t data[MAX_PACKET_BYTES];

  int gain_register = 27;
  int num_parameters_tx = MakeWriteWordPacket(data, dxl_id, gain_register, value);

  uint16_t num_parameters_rx = 1; // Error
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    return true;
  } else {
    return false;
  }
}

bool SetIGain(uint8_t dxl_id, uint8_t value) {
  uint8_t data[MAX_PACKET_BYTES];

  int gain_register = 28;
  int num_parameters_tx = MakeWriteWordPacket(data, dxl_id, gain_register, value);

  uint16_t num_parameters_rx = 1; // Error
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    return true;
  } else {
    return false;
  }
}

bool SetPGain(uint8_t dxl_id, uint8_t value) {
  uint8_t data[MAX_PACKET_BYTES];

  int gain_register = 29;
  int num_parameters_tx = MakeWriteWordPacket(data, dxl_id, gain_register, value);

  uint16_t num_parameters_rx = 1; // Error
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    return true;
  } else {
    return false;
  }
}


bool SetPosition(uint8_t dxl_id, uint16_t position) {
  uint8_t data[MAX_PACKET_BYTES];

  int position_register = 30;
  int num_parameters_tx = MakeWriteWordPacket(data, dxl_id, position_register, position);

  uint16_t num_parameters_rx = 1; // Error
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    return true;
  } else {
    return false;
  }
}


bool SetVelocity(uint8_t dxl_id, uint16_t velocity) {
  uint8_t data[MAX_PACKET_BYTES];

  int velocity_register = 32;
  int min_velocity = 0;
  int num_parameters_tx = MakeWriteWordPacket(data, dxl_id, velocity_register, velocity);

  uint16_t num_parameters_rx = 1; // Error
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    return true;
  } else {
    return false;
  }
}


bool SetTorque(uint8_t dxl_id, uint16_t value) {
  uint8_t data[MAX_PACKET_BYTES];

  int position_register = 35;
  int num_parameters_tx = MakeWriteWordPacket(data, dxl_id, position_register, value);

  uint16_t num_parameters_rx = 1; // Error
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    return true;
  } else {
    return false;
  }
}
/* READING COMMANDS ----------------------------------------------------------------------------  */


bool ReadTorque(uint8_t dxl_id, uint16_t* position) {
  uint8_t data[MAX_PACKET_BYTES];

  int position_register = 15;
  int position_bytes = 2;
  int num_parameters_tx = MakeReadPacket(data, dxl_id, position_register, position_bytes);

  uint16_t num_parameters_rx = 1 + position_bytes; // Error, Position low, position high
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    *position = GetWordParam(data, 1);
    return true;
  } else {
    return false;
  }
}

bool ReadUpperVoltage(uint8_t dxl_id, uint8_t* u_volt) {
  uint8_t data[MAX_PACKET_BYTES];

  int position_register = 14;
  int position_bytes = 1;
  int num_parameters_tx = MakeReadPacket(data, dxl_id, position_register, position_bytes);

  uint16_t num_parameters_rx = 1 + position_bytes; // Error, Position low, position high
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    *u_volt = GetWordParam(data, 1);
    return true;
  } else {
    return false;
  }
}

bool ReadLowerVoltage(uint8_t dxl_id, uint8_t* l_volt) {
  uint8_t data[MAX_PACKET_BYTES];

  int position_register = 13;
  int position_bytes = 1;
  int num_parameters_tx = MakeReadPacket(data, dxl_id, position_register, position_bytes);

  uint16_t num_parameters_rx = 1 + position_bytes; // Error, Position low, position high
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    *l_volt = GetWordParam(data, 1);
    return true;
  } else {
    return false;
  }
}

bool ReadCurrentVoltage(uint8_t dxl_id, uint8_t* c_volt) {
  uint8_t data[MAX_PACKET_BYTES];

  int position_register = 45;
  int position_bytes = 1;
  int num_parameters_tx = MakeReadPacket(data, dxl_id, position_register, position_bytes);

  uint16_t num_parameters_rx = 1 + position_bytes; // Error, Position low, position high
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    *c_volt = GetWordParam(data, 1);
    return true;
  } else {
    return false;
  }
}
bool ReadCurrentTemp(uint8_t dxl_id, uint8_t* temp) {
  uint8_t data[MAX_PACKET_BYTES];

  int position_register = 46;
  int position_bytes = 1;
  int num_parameters_tx = MakeReadPacket(data, dxl_id, position_register, position_bytes);

  uint16_t num_parameters_rx = 1 + position_bytes; // Error, Position low, position high
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    *temp = GetWordParam(data, 1);
    return true;
  } else {
    return false;
  }
}
bool ReadCurrentLoad(uint8_t dxl_id, uint16_t* c_load) {
  uint8_t data[MAX_PACKET_BYTES];

  int position_register = 41;
  int position_bytes = 2;
  int num_parameters_tx = MakeReadPacket(data, dxl_id, position_register, position_bytes);

  uint16_t num_parameters_rx = 1 + position_bytes; // Error, Position low, position high
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    *c_load = GetWordParam(data, 1);
    return true;
  } else {
    return false;
  }
}

bool ReadCWAngle(uint8_t dxl_id, uint16_t* angle) {
  uint8_t data[MAX_PACKET_BYTES];

  int position_register = 6;
  int position_bytes = 2;
  int num_parameters_tx = MakeReadPacket(data, dxl_id, position_register, position_bytes);

  uint16_t num_parameters_rx = 1 + position_bytes; // Error, Position low, position high
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    *angle = GetWordParam(data, 1);
    return true;
  } else {
    return false;
  }
}

bool ReadCWWAngle(uint8_t dxl_id, uint16_t* angle) {
  uint8_t data[MAX_PACKET_BYTES];

  int position_register = 8;
  int position_bytes = 2;
  int num_parameters_tx = MakeReadPacket(data, dxl_id, position_register, position_bytes);

  uint16_t num_parameters_rx = 1 + position_bytes; // Error, Position low, position high
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    *angle = GetWordParam(data, 1);
    return true;
  } else {
    return false;
  }
}

bool ReadAlarm(uint8_t dxl_id, uint8_t* alarm) {
  uint8_t data[MAX_PACKET_BYTES];

  int position_register = 18;
  int position_bytes = 1;
  int num_parameters_tx = MakeReadPacket(data, dxl_id, position_register, position_bytes);

  uint16_t num_parameters_rx = 1 + position_bytes; // Error, Position low, position high
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    *alarm = GetWordParam(data, 1);
    return true;
  } else {
    return false;
  }
}
bool ReadPosition(uint8_t dxl_id, uint16_t* position) {
  uint8_t data[MAX_PACKET_BYTES];

  int position_register = 37;
  int position_bytes = 2;
  int num_parameters_tx = MakeReadPacket(data, dxl_id, position_register, position_bytes);

  uint16_t num_parameters_rx = 1 + position_bytes; // Error, Position low, position high
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    *position = GetWordParam(data, 1);
    return true;
  } else {
    return false;
  }
}

bool ReadLoad(uint8_t dxl_id, uint16_t* load) {
  uint8_t data[MAX_PACKET_BYTES];

  int position_register = 41;
  int position_bytes = 2;
  int num_parameters_tx = MakeReadPacket(data, dxl_id, position_register, position_bytes);

  uint16_t num_parameters_rx = 1 + position_bytes; // Error, Position low, position high
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    *load = GetWordParam(data, 1);
    return true;
  } else {
    return false;
  }
}

bool ReadStatusReturnLevel(uint8_t dxl_id, uint8_t* return_level) {
  uint8_t data[MAX_PACKET_BYTES];

  int return_level_register = 17;
  int return_level_bytes = 1;
  int num_parameters_tx = MakeReadPacket(data, dxl_id, return_level_register, return_level_bytes);

  uint16_t num_parameters_rx = 1 + return_level_bytes; // Error, Position low, position high
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    *return_level = GetByteParam(data, 1);
    return true;
  } else {
    return false;
  }
}

bool ReadControl(uint8_t dxl_id, uint8_t* mode) {
  uint8_t data[MAX_PACKET_BYTES];

  int control_register = 11;
  int control_bytes = 1;
  int num_parameters_tx = MakeReadPacket(data, dxl_id, control_register, control_bytes);

  uint16_t num_parameters_rx = 1 + control_bytes; // Error, Position low, position high
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    *mode = GetByteParam(data, 1);
    return true;
  } else {
    return false;
  }
}
bool ReadMovingStatus(uint8_t dxl_id, bool* moving) {
  uint8_t data[MAX_PACKET_BYTES];

  int moving_register = 49;
  int moving_bytes = 1;
  int num_parameters_tx = MakeReadBytePacket(data, dxl_id, moving_register);

  uint16_t num_parameters_rx = 1 + 1; // Error, moving byte
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    *moving = GetByteParam(data, 1);
    return true;
  } else {
    return false;
  }
}

bool ReadError(uint8_t dxl_id, uint8_t* error_return) {
  uint8_t data[MAX_PACKET_BYTES];

  int position_register = 50;
  int position_bytes = 1;
  int num_parameters_tx = MakeReadPacket(data, dxl_id, position_register, position_bytes);

  uint16_t num_parameters_rx = 1 + position_bytes; // Error, Position low, position high
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    *error_return = GetWordParam(data, 1);
    return true;
  } else {
    return false;
  }
}
