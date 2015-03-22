#include <stdio.h>
#include <assert.h>

#include "high_level_commands.h"
#include "packet_utils.h"
#include "communications.h"
#include "xl320_control_table.h"


/* READING COMMANDS */

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

/* WRITING COMMANDS */

bool SetID(uint8_t dxl_id, uint8_t new_id) {
  uint8_t data[MAX_PACKET_BYTES];

  int led_register = 3;
  int num_parameters_tx = MakeWriteBytePacket(data, dxl_id, led_register, new_id);

  uint16_t num_parameters_rx = 1; // Error
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    return true;
    printf("ID change succesful\n");
    printf("ID is now%d\n", new_id);
  } else {
    return false;
    printf("ID change unsuccesful\n");
  }
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


bool SetVelocity(uint8_t dxl_id, uint16_t velocity) {
  uint8_t data[MAX_PACKET_BYTES];

  int velocity_register = 32;
  int max_velocity = 1023;
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

/*
bool SetPositionAndVelocity(uint8_t dxl_id, uint16_t position, uint16_t velocity) {
  uint8_t data[MAX_PACKET_BYTES];

  int velocity_register = 32;
  int max_velocity = 1023;
  int min_velocity = 0;
  int num_parameters_tx = MakeWritePacket(data, dxl_id, velocity_register, velocity);

  uint16_t num_parameters_rx = 1; // Error
  bool result = TXRXPacket(data, num_parameters_tx, num_parameters_rx);

  uint8_t error = GetByteParam(data, 0);
  if (result && error == 0) {
    return true;
  } else {
    return false;
  }
}
*/
