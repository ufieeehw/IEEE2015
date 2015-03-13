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

bool KILL_SIGNAL = false;
void KillSwitch() {
  KILL_SIGNAL = true;
  bool result = TorqueDisable(BROADCAST_ID); 
}

void PositionLoopback(int usb2ax_index, uint8_t master_id, uint8_t slave_id) {
  int baud_num = 3;
  if(InitDXL(usb2ax_index, baud_num) == 0 ) {
    printf("Failed to open device: %d, baud num: %d\n", usb2ax_index, baud_num);
    TerminateDXL();
    return;
  }

  bool result = TorqueEnable(slave_id);
  uint16_t pos;
  while (!KILL_SIGNAL) {
    result = ReadPosition(master_id, &pos);
    if (result) {
      result = SetPosition(slave_id, pos);
    }
    usleep(1000);
  }

  TorqueDisable(slave_id);

  TerminateDXL();
}

// Enables torque, centers servo, waits until movement stops, disables torque.
void CenterPosition(int usb2ax_index, int dxl_id) {
  int baud_num = 3;
  if(InitDXL(usb2ax_index, baud_num) == 0 ) {
    printf("Failed to open device: %d, baud num: %d\n", usb2ax_index, baud_num);
    TerminateDXL();
    return;
  }

  bool result = TorqueEnable(dxl_id);
  bool moving = false;
  if (result) {
    printf("Enabled torque\n");
    result = SetPosition(dxl_id, 512);
    moving = true;
    if (result) {
      printf("Set position\n");
      while (moving) {
        usleep(10000);
        ReadMovingStatus(dxl_id, &moving);
      }
      printf("Done moving\n");
    }
  }
  result = TorqueDisable(dxl_id);
  if (result) {
    printf("Disabled torque\n");
  }
  TerminateDXL();
}

void ListDevices() {
  DIR *dir;
  struct dirent *ent;
  char* devices_dir = "/dev/";
  char* expected_prefix = "ttyACM";
  int i;
  if ((dir = opendir (devices_dir)) != NULL) {
    while ((ent = readdir (dir)) != NULL) {
      if (strlen(expected_prefix) <= strlen(ent->d_name) &&
        memcmp(ent->d_name, expected_prefix, strlen(expected_prefix)) == 0) {
        printf("%s\n", ent->d_name);
      }
    }
    closedir (dir);
  } else {
    printf("Couldn't find device directory: %s\n", devices_dir);
  }
}

int main(int argc, char* argv[]) {

  struct sigaction action;
  memset(&action, 0, sizeof(struct sigaction));
  action.sa_handler = KillSwitch;
  sigaction(SIGTERM, &action, NULL);
  sigaction(SIGINT, &action, NULL);

  int device_num = 0; // Default to device ID 0
  if (argc >= 2) {
    char *end;
    long value = strtol(argv[1], &end, 10); 
    if (!(end == argv[1] || *end != '\0' || errno == ERANGE)) {
      PositionLoopback(value, 1, 2);
      //CenterPosition(value, 1);
    } else {
      printf("Arg 1 (%s) was not recognized as an integer device number.\n", argv[1]);
    }
  } else {
    printf("Enter USB2AX ttyACM* number as argument 1!\n");
    printf("Here are some possible device ID's from /dev/*.\n");
    ListDevices();
  }
  return 0;
}
