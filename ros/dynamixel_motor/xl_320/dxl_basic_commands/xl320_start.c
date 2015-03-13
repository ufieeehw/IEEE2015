#include <stdio.h>
#include <stdbool.h>
#include <termio.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <dirent.h>
#include <string.h>
#include <time.h>

#include "high_level_commands.h"
#include "communications.h"

void main_loop(){
  while(1){
    SetVelocity(1,1000);
    SetPosition(1,500);
    sleep(1);
  }
}

void ListAllDynamixels(int usb2ax_index) {
  int baud_num, dxl_id;
  int model_num, firmware_version;
  for (baud_num = 3; baud_num >= 0; baud_num--) {
    if(InitDXL(usb2ax_index, baud_num) == 0 ) {
      printf("Failed to open device: %d, baud num: %d\n", usb2ax_index, baud_num);
      TerminateDXL();
      return;
    } else {
      printf("Searching on device: %d, baud num: %d\n", usb2ax_index, baud_num);
    }

    for (dxl_id = 0; dxl_id < 200; dxl_id++) {
      if (SendPing(dxl_id, &model_num, &firmware_version)) {
        
        printf("Device found| Baud rate %d, ID: %d, Model number: %d, Firmware version: %d\n", baud_num, dxl_id, model_num, firmware_version);
        SetID(1,2);
        main_loop();
      }
    }
    }
    // ID 200 is one of the controllers or something.
    // ID 253 is USB2AX
    // ID 254 is broadcast
    for (dxl_id = 201; dxl_id < 253; dxl_id++) {
      if (SendPing(dxl_id, &model_num, &firmware_version)) {
        printf("Device found| Baud rate: %d, ID: %d, Model number: %d, Firmware version: %d\n", baud_num, dxl_id, model_num, firmware_version);
      }
    }
  
}

void ListDevices() {
  DIR * dir;
  struct dirent * ent;
  char* devices_dir = "/dev/";
  char* expected_prefix = "ttyACM";
  int i;
  if ((dir = opendir (devices_dir)) != NULL) {
    while ((ent = readdir (dir)) != NULL) {
      if (strlen(expected_prefix) <= strlen(ent->d_name)) {
        printf("%s\n", ent->d_name);
      }
      else{
        printf("%s\n", "wrong" );
      }
    }
    closedir (dir);
  } else {
    printf("Couldn't find device directory: %s\n", devices_dir);
  }
}

int main(int argc, char* argv[]) {
  int device_num = 0; // Default to device ID 0
  if (argc >= 2) {
    char *end;
    long value = strtol(argv[1], &end, 10); 
    if (!(end == argv[1] || *end != '\0' || errno == ERANGE)) {
      ListAllDynamixels(device_num);
    } else {
      printf("Arg 1 (%s) was not recognized as an integer device number.\n", argv[1]);
    }
  } else {
    printf("Enter USB2AX ttyACM number as argument 1!\n");
    printf("Here are some possible device ID's from /dev/*.\n");
    ListDevices();
  }
  return 0;
}
