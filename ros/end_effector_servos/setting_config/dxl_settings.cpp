#include <stdio.h>
#include <signal.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <dirent.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <stdlib.h>
#include <typeinfo>



#include "high_level_commands.h"
#include "communications.h"

void tutorial(){

  std::cout << "\nPROPER INPUT:\n";
  std::cout << "./dxl_settings [ID] [ACTION] [COMMAND]\n";
  std::cout << "Please enter all values as integers\n\n";
  std::cout << "Available Actions:\n";
  std::cout << "Change ID --> 1\n\n";
  std::cout << "Available Commands:\n";
  std::cout << "ID Values --> 1-252\n";
}

void do_action(uint8_t dxl_action, uint8_t dxl_id, uint8_t dxl_command){

    switch(dxl_action){
      case 1:  SetID(dxl_id, dxl_command);
               std::cout << "ID change called\n";
      break;
      default: std::cout << "That action was not recognized. Use -h to view all available options.";
      break;
    }

 }

int main(int argc, char **argv)
{

  uint8_t dxl_id, dxl_action, dxl_command;
  

  std::cout << "ID: ";
  std::cin >> dxl_id;
  std::cout << "Command: ";
  std::cin >> dxl_action;
  std::cout << "Setting: ";
  std::cin >> dxl_command;


  if (argc == 2)
  {
    std::string help = argv[1];
    if (help.compare("-h") == 0)
    {
      tutorial();
      exit(0);
    }
  }else{
    InitDXL(dxl_id, 3);
    SetLED(dxl_id, 7);
    do_action(dxl_action, dxl_command, dxl_id);
  }
}