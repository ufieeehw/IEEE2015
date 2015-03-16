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

void do_action(int dxl_action, int dxl_id, int dxl_command){

    switch(dxl_action){
      case 1:  SetID(dxl_id, dxl_command);
      default: std::cout << "That action was not recognized. Use -h to view all available options.";
      break;
    }

 }

int main(int argc, char **argv)
{

  int dxl_id = (int)argv[1];
  int dxl_action = (int)argv[2];
  int dxl_command = (int)argv[3];

  std::string help = argv[1];

  if (argc == 2)
  {
    if (help.compare("-h") == 0)
    {
      tutorial();
      exit(0);
    }
  }


  if (argc >= 4)
  {
    InitDXL(dxl_id, 3);
    do_action(dxl_action, dxl_command, dxl_id);
  }else{
    std::cout << "Please enter a proper input command\n";
    std::cout << "Enter -h for input layout.";
    std::cout << help;
  }
}