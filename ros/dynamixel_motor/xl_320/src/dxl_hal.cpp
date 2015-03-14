#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#include "dxl_hal.h"

// Test this out and see & modify.
int latency_time_ms = 10;

int gSocket_fd  = -1;
long  glStartTime = 0;
float gfRcvWaitTime = 0.0f;
float gfByteTransTime = 0.0f;

char  gDeviceName[20];

int dxl_hal_open(int deviceIndex, int baudrate)
{
  struct termios newtio;
  // struct serial_struct serinfo;
  char dev_name[100] = {0, };

  sprintf(dev_name, "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Cable_FTTKRX9B-if03-port0");

  strcpy(gDeviceName, dev_name);
  memset(&newtio, 0, sizeof(newtio));
  dxl_hal_close();
  
  if((gSocket_fd = open(gDeviceName, O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0) {
    fprintf(stderr, "device open error: %s\n", dev_name);
    dxl_hal_close();
    return 0;
  }

  int baud_num;

  switch (baudrate) {
  case 9600:
    baud_num = B9600;
    break;
  case 57600:
    baud_num = B57600;
    break;
  case 115200:
    baud_num = B115200;
    break;
  case 1000000:
    baud_num = B1000000;
    break;
  default:
    baud_num = B1000000;
  }
  newtio.c_cflag    = baud_num | CS8|CLOCAL|CREAD;
  newtio.c_iflag    = IGNPAR;
  newtio.c_oflag    = 0;
  newtio.c_lflag    = 0;
  newtio.c_cc[VTIME]  = 0;  // time-out 값 (TIME * 0.1초) 0 : disable
  newtio.c_cc[VMIN] = 0;  // MIN 은 read 가 return 되기 위한 최소 문자 개수

  tcflush(gSocket_fd, TCIFLUSH);
  tcsetattr(gSocket_fd, TCSANOW, &newtio);
  
  if(gSocket_fd == -1)
    return 0;
  
  dxl_hal_close();
  
  gfByteTransTime = (float)((1000.0f / baudrate) * 10.0f);
  
  strcpy(gDeviceName, dev_name);
  memset(&newtio, 0, sizeof(newtio));
  dxl_hal_close();
  
  if((gSocket_fd = open(gDeviceName, O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0) {
    fprintf(stderr, "device open error: %s\n", dev_name);
    dxl_hal_close();
    return 0;
  }

  newtio.c_cflag    = baud_num | CS8|CLOCAL|CREAD;
  newtio.c_iflag    = IGNPAR;
  newtio.c_oflag    = 0;
  newtio.c_lflag    = 0;
  newtio.c_cc[VTIME]  = 0;  // time-out 값 (TIME * 0.1초) 0 : disable
  newtio.c_cc[VMIN] = 0;  // MIN 은 read 가 return 되기 위한 최소 문자 개수

  tcflush(gSocket_fd, TCIFLUSH);
  tcsetattr(gSocket_fd, TCSANOW, &newtio);

  /*
    speed_t ispeed = cfgetispeed(&newtio);
    speed_t ospeed = cfgetospeed(&newtio);
    printf("ISPEED %d, OSPEED %d\n", ispeed, ospeed);
    printf("B1000000 %d, B115200 %d, B57600 %d, B9600 %d\n", B1000000, B115200, B57600, B9600);
  */
  
  return 1;
}

void dxl_hal_close()
{
  if(gSocket_fd != -1)
    close(gSocket_fd);
  gSocket_fd = -1;
}

int dxl_hal_set_baud( int baudrate )
{
  // struct serial_struct serinfo;
  
  if(gSocket_fd == -1)
    return 0;
  
  
  //dxl_hal_close();
  //dxl_hal_open(gDeviceName, baudrate);
  
  gfByteTransTime = (float)((1000.0f / baudrate) * 10.0f);
  return 1;
}

void dxl_hal_clear(void)
{
  tcflush(gSocket_fd, TCIFLUSH);
}

int dxl_hal_tx( unsigned char *pPacket, int numPacket )
{
  return write(gSocket_fd, pPacket, numPacket);
}

int dxl_hal_rx( unsigned char *pPacket, int numPacket )
{
  memset(pPacket, 0, numPacket);
  return read(gSocket_fd, pPacket, numPacket);
}

static inline long myclock()
{
  struct timeval tv;
  gettimeofday (&tv, NULL);
  return (tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

void dxl_hal_set_timeout( int NumRcvByte )
{
  glStartTime = myclock();
  gfRcvWaitTime = (float)(gfByteTransTime*(float)NumRcvByte + 2*latency_time_ms + 2.0f);
}

int dxl_hal_timeout(void)
{
  long time;
  
  time = myclock() - glStartTime;
  
  if(time > gfRcvWaitTime)
    return 1;
  else if(time < 0)
    glStartTime = myclock();
    
  return 0;
}
