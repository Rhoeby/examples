/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Rhoeby Dynamics LLC
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rhoeby Dynamics nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*********************************************************************/

#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>

void start(void);
void stop(void);
unsigned char calcChecksum(const unsigned char *data, const unsigned int length);
void sig_handler(int s);

int got_ctrl_c = 0;
int port_fd;


int main(int argc, char *argv[])
{
  struct termios new_termios;
  char read_byte;
  char prev_read_byte;

  printf("Welcome to scanner2d program!\n");

  signal(SIGINT, sig_handler);
  
  port_fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
  if (port_fd == -1)
  {
    printf("port open failed!\n");
    exit(-1);
  }
  else
  {
    fcntl(port_fd, F_SETFL, 0);
  }

  tcgetattr(port_fd, &new_termios);
  cfmakeraw(&new_termios);
  cfsetospeed(&new_termios, B115200);
  tcsetattr(port_fd, TCSANOW, &new_termios);

  start();

  while (1) 
  {
    read(port_fd, &read_byte, 1);
    printf("%d ", read_byte);
    if(prev_read_byte==-1 && read_byte==-1)
      printf("\n");

    if (got_ctrl_c) {
      printf("Got Ctrl-C\n");
      stop();
      break;
    }

    prev_read_byte = read_byte;
  }

  printf("Exiting.\n");
}

void start(void)
{
  ssize_t msg_length = 5;
  ssize_t bytes_written;
  unsigned char command_buffer[5];

  command_buffer[0] = 0xFF;
  command_buffer[1] = 0xFF;
  command_buffer[2] = 0; // SCANNER2D_CMD_START
  command_buffer[3] = 0; // payload length
  command_buffer[4] = calcChecksum(command_buffer, msg_length-1);

  bytes_written = write(port_fd, command_buffer, msg_length);
  if (bytes_written != msg_length)
  {
    printf("start command failed!\n");
  }
}

void stop(void)
{
  ssize_t msg_length = 5;
  ssize_t bytes_written;
  unsigned char command_buffer[5];

  command_buffer[0] = 0xFF;
  command_buffer[1] = 0xFF;
  command_buffer[2] = 1; // SCANNER2D_CMD_STOP
  command_buffer[3] = 0; // payload length
  command_buffer[4] = calcChecksum(command_buffer, msg_length-1);

  bytes_written = write(port_fd, command_buffer, msg_length);
  if (bytes_written != msg_length)
  {
    printf("start command failed!\n");
  }
}

unsigned char calcChecksum(const unsigned char *data, const unsigned int length)
{
  unsigned int i;
  unsigned char checksum = 0;

  for (i=0; i<length; i++)
  {
    checksum += data[i];
  }

  return ~checksum;
}

void sig_handler(int s)
{
  if (s == 2) {
    got_ctrl_c = 1; 
  }
}

