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

#define STATUS_MSG_LEN_MAX 20

void start(void);
void stop(void);
void setScanPeriod(const unsigned int period);
void parseMsgData(const unsigned char msg_byte);
unsigned char calcChecksum(const unsigned char *data, const unsigned int length);
void sig_handler(int s);

int port_fd;
unsigned int got_ctrl_c = 0;
unsigned char msg_buffer[STATUS_MSG_LEN_MAX];


/*--------------------------------------------------------------------------
  
  This program does the following: 
    - connects to the R2D Scanner 
    - starts the scanner
    - then recieves and prints the status messages 

--------------------------------------------------------------------------*/

int main(int argc, char *argv[])
{
  struct termios new_termios;
  unsigned char read_byte;

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

  setScanPeriod(500);

  while (1) 
  {
    read(port_fd, &read_byte, 1);

    parseMsgData(read_byte);

    if (got_ctrl_c) {
      printf("Got Ctrl-C\n");
      stop();
      break;
    }
  }

  printf("Exiting.\n");
}

/*--------------------------------------------------------------------------*/
void parseMsgData(const unsigned char msg_byte)
{
  unsigned char checksum;
  static unsigned int parser_state = 0;
  static unsigned int payload_length = 0;
  static unsigned int payload_byte_count = 0;

  switch (parser_state)
  {
  case 0:
    msg_buffer[0] = msg_byte;
    if (msg_byte == 0xFF)
      parser_state = 1;
    break;
  case 1:
    msg_buffer[1] = msg_byte;
    if (msg_byte == 0xFF)
      parser_state = 2;
    else
      parser_state = 0;
    break;
  case 2:
    msg_buffer[2] = msg_byte;
    if (msg_byte == 1) {
      printf("Got msg_type 1 (Status)\n");
      parser_state = 3;
    }
    else {
      parser_state = 0;
    }
    break;
  case 3:
    msg_buffer[3] = msg_byte;
    payload_length = (msg_byte << 8);
    parser_state = 4;
    break;
  case 4:
    msg_buffer[4] = msg_byte;
    payload_length |= msg_byte;
    if (payload_length >= STATUS_MSG_LEN_MAX)
    {
      printf("Got bad payload length (%d)!\n", payload_length);
      parser_state = 0;
    }
    else if (payload_length)
    {
      payload_byte_count = 0;
      parser_state = 5;
    }
    else
    {
      parser_state = 6;
    }
    break;
  case 5:
    msg_buffer[payload_byte_count + 5] = msg_byte;
    payload_byte_count++;
    if (payload_byte_count + 5 > STATUS_MSG_LEN_MAX)
    {
      printf("Got bad payload byte count (%d)!\n", payload_byte_count);
      parser_state = 0;
    }
    else if (payload_byte_count >= payload_length)
    {
      parser_state = 6;
    }
    break;
  case 6:
    checksum = calcChecksum(msg_buffer, payload_byte_count + 5);
    if (msg_byte == checksum)
    {
      printf("  Samples per scan: %d\n", msg_buffer[5] | (msg_buffer[6] << 8));
      printf("  Scan period: %d\n", msg_buffer[7] | (msg_buffer[8] << 8));
      printf("  Flags: 0x%x\n", msg_buffer[9]);

      if (payload_length > 5) {
        unsigned int range_reading = msg_buffer[10] | (msg_buffer[11] << 8);
        printf("  Range reading in millimeters [inches]: %d [%f]\n", range_reading, range_reading/25.4);
      }

      if (payload_length > 7) {
        printf("  Firmware version: %x.%02x\n", msg_buffer[13], msg_buffer[12]);
      }
    }
    else
    {
      printf("Message checksum failed!\n");
    }
    parser_state = 0;
    break;
  default:
    printf("Bad parser state!\n");
    parser_state = 0;
    break;
  }
}

/*--------------------------------------------------------------------------*/
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

/*--------------------------------------------------------------------------*/
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

/*--------------------------------------------------------------------------*/
void setScanPeriod(const unsigned int period)
{
  ssize_t msg_length = 7;
  ssize_t bytes_written;
  unsigned char command_buffer[7];

  command_buffer[0] = 0xFF;
  command_buffer[1] = 0xFF;
  command_buffer[2] = 2; // SCANNER2D_CMD_SET_SCAN_PERIOD;
  command_buffer[3] = 2; // payload length
  command_buffer[4] = (period & 0xFF);
  command_buffer[5] = (period & 0xFF00) >> 8;
  command_buffer[6] = calcChecksum(command_buffer, msg_length-1);

  bytes_written = write(port_fd, command_buffer, msg_length);
  if (bytes_written != msg_length)
  {
    printf("setScanPeriod command failed!\n");
  }
}

/*--------------------------------------------------------------------------*/
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

/*--------------------------------------------------------------------------*/
void sig_handler(int s)
{
  if (s == 2) {
    got_ctrl_c = 1; 
  }
}

