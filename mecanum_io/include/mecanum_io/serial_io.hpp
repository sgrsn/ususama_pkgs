/*******************************************************************************
Copyright (c) 2021, Hidaka Sato
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met: 
1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution. 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies, 
either expressed or implied, of the FreeBSD Project.
*******************************************************************************/

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>

class SerialIO
{
  private:
  int fd_;  // file descriptor for smc

  public:
  SerialIO(){}
  SerialIO(const char * device, uint32_t baud_rate)
  {
    fd_ = openSerialPort(device, baud_rate);
  }

  bool hasFileDescriptor()
  {
    if(fd_ < 0){ return false; }
    else{ return true; }
  }

  int openSerialPort(const char * device, uint32_t baud_rate)
  {
    int fd = open(device, O_RDWR | O_NOCTTY);
    if (fd == -1)
    {
      perror(device);
      return -1;
    }
    
    // Flush away the input and output data.
    int result = tcflush(fd, TCIOFLUSH);
    if (result)
    {
      perror("tcflush failed");  // just a warning, not a fatal error
    }
    
    // Get the parameters for fd and set them to termios, the structure referred to by options.
    struct termios options;
    result = tcgetattr(fd, &options);
    if (result)
    {
      perror("tcgetattr failed");
      close(fd);
      return -1;
    }
    
    // Turn off any options that might interfere with our ability to send and
    // receive raw binary bytes.
    options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
    options.c_oflag &= ~(ONLCR | OCRNL);
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    
    // Set up timeouts: Calls to read() will return as soon as there is
    // at least one byte available or when 100 ms has passed.
    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN] = 0;
    
    // This code only supports certain standard baud rates. Supporting
    // non-standard baud rates should be possible but takes more work.
    switch (baud_rate)
    {
    case 4800:   cfsetospeed(&options, B4800);   break;
    case 9600:   cfsetospeed(&options, B9600);   break;
    case 19200:  cfsetospeed(&options, B19200);  break;
    case 38400:  cfsetospeed(&options, B38400);  break;
    case 115200: cfsetospeed(&options, B115200); break;
    default:
        fprintf(stderr, "warning: baud rate %u is not supported, using 9600.\n",
        baud_rate);
        cfsetospeed(&options, B9600);
        break;
    }
    cfsetispeed(&options, cfgetospeed(&options));
    
    // Apply the settings immediately.
    result = tcsetattr(fd, TCSANOW, &options);
    if (result)
    {
      perror("tcsetattr failed");
      close(fd);
      return -1;
    }

    fd_ = fd;

    return fd;
  }
  void closePort()
  {
    close(fd_);
  }

  // Writes bytes to the serial port from the buffer, size is the maximum bytes
  // The return value is 0: success, -1: failure.
  int writePort(const uint8_t * buffer, size_t size)
  {
    // Write a maximum of size bytes from the buffer pointed to by buffer
    // to the file referenced by the file descriptor fd_.
    ssize_t result = write(fd_, buffer, size);
    if (result != (ssize_t)size)
    {
      perror("failed to write to port");
      return -1;
    }
    return 0;
  }


  // Reads bytes from the serial port.
  // Returns after all the desired bytes have been read, or if there is a
  // timeout or other error.
  // The received bytes are stored in buffer, size is the maximum bytes.
  // Returns the number of bytes successfully read into the buffer, or -1 if
  // there was an error reading.
  ssize_t readPort(uint8_t * buffer, size_t size)
  {
    size_t received = 0;
    while (received < size)
    {
      ssize_t r = read(fd_, buffer + received, size - received);
      if (r < 0)
      {
        perror("failed to read from port");
        return -1;
      }
      if (r == 0)
      {
        // Timeout
        break;
      }
      received += r;
    }
    return received;
  }
 
};