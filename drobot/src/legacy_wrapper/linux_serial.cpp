#if !defined(LINUX_SERIAL_H) && !defined(win_x86)
#define LINUX_SERIAL_H

#include "drobot/legacy_wrapper/serial.h"  /* Std. function protos */
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>  /* Malloc */
#include <assert.h>

int OpenSerial(void **handle, const char *port_name)
{
	int fd; /* File descriptor for the port */

	fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)
	{
		fprintf(stderr, "Unable to open %s\n\r", port_name);
		return -3;
	}

	// Verify it is a serial port
	if (!isatty(fd))
	{
		close(fd);
		fprintf(stderr, "%s is not a serial port\n", port_name);
		return -3;
	}

	*handle = (int *) malloc(sizeof(int));
	**(int **) handle = fd;
	return fd;
}

int SetupSerial(void *handle)
{
  struct termios options;

  // Get the current options for the port...
  tcgetattr(*(int *) handle, &options);

  // 8 bits, 1 stop, no parity
  options.c_cflag = 0;
  options.c_cflag |= CS8;         // 8-bit input

  // Enable the receiver and set local mode...
  options.c_cflag |= (CLOCAL | CREAD);

  // Set the baud rates to 115200...
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);

  // No input processing
  options.c_iflag = 0;

  // No output processing
  options.c_oflag = 0;

  // No line processing
  options.c_lflag = 0;

  // read timeout
  options.c_cc[VMIN] = 0;    // non-blocking
  options.c_cc[VTIME] = 1;    // always return after 0.1 seconds

  // Set the new options for the port...
  tcsetattr(*(int *) handle, TCSAFLUSH, &options);

  return 0;
}

int WriteData(void *handle, const char *buffer, int length)
{
  int n = write(*(int *) handle, buffer, length);
  if (n < 0)
  {
    fprintf(stderr, "Error in serial write\r\n");
    return -1;
  }

  // serial port output monitor
  //#define TX_DEBUG
  #ifdef TX_DEBUG
  printf("TX:");
  int i;
  for (i=0; i<length; ++i) printf(" %x", (unsigned char)(buffer[i]));
  printf("\r\n");
  #endif

  return n;
}

int ReadData(void *handle, char *buffer, int length)
{
  int bytesRead = read(*(int *) handle, buffer, length);
  if (bytesRead <= 0)
  {
    return 0;
  }

  // serial port input monitor
  //#define RX_DEBUG
  #ifdef RX_DEBUG
  printf("RX:");
  int i;
  for (i=0; i<bytesRead; ++i) printf(" %x", (unsigned char)buffer[i]);
  printf("\r\n");
  #endif

  return bytesRead;
}

int CloseSerial(void *handle)
{
  if (NULL == handle)
  {
    return 0;
  }
  close(*(int *) handle);
  free(handle);
  return 0;
}

#endif // LINUX_SERIAL