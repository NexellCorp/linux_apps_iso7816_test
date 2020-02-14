#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#define TIOCGISO7816	0x5430
#define TIOCSISO7816	0x5431

struct serial_iso7816 {
	unsigned int	flags;			/* ISO7816 feature flags in UART6CTRL */
#define SER_ISO7816_ENABLED			(1 << 2)
#define SER_ISO7816_TX_ENABLED	(1 << 1)
#define SER_ISO7816_RX_ENABLED	(1 << 0)
#define SER_ISO7816_TX_STATUS		(1 << 3)
	unsigned int	status;			/* ISO7816 tx status return value */
	unsigned int	reserved[2];
};

struct serial_iso7816 iso7816conf;

#define TX_COMPLETE     3

/* use UART6 */
const char *portname = "/dev/ttyS0";
int uartFd = -1;

const unsigned char select_com[5] = { 0x00, 0xA4, 0x04, 0x00, 0x00 };
const unsigned char select_cmd[12] =  { 0x00, 0xA4, 0x04, 0x00, 0x07, 0xA0, 0x00, 0x00, 0x00, 0x03, 0x10, 0x10 };
unsigned char read_buf[256];

int set_interface_attribs (int fd, int speed, int parity)
{
  struct termios tty;

  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0) {
    fprintf (stderr, "error %d from tcgetattr", errno);
    return -1;
  }

  tty.c_cflag = speed | CS8 | CLOCAL | CREAD | CSTOPB | parity;

  tty.c_iflag = 0;                // disable break processing
  tty.c_oflag = 0;                // no remapping, no delays
  tty.c_lflag = 0;                // no signaling chars, no echo,
                                  // no canonical processing
  tty.c_cc[VMIN]  = 0;            // read doesn't block
  tty.c_cc[VTIME] = 0;            // 0.1 seconds read timeout

  /* now clean the modem line and activate the settings for the port */
	tcflush(fd, TCIOFLUSH);

  if (tcsetattr (fd, TCSANOW, &tty) != 0) {
    fprintf (stderr, "error %d from tcsetattr", errno);
    return -1;
  }

  return 0;
}

void set_blocking (int fd, int should_block)
{
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0) {
    fprintf (stderr, "error %d from tggetattr", errno);
    return;
  }

  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 1;            // 0.1 seconds read timeout

  if (tcsetattr (fd, TCSANOW, &tty) != 0)
    fprintf (stderr, "error %d setting term attributes", errno);

  return;
}

int uart_writestr(const char* string)
{
  int length = 0;

	length = write(uartFd, string, strlen(string));

  return length;
}

int uart_write(void* data, size_t len)
{
  int length;

	length = write(uartFd, data, len);

  return length;
}

ssize_t uart_read(void* buffer, size_t charsToRead)
{
	return read(uartFd, buffer, charsToRead);
}

// converts integer baud to Linux define
static int speed(int baud)
{
	switch (baud) {
	case B9600:
		return 9600;
	case B19200:
		return 19200;
	case B38400:
		return 38400;
	case B57600:
		return 57600;
  case B115200:
  	return 115200;
  }
}

int uart_open(const char* port, int baud, int blocking)
{
	uartFd = open (port, O_RDWR|O_NONBLOCK);
	if (uartFd < 0) {
			fprintf (stderr, "error %d opening %s: %s", errno, port, strerror (errno));
			return -1;
	}
	set_interface_attribs(uartFd, baud, PARENB);	// set speed, 8E2 (even parity)
	set_blocking(uartFd, blocking);	//set blocking mode
	printf("Port %s opened (Baud:%d).\n", port, speed(baud));
	return 1;
}

int uart_close(void)
{
  close(uartFd);
}

int iso_7816_reset(void)
{
  // read mode
  iso7816conf.flags = SER_ISO7816_ENABLED|SER_ISO7816_RX_ENABLED;
  ioctl(uartFd, TIOCSISO7816, &iso7816conf);

  printf("Reset card.\n");fflush(stdout);

  // Export the desired pin by writing to /sys/class/gpio/export
  int fdg = open("/sys/class/gpio/export", O_WRONLY);

  if (fdg == -1) {
    perror("Unable to open /sys/class/gpio/export");
    exit(1);
  }
  if (write(fdg, "76", 2) != 2) {
    perror("Error writing to /sys/class/gpio/export");
    exit(1);
  }
  close(fdg);

  // Set the pin to be an output by writing "out" to /sys/class/gpio/gpio76/direction
  fdg = open("/sys/class/gpio/gpio76/direction", O_WRONLY);
  if (fdg == -1) {
    perror("Unable to open /sys/class/gpio/gpio76/direction");
    exit(1);
  }
  if (write(fdg, "out", 3) != 3) {
    perror("Error writing to /sys/class/gpio/gpio76/direction");
    exit(1);
  }
  close(fdg);

  fdg = open("/sys/class/gpio/gpio76/value", O_WRONLY);
  if (fdg == -1) {
    perror("Unable to open /sys/class/gpio/gpio76/value");
    exit(1);
  }

  // nReset pin 200 usec on.
  if (write(fdg, "0", 1) != 1) {
    perror("Error writing to /sys/class/gpio/gpio76/value");
    exit(1);
  }
  if (write(fdg, "1", 1) != 1) {
    perror("Error writing to /sys/class/gpio/gpio76/value");
    exit(1);
  }
  usleep(200);
  if (write(fdg, "0", 1) != 1) {
    perror("Error writing to /sys/class/gpio/gpio76/value");
    exit(1);
  }
  close(fdg);

  // Unexport the pin by writing to /sys/class/gpio/unexport
  fdg = open("/sys/class/gpio/unexport", O_WRONLY);
  if (fdg == -1) {
    perror("Unable to open /sys/class/gpio/unexport");
    exit(1);
  }
  if (write(fdg, "76", 2) != 2) {
    perror("Error writing to /sys/class/gpio/unexport");
    exit(1);
  }
  close(fdg);

  return 0;
}

int iso_7816_read(unsigned char *rd_buf, int length)
{
  int i, j, k;
  int rd_cnt = 0;

  for(i=0, j=0; (i<length)&&(j<1000); j++) {
    usleep(10000);
    memset (&read_buf, 0, sizeof read_buf);
    rd_cnt = uart_read(read_buf, 256);

    if (rd_cnt>0) {
      memcpy(rd_buf+i, read_buf, rd_cnt);
      i += rd_cnt;
    }
  }

  return i;
}

int iso_7816_write(const unsigned char *wr_buf, int length)
{
  int len=0;

  iso7816conf.flags = SER_ISO7816_ENABLED|SER_ISO7816_TX_ENABLED;
  ioctl(uartFd, TIOCSISO7816, &iso7816conf);

  len = uart_write((void*)wr_buf, length);

  usleep(10);
  do {
    iso7816conf.flags = SER_ISO7816_TX_STATUS;
    ioctl(uartFd, TIOCSISO7816, &iso7816conf);
    ioctl(uartFd, TIOCGISO7816, &iso7816conf);
    usleep(10);
  } while (iso7816conf.status!=TX_COMPLETE);
  usleep(100);

  iso7816conf.flags = SER_ISO7816_ENABLED|SER_ISO7816_RX_ENABLED;
  ioctl(uartFd, TIOCSISO7816, &iso7816conf);

  return len;
}

int main(int argc, char* argv[] )
{
  int i, j;
  int rx_cnt=0;
  char rx_buffer[256];

	if(!uart_open(portname, B9600, 0))
		return -1;

  iso_7816_reset();

  memset (&rx_buffer, 0, sizeof rx_buffer);
  printf("ATR : \t");fflush(stdout);
  rx_cnt = iso_7816_read(rx_buffer, 23);
  for(i=0; i<rx_cnt; i++) {
    printf("%02X ", *(rx_buffer+i));
  }
  printf(" (%d)\n", rx_cnt);fflush(stdout);

  sleep(1);

  memset (&rx_buffer, 0, sizeof rx_buffer);
  printf("APDU (->SC) : \t");
  for(i=0; i<5; i++) {
    printf("%02X ", *(select_com+i));
  }
  printf("\nANS  (<-SC) : \t\t");fflush(stdout);

  rx_cnt=iso_7816_write(select_com, 5);

  rx_cnt = iso_7816_read(rx_buffer, 2);
  for(i=0; i<rx_cnt; i++) {
    printf("%02X ", *(rx_buffer+i));
  }
  printf(" (%d)\n", rx_cnt);fflush(stdout);

  iso7816conf.flags = 0;
  ioctl(uartFd, TIOCSISO7816, &iso7816conf);

  uart_close();

	return 0;
}
