/*
 * Ethernet over serial link (master/slave).
 *
 * Author: Rafal Vonau <rafal.vonau@elfin-pe.pl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or above as
 * published by the Free Software Foundation.
 */
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <linux/types.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <asm-generic/errno.h>
#include <errno.h>
#include <stdio.h>
#include <ctype.h>         /* for isspace           */
#include <string.h>
#include <stdlib.h>        /* for malloc            */
#include <time.h>
#include <unistd.h>        /* for close             */
#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>    /* for connect and socket*/
#include <sys/stat.h>
#include <sys/sendfile.h>
#include <netinet/in.h>    /* for sockaddr_in       */
#include <netinet/tcp.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/poll.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <fcntl.h>         /* for open modes        */
#include <dlfcn.h>
#include <arpa/inet.h>
#include <linux/if.h>
#include <linux/if_tun.h>
#include "tun-driver.h"
#include <termios.h>

#ifndef  SD_SEND
#define SD_SEND (1)
#endif

#define RS232_MAX_MEMORY_BUFF (4096)


void set_NDELAY(int fd, int ndelay);
void set_NDELAY(int fd, int ndelay)
{
	setsockopt( fd, SOL_TCP, TCP_NODELAY, (void*)&ndelay, sizeof(ndelay) );
}
//====================================================================================================

/*!
 * \brief Flush and close network socket
 * \param fd - socket handle.
 */
void rs232_closeSocket ( int fd )
{
	shutdown ( fd, SD_SEND ); /* Flush data            */
	close ( fd );             /* Close socket          */
}
//====================================================================================================


int setNonBlocking ( int fd )
{
	int flags;
	if ( -1 == ( flags = fcntl ( fd, F_GETFL, 0 ) ) )
		flags = 0;
	return fcntl ( fd, F_SETFL, flags | O_NONBLOCK | O_NDELAY );
}
//====================================================================================================

int setBlocking ( int fd )
{
	int flags;
	if ( -1 == ( flags = fcntl ( fd, F_GETFL, 0 ) ) )
		flags = 0;
	return fcntl ( fd, F_SETFL, flags & ( ~O_NONBLOCK ) );
}
//====================================================================================================
     
int read_n(int fd, char *buf, int n)
{
	int cnt=0, nread, left = n;

	while(left > 0) {
		nread = read(fd, buf, left);
		if (nread<=0) {
			return cnt;
		} else {
			left -= nread;
			buf += nread;
			cnt += nread;
		}
	}
	return cnt;
}
//===========================================================================

int eth_over_serial_read_rs(int fd, char *buf, int maxsize)
{
	struct pollfd fds; // poll
	int res = -2;

	fds.fd = fd;
	fds.events = POLLIN;

	if (poll(&fds, 1, 1000) > 0) {
		res = read_n(fd, buf, maxsize);
	}
	return res;
}
//==========================================================================================

/*!
 * \brief Configure RS port.
 * \param fd - file handle,
 */
void configureRS(int fd, int speed);
void configureRS(int fd, int speed)
{
	struct termios options;

	/*Get the current options for the port*/
	tcgetattr(fd, &options);
	/*Set Baud rate*/
	cfsetispeed(&options, speed);
	cfsetospeed(&options, speed);

	/*Enable received and set local mode*/
	options.c_cflag |= (CLOCAL | CREAD);
	/*Set new options for port*/
	tcsetattr( fd, TCSANOW, &options );

	/*Set data bits*/
	options.c_cflag &= ~CSIZE;                   /* Mask the character size bits */
	options.c_cflag |= CS8;          /* Select 8 data bits */

	/* Parity = None */
	options.c_cflag &= ~PARENB; // Disable parity

	// Set Stop bits
	options.c_cflag &= ~(CSTOPB);

	/*Set RAW input*/
	options.c_lflag &= ~(ICANON | ECHO | ISIG);
	options.c_cflag &= ~CRTSCTS;
	/*Set Raw output*/
	options.c_oflag &= ~OPOST;
	/* RTS CTS controll */
	//options.c_cflag |= CRTSCTS;


	/*Set timeout to 1,5 sec*/

	//don't map CR LF
	options.c_iflag &= ~(INLCR | ICRNL | IGNBRK | IGNCR | IXON | IXOFF | IXANY);
	options.c_cc[VMIN] = 0;
	options.c_cc[VTIME] = 15;
	tcflush (fd, TCIFLUSH);
	// Set the new options for the port...
	tcsetattr(fd, TCSANOW, &options);
}
//==========================================================================================

typedef struct 
{
	char st;
	char cmd;
	__u16 size;
	char buf[RS232_MAX_MEMORY_BUFF+100];
} ms_hdr_t;

int main(int argc, char *argv[])
{
	int c,handle,i,err,idx, server,baudrate=115200,b_rate;
	struct pollfd fds[2];
	char adapterName[4095];
	int master=1;
	char dev[255];
	unsigned short nr;
	ms_hdr_t hdr;

	printf("eth_over_serial -d device -c -B boudrate\n\n");
	strcpy(dev,"/dev/ttyUSB0");
	while ((c = getopt (argc, argv, "cB:d:")) != -1)
	switch (c) {
		case 'c': master=0;break;
		case 'B': baudrate = atoi(optarg);break;
		case 'd': strcpy(dev,optarg);break;
		default:break;
	}
	printf("parse ready\n");
	switch (baudrate) {
		case 1200:   b_rate = B1200;break;
		case 1800:   b_rate = B1800;break;
		case 2400:   b_rate = B2400;break;
		case 4800:   b_rate = B4800;break;
		case 9600:   b_rate = B9600;break;
		case 19200:  b_rate = B19200;break;
		case 38400:  b_rate = B38400;break;
		case 57600:  b_rate = B57600;break;
		case 115200: b_rate = B115200;break;
		case 460800: b_rate = B460800;break;
		case 500000: b_rate = B500000;break;
		case 576000: b_rate = B576000;break;
		case 921600: b_rate = B921600;break;
		case 1000000: b_rate = B1000000;break;
		case 1500000: b_rate = B1500000;break;
		case 2000000: b_rate = B2000000;break;
		case 2500000: b_rate = B2500000;break;
		case 3000000: b_rate = B3000000;break;
		case 3500000: b_rate = B3500000;break;
		case 4000000: b_rate = B4000000;break;
		default:printf("unknown baudrate %d !!!\n", baudrate);break;
	}


	handle = open(dev, O_RDWR | O_NOCTTY );
	if (handle < 0) { printf("Port open error (%d): %s\n", errno, strerror(errno));exit(-1); }
	printf("open ready\n");
	configureRS(handle, b_rate);
	printf("configure ready (speed = %d)\n",b_rate);

	
	while (1) {
		/* Allocate TAP interface */
		sprintf(adapterName,"tap%d",0);
		server = tun_alloc(adapterName, IFF_TAP | IFF_NO_PI);
		//set_NDELAY(server, 1);
		/* Add TAP to poll */
		fds[0].fd = server;
		fds[0].events = POLLIN;
		fds[0].revents = 0;
		/* Add RS232 to poll */
		fds[1].fd = handle;
		fds[1].events = POLLIN;
		fds[1].revents = 0;
		if (master) {
			/* ===================================== */
			/* =========-- Server mode --=========== */
			/* ===================================== */
			printf("eth_over_serial - master mode\n");
			for (;;) {
				fds[0].revents = 0;
				fds[1].revents = 0;
				i = poll(fds, 1, 10);
				/* Process requests */
				if (i > 0) {
					/* read data from TAP interface */
					if (fds[0].revents & POLLIN) {
							fds[0].revents = 0;
							idx = read(fds[0].fd, hdr.buf, RS232_MAX_MEMORY_BUFF);
							if (idx <= 0) {continue;}
							printf("Got packet from TUN (size=%d)\n", idx);
							/* Send new data over UART */
							hdr.st=':';
							hdr.cmd='>';
							hdr.size=idx;
							err = write(fds[1].fd, &hdr, idx + 4);
							if (err != (idx+4)) {printf("UART write error ! (%d != %d)\n",err, idx+4);}
					}
				} else {
					/* Ask for new data */
					while (1) {
						printf("POLL: ask for new data\n");
						hdr.st=':';
						hdr.cmd='<';
						hdr.size = 0;
						err = write(fds[1].fd, &hdr, 4);
						/* Wait for answer */
						err = eth_over_serial_read_rs(fds[1].fd, &hdr, 4);
						if (err < 4) {printf("No answer!\n");break;}
						if ((hdr.st==':') && (hdr.cmd=='?')) {
							nr = hdr.size;
							if ((nr > 0) && (nr <= RS232_MAX_MEMORY_BUFF)) {
								err = eth_over_serial_read_rs(fds[1].fd, hdr.buf, nr);
								if (err != nr) {
									printf("POLL: Read Error!\n");
									break;
								} else {
									/* Write to TAP interface */
									err = write(fds[0].fd, hdr.buf, nr);
									if (err != nr) {printf("TAP: Write Error!\n");break;}
								}
							} else {
								break;
							}
						} else {
							printf("POLL: No answer!\n");
							break;
						}
					}
				}
			}
			/* Cleanup */
			printf("eth_over_serial - Cleanup master\n");
			fsockclose(server);
		} else {
			/* ===================================== */
			/* =========-- Client mode --=========== */
			/* ===================================== */
			printf("eth_over_serial - slave mode\n");
			//setBlocking(fds[2].fd);       /* Blocking serial */
			for (;;) {
				fds[0].revents = 0;
				fds[1].revents = 0;
				i = poll(&fds[1], 1, -1);
				/* Process requests */
				if (i > 0) {
					/* Read command from RS232 */
					if (fds[1].revents & POLLIN) {
						fds[1].revents = 0;
						idx = read_n(fds[1].fd, &hdr, 4);
						if (idx < 4) {printf("UART header read error!\n");continue;}
						if (hdr.st==':') {
							nr = hdr.size;
							if (nr > RS232_MAX_MEMORY_BUFF) {
								printf("Bad frame size (%d) cmd = %c\n",nr,hdr.st);
								continue;
							}
							printf("Cmd %c, size = %d\n",hdr.cmd, nr);
							if (hdr.cmd == '>') {
								/* CMD: Write data to TAP interface */
								idx = read_n(fds[1].fd, hdr.buf, nr);
								if (idx == nr) {
									setBlocking(fds[0].fd);
									err = write(fds[0].fd, hdr.buf, idx);
								} else {
									printf("UART: read error! (%d != %d)\n", idx, nr);
								}
							} else if ((hdr.cmd == '<') && (nr == 0)) {
								/* CMD: read data from TAP interface */
								setNonBlocking(fds[0].fd);
								idx = read(fds[0].fd, hdr.buf, RS232_MAX_MEMORY_BUFF);
								if (idx<0) idx = 0;
								/* Send result over serial */
								hdr.st=':';
								hdr.cmd='?';
								hdr.size = idx;
								err = write(fds[1].fd, &hdr, idx+4);
								if (err == idx+4) {
								} else {
									printf("UART write error! (%d != %d)\n", err, idx+4);
								}
							} else {
								printf("Bad command code - frame damaged!!!!\n");
							}
						}
					}
				}
			}
			printf("eth_over_serial - Cleanup slave\n");
			fsockclose(server);
		}
	}
	printf("eth_over_serial - Exit from thread\n");
	return 0;
}
//===========================================================================

