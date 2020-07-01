#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <linux/if_tun.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
//#include <linux/if.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include "tun-driver.h"

int tun_alloc(char dev[IFNAMSIZ], short flags) 
{
	// Interface request structure
	const char *cloneDevice = "/dev/net/tun";
	struct ifreq ifr;
	int fileDescriptor,err;
	char buf[4096];

	printf("Create tap device (%s)....\n",dev);
	// Open the tun device, if it doesn't exists return the error
	if ((fileDescriptor = open(cloneDevice, O_RDWR)) < 0) {
		printf("Can not open /dev/net/tun\n");
		return fileDescriptor;
	}

	// Initialize the ifreq structure with 0s and the set flags
	memset(&ifr, 0, sizeof(ifr));
	ifr.ifr_flags = flags;

	// If a device name is passed we should add it to the ifreq struct
	// Otherwise the kernel will try to allocate the next available
	// device of the given type
	if (*dev) {strncpy(ifr.ifr_name, dev, IFNAMSIZ);}

	// Ask the kernel to create the new device
	err = ioctl(fileDescriptor, TUNSETIFF, (void *) &ifr);
	if (err < 0) {
		// If something went wrong close the file and return
		printf("ioctl TUNSETIFF\n");
		close(fileDescriptor);
		return err;
	}
	// Write the device name back to the dev variable so the caller
	// can access it
	strcpy(dev, ifr.ifr_name);

	sprintf(buf,"/sbin/ifconfig %s up", dev);
	system(buf);
	//sprintf(buf,"brctl addif br0 %s", dev);
	//system(buf);

	// Return the file descriptor
	printf("TAP ready :-) \n");
	return fileDescriptor;
}
//==========================================================================================
