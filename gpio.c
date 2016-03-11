#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>

#include "gpio.h"

#define SYSFS_GPIO_DIR "/sys/class/gpio"

#define SYSFS_GPIO_MAX_BUF 64

int8_t gpio_export(uint8_t gpio) { //this exports gpio and sets the direction to out
	char buf[SYSFS_GPIO_MAX_BUF];
	int fd;
	uint8_t len;
 
	fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
	if (-1 == fd) {
		perror("gpio/export");
		return -1;
	}
 
	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);

	//set direction to out
	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR  "/gpio%d/direction", gpio);

	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/direction");
		return fd;
	}


	write(fd, "out", 4);

	close(fd);

	return 0;
}

int8_t gpio_unexport(uint8_t gpio) {
	char buf[SYSFS_GPIO_MAX_BUF];
	int fd;
	uint8_t len;
 
	fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
	if (-1 == fd) {
		perror("gpio/unexport");
		return -1;
	}
 
	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);

	return 0;
}

int8_t gpio_set_value(uint8_t gpio, uint8_t value) {
	int fd;
	char buf[SYSFS_GPIO_MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-value");
		return -1;
	}

	if (value==GPIO_LOW)
		write(fd, "0", 2);
	else
		write(fd, "1", 2);

	close(fd);
	return 0;	
}

