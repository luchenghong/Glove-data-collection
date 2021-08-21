#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>

#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "spi_if.h"

#define SPI_CE0           0
#define SPI_CE1           1
#define SPI_SCLK_ICM	  1000000 //Hz

static int global_cs = SENSOR_ID_10;

void sensor_select(int cs)
{
	//printf("select sensor:%d\r\n",cs);
	global_cs = cs;
}

void spi_init()
{
	int fd;
	int spiMode = 3;
     
	printf("Setup WiringPi\r\n");
	wiringPiSetup () ;

	fd = wiringPiSPISetup(SPI_CE1, SPI_SCLK_ICM);
	ioctl(fd,SPI_IOC_WR_MODE,&spiMode);
	
	pinMode(SENSOR_ID_11,OUTPUT);
	digitalWrite(SENSOR_ID_11,HIGH);
	pinMode(SENSOR_ID_10,OUTPUT);
	digitalWrite(SENSOR_ID_10,HIGH);
	pinMode(SENSOR_ID_9,OUTPUT);
	digitalWrite(SENSOR_ID_9,HIGH);
	pinMode(SENSOR_ID_8,OUTPUT);
	digitalWrite(SENSOR_ID_8,HIGH);
	pinMode(SENSOR_ID_7,OUTPUT);
	digitalWrite(SENSOR_ID_7,HIGH);
	pinMode(SENSOR_ID_6,OUTPUT);
	digitalWrite(SENSOR_ID_6,HIGH);
	pinMode(SENSOR_ID_5,OUTPUT);
	digitalWrite(SENSOR_ID_5,HIGH);
	pinMode(SENSOR_ID_4,OUTPUT);
	digitalWrite(SENSOR_ID_4,HIGH);
	pinMode(SENSOR_ID_3,OUTPUT);
	digitalWrite(SENSOR_ID_3,HIGH);
	pinMode(SENSOR_ID_2,OUTPUT);
	digitalWrite(SENSOR_ID_2,HIGH);
	pinMode(SENSOR_ID_1,OUTPUT);
	digitalWrite(SENSOR_ID_1,HIGH);

	printf("ICM20948 spi setup result: %d\r\n",fd);
}

int icm_read_reg(uint8_t reg, const uint8_t * rbuffer, uint32_t rlen)
{
	uint8_t *buffer = (uint8_t*) malloc( (size_t)rlen+1);
	
	memset(buffer,0,rlen+1);
	buffer[0] = reg|0x80;

	digitalWrite(global_cs,LOW);
	wiringPiSPIDataRW(SPI_CE1,buffer,rlen+1);
	digitalWrite(global_cs,HIGH);	

	memcpy(rbuffer,buffer+1,rlen);
	free(buffer);

	return 0;
} 

int icm_write_reg(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{

	uint8_t *buffer = (uint8_t*)malloc( (size_t)wlen+1);
	
	memset(buffer,0,wlen+1);
	buffer[0] = reg;
	memcpy(buffer+1,wbuffer,wlen);

	digitalWrite(global_cs,LOW);
	wiringPiSPIDataRW(SPI_CE1,buffer,wlen+1);
	digitalWrite(global_cs,HIGH);

	free(buffer);

	return 0;
}
