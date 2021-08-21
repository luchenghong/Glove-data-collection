#ifndef SPI_IF_H
#define SPI_IF_H

#include <stdio.h>
#include <stdint.h>

#define SENSOR_ID_11	  27
#define SENSOR_ID_10	  28
#define SENSOR_ID_9	  29
#define SENSOR_ID_8	  25
#define SENSOR_ID_7	  24
#define SENSOR_ID_6	  23
#define SENSOR_ID_5	  22
#define SENSOR_ID_4	  21
#define SENSOR_ID_3	  3
#define SENSOR_ID_2	  2
#define SENSOR_ID_1	  0


void spi_init();
int icm_read_reg(uint8_t reg, const uint8_t * rbuffer, uint32_t rlen);
int icm_write_reg(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
void sensor_select(int cs);
#endif
