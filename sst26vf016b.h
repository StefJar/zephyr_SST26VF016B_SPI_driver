/*
 * sst26vf016b.h
 *
 *  Created on: 8 Jul 2019
 *      Author: Stefan Jaritz
 */

#ifndef DRV_SST26VF016B_H_
#define DRV_SST26VF016B_H_

#include <zephyr.h>

#define LOG_LEVEL_APP_SST26VF016B 4

#define SST26VF016B_DEV_NAME "SST26VF016B"
#define SST26VF016B_SIZE (2*1024*1024)
#define SST26VF016B_SECTORS (40)

#define SST26VF016B_CS_GPIO_NAME "GPIOA"
#define SST26VF016B_CS_GPIO_NUM 4
#define SST26VF016B_SPI "SPI_1"

// memory organisation
// start at addr 0x0
// 4 x 8KBytes sectors
// 1 x 32kBytes sectors
// 30 x 64kBytes sectors
// 1 x 32kBytes sectors
// 4 x 8KBytes sectors
// total 40 sectors

typedef enum sst26vf016b_errors {
	sst26vf016b_error_none = 0,
	sst26vf016b_error_SPI = 1,
	sst26vf016b_error_jedec = 2,
	sst26vf016b_error_wakeup_wrongid = 3
} sst26vf016b_error_t;

typedef struct sst26vf016b_api_s {
	sst26vf016b_error_t (* init) (void);
	sst26vf016b_error_t (* sleep) (void);
	sst26vf016b_error_t (* wakeup) (void);
	sst26vf016b_error_t (* readBlock) (u32_t addr, u8_t * dest, size_t len);
	sst26vf016b_error_t (* sectorErase) (const u32_t secNbr);
	sst26vf016b_error_t (* chipErase) (void);
	sst26vf016b_error_t (* writeBlock) (u32_t addr, u8_t * src, u8_t len); // note only 1-255 Bytes transfers are supported
	u32_t (* sectorNumberToAddr) (u32_t secNbr);
} sst26vf016b_api_t;

#endif /* DRV_SST26VF016B_H_ */
