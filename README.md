# zephyr_SST26VF016B_SPI_driver
A simple zephyr straight forward driver for the SST26VF016B flash IC connected through SPI.

How to use:
- activate SPI1 (defconfig)
- take care that the gpio pins are configured(CS, CLK, MISO, MOSI at pinmux.c in the board folder)
- set the debug level to see what happens (at the header file)
- add c-file to the CMakeLists.txt
- try sample code

Sample code:
```C
#include "sst26vf016b.h"
#include <ctype.h>

#define FLASH_SIZE (2*1024*1024)
#define FLASH_PAGE 256
#define FLASH_RN ((FLASH_SIZE-8*1024)/FLASH_PAGE)

u8_t flash_chunk[FLASH_PAGE+1];

void init_flash(void) {
	struct device * dev;
	sst26vf016b_api_t * api;

	dev = device_get_binding(SST26VF016B_DEV_NAME);
	if (NULL == dev) {
		LOG_ERR("can not find device " SST26VF016B_DEV_NAME);
		return;
	}

	api = (sst26vf016b_api_t *) dev->driver_api;

	api->init();
	// print sector addresses
	for (unsigned int i = 0; i < SST26VF016B_SECTORS; i++) {
		LOG_DBG("sector %u addr=0x%X", i, api->sectorNumberToAddr(i));
	}

	api->sleep();
	api->wakeup();
	api->sleep();
	api->wakeup();
	// read and print flash
	u32_t addr = api->sectorNumberToAddr(39);
	flash_chunk[FLASH_PAGE] = 0;
	while(addr < FLASH_SIZE) {
		api->readBlock(addr, flash_chunk, FLASH_PAGE);
		for (unsigned j = 0; j < FLASH_PAGE; j++) {
			flash_chunk[j] = isgraph(flash_chunk[j]) ? flash_chunk[j] : ' ';
		}
		LOG_DBG("%s", flash_chunk);
		addr += FLASH_PAGE;
	}
	LOG_DBG("erase sector and write");
	api->sectorErase(39);

	const char txt [] = "hello world!";
	api->writeBlock(api->sectorNumberToAddr(39), txt, sizeof(txt));

	addr = api->sectorNumberToAddr(39);
	flash_chunk[FLASH_PAGE] = 0;
	while(addr < FLASH_SIZE) {
		api->readBlock(addr, flash_chunk, FLASH_PAGE);
		for (unsigned j = 0; j < FLASH_PAGE; j++) {
			flash_chunk[j] = isgraph(flash_chunk[j]) ? flash_chunk[j] : ' ';
		}
		LOG_DBG("%s", flash_chunk);
		addr += FLASH_PAGE;
	}
	api->sleep();
}
```

sources:
- SST26VF016B [datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/20005262D.pdf)
- Zephyr OS [SPI API](https://docs.zephyrproject.org/latest/reference/peripherals/spi.html)



