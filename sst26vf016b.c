/*
 * sst26vf016b.c
 *
 *  Created on: 8 Jul 2019
 *      Author: Stefan Jaritz
 */

#include <zephyr.h>
#include <errno.h>
#include <misc/util.h>
#include <device.h>
#include <spi.h>
#include <stdbool.h>

#include <logging/log.h>

#include "sst26vf016b.h"

LOG_MODULE_REGISTER(SST26VF016B, LOG_LEVEL_APP_SST26VF016B);

#define SST26VF016B_JEDEC_ManufacturerID 0xBF
#define SST26VF016B_JEDEC_DeviceType 0x26
#define SST26VF016B_JEDEC_DeviceID 0x41

typedef enum sst26vf016b_ops {
	sst26vf016b_op_RSTEN = 0x66,
	sst26vf016b_op_RST = 0x99,
	sst26vf016b_op_RDSR = 0x05,
	sst26vf016b_op_WRSR = 0x01,
	sst26vf016b_op_READ = 0x03,
	sst26vf016b_op_JEDEC_ID = 0x9F,
	sst26vf016b_op_WREN = 0x06,
	sst26vf016b_op_WRDI = 0x04,
	sst26vf016b_op_SE = 0x20,
	sst26vf016b_op_BE = 0xD8,
	sst26vf016b_op_CE = 0xC7,
	sst26vf016b_op_PP = 0x02,
	sst26vf016b_op_RBPR = 0x72,
	sst26vf016b_op_WBPR = 0x42,
	sst26vf016b_op_RSID = 0x88,
	sst26vf016b_op_DPD = 0xB9,
	sst26vf016b_op_RDPD = 0xAB,
	sst26vf016b_op_ULBPR = 0x98
} sst26vf016b_op_t;

typedef enum sst26vf016b_statusreg_flags {
	sst26vf016b_statusreg_BUSY = BIT(0), // Write operation status 1 = Internal Write operation is in progress
	sst26vf016b_statusreg_WEL = BIT(1), // Write-Enable Latch status 1 = Device is write-enabled
	sst26vf016b_statusreg_WSE = BIT(2), // Write Suspend-Erase status 1 = Erase suspended
	sst26vf016b_statusreg_WSP = BIT(3), // Write Suspend-Program status 1 = Program suspended
	sst26vf016b_statusreg_WPLD = BIT(4), // Write Protection Lock-Down status 1 = Write Protection Lock-Down enabled
	sst26vf016b_statusreg_SEC = BIT(5), // Security ID status 1 = Security ID space locked
	sst26vf016b_statusreg_RES = BIT(6), // Reserved for future use
	sst26vf016b_statusreg_BUSY2 = BIT(7), // Write operation status 1 = Internal Write operation is in progress

	sst26vf016b_statusreg_BUSYMask = BIT(0) | BIT(7)
} sst26vf016b_statusreg_flag_t;

typedef struct sst26vf016b_s {
	struct device * spiDev; // spi device
	struct spi_cs_control spi_cs_conf; // spi chip select config
	struct spi_config spi_conf; // spi config
} sst26vf016b_t;

sst26vf016b_t sst26vf016b = {
	.spiDev = NULL,
	.spi_cs_conf = {
		.gpio_dev = NULL,
		.gpio_pin = SST26VF016B_CS_GPIO_NUM,
		.delay = 1, // 20ns chip select time
	},
	.spi_conf = {
		.frequency = 40000000, // 40MHz clk speed
		.operation = (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_LINES_SINGLE), // master,
		.slave     = 0, // slave number 0
		.cs        = NULL, // chip select pin
	},
};

// -----------------------------------------
// helper
// -----------------------------------------
static bool sst26vf016b_helper_checkJEDECid(void) {
	u8_t buffer_tx[] = {sst26vf016b_op_JEDEC_ID};
	u8_t buffer_rx[3];
	struct spi_buf tx_buf [] = {
		{
		.buf = buffer_tx,
		.len = sizeof(buffer_tx)/sizeof(u8_t)
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = sizeof(tx_buf)/sizeof(struct spi_buf)
	};
	const struct spi_buf rx_buf []= {
		{
		.buf = NULL,
		.len = 1,
		},
		{
		.buf = buffer_rx,
		.len = sizeof(buffer_rx),
		}
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = sizeof(rx_buf)/sizeof(struct spi_buf)
	};

	if (spi_transceive(sst26vf016b.spiDev, &sst26vf016b.spi_conf, &tx, &rx)) {
		return false;
	}

	if (
			(SST26VF016B_JEDEC_ManufacturerID == buffer_rx[0]) &&
			(SST26VF016B_JEDEC_DeviceType == buffer_rx[1]) &&
			(SST26VF016B_JEDEC_DeviceID == buffer_rx[2])
		) {
		return true;
	}
	LOG_ERR("wrong JEDC id! current ManufacturerID=0x%X DeviceType=0x%X DeviceID=0x%X vs ManufacturerID=0x%X DeviceType=0x%X DeviceID=0x%X",
			buffer_rx[0], buffer_rx[1], buffer_rx[2],
			SST26VF016B_JEDEC_ManufacturerID, SST26VF016B_JEDEC_DeviceType, SST26VF016B_JEDEC_DeviceID
		);
	return false;
}

static sst26vf016b_error_t sst26vf016b_helper_readRegister(const u8_t opID,  u8_t * v) {
	u8_t buffer_tx[] = {opID,};
	struct spi_buf tx_buf [] = {
		{
		.buf = buffer_tx,
		.len = sizeof(buffer_tx)/sizeof(u8_t)
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = sizeof(tx_buf)/sizeof(struct spi_buf)
	};
	const struct spi_buf rx_buf []= {
		{
		.buf = NULL,
		.len = 1,
		},
		{
		.buf = v,
		.len = 1,
		}
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = sizeof(rx_buf)/sizeof(struct spi_buf)
	};

	if (spi_transceive(sst26vf016b.spiDev, &sst26vf016b.spi_conf, &tx, &rx)) {
		return sst26vf016b_error_SPI;
	}
	return sst26vf016b_error_none;
}

static sst26vf016b_error_t sst26vf016b_helper_writeRegister(const u8_t statusReg, const u8_t configReg) {
	u8_t buffer_tx[] = {sst26vf016b_op_WRSR, 0x01, statusReg, configReg};
	struct spi_buf tx_buf [] = {
		{
		.buf = buffer_tx,
		.len = sizeof(buffer_tx)/sizeof(u8_t)
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = sizeof(tx_buf)/sizeof(struct spi_buf)
	};
	if (spi_write(sst26vf016b.spiDev, &sst26vf016b.spi_conf, &tx)) {
		return sst26vf016b_error_SPI;
	}
	return sst26vf016b_error_none;
}

static sst26vf016b_error_t sst26vf016b_helper_writeCmd(const u8_t cmd) {
	u8_t buffer_tx[] = {cmd};
	struct spi_buf tx_buf [] = {
		{
		.buf = buffer_tx,
		.len = sizeof(buffer_tx)/sizeof(u8_t)
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = sizeof(tx_buf)/sizeof(struct spi_buf)
	};
	if (spi_write(sst26vf016b.spiDev, &sst26vf016b.spi_conf, &tx)) {
		return sst26vf016b_error_SPI;
	}
	return sst26vf016b_error_none;
}

static sst26vf016b_error_t sst26vf016b_helper_waitForReady(u32_t sleepTime) {
	u8_t v;
	sst26vf016b_error_t r;

	while(true) {
		r = sst26vf016b_helper_readRegister(sst26vf016b_op_RDSR, &v);
		if (sst26vf016b_error_none != r) {
			LOG_ERR("reading status register failed with %u", r);
			return r;
		}
		if ((v & sst26vf016b_statusreg_BUSYMask) == 0) break;
		k_sleep(sleepTime);
	};
	return sst26vf016b_error_none;;
}

static sst26vf016b_error_t sst26vf016b_helper_ReadBlockProtectionRegister(u8_t regs[6]) {
	u8_t buffer_tx[] = {sst26vf016b_op_RBPR};
	struct spi_buf tx_buf [] = {
		{
		.buf = buffer_tx,
		.len = sizeof(buffer_tx)/sizeof(u8_t)
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = sizeof(tx_buf)/sizeof(struct spi_buf)
	};
	const struct spi_buf rx_buf []= {
		{
		.buf = NULL,
		.len = 1,
		},
		{
		.buf = regs,
		.len = 6,
		}
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = sizeof(rx_buf)/sizeof(struct spi_buf)
	};

	if (spi_transceive(sst26vf016b.spiDev, &sst26vf016b.spi_conf, &tx, &rx)) {
		return sst26vf016b_error_SPI;
	}
	return sst26vf016b_error_none;
}


// -----------------------------------------
// api
// -----------------------------------------
static u32_t sst26vf016b_sectorNumberToAddr(u32_t secNbr) {
	// start at addr 0x0
	// 4 x 8KBytes sectors
	// 1 x 32kBytes sectors
	// 30 x 64kBytes sectors
	// 1 x 32kBytes sectors
	// 4 x 8KBytes sectors

	u32_t addr;

	// 4 x 8KBytes sectors
	if (secNbr < 4) {
		return (8 * 1024) * secNbr;
	}
	addr = 32 * 1024;
	secNbr -= 4;

	// 1 x 32kBytes sectors
	if (secNbr < 1) {
		return addr;
	}
	addr += 32 * 1024;
	secNbr -= 1;

	// 30 x 64kBytes sectors
	if (secNbr < 30) {
		return addr + 64*1024*secNbr;
	}
	addr += 30*64*1024;
	secNbr -= 30;

	// 1 x 32kBytes sectors
	if (secNbr < 1) {
		return addr;
	}
	addr += 32 * 1024;
	secNbr -= 1;

	// 4 x 8KBytes sectors
	if (secNbr < 4) {
		return addr + (8 * 1024) * secNbr;
	}
	return SST26VF016B_SIZE;
}

static sst26vf016b_error_t sst26vf016b_sleep(void) {
	sst26vf016b_error_t r;
	LOG_DBG("sleep");
	r = sst26vf016b_helper_writeCmd(sst26vf016b_op_DPD);
	k_sleep(K_MSEC(3)); // 3 msec till the device is in deep sleep
	return r;
}

static sst26vf016b_error_t sst26vf016b_wakeup(void) {
	LOG_DBG("wakeup");
	u8_t buffer_tx[] = {sst26vf016b_op_RDPD};
	u8_t buffer_rx[1];
	struct spi_buf tx_buf [] = {
		{
		.buf = buffer_tx,
		.len = sizeof(buffer_tx)/sizeof(u8_t)
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = sizeof(tx_buf)/sizeof(struct spi_buf)
	};
	const struct spi_buf rx_buf []= {
		{
		.buf = NULL,
		.len = 4,
		},
		{
		.buf = buffer_rx,
		.len = sizeof(buffer_rx),
		}
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = sizeof(rx_buf)/sizeof(struct spi_buf)
	};

	if (spi_transceive(sst26vf016b.spiDev, &sst26vf016b.spi_conf, &tx, &rx)) {
		return sst26vf016b_error_SPI;
	}

	if (SST26VF016B_JEDEC_DeviceID != buffer_rx[0]) {
		LOG_ERR("wrong device id! current DeviceID=0x%X vs DeviceID=0x%X",
				buffer_rx[0],
				SST26VF016B_JEDEC_DeviceID
			);
		return sst26vf016b_error_wakeup_wrongid;
	}
	k_sleep(K_MSEC(10)); // 10 msec till the device has left deep sleep
	return sst26vf016b_error_none;
}

static sst26vf016b_error_t sst26vf016b_readBlock(u32_t addr, u8_t * dest, size_t len) {
	LOG_DBG("read from 0x%X %uBytes", addr, len);
	u8_t buffer_tx[] = {
		sst26vf016b_op_READ,
		((addr & 0xFFFFFF) >> 16),
		((addr & 0xFFFF) >> 8),
		(addr & 0xFF)
	};
	struct spi_buf tx_buf [] = {
		{
		.buf = buffer_tx,
		.len = sizeof(buffer_tx)/sizeof(u8_t)
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = sizeof(tx_buf)/sizeof(struct spi_buf)
	};
	const struct spi_buf rx_buf []= {
		{
		.buf = NULL,
		.len = 4,
		},
		{
		.buf = dest,
		.len = len,
		}
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = sizeof(rx_buf)/sizeof(struct spi_buf)
	};

	if (spi_transceive(sst26vf016b.spiDev, &sst26vf016b.spi_conf, &tx, &rx)) {
		return sst26vf016b_error_SPI;
	}
	return sst26vf016b_error_none;
}

static sst26vf016b_error_t sst26vf016b_sectorErase(const u32_t secNbr) {
	sst26vf016b_error_t r;
	u32_t addr = sst26vf016b_sectorNumberToAddr(secNbr);
	u8_t buffer_tx[] = {
		sst26vf016b_op_SE,
		((addr & 0xFFFFFF) >> 16),
		((addr & 0xFFFF) >> 8),
		(addr & 0xFF)
	};
	struct spi_buf tx_buf [] = {
		{
		.buf = buffer_tx,
		.len = sizeof(buffer_tx)/sizeof(u8_t)
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = sizeof(tx_buf)/sizeof(struct spi_buf)
	};

	LOG_DBG("start erase 4kByte sector %u(addr=0x%X)", secNbr, addr);

	// always an write enable before operation!
	r = sst26vf016b_helper_writeCmd(sst26vf016b_op_WREN);
	if (r != sst26vf016b_error_none) {
		return r;
	}

	// erase sector
	if (spi_write(sst26vf016b.spiDev, &sst26vf016b.spi_conf, &tx)) {
		return sst26vf016b_error_SPI;
	}

	// wait till we are done
	r = sst26vf016b_helper_waitForReady(K_MSEC(10));
	if (r != sst26vf016b_error_none) {
		return r;
	}
	LOG_DBG("erase done");
	return sst26vf016b_error_none;
}

static sst26vf016b_error_t sst26vf016b_chipErase(void) {
	sst26vf016b_error_t r;

	LOG_DBG("start chip erase");

	// always an write enable before operation!
	r = sst26vf016b_helper_writeCmd(sst26vf016b_op_WREN);
	if (r != sst26vf016b_error_none) {
		return r;
	}
	// erase chip
	r = sst26vf016b_helper_writeCmd(sst26vf016b_op_CE);
	if (r != sst26vf016b_error_none) {
		return r;
	}

	// wait till we are done
	r = sst26vf016b_helper_waitForReady(K_MSEC(10));
	if (r != sst26vf016b_error_none) {
		return r;
	}
	LOG_DBG("chip erase done");
	return sst26vf016b_error_none;
}

static sst26vf016b_error_t sst26vf016b_writeBlock(u32_t addr, u8_t * src, u8_t len) {
	sst26vf016b_error_t r;
	// page program works with chunks of 1-255 Bytes
	u8_t buffer_tx[] = {
		sst26vf016b_op_PP,
		((addr & 0xFFFFFF) >> 16),
		((addr & 0xFFFF) >> 8),
		(addr & 0xFF)
	};
	struct spi_buf tx_buf [] = {
		{
		.buf = buffer_tx,
		.len = sizeof(buffer_tx)/sizeof(u8_t)
		},
		{
		.buf = src,
		.len = len
		},
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = sizeof(tx_buf)/sizeof(struct spi_buf)
	};

	LOG_DBG("write block %uBytes to flash addr 0x%X ", len, addr);

	// always an write enable before operation!
	r = sst26vf016b_helper_writeCmd(sst26vf016b_op_WREN);
	if (r != sst26vf016b_error_none) {
		return r;
	}

	// write data to page
	if (spi_write(sst26vf016b.spiDev, &sst26vf016b.spi_conf, &tx)) {
		return sst26vf016b_error_SPI;
	}
	// wait till we are done
	r = sst26vf016b_helper_waitForReady(K_MSEC(10));
	if (r != sst26vf016b_error_none) {
		return r;
	}
	LOG_DBG("write done");
	return sst26vf016b_error_none;
}

static sst26vf016b_error_t sst26vf016b_init(void) {
	sst26vf016b_error_t r;

	LOG_DBG("init");

	sst26vf016b_wakeup();
	// wait for ready
	r = sst26vf016b_helper_waitForReady(K_MSEC(10));
	if (r != sst26vf016b_error_none) {
		return r;
	}
	// reset enable
	LOG_DBG("reset");
	r = sst26vf016b_helper_writeCmd(sst26vf016b_op_RSTEN);
	if (r != sst26vf016b_error_none) {
		LOG_ERR("enable reset failed with error %u", r);
		return r;
	}
	k_sleep(K_MSEC(1));
	r = sst26vf016b_helper_writeCmd(sst26vf016b_op_RST);
	if (r != sst26vf016b_error_none) {
		LOG_ERR("reset failed with error %u", r);
		return r;
	}

	// enable writing
	LOG_DBG("enable writing");
	r = sst26vf016b_helper_writeCmd(sst26vf016b_op_WREN);
	if (r != sst26vf016b_error_none) {
		LOG_ERR("enable writing failed with error %u", r);
		return r;
	}

	// Global Block Protection Unlock
	LOG_DBG("unlock Global Block Protection");
	r = sst26vf016b_helper_writeCmd(sst26vf016b_op_ULBPR);
	if (r != sst26vf016b_error_none) {
		LOG_ERR("Global Block Protection Unlock failed with error %u", r);
	}

	u8_t regs[6] = {0, 0, 0, 0, 0, 0};
	r = sst26vf016b_helper_ReadBlockProtectionRegister(regs);
	if (r != sst26vf016b_error_none) {
		LOG_ERR("Read Block-Protection Register failed with error %u", r);
	}

	// check id
	if (false == sst26vf016b_helper_checkJEDECid()) return sst26vf016b_error_jedec;
	LOG_DBG("JEDC id successfully matched! ManufacturerID=0x%X DeviceType=0x%X DeviceID=0x%X",
		SST26VF016B_JEDEC_ManufacturerID, SST26VF016B_JEDEC_DeviceType, SST26VF016B_JEDEC_DeviceID
	);
	return sst26vf016b_error_none;
}

// -----------------------------------------
// driver
// -----------------------------------------

static int sst26vf016b_initialize(struct device *dev) {
	LOG_INF("Initialise SST26VF016B");

	// get spi1 device
	sst26vf016b.spiDev = device_get_binding(SST26VF016B_SPI);
	if (NULL == sst26vf016b.spiDev) {
		LOG_ERR("can not find device " SST26VF016B_SPI);
		return -1;
	}

	// get chip select pin device
	sst26vf016b.spi_cs_conf.gpio_dev = device_get_binding(SST26VF016B_CS_GPIO_NAME);
	if (NULL == sst26vf016b.spi_cs_conf.gpio_dev) {
		LOG_ERR("can not find device " SST26VF016B_CS_GPIO_NAME);
		return -1;
	}
	// set chip select
	sst26vf016b.spi_conf.cs = &sst26vf016b.spi_cs_conf;
	return 0;
}

static const sst26vf016b_api_t sst26vf016b_api = {
	.init = sst26vf016b_init,
	.sleep = sst26vf016b_sleep,
	.wakeup = sst26vf016b_wakeup,
	.readBlock = sst26vf016b_readBlock,
	.sectorErase = sst26vf016b_sectorErase,
	.chipErase = sst26vf016b_chipErase,
	.writeBlock = sst26vf016b_writeBlock,
	.sectorNumberToAddr = sst26vf016b_sectorNumberToAddr,
};

DEVICE_AND_API_INIT(sst26vf016b_flash, SST26VF016B_DEV_NAME, &sst26vf016b_initialize,
	NULL, NULL,
	POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
	&sst26vf016b_api
	);
