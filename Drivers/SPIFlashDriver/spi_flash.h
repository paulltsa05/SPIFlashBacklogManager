/*
 * spi_flash.h
 *
 *  Created on: Jun 2, 2017
 *      Author: paull
 */

#ifndef THIRD_PARTY_FATFS_SRC_SPI_FLASH_H_
#define THIRD_PARTY_FATFS_SRC_SPI_FLASH_H_


//***************** ENABLE WHICH FLASH VENDOR YOU ARE USING HERE
#ifndef CONFIG_SPI_FLASH_WINBOND
	#define CONFIG_SPI_FLASH_WINBOND
#endif

/*
 * Interface to SPI flash
 *
 * Copyright (C) 2008 Atmel Corporation
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#ifndef _SPI_FLASH_H_
#define _SPI_FLASH_H_

#include <stdint.h>
#include <stddef.h>
//#include <console/console.h>
//#include <spi-generic.h>
#include  <main.h>

/**
 * container_of - cast a member of a structure out to the containing structure
 * @ptr:	the pointer to the member.
 * @type:	the type of the container struct this is embedded in.
 * @member:	the name of the member within the struct.
 *
 */
#define container_of(ptr, type, member) ({			\
	const typeof( ((type *)0)->member ) *__mptr = (ptr);	\
	(type *)( (char *)__mptr - offsetof(type,member) );})

#define min(a, b) ((a)<(b)?(a):(b))

#define CONFIG_ICH_SPI
#ifdef CONFIG_ICH_SPI
#define CONTROLLER_PAGE_LIMIT	64
#else
/* any number larger than 4K would do, actually */
#define CONTROLLER_PAGE_LIMIT	((int)(~0U>>1))
#endif

/* Common parameters -- kind of high, but they should only occur when there
 * is a problem (and well your system already is broken), so err on the side
 * of caution in case we're dealing with slower SPI buses and/or processors.
 */
#define CONFIG_SYS_HZ 100
#define SPI_FLASH_PROG_TIMEOUT		(2 * CONFIG_SYS_HZ)
#define SPI_FLASH_PAGE_ERASE_TIMEOUT	(5 * CONFIG_SYS_HZ)
#define SPI_FLASH_SECTOR_ERASE_TIMEOUT	(10 * CONFIG_SYS_HZ)

/* SPI transfer flags */
#define SPI_XFER_BEGIN	0x01			/* Assert CS before transfer */
#define SPI_XFER_END	0x02			/* Deassert CS after transfer */

/* SPI opcodes */
#define SPI_OPCODE_WREN 0x06
#define SPI_OPCODE_FAST_READ 0x0b

#define SPI_READ_FLAG	0x01
#define SPI_WRITE_FLAG	0x02


#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

/* Common commands */
#define CMD_READ_ID			0x9f

#define CMD_READ_ARRAY_SLOW		0x03
#define CMD_READ_ARRAY_FAST		0x0b
#define CMD_READ_ARRAY_LEGACY		0xe8

#define CMD_READ_STATUS			0x05
#define CMD_WRITE_ENABLE		0x06

/* Common status */
#define STATUS_WIP			0x01

/*-----------------------------------------------------------------------
 * Representation of a SPI slave, i.e. what we're communicating with.
 *
 * Drivers are expected to extend this with controller-specific data.
 *
 *   bus:	ID of the bus that the slave is attached to.
 *   cs:	ID of the chip select connected to the slave.
 *   rw: 	Read or Write flag
 */
struct spi_slave {
	unsigned int	bus;
	unsigned int	cs;
	unsigned int	rw;
};



struct spi_flash {
	struct spi_slave *spi;

	const char	*name;

	uint32_t		size;

	uint32_t		sector_size;

	int		(*read)(struct spi_flash *flash, uint32_t offset,
				size_t len, void *buf);
	int		(*write)(struct spi_flash *flash, uint32_t offset,
				size_t len, const void *buf);
	int		(*erase)(struct spi_flash *flash, uint32_t offset,
				size_t len);
};

struct spi_flash *spi_flash_probe();

static inline int spi_flash_read(struct spi_flash *flash, uint32_t offset,
		size_t len, void *buf)
{
	return flash->read(flash, offset, len, buf);
}

static inline int spi_flash_write(struct spi_flash *flash, uint32_t offset,
		size_t len, const void *buf)
{
	return flash->write(flash, offset, len, buf);
}

static inline int spi_flash_erase(struct spi_flash *flash, uint32_t offset,
		size_t len)
{
	return flash->erase(flash, offset, len);
}

/*
 * SPI flash internal definitions
 *
 * Copyright (C) 2008 Atmel Corporation
 */



/* Send a single-byte command to the device and read the response */
int spi_flash_cmd(struct spi_slave *spi, uint8_t cmd, void *response, size_t len);

/*
 * Send a multi-byte command to the device and read the response. Used
 * for flash array reads, etc.
 */
int spi_flash_cmd_read(struct spi_slave *spi, const uint8_t *cmd,
		size_t cmd_len, void *data, size_t data_len);

int spi_flash_cmd_read_fast(struct spi_flash *flash, uint32_t offset,
		size_t len, void *data);

int spi_flash_cmd_read_slow(struct spi_flash *flash, uint32_t offset,
		size_t len, void *data);

/*
 * Send a multi-byte command to the device followed by (optional)
 * data. Used for programming the flash array, etc.
 */
int spi_flash_cmd_write(struct spi_slave *spi, const uint8_t *cmd, size_t cmd_len,
		const void *data, size_t data_len);

/*
 * Same as spi_flash_cmd_read() except it also claims/releases the SPI
 * bus. Used as common part of the ->read() operation.
 */
int spi_flash_read_common(struct spi_flash *flash, const uint8_t *cmd,
		size_t cmd_len, void *data, size_t data_len);

/* Send a command to the device and wait for some bit to clear itself. */
int spi_flash_cmd_poll_bit(struct spi_flash *flash, unsigned long timeout,
			   uint8_t cmd, uint8_t poll_bit);

/*
 * Send the read status command to the device and wait for the wip
 * (write-in-progress) bit to clear itself.
 */
int spi_flash_cmd_wait_ready(struct spi_flash *flash, unsigned long timeout);

/* Erase sectors. */
int spi_flash_cmd_erase(struct spi_flash *flash, uint8_t erase_cmd,
			uint32_t offset, size_t len);

/* Manufacturer-specific probe functions */
//struct spi_flash *spi_flash_probe_spansion(struct spi_slave *spi, uint8_t *idcode);
//struct spi_flash *spi_flash_probe_atmel(struct spi_slave *spi, uint8_t *idcode);
//struct spi_flash *spi_flash_probe_eon(struct spi_slave *spi, uint8_t *idcode);
//struct spi_flash *spi_flash_probe_macronix(struct spi_slave *spi, uint8_t *idcode);
//struct spi_flash *spi_flash_probe_sst(struct spi_slave *spi, uint8_t *idcode);
//struct spi_flash *spi_flash_probe_stmicro(struct spi_slave *spi, uint8_t *idcode);
struct spi_flash *spi_flash_probe_winbond(struct spi_slave *spi, uint8_t *idcode);
//struct spi_flash *spi_flash_probe_gigadevice(struct spi_slave *spi,
//					     uint8_t *idcode);
//struct spi_flash *spi_fram_probe_ramtron(struct spi_slave *spi, uint8_t *idcode);



//SPI function to be implemented as per STM32
/*-----------------------------------------------------------------------
 * Initialization, must be called once on start up.
 *
 */
//void spi_init(void);

/*-----------------------------------------------------------------------
 * Set up communications parameters for a SPI slave.
 *
 * This must be called once for each slave. Note that this function
 * usually doesn't touch any actual hardware, it only initializes the
 * contents of spi_slave so that the hardware can be easily
 * initialized later.
 *
 *   bus:     Bus ID of the slave chip.
 *   cs:      Chip select ID of the slave chip on the specified bus.
 *   max_hz:  Maximum SCK rate in Hz.
 *   mode:    Clock polarity, clock phase and other parameters.
 *
 * Returns: A spi_slave reference that can be used in subsequent SPI
 * calls, or NULL if one or more of the parameters are not supported.
 */
struct spi_slave *spi_setup_slave(void);





/*-----------------------------------------------------------------------
 * SPI transfer
 *
 * This writes "bitlen" bits out the SPI MOSI port and simultaneously clocks
 * "bitlen" bits in the SPI MISO port.  That's just the way SPI works.
 *
 * The source of the outgoing bits is the "dout" parameter and the
 * destination of the input bits is the "din" parameter.  Note that "dout"
 * and "din" can point to the same memory location, in which case the
 * input data overwrites the output data (since both are buffered by
 * temporary variables, this is OK).
 *
 * spi_xfer() interface:
 *   slave:	The SPI slave which will be sending/receiving the data.
 *   dout:	Pointer to a string of bits to send out.  The bits are
 *		held in a byte array and are sent MSB first.
 *   bitsout:	How many bits to write.
 *   din:	Pointer to a string of bits that will be filled in.
 *   bitsin:	How many bits to read.
 *
 *   Returns: 0 on success, not 0 on failure
 */
int  spi_xfer(struct spi_slave *slave, const void *dout, unsigned int bytesout,
		void *din, unsigned int bytesin);









#endif /* _SPI_FLASH_H_ */


#endif /* THIRD_PARTY_FATFS_SRC_SPI_FLASH_H_ */
