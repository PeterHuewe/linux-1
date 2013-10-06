/*
 * STMicroelectronics TPM SPI Linux driver for TPM ST33NP18
 * Copyright (C) 2009, 2013  STMicroelectronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * STMicroelectronics version 1.2.0, Copyright (C) 2010
 * STMicroelectronics comes with ABSOLUTELY NO WARRANTY.
 * This is free software, and you are welcome to redistribute it
 * under certain conditions.
 *
 * @Author: Christophe RICARD tpmsupport@st.com
 *
 * @File: tpm_spi_stm_st33.h
 *
 * @Date: 09/15/2010
 */
#ifndef __STM_ST33_TPM_SPI_MAIN_H__
#define __STM_ST33_TPM_SPI_MAIN_H__

#define TPM_ACCESS			(0x0)
#define TPM_STS				(0x18)
#define TPM_DATA_FIFO			(0x24)
#define TPM_HASH_DATA			(0x24)
#define TPM_INTF_CAPABILITY		(0x14)
#define TPM_INT_STATUS			(0x10)
#define TPM_INT_ENABLE			(0x08)

#define TPM_DUMMY_BYTE			0x00
#define TPM_WRITE_DIRECTION		0x80
#define TPM_HEADER_SIZE			10
#define TPM_BUFSIZE			2048

#define LOCALITY0		0

#define MAX_SPI_LATENCY		15

#define TPM_ST33_SPI		"st33zp24_spi"

struct st33zp24_platform_data {
	int io_serirq;
	int io_lpcpd;
	int latency;
	bool bchipf;
	u8 *tpm_spi_buffer[2]; /* 0 Request 1 Response */
	struct completion irq_detection;
};

#endif /* __STM_ST33_TPM_SPI_MAIN_H__ */
