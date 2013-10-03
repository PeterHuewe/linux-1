/*
 * STMicroelectronics TPM SPI Linux driver for TPM ST33ZP24
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
 * @File: tpm_stm_st33_spi.c
 *
 * @Synopsis:
 *	09/15/2010: First shot driver tpm_tis driver for lpc is used as model.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/wait.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include "tpm.h"
#include "tpm_spi_stm_st33.h"

enum stm33zp24_access {
	TPM_ACCESS_VALID = 0x80,
	TPM_ACCESS_ACTIVE_LOCALITY = 0x20,
	TPM_ACCESS_REQUEST_PENDING = 0x04,
	TPM_ACCESS_REQUEST_USE = 0x02,
};

enum stm33zp24_status {
	TPM_STS_VALID = 0x80,
	TPM_STS_COMMAND_READY = 0x40,
	TPM_STS_GO = 0x20,
	TPM_STS_DATA_AVAIL = 0x10,
	TPM_STS_DATA_EXPECT = 0x08,
};

enum stm33zp24_int_flags {
	TPM_GLOBAL_INT_ENABLE = 0x80,
	TPM_INTF_CMD_READY_INT = 0x80,
	TPM_INTF_FIFO_AVALAIBLE_INT = 0x40,
	TPM_INTF_WAKE_UP_READY_INT = 0x20,
	TPM_INTF_LOC4SOFTRELEASE_INT = 0x08,
	TPM_INTF_LOCALITY_CHANGE_INT = 0x04,
	TPM_INTF_STS_VALID_INT = 0x02,
	TPM_INTF_DATA_AVAIL_INT = 0x01,
};

enum tis_defaults {
	TIS_SHORT_TIMEOUT = 750,	/* ms */
	TIS_LONG_TIMEOUT = 2000,	/* 2 sec */
};


static int interrupts;
module_param(interrupts, int, 0444);
MODULE_PARM_DESC(interrupts, "Enable interrupts");

static int power_mgt = 1;
module_param(power_mgt, int, 0444);
MODULE_PARM_DESC(power_mgt, "Power Management");

/*
 * spi_write8_reg
 * Send byte to the TIS register according to the ST33ZP24 SPI protocol.
 * @param: tpm, the chip description
 * @param: tpm_register, the tpm tis register where the data should be written
 * @param: tpm_data, the tpm_data to write inside the tpm_register
 * @param: tpm_size, The length of the data
 * @return: should be zero if success else a negative error code.
 */
static int spi_write8_reg(struct tpm_chip *tpm, u8 tpm_register,
		      u8 *tpm_data, u16 tpm_size)
{
	u8 data = 0;
	int total_length = 0, nbr_dummy_bytes = 0;
	int value = 0;
	struct spi_device *dev =
		 (struct spi_device __force *)tpm->vendor.iobase;
	struct st33zp24_platform_data *platform_data = dev->dev.platform_data;
	u8 *data_buffer = platform_data->tpm_spi_buffer[0];
	struct spi_transfer xfer = {
		.tx_buf	 = data_buffer,
		.rx_buf	 = platform_data->tpm_spi_buffer[1],
	};

	/* Pre-Header */
	data = TPM_WRITE_DIRECTION | tpm->vendor.locality;
	memcpy(data_buffer + total_length, &data, sizeof(data));
	total_length++;
	data = tpm_register;
	memcpy(data_buffer + total_length, &data, sizeof(data));
	total_length++;

	if (tpm_size > 0 &&
	 (tpm_register == TPM_DATA_FIFO || tpm_register == TPM_HASH_DATA)) {
			data_buffer[total_length++] = tpm_size >> 8;
			data_buffer[total_length++] = tpm_size;
	}

	memcpy(&data_buffer[total_length], tpm_data, tpm_size);
	total_length += tpm_size;

	nbr_dummy_bytes = platform_data->latency + 1;
	memset(&data_buffer[total_length], TPM_DUMMY_BYTE,
					 platform_data->latency + 1);

	xfer.len = total_length;

	value = spi_sync_transfer(dev, &xfer, 1);

	if (value == 0) {
		nbr_dummy_bytes = total_length - 1 - nbr_dummy_bytes;
		while (nbr_dummy_bytes < total_length &&
			 ((u8 *)xfer.rx_buf)[nbr_dummy_bytes] == 0)
					nbr_dummy_bytes++;

		if (((u8 *)xfer.rx_buf)[nbr_dummy_bytes] != 0)
			value = ((u8 *)xfer.rx_buf)[nbr_dummy_bytes];
	}

	return value;
} /* spi_write8_reg() */

/*
 * spi_read8_reg
 * Recv byte from the TIS register according to the ST33ZP24 SPI protocol.
 * @param: tpm, the chip description
 * @param: tpm_register, the tpm tis register where the data should be read
 * @param: tpm_data, the TPM response
 * @param: tpm_size, tpm TPM response size to read.
 * @return: should be zero if success else a negative error code.
 */
static u8 spi_read8_reg(struct tpm_chip *tpm, u8 tpm_register,
		    u8 *tpm_data, u16 tpm_size)
{
	u8 data = 0;
	int total_length = 0, nbr_dummy_bytes;
	int value = 0;
	struct spi_device *dev =
		 (struct spi_device __force *)tpm->vendor.iobase;
	struct st33zp24_platform_data *platform_data = dev->dev.platform_data;
	u8 *data_buffer = platform_data->tpm_spi_buffer[0];
	struct spi_transfer xfer = {
		.tx_buf	 = data_buffer,
		.rx_buf	 = platform_data->tpm_spi_buffer[1],
	};

	/* Pre-Header */
	data = tpm->vendor.locality;
	memcpy(data_buffer + total_length, &data, sizeof(data));
	total_length++;
	data = tpm_register;
	memcpy(data_buffer + total_length, &data, sizeof(data));
	total_length++;

	nbr_dummy_bytes = platform_data->latency + 1;
	memset(&data_buffer[total_length], TPM_DUMMY_BYTE,
		 platform_data->latency + 1);

	xfer.len = total_length;

	/* header + status byte + size of the data + status byte */
	value = spi_sync_transfer(dev, &xfer, 1);

	if (tpm_size > 0 && value == 0) {
		nbr_dummy_bytes = 2;
		while (nbr_dummy_bytes < total_length &&
			 ((u8 *)xfer.rx_buf)[nbr_dummy_bytes] == 0)
				nbr_dummy_bytes++;

		if (nbr_dummy_bytes + 1 < total_length) {
			value = ((u8 *)xfer.rx_buf)[nbr_dummy_bytes];

			if (tpm_size > 0)
				memcpy(tpm_data, xfer.rx_buf +
							 nbr_dummy_bytes + 1,
								 tpm_size);
		}
	}
	return value;
} /* spi_read8_reg() */

/*
 * clear_interruption
 * clear the TPM interrupt register.
 * @param: tpm, the chip description
 */
static void clear_interruption(struct tpm_chip *tpm)
{
	u8 interrupt;
	spi_read8_reg(tpm, TPM_INT_STATUS, &interrupt, 1);
	/* Clear interrupts handled with TPM_EOI */
	spi_write8_reg(tpm, TPM_INT_STATUS, &interrupt, 1);
	spi_read8_reg(tpm, TPM_INT_STATUS, &interrupt, 1);
} /* clear_interruption() */

/*
 * _wait_for_serirq_timeout
 * @param: tpm, the chip description
 * @param: timeout, the timeout of the interrupt
 * @return: the status of the interruption.
 */
static unsigned long wait_for_serirq_timeout(struct tpm_chip *chip,
	 bool condition, unsigned long timeout)
{
	long status = 0;
	struct spi_device *client;
	struct st33zp24_platform_data *pin_infos;

	client = (struct spi_device __force *)chip->vendor.iobase;
	pin_infos = client->dev.platform_data;

	status = wait_for_completion_interruptible_timeout(
				&pin_infos->irq_detection, timeout);
	if (status > 0)
		enable_irq(gpio_to_irq(pin_infos->io_serirq));
	gpio_direction_input(pin_infos->io_serirq);

	if (!status)
		return -EBUSY;

	clear_interruption(chip);
	if (condition)
		status = 1;
return status;
}


/*
 * tpm_stm_spi_cancel, cancel is not implemented.
 * @param: chip, the tpm chip description as specified in
 * driver/char/tpm/tpm.h.
 */
static void tpm_stm_spi_cancel(struct tpm_chip *chip)
{
	u8 data = TPM_STS_COMMAND_READY;

	/* this causes the current command to be aborted */
	spi_write8_reg(chip, TPM_STS, &data, 1);
} /* tpm_stm_spi_cancel() */

/*
 * tpm_stm_spi_status return the TPM_STS register
 * @param: chip, the tpm chip description
 * @return: the TPM_STS register value.
 */
static u8 tpm_stm_spi_status(struct tpm_chip *chip)
{
	u8 data = 0;

	spi_read8_reg(chip, TPM_STS, &data, 1);
	return data;
} /* tpm_stm_spi_status() */



/*
 * check_locality if the locality is active
 * @param: chip, the tpm chip description
 * @return: the active locality or -EACCESS.
 */
static int check_locality(struct tpm_chip *chip)
{
	u8 data = 0;
	u8 status;

	status = spi_read8_reg(chip, TPM_ACCESS, &data, 1);
	if (status && (data &
		(TPM_ACCESS_ACTIVE_LOCALITY | TPM_ACCESS_VALID)) ==
		(TPM_ACCESS_ACTIVE_LOCALITY | TPM_ACCESS_VALID))
		return chip->vendor.locality;

	return -EACCES;
} /* check_locality() */

/*
 * request_locality request the TPM locality
 * @param: chip, the chip description
 * @return: the active locality or EACCESS.
 */
static int request_locality(struct tpm_chip *chip)
{
	unsigned long stop;
	long rc;
	u8 data = 0;

	/* Check locality */
	if (check_locality(chip) == chip->vendor.locality)
		return chip->vendor.locality;

	/* Request locality */
	data = TPM_ACCESS_REQUEST_USE;
	rc = spi_write8_reg(chip, TPM_ACCESS, &data, 1);
	if (rc < 0)
		return -EACCES;

	/* wait for burstcount */
	if (chip->vendor.irq) {
		rc = wait_for_serirq_timeout(chip, (check_locality
						(chip) >= 0),
						chip->vendor.timeout_a);
		if (rc > 0)
			return chip->vendor.locality;
	} else {
		stop = jiffies + chip->vendor.timeout_a;
		do {
			if (check_locality(chip) >= 0)
				return chip->vendor.locality;
			msleep(TPM_TIMEOUT);
		} while (time_before(jiffies, stop));
	}
return -EACCES;
} /* request_locality() */

/*
 * release_locality release the active locality
 * @param: chip, the tpm chip description.
 */
static void release_locality(struct tpm_chip *chip)
{
	u8 data = 0;

	data = TPM_ACCESS_ACTIVE_LOCALITY;
	spi_write8_reg(chip, TPM_ACCESS, &data, 1);
} /* release_locality()*/

/*
 * get_burstcount return the burstcount address 0x19 0x1A
 * @param: chip, the chip description
 * return: the burstcount.
 */
static int get_burstcount(struct tpm_chip *chip)
{
	unsigned long stop;
	u32 burstcnt;
	u8 tpm_reg, temp = 0;
	long status = 0;

	/* wait for burstcount */
	/* which timeout value, spec has 2 answers (c & d) */
	stop = jiffies + chip->vendor.timeout_d;
	do {
		tpm_reg = TPM_STS + 1;
		status = spi_read8_reg(chip, tpm_reg, &temp, 1);
		if (status < 0)
			return -EBUSY;

		burstcnt = temp;
		status = spi_read8_reg(chip, ++tpm_reg, &temp, 1);
		if (status < 0)
			return -EBUSY;

		burstcnt |= temp << 8;
		if (burstcnt)
			return burstcnt;
		msleep(TPM_TIMEOUT);
	} while (time_before(jiffies, stop));
return -EBUSY;
} /* get_burstcount() */

/*
 * wait_for_stat wait for a TPM_STS value
 * @param: chip, the tpm chip description
 * @param: mask, the value mask to wait
 * @param: timeout, the timeout
 * @param: queue, the wait queue.
 * @return: the tpm status, 0 if success, -ETIME if timeout is reached.
 */
static int wait_for_stat(struct tpm_chip *chip, u8 mask, unsigned long timeout,
			 wait_queue_head_t *queue)
{
	unsigned long stop;
	long rc;
	u8 status;

	/* check current status */
	status = tpm_stm_spi_status(chip);
	if (!chip->vendor.irq && (status & mask) == mask)
		return 0;

	if (chip->vendor.irq) {
		rc = wait_for_serirq_timeout(chip, ((tpm_stm_spi_status
						    (chip) & mask) ==
						     mask), timeout);
		if (rc > 0)
			return 0;
	} else {
		stop = jiffies + timeout;
		do {
			msleep(TPM_TIMEOUT);
			status = tpm_stm_spi_status(chip);
			if ((status & mask) == mask)
				return 0;
		} while (time_before(jiffies, stop));
	}
return -ETIME;
} /* wait_for_stat() */

/*
 * recv_data receive data
 * @param: chip, the tpm chip description
 * @param: buf, the buffer where the data are received
 * @param: count, the number of data to receive
 * @return: the tpm status, 0 if success, -ETIME if timeout is reached.
 */
static int recv_data(struct tpm_chip *chip, u8 *buf, size_t count)
{
	u32 size = 0, burstcnt, len;
	long status = 0;

	while (size < count &&
	       wait_for_stat(chip,
			     TPM_STS_DATA_AVAIL | TPM_STS_VALID,
			     chip->vendor.timeout_c,
			     &chip->vendor.read_queue)
						== 0) {
		burstcnt = get_burstcount(chip);
		len = min_t(int, burstcnt, count - size);
		status = spi_read8_reg(chip, TPM_DATA_FIFO, buf + size, len);
		if (status < 0)
			return status;


		size += len;
	}
return size;
}

/*
 * tpm_ioserirq_handler the serirq irq handler
 * @param: irq, the tpm chip description
 * @param: dev_id, the description of the chip
 * @return: the status of the handler.
 */
static irqreturn_t tpm_ioserirq_handler(int irq, void *dev_id)
{
	struct tpm_chip *chip = dev_id;
	struct spi_device *client;
	struct st33zp24_platform_data *pin_infos;

	disable_irq_nosync(irq);

	client = (struct spi_device __force *)chip->vendor.iobase;
	pin_infos = client->dev.platform_data;

	complete(&pin_infos->irq_detection);
	return IRQ_HANDLED;
} /* tpm_ioserirq_handler() */

/*
 * tpm_stm_spi_send send TPM commands through the SPI bus.
 * @param: chip, the tpm chip description
 * @param: buf, the tpm command buffer
 * @param: len, the tpm command size
 * @return: should be zero if success else the negative error code.
 */
static int tpm_stm_spi_send(struct tpm_chip *chip, unsigned char *buf,
			    size_t len)
{
	u32 burstcnt = 0, i, size = 0;
	u8 data = 0;
	long status = 0, ret = 0;

	if (chip == NULL)
		return -EINVAL;
	if (len < TPM_HEADER_SIZE)
		return -EINVAL;

	ret = request_locality(chip);
	if (ret < 0)
		return ret;

	status = tpm_stm_spi_status(chip);
	if ((status & TPM_STS_COMMAND_READY) == 0) {
		tpm_stm_spi_cancel(chip);
		if (wait_for_stat
		    (chip, TPM_STS_COMMAND_READY, chip->vendor.timeout_b,
		     &chip->vendor.int_queue) < 0) {
			ret = -ETIME;
			goto out_err;
		}
	}

	for (i = 0; i < len - 1;) {
		burstcnt = get_burstcount(chip);
		size = min_t(int, len - i - 1, burstcnt);
		ret = spi_write8_reg(chip, TPM_DATA_FIFO, buf, size);
		if (ret < 0)
			goto out_err;
		i += size;
	}

	status = tpm_stm_spi_status(chip);
	if ((status & TPM_STS_DATA_EXPECT) == 0) {
		ret = -EIO;
		goto out_err;
	}

	/* write last byte */
	spi_write8_reg(chip, TPM_DATA_FIFO, buf + len - 1, 1);

	status = tpm_stm_spi_status(chip);
	if ((status & TPM_STS_DATA_EXPECT) != 0) {
		ret = -EIO;
		goto out_err;
	}

	/* go and do it */
	data = TPM_STS_GO;
	ret = spi_write8_reg(chip, TPM_STS, &data, 1);
	if (ret < 0)
		goto out_err;

	return len;
out_err:
	tpm_stm_spi_cancel(chip);
	release_locality(chip);
	return ret;
}

/*
 * tpm_stm_spi_recv received TPM response through the SPI bus.
 * @param: chip, the tpm chip description
 * @param: buf, the tpm command buffer
 * @param: len, the tpm command size
* @return: should be zero if success else the negative error code.
 */
static int tpm_stm_spi_recv(struct tpm_chip *chip, unsigned char *buf,
			    size_t count)
{
	int size = 0;
	int expected;

	if (chip == NULL)
		return -EINVAL;
	if (count < TPM_HEADER_SIZE) {
		size = -EIO;
		goto out;
	}

	size = recv_data(chip, buf, TPM_HEADER_SIZE);

	/* read first 10 bytes, including tag, paramsize, and result */
	if (size < TPM_HEADER_SIZE) {
		dev_err(chip->dev, "Unable to read header\n");
		goto out;
	}

	expected = be32_to_cpu(*(__be32 *)(buf + 2));
	if (expected > count) {
		size = -EIO;
		goto out;
	}

	size += recv_data(chip, &buf[TPM_HEADER_SIZE],
					expected - TPM_HEADER_SIZE);
	if (size < expected) {
		dev_err(chip->dev, "Unable to read remainder of result\n");
		size = -ETIME;
		goto out;
	}

out:
	chip->vendor.cancel(chip);
	release_locality(chip);
	return size;
}

static DEVICE_ATTR(pubek, S_IRUGO, tpm_show_pubek, NULL);
static DEVICE_ATTR(pcrs, S_IRUGO, tpm_show_pcrs, NULL);
static DEVICE_ATTR(enabled, S_IRUGO, tpm_show_enabled, NULL);
static DEVICE_ATTR(active, S_IRUGO, tpm_show_active, NULL);
static DEVICE_ATTR(owned, S_IRUGO, tpm_show_owned, NULL);
static DEVICE_ATTR(temp_deactivated, S_IRUGO, tpm_show_temp_deactivated, NULL);
static DEVICE_ATTR(caps, S_IRUGO, tpm_show_caps, NULL);
static DEVICE_ATTR(cancel, S_IWUSR | S_IWGRP, NULL, tpm_store_cancel);

static struct attribute *stm_tpm_attrs[] = {
	&dev_attr_pubek.attr,
	&dev_attr_pcrs.attr,
	&dev_attr_enabled.attr,
	&dev_attr_active.attr,
	&dev_attr_owned.attr,
	&dev_attr_temp_deactivated.attr,
	&dev_attr_caps.attr,
	&dev_attr_cancel.attr, NULL,
};

static struct attribute_group stm_tpm_attr_grp = {
	.attrs = stm_tpm_attrs
};

static struct tpm_vendor_specific st_spi_tpm = {
	.send = tpm_stm_spi_send,
	.recv = tpm_stm_spi_recv,
	.cancel = tpm_stm_spi_cancel,
	.status = tpm_stm_spi_status,
	.attr_group = &stm_tpm_attr_grp,
};

static int evaluate_latency(struct tpm_chip *chip)
{
	int latency = 0;
	struct spi_device *dev =
		 (struct spi_device __force *)chip->vendor.iobase;
	struct st33zp24_platform_data *platform_data = dev->dev.platform_data;
	int status = 0;
	u8 data = 0;

	while (status == 0x00 && latency < MAX_SPI_LATENCY) {
		platform_data->latency = latency;
		status = spi_read8_reg(chip, TPM_INTF_CAPABILITY, &data, 1);
		latency++;
	}
	return latency - 1;
} /* evaluate_latency() */

/*
 * tpm_st33_spi_probe initialize the TPM device
 * @param: client, the spi_device drescription (TPM SPI description).
 * @param: id, the spi_device_id struct.
 * @return: 0 in case of success.
 *	 -1 in other case.
 */
static int
tpm_st33_spi_probe(struct spi_device *dev)
{
	long err = 0;
	u8 intmask;
	struct tpm_chip *chip;
	struct st33zp24_platform_data *platform_data;

	/* Check SPI platform functionnalities */
	if (dev == NULL) {
		pr_info("dev is NULL. exiting.\n");
		err = -ENODEV;
		goto end;
	}

	chip = tpm_register_hardware(&dev->dev, &st_spi_tpm);
	if (!chip) {
		err = -ENODEV;
		goto end;
	}

	platform_data = dev->dev.platform_data;

	if (!platform_data)
		return -ENODEV;

	platform_data->tpm_spi_buffer[0] =
	kmalloc((TPM_BUFSIZE + (TPM_BUFSIZE / 2) +
		 TPM_DIGEST_SIZE) * sizeof(u8), GFP_KERNEL);

	if (platform_data->tpm_spi_buffer[0] == NULL) {
		err = -ENOMEM;
		goto _tpm_clean_answer;
	}

	platform_data->tpm_spi_buffer[1] =
	    kmalloc((TPM_BUFSIZE + (TPM_BUFSIZE / 2) +
				 TPM_DIGEST_SIZE) * sizeof(u8) , GFP_KERNEL);

	if (platform_data->tpm_spi_buffer[1] == NULL) {
		err = -ENOMEM;
		goto _tpm_clean_response;
	}

	chip->vendor.iobase = (void __iomem *)dev;

	/* Default timeouts */
	chip->vendor.timeout_a = msecs_to_jiffies(TIS_SHORT_TIMEOUT);
	chip->vendor.timeout_b = msecs_to_jiffies(TIS_LONG_TIMEOUT);
	chip->vendor.timeout_c = msecs_to_jiffies(TIS_SHORT_TIMEOUT);
	chip->vendor.timeout_d = msecs_to_jiffies(TIS_SHORT_TIMEOUT);
	chip->vendor.locality = LOCALITY0;

	if (power_mgt) {
		err = gpio_request(platform_data->io_lpcpd, "TPM IO_LPCPD");
		if (err)
			goto _gpio_init1;
		gpio_set_value(platform_data->io_lpcpd, 1);
	}

	platform_data->latency = evaluate_latency(chip);
	if (platform_data->latency <= 0x00) {
		err = -ENODEV;
		goto _gpio_init1;
	}

	/* Enable interrupt */
	/* Register GPIO pin through generic Linux GPIO API */
	if (interrupts) {
		init_completion(&platform_data->irq_detection);
		if (request_locality(chip) != LOCALITY0) {
			err = -ENODEV;
			goto _tpm_clean_response;
		}
		err = gpio_request(platform_data->io_serirq, "TPM IO_SERIRQ");
		if (err)
			goto _gpio_init2;

		/* Clear all existing */
		clear_interruption(chip);
		err = request_irq(gpio_to_irq(platform_data->io_serirq),
							&tpm_ioserirq_handler,
				IRQF_TRIGGER_HIGH | IRQF_SHARED,
				"TPM SERIRQ management", chip);
		if (err < 0) {
			pr_info("TPM SERIRQ signals %d not available\n",
				gpio_to_irq(platform_data->io_serirq));
			goto _irq_set;
		}
		err = spi_read8_reg(chip, TPM_INT_ENABLE, &intmask, 1);
		if (err < 0)
			goto _irq_set;

		intmask |= TPM_INTF_CMD_READY_INT
			|  TPM_INTF_FIFO_AVALAIBLE_INT
			|  TPM_INTF_WAKE_UP_READY_INT
			|  TPM_INTF_LOC4SOFTRELEASE_INT
			|  TPM_INTF_LOCALITY_CHANGE_INT
			|  TPM_INTF_STS_VALID_INT
			|  TPM_INTF_DATA_AVAIL_INT;

		err = spi_write8_reg(chip, TPM_INT_ENABLE, &intmask, 1);
		if (err < 0)
			goto _irq_set;

		intmask = TPM_GLOBAL_INT_ENABLE;
		err = spi_write8_reg(chip, TPM_INT_ENABLE + 3, &intmask, 1);
		if (err < 0)
			goto _irq_set;

		err = spi_read8_reg(chip, TPM_INT_STATUS, &intmask, 1);
		if (err < 0)
			goto _irq_set;

		chip->vendor.irq = interrupts;

		tpm_gen_interrupt(chip);
	}

	tpm_get_timeouts(chip);

	platform_data->bchipf = false;

	pr_info("TPM SPI Initialized\n");
	return 0;
_irq_set:
	free_irq(gpio_to_irq(platform_data->io_serirq), chip);
_gpio_init2:
	if (platform_data && interrupts)
		gpio_free(platform_data->io_serirq);
_gpio_init1:
	if (platform_data && power_mgt)
		gpio_free(platform_data->io_lpcpd);
_tpm_clean_response:
	tpm_remove_hardware(chip->dev);
	if (platform_data->tpm_spi_buffer[1] != NULL) {
		kfree(platform_data->tpm_spi_buffer[1]);
		platform_data->tpm_spi_buffer[1] = NULL;
	}
_tpm_clean_answer:
	if (platform_data->tpm_spi_buffer[0] != NULL) {
		kfree(platform_data->tpm_spi_buffer[0]);
		platform_data->tpm_spi_buffer[0] = NULL;
	}

	platform_data->bchipf = true;
end:
	pr_info("TPM SPI initialisation fail\n");
	return err;
}

/*
 * tpm_st33_spi_remove remove the TPM device
 * @param: client, the spi_device drescription (TPM SPI description).
		clear_bit(0, &chip->is_open);
 * @return: 0 in case of success.
 */
static int tpm_st33_spi_remove(struct spi_device *client)
{
	struct tpm_chip *chip = (struct tpm_chip *)spi_get_drvdata(client);
	struct st33zp24_platform_data *pin_infos =
	 ((struct spi_device __force *)chip->vendor.iobase)->dev.platform_data;

	if (pin_infos != NULL) {
		gpio_free(pin_infos->io_lpcpd);

		/* Check if chip has been previously clean */
		if (pin_infos->bchipf != true)
			tpm_remove_hardware(chip->dev);
		if (pin_infos->tpm_spi_buffer[1] != NULL) {
			kfree(pin_infos->tpm_spi_buffer[1]);
			pin_infos->tpm_spi_buffer[1] = NULL;
		}
		if (pin_infos->tpm_spi_buffer[0] != NULL) {
			kfree(pin_infos->tpm_spi_buffer[0]);
			pin_infos->tpm_spi_buffer[0] = NULL;
		}
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP
/*
 * tpm_st33_spi_pm_suspend suspend the TPM device
 * @return: 0 in case of success.
 */
static int tpm_st33_spi_pm_suspend(struct device *dev)
{
	struct st33zp24_platform_data *pin_infos = dev->platform_data;
	int ret = 0;

	if (power_mgt)
		gpio_set_value(pin_infos->io_lpcpd, 0);
	else {
		ret = tpm_pm_suspend(dev);
		}
	return ret;
}				/* tpm_st33_spi_suspend() */

/*
 * tpm_st33_spi_pm_resume resume the TPM device
 * @param: spi, the spi_device drescription (TPM SPI description).
 * @return: 0 in case of success.
 */
static int tpm_st33_spi_pm_resume(struct device *dev)
{
	struct tpm_chip *chip = dev_get_drvdata(dev);
	struct st33zp24_platform_data *pin_infos = dev->platform_data;

	int ret = 0;
	if (power_mgt) {
		gpio_set_value(pin_infos->io_lpcpd, 1);
		ret = wait_for_serirq_timeout(chip,
					 (chip->vendor.status(chip) &
					  TPM_STS_VALID) == TPM_STS_VALID,
					  chip->vendor.timeout_b);
	} else {
		ret = tpm_pm_resume(dev);
		if (!ret)
			tpm_do_selftest(chip);
	}
	return ret;
} /* tpm_st33_spi_pm_resume() */
#endif

static SIMPLE_DEV_PM_OPS(tpm_st33_spi_ops, tpm_st33_spi_pm_suspend,
tpm_st33_spi_pm_resume);
static struct spi_driver tpm_st33_spi_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = TPM_ST33_SPI,
		   .pm = &tpm_st33_spi_ops,
		   },
	.probe = tpm_st33_spi_probe,
	.remove = tpm_st33_spi_remove,
};

module_spi_driver(tpm_st33_spi_driver);

MODULE_AUTHOR("Christophe Ricard (tpmsupport@st.com)");
MODULE_DESCRIPTION("STM TPM SPI ST33 Driver");
MODULE_VERSION("1.2.0");
