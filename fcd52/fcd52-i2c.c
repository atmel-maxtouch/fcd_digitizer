// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Microchip ATFCD52 Fast Capacitive Digitizer - I2C interface
 *
 * I2C interface driver using Object-Based Protocol I2C interface.
 * Used to configure, control upgrade device. Sample data is
 * delivered over the SPI frame data.  Refer to fcd52-spi.c driver.
 *
 * Copyright (c) 2026 Microchip Technology Inc.
 */

#include <linux/i2c.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include "fcd52.h"

/*
 * Analog and digital supplies. Assume vddio same as vdd.
 * xvdd -- Internal tripler or doubler, set in config
 */
static const char * const fcd52_supplies[] = { "vdd", "avdd" };

/*
 * Reads take two bus transactions: a write of the 16-bit little-endian
 * address pointer followed by the data phase.
 * Encryption handling not included
 * CRC bit not enabled.
 */
static int fcd52_i2c_read(struct fcd52_data *data, u16 reg, u16 len, void *val)
{
	struct i2c_client *client = to_i2c_client(data->dev);
	u8 *buf = val;
	int ret;

	while (len) {
		u16 chunk = min_t(u16, len, FCD52_MAX_BLOCK_RW);
		struct i2c_msg xfer[2];
		u8 addr[2];

		addr[0] = reg & 0xff;
		addr[1] = (reg >> 8) & 0xff;

		xfer[0].addr = client->addr;
		xfer[0].flags = 0;
		xfer[0].len = sizeof(addr);
		xfer[0].buf = addr;

		xfer[1].addr = client->addr;
		xfer[1].flags = I2C_M_RD;
		xfer[1].len = chunk;
		xfer[1].buf = buf;

		ret = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
		if (ret != ARRAY_SIZE(xfer)) {
			ret = ret < 0 ? ret : -EIO;
			dev_err(data->dev,
				"read of %u bytes at 0x%04x failed: %d\n",
				chunk, reg, ret);
			return ret;
		}

		reg += chunk;
		buf += chunk;
		len -= chunk;
	}

	return 0;
}

/*
 * Writes are a single transaction holding the 16-bit little-endian address
 * pointer followed by the data (Data Sheet Section 8.2).
 */
static int fcd52_i2c_write(struct fcd52_data *data, u16 reg, u16 len,
			   const void *val)
{
	struct i2c_client *client = to_i2c_client(data->dev);
	const u8 *src = val;
	u8 *buf;
	int ret = 0;

	buf = kmalloc(min_t(u16, len, FCD52_MAX_BLOCK_RW) + 2, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	while (len) {
		u16 chunk = min_t(u16, len, FCD52_MAX_BLOCK_RW);

		buf[0] = reg & 0xff;
		buf[1] = (reg >> 8) & 0xff;
		memcpy(&buf[2], src, chunk);

		ret = i2c_master_send(client, buf, chunk + 2);
		if (ret != chunk + 2) {
			ret = ret < 0 ? ret : -EIO;
			dev_err(data->dev,
				"write of %u bytes at 0x%04x failed: %d\n",
				chunk, reg, ret);
			goto out_free;
		}

		reg += chunk;
		src += chunk;
		len -= chunk;
	}

	ret = 0;

out_free:
	kfree(buf);
	return ret;
}

/* Bootloader reads and writes: no register address, one raw transfer */
static int fcd52_i2c_bl_read(struct fcd52_data *data, u8 *val,
			     unsigned int len)
{
	struct i2c_client *client = to_i2c_client(data->dev);
	struct i2c_msg msg = {
		.addr = data->bl_addr,
		.flags = I2C_M_RD,
		.len = len,
		.buf = val,
	};
	int ret;

	ret = i2c_transfer(client->adapter, &msg, 1);
	return ret == 1 ? 0 : (ret < 0 ? ret : -EIO);
}

static int fcd52_i2c_bl_write(struct fcd52_data *data, const u8 *val,
			      unsigned int len)
{
	struct i2c_client *client = to_i2c_client(data->dev);
	struct i2c_msg msg = {
		.addr = data->bl_addr,
		.flags = 0,
		.len = len,
		.buf = (u8 *)val,
	};
	int ret;

	ret = i2c_transfer(client->adapter, &msg, 1);
	return ret == 1 ? 0 : (ret < 0 ? ret : -EIO);
}

static const struct fcd52_bus_ops fcd52_i2c_bus_ops = {
	.reg_read = fcd52_i2c_read,
	.reg_write = fcd52_i2c_write,
	.bl_read = fcd52_i2c_bl_read,
	.bl_write = fcd52_i2c_bl_write,
};

static int fcd52_i2c_probe(struct i2c_client *client)
{
	int error;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;

	/* Enabled voltages before reset is released */
	error = devm_regulator_bulk_get_enable(&client->dev,
					       ARRAY_SIZE(fcd52_supplies),
					       fcd52_supplies);
	if (error)
		return dev_err_probe(&client->dev, error,
				     "Failed to enable regulators\n");

	return fcd52_core_probe(&client->dev, &fcd52_i2c_bus_ops,
				client->name, client->addr, client->irq);
}

static const struct i2c_device_id fcd52_i2c_id[] = {
	{ "atfcd52" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fcd52_i2c_id);

static const struct of_device_id fcd52_of_match[] = {
	{ .compatible = "microchip,atfcd52" },
	{ }
};
MODULE_DEVICE_TABLE(of, fcd52_of_match);

static struct i2c_driver fcd52_i2c_driver = {
	.driver = {
		.name = "atfcd52",
		.of_match_table = fcd52_of_match,
	},
	.probe = fcd52_i2c_probe,
	.id_table = fcd52_i2c_id,
};
module_i2c_driver(fcd52_i2c_driver);

MODULE_AUTHOR("Michael Gong <michael.gong@microchip.com>");
MODULE_DESCRIPTION("Microchip ATFCD52 Fast Capacitive Digitizer I2C interface");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_FCD52);
