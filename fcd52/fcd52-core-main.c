// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Microchip ATFCD52 Fast Capacitive Digitizer - core driver
 *
 * Implements the Object-Based Protocol (OBP) control plane: reading the
 * information block, building the object table (including the 16-bit types
 * held in the Extended Object Table T254) and register access.
 *
 * Copyright (c) 2026 Microchip Technology Inc.
 */

#include <linux/crc32.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "fcd52.h"
#include "fcd52-cfg.h"

/*
 * 24-bit CRC, initial value zero.
 * The checksum consumes two bytes per iteration.
 */
u32 fcd52_crc24(u32 crc, u8 firstbyte, u8 secondbyte)
{
	static const u32 crcpoly = 0x80001b;
	u32 data_word = (u32)firstbyte | ((u32)secondbyte << 8);
	u32 result = data_word ^ (crc << 1);

	if (result & 0x1000000)
		result ^= crcpoly;

	return result;
}
EXPORT_SYMBOL_NS_GPL(fcd52_crc24, IIO_FCD52);

/*
 * Calculate the 24-bit CRC over @len bytes of @base.
 * An odd length is padded with zero.
 */
u32 fcd52_calculate_crc(const u8 *base, size_t len)
{
	u32 crc = 0;
	size_t i;

	for (i = 0; i + 1 < len; i += 2)
		crc = fcd52_crc24(crc, base[i], base[i + 1]);

	if (i < len)
		crc = fcd52_crc24(crc, base[i], 0);

	return crc & 0x00ffffff;
}
EXPORT_SYMBOL_NS_GPL(fcd52_calculate_crc, IIO_FCD52);

/*
 * 8-bit CRC, polynomial 0x8C, initial value zero.
 * Used for optional checksum mode
 */
u8 fcd52_crc8(u8 crc, u8 data)
{
	static const u8 crcpoly = 0x8c;
	unsigned int i;

	for (i = 0; i < 8; i++) {
		u8 fb = (crc ^ data) & 0x01;

		data >>= 1;
		crc >>= 1;
		if (fb)
			crc ^= crcpoly;
	}

	return crc;
}
EXPORT_SYMBOL_NS_GPL(fcd52_crc8, IIO_FCD52);

/*
 * Bus access for the CHG handler. It is not gated, because it must read the
 * RESET message to complete a software reset.
 */
static int fcd52_bus_read(struct fcd52_data *data, u16 reg, u16 len, void *val)
{
	int ret;

	mutex_lock(&data->bus_lock);
	ret = data->bops->reg_read(data, reg, len, val);
	mutex_unlock(&data->bus_lock);

	return ret;
}

/* Ungated write, used by the reset helper to issue the command itself */
static int fcd52_bus_write(struct fcd52_data *data, u16 reg, u16 len,
			   const void *val)
{
	int ret;

	mutex_lock(&data->bus_lock);
	ret = data->bops->reg_write(data, reg, len, val);
	mutex_unlock(&data->bus_lock);

	return ret;
}

/*
 * Caller access. Refused while a software reset is in progress, so the
 * device is not addressed until its reset sequence has finished.
 */
int fcd52_read_reg(struct fcd52_data *data, u16 reg, u16 len, void *val)
{
	if (READ_ONCE(data->reset_busy))
		return -EBUSY;

	return fcd52_bus_read(data, reg, len, val);
}
EXPORT_SYMBOL_NS_GPL(fcd52_read_reg, IIO_FCD52);

int fcd52_write_reg(struct fcd52_data *data, u16 reg, u16 len, const void *val)
{
	int ret;

	if (READ_ONCE(data->reset_busy))
		return -EBUSY;

	mutex_lock(&data->bus_lock);
	ret = data->bops->reg_write(data, reg, len, val);
	mutex_unlock(&data->bus_lock);

	return ret;
}
EXPORT_SYMBOL_NS_GPL(fcd52_write_reg, IIO_FCD52);

/*
 * Look up an object by type. Covers both the main object table and the
 * extended objects. Returns the first instance.
 */
struct fcd52_object *fcd52_get_object(struct fcd52_data *data, u16 type)
{
	unsigned int i;

	if (!data->object_table)
		return NULL;

	for (i = 0; i < data->object_num; i++) {
		if (data->object_table[i].type == type)
			return &data->object_table[i];
	}

	return NULL;
}
EXPORT_SYMBOL_NS_GPL(fcd52_get_object, IIO_FCD52);

static void fcd52_free_object_table(struct fcd52_data *data)
{
	kfree(data->dbg_ring);
	data->dbg_ring = NULL;
	data->dbg_ring_slots = 0;
	data->debug_v2 = false;

	kfree(data->t5_msg);
	data->t5_msg = NULL;
	data->t5_msg_len = 0;

	kfree(data->object_table);
	data->object_table = NULL;
	data->object_num = 0;

	kfree(data->raw_info_block);
	data->raw_info_block = NULL;
	data->info = NULL;

	data->max_reportid = 0;
	data->mem_size = 0;
	data->t254_address = 0;
	data->t254_size = 0;
	memset(&data->cached, 0, sizeof(data->cached));
}

/*
 * Record the memory map size of object table.
 */
static void fcd52_account_object(struct fcd52_data *data,
				 const struct fcd52_object *object)
{
	unsigned int end = object->start_address +
			   object->size * object->instances;

	if (end > data->mem_size)
		data->mem_size = end;
}

/*
 * Allocate report IDs for @object, continuing the sequence in @reportid.
 * Valid report IDs start at 1; 0x00 and 0xFF is reserved
 */
static void fcd52_assign_report_ids(struct fcd52_data *data,
				    struct fcd52_object *object,
				    u16 *reportid)
{
	if (!object->num_report_ids) {
		object->min_reportid = 0;
		object->max_reportid = 0;
		return;
	}

	object->min_reportid = *reportid;
	*reportid += object->num_report_ids * object->instances;
	object->max_reportid = *reportid - 1;

	data->max_reportid = object->max_reportid;
}

/*
 * Decode the main object table. Entries are six bytes wide and carry 8-bit
 * type codes (Protocol Guide Table 1-3).
 */
/*
 * Record the location of an object the driver configures. Called for every
 * object, main table or extended, so that both tables are covered.
 */
static void fcd52_cache_object(struct fcd52_data *data,
			       const struct fcd52_object *object)
{
	struct fcd52_obj_ref *ref;

	switch (object->type) {
	case FCD52_GEN_MESSAGE_T5:
		ref = &data->cached.t5;
		break;
	case FCD52_GEN_COMMAND_T6:
		ref = &data->cached.t6;
		break;
	case FCD52_GEN_POWER_T7:
		ref = &data->cached.t7;
		break;
	case FCD52_SPT_SPIDEBUGCONFIG_T17:
		ref = &data->cached.t17;
		break;
	case FCD52_SPT_COMMSCONFIG_T18:
		ref = &data->cached.t18;
		break;
	case FCD52_SPT_USERDATA_T38:
		ref = &data->cached.t38;
		break;
	case FCD52_DEBUG_DIAGNOSTIC_T37:
		ref = &data->cached.t37;
		break;
	case FCD52_SPT_MESSAGECOUNT_T44:
		ref = &data->cached.t44;
		break;
	case FCD52_SPT_SERIALDATACOMMAND_T68:
		ref = &data->cached.t68;
		break;
	case FCD52_SPT_FASTCAPDIGITIZER_T700:
		ref = &data->cached.t700;
		break;
	default:
		return;
	}

	ref->present = true;
	ref->start_address = object->start_address;
	ref->size = object->size;
	ref->instances = object->instances;
	ref->min_reportid = object->min_reportid;
}

static int fcd52_parse_object_table(struct fcd52_data *data, const u8 *raw,
				    u16 *reportid)
{
	unsigned int i;

	for (i = 0; i < data->info->object_num; i++) {
		const u8 *entry = raw + i * FCD52_OBJECT_ELEM_SIZE;
		struct fcd52_object *object = &data->object_table[i];

		object->type = entry[0];
		object->start_address = get_unaligned_le16(&entry[1]);
		object->size = entry[3] + 1;
		object->instances = entry[4] + 1;
		object->num_report_ids = entry[5];

		fcd52_assign_report_ids(data, object, reportid);
		fcd52_account_object(data, object);
		fcd52_cache_object(data, object);

		if (object->type == FCD52_GEN_EXTOBJECTTABLE_T254) {
			data->t254_address = object->start_address;
			data->t254_size = object->size;
		}

		dev_info(data->dev,
			 "T%u Start:%u Size:%u Instances:%u Rpt IDs:%u-%u\n",
			 object->type, object->start_address, object->size,
			 object->instances, object->min_reportid,
			 object->max_reportid);
	}

	data->object_num = data->info->object_num;

	/*
	 * Message reads rely on the Message Count T44 sitting immediately
	 * before the Message Processor T5 (Protocol Guide Section 3.2.1.1). A
	 * continuous DMA read of T44 followed by T5 breaks if there is a gap.
	 * Check the addresses here, since the cache is filled by this point.
	 */
	if (data->cached.t44.present && data->cached.t5.present &&
	    data->cached.t5.start_address !=
	    data->cached.t44.start_address + data->cached.t44.size) {
		dev_err(data->dev,
			"T5 at %u is not directly after T44 at %u\n",
			data->cached.t5.start_address,
			data->cached.t44.start_address);
		return -EINVAL;
	}

	return 0;
}

/*
 * Read and decode the Extended Object Table T254. Object holds
 * a 7-byte element with a 16-bit type code, followed 24-bit CRC.
 * An element with a zero start address is unused.
 */
static int fcd52_parse_ext_object_table(struct fcd52_data *data, u16 *reportid)
{
	struct fcd52_object *object;
	u32 calculated_crc, stored_crc;
	const u8 *entry;
	u8 *buf;
	int error;

	if (!data->t254_address)
		return 0;

	if (data->t254_size < FCD52_T254_SIZE) {
		dev_err(data->dev, "T254 too small: %u bytes\n",
			data->t254_size);
		return -EINVAL;
	}

	buf = kzalloc(data->t254_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	error = fcd52_read_reg(data, data->t254_address, data->t254_size, buf);
	if (error) {
		dev_err(data->dev, "Failed to read T254: %d\n", error);
		goto out_free;
	}

	stored_crc = get_unaligned_le24(buf + data->t254_size -
					FCD52_INFO_CRC_SIZE);
	calculated_crc = fcd52_calculate_crc(buf, data->t254_size -
					     FCD52_INFO_CRC_SIZE);
	if (stored_crc != calculated_crc) {
		dev_err(data->dev,
			"T254 CRC error calculated=0x%06X read=0x%06X\n",
			calculated_crc, stored_crc);
		error = -EIO;
		goto out_free;
	}

	entry = buf;
	if (!get_unaligned_le16(&entry[2])) {
		dev_dbg(data->dev, "T254 holds no extended objects\n");
		goto out_free;
	}

	object = &data->object_table[data->object_num];
	object->type = get_unaligned_le16(&entry[0]);
	object->start_address = get_unaligned_le16(&entry[2]);
	object->size = entry[4] + 1;
	object->instances = entry[5] + 1;
	object->num_report_ids = entry[6];

	fcd52_assign_report_ids(data, object, reportid);
	fcd52_account_object(data, object);
	fcd52_cache_object(data, object);
	data->object_num++;

	dev_info(data->dev,
		 "T%u (ext) Start:%u Size:%u Instances:%u Rpt IDs:%u-%u\n",
		 object->type, object->start_address, object->size,
		 object->instances, object->min_reportid, object->max_reportid);

out_free:
	kfree(buf);
	return error;
}

/* Object table read after an update: retries and spacing */
#define FCD52_OBJ_READ_TRIES		50
#define FCD52_OBJ_READ_RETRY_MS		20

/*
 * Read the infoblock and build the object table.
 */
static int fcd52_read_info_block(struct fcd52_data *data)
{
	struct fcd52_info *info;
	u32 calculated_crc;
	size_t size;
	void *id_buf, *buf;
	u16 reportid = 1;
	int retry;
	int error;

	fcd52_free_object_table(data);

	id_buf = kzalloc(FCD52_ID_INFO_SIZE, GFP_KERNEL);
	if (!id_buf)
		return -ENOMEM;

	error = fcd52_read_reg(data, 0, FCD52_ID_INFO_SIZE, id_buf);
	if (error) {
		dev_err(data->dev, "Failed to read ID information: %d\n",
			error);
		goto err_free_mem;
	}

	info = id_buf;
	if (!info->object_num) {
		dev_err(data->dev, "Device reports an empty object table\n");
		error = -EIO;
		goto err_free_mem;
	}

	dev_info(data->dev,
		 "Family: 0x%02X Variant: 0x%02X Firmware V%u.%u.%02X Objects: %u\n",
		 info->family_id, info->variant_id, info->version >> 4,
		 info->version & 0xf, info->build, info->object_num);

	if (info->family_id != FCD52_FAMILY_ID)
		dev_warn(data->dev,
			 "Unexpected family ID 0x%02X, expected 0x%02X\n",
			 info->family_id, FCD52_FAMILY_ID);

	/* Grow the buffer to hold the object table and its checksum */
	size = FCD52_ID_INFO_SIZE +
	       info->object_num * FCD52_OBJECT_ELEM_SIZE +
	       FCD52_INFO_CRC_SIZE;

	buf = krealloc(id_buf, size, GFP_KERNEL);
	if (!buf) {
		error = -ENOMEM;
		goto err_free_mem;
	}
	id_buf = buf;
	info = id_buf;

	/*
	 * The device can NACK a read that follows the information block soon
	 * after a firmware update, while it finishes start-up. Retry for a bounded
	 * time, and log how many attempts it took.
	 */
	for (retry = 0; retry < FCD52_OBJ_READ_TRIES; retry++) {
		error = fcd52_read_reg(data, FCD52_OBJECT_START,
				       size - FCD52_OBJECT_START,
				       id_buf + FCD52_OBJECT_START);
		if (!error)
			break;
		msleep(FCD52_OBJ_READ_RETRY_MS);
	}
	if (error) {
		dev_err(data->dev, "Failed to read object table after %d tries: %d\n",
			retry, error);
		goto err_free_mem;
	}
	if (retry)
		dev_info(data->dev, "object table read after %d retries\n", retry);

	data->info_crc = get_unaligned_le24(id_buf + size -
					    FCD52_INFO_CRC_SIZE);
	calculated_crc = fcd52_calculate_crc(id_buf,
					     size - FCD52_INFO_CRC_SIZE);

	/*
	 * A mismatch means either the information block was corrupted on the
	 * bus or the device does not speak the Object-Based Protocol.
	 */
	if (!data->info_crc || data->info_crc != calculated_crc) {
		dev_err(data->dev,
			"Info Block CRC error calculated=0x%06X read=0x%06X\n",
			calculated_crc, data->info_crc);
		error = -EIO;
		goto err_free_mem;
	}

	/* Room for the main table plus the extended objects held in T254 */
	data->object_table = kcalloc(info->object_num + FCD52_EXT_OBJECT_NUM,
				     sizeof(*data->object_table), GFP_KERNEL);
	if (!data->object_table) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	data->raw_info_block = id_buf;
	data->info = info;

	error = fcd52_parse_object_table(data, id_buf + FCD52_OBJECT_START,
					 &reportid);
	if (error)
		goto err_free_table;

	error = fcd52_parse_ext_object_table(data, &reportid);
	if (error)
		goto err_free_table;

	/*
	 * The T5 size comes from the object table: size - 1 is the message
	 * length without the checksum byte, so the message is sized from the
	 * device, not a fixed bound.
	 */
	if (data->cached.t5.present) {
		/* A T6 message needs the report ID, STATUS, checksum and DATA */
		if (data->cached.t5.size < FCD52_T6_MSG_MIN_SIZE) {
			dev_err(data->dev, "T5 is %u bytes, too short for T6\n",
				data->cached.t5.size);
			error = -EINVAL;
			goto err_free_table;
		}

		data->t5_msg_len = data->cached.t5.size - 1;
		data->t5_msg = kzalloc(data->t5_msg_len, GFP_KERNEL);
		if (!data->t5_msg) {
			error = -ENOMEM;
			goto err_free_table;
		}
	}

	dev_info(data->dev,
		 "Matrix: %ux%u Objects: %u Memory map: %u bytes Report IDs: %u\n",
		 info->matrix_xsize, info->matrix_ysize, data->object_num,
		 data->mem_size, data->max_reportid);

	return 0;

err_free_table:
	fcd52_free_object_table(data);
	return error;

err_free_mem:
	kfree(id_buf);
	return error;
}

/*
 * Fill in the frame geometry implied by a matrix size and data configuration.
 * Split out from fcd52_read_frame_layout() so that the arithmetic can be
 * exercised without a device present.
 */
void fcd52_frame_layout_from_matrix(struct fcd52_frame_layout *layout,
				    u8 xsize, u8 ysize, u16 data_type,
				    bool crc_enabled)
{
	layout->xsize = xsize;
	layout->ysize = ysize;
	layout->nodes = (unsigned int)xsize * ysize;
	layout->data_type = data_type;
	layout->crc_enabled = crc_enabled;

	/* A Frame ID byte, then the optional data type and length header */
	layout->data_offset = FCD52_FRAME_ID_SIZE;
	if (crc_enabled)
		layout->data_offset += FCD52_FRAME_HDR_SIZE;

	layout->frame_size = layout->data_offset + layout->nodes * 2 +
			     FCD52_FRAME_ID_SIZE;
	if (crc_enabled)
		layout->frame_size += FCD52_FRAME_CRC_SIZE;
}
EXPORT_SYMBOL_NS_GPL(fcd52_frame_layout_from_matrix, IIO_FCD52);

/*
 * Read the Fast Capacitive Digitizer T700 configuration and work out the
 * resulting frame geometry. The frame data interface carries no addressing
 * of its own, so the layout has to come from the object over I2C.
 */
int fcd52_read_frame_layout(struct fcd52_data *data,
			    struct fcd52_frame_layout *layout)
{
	struct fcd52_object *object;
	bool crc_enabled;
	u16 data_type;
	u8 cfg[6];
	u8 xsize, ysize;
	int error;

	object = fcd52_get_object(data, FCD52_SPT_FASTCAPDIGITIZER_T700);
	if (!object) {
		dev_err(data->dev, "No T700 object, cannot decode frame data\n");
		return -ENODEV;
	}

	if (object->size < sizeof(cfg)) {
		dev_err(data->dev, "T700 is %u bytes, expected at least %zu\n",
			object->size, sizeof(cfg));
		return -EINVAL;
	}

	error = fcd52_read_reg(data, object->start_address, sizeof(cfg), cfg);
	if (error)
		return error;

	if (!(cfg[FCD52_T700_CTRL] & FCD52_T700_CTRL_ENABLE)) {
		dev_err(data->dev, "T700 is disabled, no frame data is output\n");
		return -ENODATA;
	}

	/* A zero size means one line (Protocol Guide "XSIZE Field") */
	xsize = cfg[FCD52_T700_XSIZE] ?: 1;
	ysize = cfg[FCD52_T700_YSIZE] ?: 1;

	if (xsize > FCD52_MAX_XSIZE || ysize > FCD52_MAX_YSIZE) {
		dev_err(data->dev, "T700 matrix %ux%u out of range\n",
			xsize, ysize);
		return -ERANGE;
	}

	crc_enabled = cfg[FCD52_T700_DATACFG2] & FCD52_T700_CRCEN;

	/*
	 * Exactly one data type may be selected. The codes pair each type
	 * with its CRC variant (Data Sheet Table 5-1).
	 */
	switch (cfg[FCD52_T700_DATACFG] & (FCD52_T700_DELTASEN |
					   FCD52_T700_SIGNALSEN |
					   FCD52_T700_REFSEN)) {
	case FCD52_T700_DELTASEN:
		data_type = crc_enabled ? FCD52_DATA_DELTAS_CRC :
					  FCD52_DATA_DELTAS;
		break;
	case FCD52_T700_SIGNALSEN:
		data_type = crc_enabled ? FCD52_DATA_SIGNALS_CRC :
					  FCD52_DATA_SIGNALS;
		break;
	case FCD52_T700_REFSEN:
		data_type = crc_enabled ? FCD52_DATA_REFERENCES_CRC :
					  FCD52_DATA_REFERENCES;
		break;
	default:
		dev_err(data->dev,
			"T700 DATACFG 0x%02X does not select one data type\n",
			cfg[FCD52_T700_DATACFG]);
		return -EINVAL;
	}

	fcd52_frame_layout_from_matrix(layout, xsize, ysize, data_type,
				       crc_enabled);

	if (layout->nodes * 2 > FCD52_FRAME_MAX_DATA) {
		dev_err(data->dev, "%u nodes exceed the %u byte frame limit\n",
			layout->nodes, FCD52_FRAME_MAX_DATA);
		return -ERANGE;
	}

	dev_info(data->dev,
		 "Frame data: %ux%u nodes, type 0x%04X, %s, %u bytes/frame\n",
		 xsize, ysize, data_type,
		 crc_enabled ? "CRC" : "no CRC", layout->frame_size);

	return 0;
}
EXPORT_SYMBOL_NS_GPL(fcd52_read_frame_layout, IIO_FCD52);

/*
 * Check one captured frame. The device brackets each frame with a matching
 * Frame ID byte; a mismatch means the frame was truncated or the capture is
 * out of sync (Protocol Guide Section 3.3.2). With CRCs enabled the embedded
 * header and 32-bit CRC are verified too.
 */
int fcd52_frame_validate(const struct fcd52_frame_layout *layout,
			 const u8 *frame)
{
	u32 crc, expected;
	u16 type, len;

	if (frame[0] != frame[layout->frame_size - FCD52_FRAME_ID_SIZE])
		return -EBADMSG;

	if (!layout->crc_enabled)
		return 0;

	type = get_unaligned_le16(&frame[FCD52_FRAME_ID_SIZE]);
	if (type != layout->data_type)
		return -EPROTO;

	len = get_unaligned_le16(&frame[FCD52_FRAME_ID_SIZE + 2]);
	if (len != layout->nodes * 2)
		return -EPROTO;

	/* The CRC covers the samples only */
	crc = crc32_le(FCD52_CRC32_SEED, &frame[layout->data_offset],
		       layout->nodes * 2);
	expected = get_unaligned_le32(&frame[layout->data_offset +
					     layout->nodes * 2]);
	if (crc != expected)
		return -EBADMSG;

	return 0;
}
EXPORT_SYMBOL_NS_GPL(fcd52_frame_validate, IIO_FCD52);

static int fcd52_objects_show(struct seq_file *s, void *p)
{
	struct fcd52_data *data = s->private;
	unsigned int i;

	for (i = 0; i < data->object_num; i++) {
		const struct fcd52_object *object = &data->object_table[i];

		seq_printf(s,
			   "T%-4u start=0x%04x size=%-4u instances=%-3u reportids=%u-%u\n",
			   object->type, object->start_address, object->size,
			   object->instances, object->min_reportid,
			   object->max_reportid);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(fcd52_objects);

/*
 * Provides the standard IIO direct_reg_access interface over the device
 * memory map, so that individual object fields can be inspected and
 * configured before the frame data interface exists.
 */
static int fcd52_debugfs_reg_access(struct iio_dev *indio_dev,
				    unsigned int reg, unsigned int writeval,
				    unsigned int *readval)
{
	struct fcd52_data *data = iio_priv(indio_dev);
	u8 val;
	int error;

	if (reg >= data->mem_size)
		return -EINVAL;

	if (!readval) {
		val = writeval;
		return fcd52_write_reg(data, reg, 1, &val);
	}

	error = fcd52_read_reg(data, reg, 1, &val);
	if (error)
		return error;

	*readval = val;

	return 0;
}

/*
 * Pulse the reset line if available. Device signals completion on the CHG
 * line, else fall back to worst-case delay.
 */
static int fcd52_hw_reset(struct fcd52_data *data)
{
	struct gpio_desc *reset_gpio;

	reset_gpio = devm_gpiod_get_optional(data->dev, "reset",
					     GPIOD_OUT_HIGH);
	if (IS_ERR(reset_gpio))
		return dev_err_probe(data->dev, PTR_ERR(reset_gpio),
				     "Failed to get reset GPIO\n");

	if (!reset_gpio) {
		dev_dbg(data->dev, "No reset GPIO, assuming power-on reset\n");
		msleep(FCD52_POWERUP_DELAY_MS);
		return 0;
	}

	/* RESET must be held for at least 90ns */
	fsleep(FCD52_RESET_GPIO_TIME_US);
	gpiod_set_value_cansleep(reset_gpio, 0);
	msleep(FCD52_RESET_DELAY_MS);

	return 0;
}

/*
 * Decode a Command Processor T6 message (Protocol Guide Table 3-7). Each
 * STATUS bit is logged by name. The configuration checksum is logged on the
 * message that reports a reset, since that is when the device has just
 * finished loading its configuration. Ongoing bits are logged only when they
 * change, so a repeated status is not reported twice.
 */
static void fcd52_proc_t6_message(struct fcd52_data *data, const u8 *msg)
{
	u8 status = msg[FCD52_T6_MSG_STATUS];
	u32 crc;

	crc = get_unaligned_le24(&msg[FCD52_T6_MSG_CHECKSUM]);
	data->config_crc = crc;
	data->config_crc_valid = true;

	if (status & FCD52_T6_STATUS_RESET)
		dev_info(data->dev,
			 "T6 RESET: device reset, config checksum 0x%06X\n",
			 crc);

	if (status != data->t6_status) {
		dev_info(data->dev,
			 "T6 status 0x%02X:%s%s%s%s%s%s%s\n", status,
			 status & FCD52_T6_STATUS_RESET ? " RESET" : "",
			 status & FCD52_T6_STATUS_OFL ? " OFL" : "",
			 status & FCD52_T6_STATUS_SIGERR ? " SIGERR" : "",
			 status & FCD52_T6_STATUS_CAL ? " CAL" : "",
			 status & FCD52_T6_STATUS_CFGERR ? " CFGERR" : "",
			 status & FCD52_T6_STATUS_COMSERR ? " COMSERR" : "",
			 status & FCD52_T6_STATUS_FROZEN ? " FROZEN" : "");
	}

	if (status & FCD52_T6_STATUS_SIGERR)
		dev_info(data->dev, "T6 SIGERR data 0x%02X 0x%02X 0x%02X\n",
			 msg[5], msg[6], msg[7]);

	/*
	 * A reset is reported as a sequence: RESET, then CAL, then a message
	 * with no bits set once calibration has finished. The sequence is
	 * complete only at that last message, so the caller waits for it.
	 */
	if (status & FCD52_T6_STATUS_RESET)
		data->reset_pending = true;

	if (data->reset_pending && !status) {
		data->reset_pending = false;
		complete(&data->reset_done);
	}

	data->t6_status = status;
}

/*
 * Print one Message Processor T5 message. Only the report ID and data are in
 * the buffer; checksum mode is off, so there is no trailing CRC byte.
 */
static void fcd52_dump_message(struct fcd52_data *data, const u8 *msg,
			       unsigned int len)
{
	dev_info(data->dev, "T5 msg rid=%u: %*ph\n", msg[0], len, msg);
}

/* Messages the T5 feed holds before it starts dropping new ones */
#define FCD52_DBG_RING_SLOTS	32

/*
 * Queue one T5 message for the bridge. When the ring is full the new message is
 * dropped and counted, so messages already queued keep their order. Called
 * from the message path with the ring lock taken.
 */
static void fcd52_dbg_push(struct fcd52_data *data, const u8 *msg, unsigned int len)
{
	unsigned int tail;

	if (!data->dbg_ring)
		return;

	mutex_lock(&data->dbg_lock);

	if (data->dbg_count == data->dbg_ring_slots) {
		data->dbg_lost++;
		mutex_unlock(&data->dbg_lock);
		return;
	}

	tail = (data->dbg_head + data->dbg_count) % data->dbg_ring_slots;
	memcpy(data->dbg_ring + tail * len, msg, len);
	data->dbg_count++;

	mutex_unlock(&data->dbg_lock);

	sysfs_notify(&data->dev->kobj, NULL, "debug_notify");
}

/*
 * Enable or disable the T5 feed. Enabling allocates the ring, sized from the
 * T5 object; disabling frees it and discards anything still queued.
 */
static int fcd52_dbg_set_enabled(struct fcd52_data *data, bool enable)
{
	unsigned int len = data->t5_msg_len;

	mutex_lock(&data->dbg_lock);

	if (enable && !data->dbg_ring) {
		if (!len) {
			mutex_unlock(&data->dbg_lock);
			return -ENODEV;
		}
		data->dbg_ring = kcalloc(FCD52_DBG_RING_SLOTS, len, GFP_KERNEL);
		if (!data->dbg_ring) {
			mutex_unlock(&data->dbg_lock);
			return -ENOMEM;
		}
		data->dbg_ring_slots = FCD52_DBG_RING_SLOTS;
	} else if (!enable) {
		kfree(data->dbg_ring);
		data->dbg_ring = NULL;
		data->dbg_ring_slots = 0;
	}

	data->dbg_head = 0;
	data->dbg_count = 0;
	data->dbg_lost = 0;
	data->debug_v2 = enable;

	mutex_unlock(&data->dbg_lock);
	return 0;
}

/*
 * Copy queued messages to @buf, whole messages only, and remove them from the
 * ring. Returns the number of bytes copied.
 */
static size_t fcd52_dbg_pop(struct fcd52_data *data, u8 *buf, size_t size)
{
	unsigned int len = data->t5_msg_len;
	size_t out = 0;

	mutex_lock(&data->dbg_lock);

	while (data->dbg_count && size - out >= len) {
		memcpy(buf + out, data->dbg_ring + data->dbg_head * len, len);
		data->dbg_head = (data->dbg_head + 1) % data->dbg_ring_slots;
		data->dbg_count--;
		out += len;
	}

	mutex_unlock(&data->dbg_lock);
	return out;
}

/*
 * Read one Message Processor T5 message and act on it. A T6 message is decoded
 * for its checksum and status; any other message is printed. Returns 1 for a
 * valid message, 0 when the device reports no message (report ID 0xFF), or a
 * negative errno.
 */
static int fcd52_read_and_process_message(struct fcd52_data *data)
{
	const struct fcd52_obj_ref *t5 = &data->cached.t5;
	const struct fcd52_obj_ref *t6 = &data->cached.t6;
	u8 *msg = data->t5_msg;
	int error;

	if (!msg)
		return -ENODEV;

	error = fcd52_bus_read(data, t5->start_address, data->t5_msg_len, msg);
	if (error)
		return error;

	if (msg[0] == FCD52_RPTID_NOMSG)
		return 0;

	if (t6->present && msg[0] == t6->min_reportid)
		fcd52_proc_t6_message(data, msg);
	else
		fcd52_dump_message(data, msg, data->t5_msg_len);

	if (data->debug_v2)
		fcd52_dbg_push(data, msg, data->t5_msg_len);

	return 1;
}

/*
 * Drain the T5 queue. The CHG line stays low until the queue is empty, so this
 * reads messages until the device reports none. Each message is decoded, not
 * discarded, so the power-up RESET and checksum messages are kept. The bound
 * is the number of report IDs the device allocated: a queue cannot hold more
 * messages than that, so a count beyond it means the device is misbehaving.
 */
static int fcd52_process_messages_until_empty(struct fcd52_data *data)
{
	unsigned int max = data->max_reportid ? data->max_reportid : 1;
	unsigned int n;
	int error;

	for (n = 0; n < max; n++) {
		error = fcd52_read_and_process_message(data);
		if (error < 0)
			return error;
		if (!error)
			return 0;
	}

	dev_warn(data->dev, "T5 queue did not empty after %u messages\n", max);
	return -EBUSY;
}

/* Device readiness after CHG before a status read (Protocol Guide D.2.3) */
#define FCD52_BL_READY_US		1000
#define FCD52_BL_RETRY_US		5000
#define FCD52_BL_READ_TRIES		200

/*
 * Read the bootloader status from the CHG handler. A NACK is retried here, while
 * the handler still holds CHG low: returning without a successful read would
 * leave CHG low indefinitely (MXTAN2016 2.2). The retries are bounded so a
 * device that never answers still fails the update.
 */
static int fcd52_bl_read_status(struct fcd52_data *data, u8 *val)
{
	int retry;
	int error = -EIO;

	usleep_range(FCD52_BL_READY_US, FCD52_BL_READY_US * 2);

	for (retry = 0; retry < FCD52_BL_READ_TRIES; retry++) {
		/* Nothing waiting: stop, do not keep addressing the bootloader */
		if (!data->bl_waiting)
			return -ECANCELED;

		error = data->bops->bl_read(data, val, sizeof(val));
		if (!error)
			return 0;

		dev_info(data->dev, "bl read NACK %d (error %d), retry\n",
			 retry + 1, error);
		usleep_range(FCD52_BL_RETRY_US, FCD52_BL_RETRY_US * 2);
	}

	return error;
}

/* Read the status in the CHG handler and queue it for the waiter, in order */
static void queue_bl_status(struct fcd52_data *data)
{
	struct fcd52_bl_status *slot;
	unsigned long flags;
	unsigned int next;

	spin_lock_irqsave(&data->bl_lock, flags);
	next = (data->bl_head + 1) % FCD52_BL_FIFO;
	if (next == data->bl_tail) {
		/* FIFO full: the waiter has fallen behind; drop the oldest */
		data->bl_tail = (data->bl_tail + 1) % FCD52_BL_FIFO;
	}
	slot = &data->bl_fifo[data->bl_head];
	slot->error = fcd52_bl_read_status(data, slot->val);
	data->bl_head = next;
	spin_unlock_irqrestore(&data->bl_lock, flags);

	complete(&data->bl_done);
}

/*
 * Handle the CHG line. Reads the pending message count from T44, then reads
 * all of the messages in one continuous transfer, as the device requires
 * (Data Sheet Section 8.6.1). A report ID of 0xFF marks the end.
 */
static irqreturn_t fcd52_irq_thread(int irq, void *p)
{
	struct iio_dev *indio_dev = p;
	struct fcd52_data *data = iio_priv(indio_dev);
	/*
	 * In bootloader mode T5 is not read. Signal the update code, which reads
	 * the bootloader status itself once the line changes.
	 */
	if (data->in_bootloader) {
		/*
		 * The bootloader holds CHG low until its status is read
		 * (MXTAN2016 2.2). Read it here, after the 1 ms the device needs
		 * (Protocol Guide D.2.3): reading releases CHG, so the level
		 * interrupt does not re-fire while the update waits.
		 */
		queue_bl_status(data);
		/*
		 * Complete on every handler run. The device holds CHG low until
		 * each status is read, so a repeated WAITING_FRAME_DATA is a real
		 * event that the waiting code must see; suppressing it stalls the
		 * next frame.
		 */
		complete(&data->bl_done);
		return IRQ_HANDLED;
	}

	/*
	 * Without a T5 buffer there is nothing to read the message into, so the
	 * line is left alone. A cache that is only momentarily empty (during the
	 * info block re-read) must not skip the drain, or CHG stays low.
	 */
	if (!data->t5_msg)
		return IRQ_HANDLED;

	/*
	 * Empty the queue on every call. CHG is held low until it is empty, and
	 * with an edge trigger no further interrupt arrives for messages that
	 * were not read here, so draining is required, not an optimisation.
	 */
	if (fcd52_process_messages_until_empty(data))
		dev_dbg(data->dev, "T5 queue not drained on this pass\n");

	return IRQ_HANDLED;
}

/*
 * Request the CHG interrupt. It is requested disabled, because the handler
 * depends on the object table and must not run before it exists. The
 * caller enables it once probing has finished.
 */
static int fcd52_request_irq(struct fcd52_data *data, struct iio_dev *indio_dev)
{
	int error;

	if (!data->irq)
		return 0;

	error = devm_request_threaded_irq(data->dev, data->irq, NULL,
					  fcd52_irq_thread,
					  IRQF_ONESHOT | IRQF_NO_AUTOEN,
					  dev_name(data->dev), indio_dev);
	if (error)
		return dev_err_probe(data->dev, error,
				     "Failed to request CHG interrupt\n");

	return 0;
}

/* Command Processor T6 RESET byte (Protocol Guide Table 3-3) */
#define FCD52_T6_RESET_OFFSET		0
#define FCD52_T6_RESET_VALUE		0x01
#define FCD52_RESET_TIMEOUT_MS		3000

/*
 * Issue a software reset and wait for the whole reset sequence to finish.
 * Caller transfers are refused while it runs. The CHG handler is not
 * gated, so it can read the RESET, CAL and final messages that complete
 * the wait.
 */
int fcd52_soft_reset(struct fcd52_data *data)
{
	const struct fcd52_obj_ref *t6 = &data->cached.t6;
	unsigned long left;
	int error;
	u8 value = FCD52_T6_RESET_VALUE;

	if (!t6->present)
		return -ENODEV;

	reinit_completion(&data->reset_done);
	data->reset_pending = false;
	WRITE_ONCE(data->reset_busy, true);

	error = fcd52_bus_write(data, t6->start_address +
				FCD52_T6_RESET_OFFSET, 1, &value);
	if (error)
		goto out_clear;

	left = wait_for_completion_timeout(&data->reset_done,
					   msecs_to_jiffies(FCD52_RESET_TIMEOUT_MS));
	if (!left) {
		dev_err(data->dev, "Timed out waiting for reset sequence\n");
		error = -ETIMEDOUT;
	}

out_clear:
	WRITE_ONCE(data->reset_busy, false);
	return error;
}
EXPORT_SYMBOL_NS_GPL(fcd52_soft_reset, IIO_FCD52);

/* Command Processor T6 REPORTALL byte (Protocol Guide Table 3-3) */
#define FCD52_T6_REPORTALL_OFFSET	3
#define FCD52_T6_REPORTALL_VALUE	1
#define FCD52_REPORTALL_POLLS		20
#define FCD52_REPORTALL_DELAY_MS	20

/*
 * Ask every object that reports messages to send its current status, then read
 * the queue. The device clears the REPORTALL byte once the request is done, and
 * the T6 message in the queue carries the configuration checksum, so this gives
 * a real config_crc at probe even when no reset has happened.
 */
int fcd52_reportall(struct fcd52_data *data)
{
	const struct fcd52_obj_ref *t6 = &data->cached.t6;
	u8 value = FCD52_T6_REPORTALL_VALUE;
	u8 cleared;
	int tries;
	int error;

	if (!t6->present)
		return -ENODEV;

	error = fcd52_bus_write(data, t6->start_address +
				FCD52_T6_REPORTALL_OFFSET, 1, &value);
	if (error)
		return error;

	for (tries = 0; tries < FCD52_REPORTALL_POLLS; tries++) {
		msleep(FCD52_REPORTALL_DELAY_MS);
		error = fcd52_bus_read(data, t6->start_address +
				       FCD52_T6_REPORTALL_OFFSET, 1, &cleared);
		if (error)
			return error;
		if (!cleared)
			break;
	}

	if (cleared) {
		dev_err(data->dev, "REPORTALL not acknowledged by device\n");
		return -ETIMEDOUT;
	}

	/* The reports are now queued on T5; decode them, which sets config_crc */
	return fcd52_process_messages_until_empty(data);
}
EXPORT_SYMBOL_NS_GPL(fcd52_reportall, IIO_FCD52);

/* Command Processor T6 BACKUPNV byte (Protocol Guide Table 3-4) */
#define FCD52_T6_BACKUPNV_OFFSET	1

/* BACKUPNV commands (Protocol Guide Table 3-4) */
#define FCD52_BACKUP_UNFREEZE		0x11	/* resume operations */
#define FCD52_BACKUP_FREEZE		0x22	/* suspend operations */
#define FCD52_BACKUP_FREEZE_ON_ERROR	0x23	/* suspend on signal error */
#define FCD52_BACKUP_STOP_T70		0x33	/* stop T70, restore from NVM */
#define FCD52_BACKUP_RESTORE		0x44	/* restore from NVM, T70 runs */
#define FCD52_BACKUP_SAVE		0x55	/* save settings to NVM */

/*
 * Empty the message queue before a configuration write. The CHG interrupt is
 * held off while this runs, because the handler reads the same queue.
 */
static int fcd52_clear_messages(struct fcd52_data *data)
{
	const struct fcd52_obj_ref *t5 = &data->cached.t5;
	const struct fcd52_obj_ref *t44 = &data->cached.t44;
	int error;

	if (!t5->present || !t44->present)
		return 0;

	if (data->irq)
		disable_irq(data->irq);

	error = fcd52_process_messages_until_empty(data);

	if (data->irq)
		enable_irq(data->irq);

	return error;
}

/*
 * Write one configuration record to the device. Volatile objects are skipped.
 * A record larger than the device's object is refused rather than truncated,
 * since a partial write would corrupt the neighbouring object.
 */
static int fcd52_write_record(struct fcd52_data *data,
			      const struct fcd52_cfg_record *rec)
{
	struct fcd52_object *obj;
	u16 addr;

	if (fcd52_cfg_record_is_volatile(rec->type))
		return 0;

	obj = fcd52_get_object(data, rec->type);
	if (!obj) {
		dev_err(data->dev, "T%u in file is not on the device\n",
			rec->type);
		return -ENOENT;
	}

	if (rec->size > obj->size) {
		dev_err(data->dev, "T%u record %u bytes, object is %u\n",
			rec->type, rec->size, obj->size);
		return -EINVAL;
	}

	addr = obj->start_address + rec->instance * obj->size;

	return fcd52_bus_write(data, addr, rec->size, rec->data);
}

/*
 * Write the configuration from a parsed file. The file header has already
 * been checked against the device by fcd52_cfg_decide().
 */
static int fcd52_apply_config(struct fcd52_data *data,
			      const struct fcd52_cfg *cfg)
{
	unsigned int i;
	int error;

	for (i = 0; i < cfg->num_records; i++) {
		error = fcd52_write_record(data, &cfg->records[i]);
		if (error)
			return error;
	}

	return 0;
}

/*
 * Save the configuration to NVM. The device answers with one T6 message that
 * carries the new configuration checksum (Protocol Guide Table 3-4).
 */
/*
 * Issue a BACKUPNV command. The device clears the byte once the command has
 * been processed, and reports the result in a T6 message, which the caller
 * sees through the T6 decoder. Only the commands in Table 3-4 are accepted.
 */
int fcd52_backup_command(struct fcd52_data *data, u8 command)
{
	const struct fcd52_obj_ref *t6 = &data->cached.t6;

	switch (command) {
	case FCD52_BACKUP_UNFREEZE:
	case FCD52_BACKUP_FREEZE:
	case FCD52_BACKUP_FREEZE_ON_ERROR:
	case FCD52_BACKUP_STOP_T70:
	case FCD52_BACKUP_RESTORE:
	case FCD52_BACKUP_SAVE:
		break;
	default:
		return -EINVAL;
	}

	if (!t6->present)
		return -ENODEV;

	return fcd52_bus_write(data, t6->start_address +
			       FCD52_T6_BACKUPNV_OFFSET, 1, &command);
}
EXPORT_SYMBOL_NS_GPL(fcd52_backup_command, IIO_FCD52);

/*
 * Update the configuration from an OBP_RAW file (V1 or V4), following the
 * sequence of the maXTouch reference:
 *   1. check the file's info block and CRC against the device, and skip the
 *      write if the configuration is already loaded;
 *   2. clear pending messages;
 *   3. write each record;
 *   4. back up to NVM, which produces a T6 message with the new checksum;
 *   5. software reset, so the new configuration takes effect.
 *
 * The file is parsed and decoded by the caller. The caller must hold the
 * reset gate clear; this function takes it for the reset step.
 */
int fcd52_update_config(struct fcd52_data *data, const struct fcd52_cfg *cfg)
{
	const struct fcd52_obj_ref *t6 = &data->cached.t6;
	u32 dev_cfg_crc = data->config_crc;
	int decision;
	int error;

	if (!t6->present)
		return -ENODEV;

	/*
	 * No T6 message has supplied the checksum yet. Zero is a valid checksum
	 * (no configuration stored), so validity is tracked separately.
	 */
	if (!data->config_crc_valid) {
		dev_info(data->dev, "config CRC not yet known, requesting REPORTALL\n");
		error = fcd52_reportall(data);
		if (error)
			return error;
		dev_cfg_crc = data->config_crc;
	}

	/* struct fcd52_info is seven single-byte fields, in file order */
	decision = fcd52_cfg_decide(cfg, (const u8 *)data->info,
				    data->info_crc, dev_cfg_crc);

	dev_info(data->dev, "config CRC: device 0x%06X, file 0x%06X\n",
		 dev_cfg_crc, cfg->config_crc);

	if (decision < 0) {
		dev_err(data->dev, "Configuration file does not match device\n");
		return decision;
	}

	if (decision == FCD52_CFG_SKIP) {
		dev_info(data->dev, "config CRC matched, skipping update\n");
		return 0;
	}

	dev_info(data->dev, "config CRC differs, programming configuration\n");

	dev_info(data->dev, "config stage: writing configuration\n");
	error = fcd52_clear_messages(data);
	if (error)
		return error;

	error = fcd52_apply_config(data, cfg);
	if (error) {
		dev_err(data->dev, "Configuration write failed: %d\n", error);
		return error;
	}

	dev_info(data->dev, "config stage: backup to NVM\n");
	error = fcd52_backup_command(data, FCD52_BACKUP_SAVE);
	if (error)
		return error;

	dev_info(data->dev, "config stage: device reset\n");
	error = fcd52_soft_reset(data);
	if (error)
		return error;

	dev_info(data->dev, "config stage: reset complete\n");
	return 0;
}
EXPORT_SYMBOL_NS_GPL(fcd52_update_config, IIO_FCD52);

/* Default configuration name, used unless the device tree overrides it */
#define FCD52_CFG_NAME_DEFAULT	"fcd52.cfg"
#define FCD52_CFG_PATH_MAX	128

/*
 * Load a configuration from the firmware directory and apply it. An empty
 * write or "default" loads the configured name (fcd52.cfg unless the device
 * tree sets microchip,cfg-name). Any other string is used as the name.
 */
static ssize_t update_cfg_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct fcd52_data *data = iio_priv(indio_dev);
	const struct firmware *fw = NULL;
	struct fcd52_cfg cfg;
	const char *name;
	char path[FCD52_CFG_PATH_MAX];
	size_t len;
	int error;

	/*
	 * Anything other than the default is passed to request_firmware()
	 * unchanged, so a sub-path under the firmware roots works. The driver
	 * does not decide where the file lives; that is the caller's choice.
	 */
	if (!*buf || sysfs_streq(buf, "default")) {
		name = data->cfg_name;
	} else {
		len = min(count, sizeof(path) - 1);
		memcpy(path, buf, len);
		path[len] = '\0';
		/* echo adds a trailing newline, which is not part of the name */
		if (len && path[len - 1] == '\n')
			path[len - 1] = '\0';
		name = path;
	}

	error = request_firmware(&fw, name, dev);
	if (error) {
		dev_err(dev, "Failed to load %s: %d\n", name, error);
		return error;
	}

	error = fcd52_cfg_parse(dev, (const char *)fw->data, fw->size, &cfg);
	if (error) {
		dev_err(dev, "Invalid configuration %s: %d\n", name, error);
		goto out_release;
	}

	cfg.records = kcalloc(cfg.num_records, sizeof(*cfg.records),
			      GFP_KERNEL);
	cfg.blob = kmalloc(cfg.total_bytes ?: 1, GFP_KERNEL);
	if (!cfg.records || !cfg.blob) {
		error = -ENOMEM;
		goto out_free;
	}

	error = fcd52_cfg_fill((const char *)fw->data, fw->size, &cfg);
	if (error) {
		dev_err(dev, "Failed to decode %s: %d\n", name, error);
		goto out_free;
	}

	error = fcd52_update_config(data, &cfg);

out_free:
	kfree(cfg.records);
	kfree(cfg.blob);
out_release:
	release_firmware(fw);

	return error ? error : count;
}

/* Driver version reported through sysfs */
#define FCD52_DRIVER_VERSION	"6.12-20261117"

/* Firmware version as Major.Minor.Build */
static ssize_t fw_version_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct fcd52_data *data = iio_priv(indio_dev);

	if (!data->info)
		return -ENODATA;

	return sysfs_emit(buf, "%u.%u.%02X\n", data->info->version >> 4,
			  data->info->version & 0xf, data->info->build);
}
static DEVICE_ATTR_RO(fw_version);

/* Hardware version as FamilyID.VariantID */
static ssize_t hw_version_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct fcd52_data *data = iio_priv(indio_dev);

	if (!data->info)
		return -ENODATA;

	return sysfs_emit(buf, "%u.%u\n", data->info->family_id,
			  data->info->variant_id);
}
static DEVICE_ATTR_RO(hw_version);

/* Configuration checksum from the most recent T6 message */
static ssize_t config_crc_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct fcd52_data *data = iio_priv(indio_dev);

	return sysfs_emit(buf, "0x%06X\n", data->config_crc);
}
static DEVICE_ATTR_RO(config_crc);

/* Driver version string */
static ssize_t drvr_version_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%s\n", FCD52_DRIVER_VERSION);
}
static DEVICE_ATTR_RO(drvr_version);

/* T5 message feed: enable switch (0/1), off by default */
static ssize_t debug_v2_enable_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct fcd52_data *data = iio_priv(indio_dev);

	return sysfs_emit(buf, "%u\n", data->debug_v2 ? 1 : 0);
}

static ssize_t debug_v2_enable_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct fcd52_data *data = iio_priv(indio_dev);
	bool enable;
	int error;

	error = kstrtobool(buf, &enable);
	if (error)
		return error;

	error = fcd52_dbg_set_enabled(data, enable);
	if (error)
		return error;

	return count;
}
static DEVICE_ATTR_RW(debug_v2_enable);

/*
 * Queued T5 messages, whole messages only; reading removes them. The size is
 * the full ring, set at probe from the T5 object, because the bridge sizes its
 * read buffer from the file size.
 */
/*
 * Raw memory access to the device map, used by the bridge for object reads
 * (including T37 DATA). Reads and writes are range-checked against the map
 * and clamped to one block, as the maXTouch driver does.
 */
#define FCD52_MEM_MAX_BLOCK		255

static ssize_t mem_access_read(struct file *filp, struct kobject *kobj,
			       struct bin_attribute *attr, char *buf,
			       loff_t off, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct fcd52_data *data = iio_priv(indio_dev);
	int error;

	if (off >= data->mem_size)
		return -EIO;

	count = min_t(size_t, count, data->mem_size - off);
	count = min_t(size_t, count, FCD52_MEM_MAX_BLOCK);

	error = fcd52_read_reg(data, off, count, buf);
	if (error)
		return error;

	return count;
}

/*
 * True if [start, start + len) overlaps any object the device changes at run
 * time. Checked against the object table, so the guard follows the device.
 */
static bool fcd52_range_is_volatile(struct fcd52_data *data, u16 start,
				    size_t len)
{
	unsigned int i;

	for (i = 0; i < data->object_num; i++) {
		const struct fcd52_object *obj = &data->object_table[i];
		u32 obj_end = obj->start_address +
			      (u32)obj->size * obj->instances;

		if (!fcd52_cfg_record_is_volatile(obj->type))
			continue;

		if (start < obj_end && start + len > obj->start_address)
			return true;
	}

	return false;
}

static ssize_t mem_access_write(struct file *filp, struct kobject *kobj,
				struct bin_attribute *attr, char *buf,
				loff_t off, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct fcd52_data *data = iio_priv(indio_dev);
	int error;

	if (off >= data->mem_size)
		return -EIO;

	count = min_t(size_t, count, data->mem_size - off);
	count = min_t(size_t, count, FCD52_MEM_MAX_BLOCK);

	/* Writes into a volatile object would corrupt message or command state */
	if (fcd52_range_is_volatile(data, off, count))
		return -EPERM;

	error = fcd52_write_reg(data, off, count, buf);
	if (error)
		return error;

	return count;
}

static struct bin_attribute fcd52_mem_access_attr;

static struct bin_attribute fcd52_debug_msg_attr;
static ssize_t debug_msg_read(struct file *filp, struct kobject *kobj,
			      struct bin_attribute *attr, char *buf,
			      loff_t off, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct fcd52_data *data = iio_priv(indio_dev);

	if (off)
		return 0;

	return fcd52_dbg_pop(data, buf, count);
}

/* Messages dropped while the ring was full */
static ssize_t debug_msg_lost_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct fcd52_data *data = iio_priv(indio_dev);

	return sysfs_emit(buf, "%u\n", data->dbg_lost);
}
static DEVICE_ATTR_RO(debug_msg_lost);

/* Poll target: notified each time a message is queued */
static ssize_t debug_notify_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "0\n");
}
static DEVICE_ATTR_RO(debug_notify);

/* Single bootloader-address read, outside any update: tests bl_read alone */
static ssize_t bl_status_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct fcd52_data *data = iio_priv(indio_dev);
	u8 val[3] = { 0 };
	int error;

	error = data->bops->bl_read(data, val, sizeof(val));
	if (error)
		return sysfs_emit(buf, "error %d\n", error);

	return sysfs_emit(buf, "%02x %02x %02x\n", val[0], val[1], val[2]);
}
static DEVICE_ATTR_RO(bl_status);

static DEVICE_ATTR_WO(update_cfg);
static ssize_t update_fw_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count);
static DEVICE_ATTR_WO(update_fw);


static struct bin_attribute *fcd52_bin_attrs[] = {
	&fcd52_mem_access_attr,
	&fcd52_debug_msg_attr,
	NULL
};

static struct attribute *fcd52_attrs[] = {
	&dev_attr_update_cfg.attr,
	&dev_attr_bl_status.attr,
	&dev_attr_update_fw.attr,
	&dev_attr_debug_v2_enable.attr,
	&dev_attr_debug_msg_lost.attr,
	&dev_attr_debug_notify.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_hw_version.attr,
	&dev_attr_config_crc.attr,
	&dev_attr_drvr_version.attr,
	NULL
};

/* Bootloader only: the update interface, with no object table to expose */
static struct attribute *fcd52_bl_attrs[] = {
	&dev_attr_update_fw.attr,
	&dev_attr_bl_status.attr,
	NULL
};

static const struct attribute_group fcd52_bl_attr_group = {
	.attrs = fcd52_bl_attrs,
};

static const struct attribute_group fcd52_attr_group = {
	.attrs = fcd52_attrs,
	.bin_attrs = fcd52_bin_attrs,
};

/* T6 DIAGNOSTIC byte and debug commands (Protocol Guide Table 3-5) */
#define FCD52_T6_DIAGNOSTIC_OFFSET	5
#define FCD52_DIAG_PAGE_UP		0x01
#define FCD52_DIAG_PAGE_DOWN		0x02
#define FCD52_DIAG_MODE_DELTAS		0x10	/* T37 holds mutual deltas */
#define FCD52_DIAG_MODE_REFS		0x11	/* T37 holds references */

/* T37 layout (Protocol Guide Section 2.2): PAGE, then 128 bytes of DATA */
#define FCD52_T37_PAGE			0
#define FCD52_T37_DATA			1
#define FCD52_T37_DATA_LEN		128

/* Poll limit for the page to be confirmed after a step (a cycle is ~50 ms) */
#define FCD52_T37_CONFIRM_TRIES		20
#define FCD52_T37_CONFIRM_DELAY_MS	10

/*
 * Select the T37 debug mode. Changing mode resets the page to zero, so the
 * caller reads the page number afterwards (Table 3-5 note).
 */
int fcd52_debug_set_mode(struct fcd52_data *data, u8 mode)
{
	const struct fcd52_obj_ref *t6 = &data->cached.t6;

	if (!t6->present)
		return -ENODEV;
	if (mode != FCD52_DIAG_MODE_DELTAS && mode != FCD52_DIAG_MODE_REFS)
		return -EINVAL;

	return fcd52_bus_write(data, t6->start_address +
			       FCD52_T6_DIAGNOSTIC_OFFSET, 1, &mode);
}
EXPORT_SYMBOL_NS_GPL(fcd52_debug_set_mode, IIO_FCD52);

/*
 * Step to a page and wait until T37 reports it. The page is valid only once
 * PAGE matches (Protocol Guide Section 3.3.2), so DATA must not be read before
 * this returns 0.
 */
int fcd52_debug_step_page(struct fcd52_data *data, u8 target_page)
{
	const struct fcd52_obj_ref *t6 = &data->cached.t6;
	const struct fcd52_obj_ref *t37 = &data->cached.t37;
	u8 page;
	u8 expect;
	u8 cmd;
	int tries;
	int error;

	if (!t6->present || !t37->present)
		return -ENODEV;

	error = fcd52_bus_read(data, t37->start_address + FCD52_T37_PAGE,
			       1, &page);
	if (error)
		return error;

	/* Step one page at a time until the device reports the target */
	while (page != target_page) {
		if (page < target_page) {
			cmd = FCD52_DIAG_PAGE_UP;
			expect = page + 1;
		} else {
			cmd = FCD52_DIAG_PAGE_DOWN;
			expect = page - 1;
		}

		error = fcd52_bus_write(data, t6->start_address +
					FCD52_T6_DIAGNOSTIC_OFFSET, 1, &cmd);
		if (error)
			return error;

		/*
		 * The device moves one page per measurement cycle. Wait until
		 * PAGE shows the step we asked for, and only then go on.
		 */
		for (tries = 0; tries < FCD52_T37_CONFIRM_TRIES; tries++) {
			msleep(FCD52_T37_CONFIRM_DELAY_MS);
			error = fcd52_bus_read(data,
					       t37->start_address + FCD52_T37_PAGE,
					       1, &page);
			if (error)
				return error;
			if (page == expect)
				break;
		}

		if (page != expect) {
			dev_err(data->dev, "T37 page stuck at %u, wanted %u\n",
				page, expect);
			return -ETIMEDOUT;
		}
	}

	return 0;
}
EXPORT_SYMBOL_NS_GPL(fcd52_debug_step_page, IIO_FCD52);

/*
 * Bootloader status byte (MXTAN2016 Table 2). The three ID-bearing states are
 * identified by the top two bits; the low six bits carry the bootloader ID.
 * The CRC states have no ID bits, so the whole byte is the code.
 */
#define FCD52_BL_STATE_MASK		0xc0
#define FCD52_BL_WAITING_CMD		0xc0	/* 0b11nnnnnn */
#define FCD52_BL_WAITING_FRAME		0x80	/* 0b10nnnnnn */
#define FCD52_BL_APP_CRC_FAIL		0x40	/* 0b01nnnnnn */
#define FCD52_BL_FRAME_CRC_CHECK	0x02
#define FCD52_BL_FRAME_CRC_FAIL		0x03
#define FCD52_BL_FRAME_CRC_PASS		0x04
/* Bit 5 set: extended ID mode, ID and version follow in a 3-byte read */
#define FCD52_BL_EXT_ID			0x20

/* Unlock command, sent in WAITING_CMD (Figure 8) */
#define FCD52_BL_UNLOCK_LSB		0xdc
#define FCD52_BL_UNLOCK_MSB		0xaa

/* Frames are at most 276 bytes; each frame is retried this many times */
#define FCD52_FW_FRAME_MAX		276
#define FCD52_FW_FRAME_RETRIES		20
#define FCD52_BL_WAIT_MS		1000
/* States skipped while waiting for one that accepts the unlock command */
#define FCD52_BL_STATE_TRIES		32
/*
 * Entry timing (Protocol Guide D.2.3): CHG low within 725 ms of the 0xA5 write,
 * then 1 ms before the device answers. Poll with a margin beyond that.
 */
#define FCD52_BL_ENTRY_MS		1000
/* Time for the application to start after the final firmware frame */
#define FCD52_FW_APP_START_MS		1000

/* True when @val is in the state @state (Table 2) */
static bool fcd52_bl_state_is(u8 val, u8 state)
{
	switch (state) {
	case FCD52_BL_WAITING_CMD:
	case FCD52_BL_WAITING_FRAME:
	case FCD52_BL_APP_CRC_FAIL:
		return (val & FCD52_BL_STATE_MASK) == state;
	default:
		return val == state;
	}
}

/* Reject ASCII hex input: a binary image contains bytes outside '0'-'9','A'-'F' */
static bool fcd52_fw_is_binary(const u8 *fw, size_t len)
{
	size_t i;

	for (i = 0; i < len; i++) {
		u8 c = fw[i];

		if (c < '0' || (c > '9' && c < 'A') || c > 'F')
			return true;
	}

	return false;
}

/*
 * Wait for the CHG handler to report the next bootloader state. The handler has
 * already read the status byte and queued it. CHG stays low until
 * that read, so each completion matches exactly one state change.
 */
static int fcd52_bl_wait_event(struct fcd52_data *data, u8 *val)
{
	struct fcd52_bl_status *slot;
	unsigned long flags;
	int error;

	/*
	 * The completion is only a wakeup. The queue is the record of what was
	 * read, so an entry is taken from it, not counted from completions. A
	 * wakeup with an empty queue (a completion already consumed alongside its
	 * entry) is not an error: wait again for the next one.
	 */
	for (;;) {
		if (!wait_for_completion_timeout(&data->bl_done,
						 msecs_to_jiffies(FCD52_BL_WAIT_MS)))
			return -ETIMEDOUT;

		spin_lock_irqsave(&data->bl_lock, flags);
		if (data->bl_tail != data->bl_head)
			break;
		spin_unlock_irqrestore(&data->bl_lock, flags);
	}

	slot = &data->bl_fifo[data->bl_tail];
	error = slot->error;
	memcpy(val, slot->val, sizeof(slot->val));
	data->bl_tail = (data->bl_tail + 1) % FCD52_BL_FIFO;
	spin_unlock_irqrestore(&data->bl_lock, flags);

	return error;
}

/* Wait for a state, skipping the transient FRAME_CRC_CHECK */
static int fcd52_bl_wait_status(struct fcd52_data *data, u8 want)
{
	u8 buf[3];
	u8 val;
	int error;

	/*
	 * CHG stays low while a status is held, so the handler re-queues the same
	 * value on each run. Consume repeats of the state just seen and wait for
	 * the next change; CHECK is transient, so it is passed over the same way.
	 */
	error = fcd52_bl_wait_event(data, buf);
	if (error)
		return error;
	val = buf[0];

	while (val == FCD52_BL_FRAME_CRC_CHECK) {
		u8 seen = val;

		do {
			error = fcd52_bl_wait_event(data, buf);
			if (error)
				return error;
			val = buf[0];
		} while (val == seen);
	}

	/*
	 * A stale WAITING_FRAME_DATA from before the frame is not the answer to
	 * this frame. Skip it, and wait for the state the frame expects.
	 */
	while (want != FCD52_BL_WAITING_FRAME &&
	       fcd52_bl_state_is(val, FCD52_BL_WAITING_FRAME)) {
		error = fcd52_bl_wait_event(data, buf);
		if (error)
			return error;
		val = buf[0];
	}

	if (val == FCD52_BL_FRAME_CRC_FAIL)
		return -EILSEQ;

	if (fcd52_bl_state_is(val, FCD52_BL_APP_CRC_FAIL)) {
		dev_err(data->dev, "application CRC failed in bootloader\n");
		return -EBADMSG;
	}

	if (!fcd52_bl_state_is(val, want)) {
		dev_err(data->dev, "bootloader status 0x%02X, wanted 0x%02X\n",
			val, want);
		return -EINVAL;
	}

	return 0;
}

/*
 * Wait for a state that accepts the unlock command. WAITING_CMD and APP_CRC_FAIL
 * both accept it (Protocol Guide D.2.4). Any other state is consumed and
 * ignored, so a stale frame-state entry cannot be taken as the answer.
 */
static int fcd52_bl_wait_unlockable(struct fcd52_data *data, u8 *val)
{
	int error;
	int tries;

	for (tries = 0; tries < FCD52_BL_STATE_TRIES; tries++) {
		error = fcd52_bl_wait_event(data, val);
		if (error)
			return error;

		if (fcd52_bl_state_is(val[0], FCD52_BL_WAITING_CMD) ||
		    fcd52_bl_state_is(val[0], FCD52_BL_APP_CRC_FAIL))
			return 0;

		dev_dbg(data->dev, "bootloader status 0x%02X, waiting for unlock state\n",
			val[0]);
	}

	return -ETIMEDOUT;
}

/*
 * Check a status read for WAITING_CMD. APP_CRC_FAIL also accepts the unlock
 * command: the bootloader is ready for it either way (Protocol Guide D.2.4).
 * The 3-byte read gives status, bootloader ID and version (MXTAN2016 Table 1-1).
 */
static int fcd52_bl_check_cmd(struct fcd52_data *data, const u8 *val)
{
	if (!fcd52_bl_state_is(val[0], FCD52_BL_WAITING_CMD) &&
	    !fcd52_bl_state_is(val[0], FCD52_BL_APP_CRC_FAIL)) {
		dev_err(data->dev, "bootloader status 0x%02X, not ready for unlock\n",
			val[0]);
		return -EINVAL;
	}

	if (val[0] & FCD52_BL_EXT_ID)
		dev_info(data->dev, "bootloader ID 0x%02X version 0x%02X\n",
			 val[1], val[2]);

	return 0;
}

/* Wait for the CHG handler's status read after the T6 RESET (0xA5) */
static int fcd52_bl_wait_cmd(struct fcd52_data *data)
{
	int error;

	u8 buf[3] = { 0 };

	error = fcd52_bl_wait_event(data, buf);
	if (error) {
		dev_err(data->dev, "no CHG from bootloader: %d\n", error);
		return error;
	}

	return fcd52_bl_check_cmd(data, buf);
}

/*
 * Read the bootloader directly, for when the device is already in bootloader
 * mode. WAITING_FRAME_DATA means it was unlocked earlier, so the unlock command
 * must not be sent again and frames can go straight out.
 */
static int fcd52_bl_check_direct(struct fcd52_data *data, bool *unlocked)
{
	u8 val[3] = { 0 };
	int error;

	/*
	 * Already in bootloader and waiting: CHG is high, so no edge will arrive
	 * to trigger a read. Read the status once, directly, to learn the state.
	 * Transitions after this are still taken from the CHG handler.
	 */
	error = data->bops->bl_read(data, val, sizeof(val));
	if (error) {
		dev_err(data->dev, "bootloader status read failed: %d\n", error);
		return error;
	}

	if (fcd52_bl_state_is(val[0], FCD52_BL_WAITING_FRAME)) {
		dev_info(data->dev, "bootloader already unlocked, status 0x%02X\n",
			 val[0]);
		*unlocked = true;
		return 0;
	}

	if (!fcd52_bl_state_is(val[0], FCD52_BL_WAITING_CMD) &&
	    !fcd52_bl_state_is(val[0], FCD52_BL_APP_CRC_FAIL)) {
		/* Not yet a command state: wait for one, ignoring the rest */
		error = fcd52_bl_wait_unlockable(data, val);
		if (error)
			return error;
	}

	return fcd52_bl_check_cmd(data, val);
}

/*
 * Write one frame and wait for the device to accept it. A CRC failure is
 * retried, since the device returns to WAITING_FRAME_DATA and accepts a resend.
 */
static int fcd52_bl_program_frame(struct fcd52_data *data, const u8 *frame,
				  unsigned int len)
{
	int error;
	int retry;

	for (retry = 0; retry < FCD52_FW_FRAME_RETRIES; retry++) {
		error = data->bops->bl_write(data, frame, len);
		if (error)
			return error;

		error = fcd52_bl_wait_status(data, FCD52_BL_FRAME_CRC_PASS);
		if (!error)
			return 0;
		if (error != -EILSEQ)
			return error;

		dev_warn(data->dev, "frame CRC failed, retry %d\n", retry + 1);
		msleep(20 * (retry + 1));
		error = fcd52_bl_wait_status(data, FCD52_BL_WAITING_FRAME);
		if (error)
			return error;
	}

	dev_err(data->dev, "firmware frame failed after %d retries\n",
		FCD52_FW_FRAME_RETRIES);
	return -EIO;
}

/*
 * Program an application image. Frames are read from the file as stored: a
 * 2-byte big-endian length (which includes the device's own CRC), then the data.
 * The device checks each frame's CRC, so the driver does not compute one.
 */
static int fcd52_bl_program(struct fcd52_data *data, const u8 *fw, size_t len,
			    bool ready)
{
	size_t pos = 0;
	int error;

	while (pos < len) {
		unsigned int frame_len;

		if (len - pos < 2)
			return -EINVAL;

		frame_len = (fw[pos] << 8) | fw[pos + 1];
		frame_len += 2;	/* the length field itself is not counted */

		if (frame_len > FCD52_FW_FRAME_MAX || pos + frame_len > len) {
			dev_err(data->dev,
				"bad frame length %u at %zu, image is %zu bytes\n",
				frame_len, pos, len);
			return -EINVAL;
		}

		/*
		 * The first frame of an already unlocked bootloader is sent at once:
		 * its WAITING_FRAME_DATA state was read by the direct check and
		 * no CHG edge will follow it.
		 */
		if (!ready) {
			error = fcd52_bl_wait_status(data, FCD52_BL_WAITING_FRAME);
			if (error)
				return error;
		}
		ready = false;

		error = fcd52_bl_program_frame(data, fw + pos, frame_len);
		if (!error)
			data->bl_last_pass = jiffies;
		if (error)
			return error;

		pos += frame_len;
	}

	return 0;
}

/*
 * Enter bootloader mode with a software reset (T6 RESET = 0xA5), unlock it, and
 * program the image. The final frame makes the device reset into the new
 * application. In-bootloader mode keeps T5 from being read until done.
 */
int fcd52_update_firmware(struct fcd52_data *data, const u8 *fw, size_t len)
{
	const struct fcd52_obj_ref *t6 = &data->cached.t6;
	u8 unlock[2] = { FCD52_BL_UNLOCK_LSB, FCD52_BL_UNLOCK_MSB };
	u8 value = 0xa5;
	bool unlocked = false;
	int error;

	if (!fcd52_fw_is_binary(fw, len)) {
		dev_err(data->dev, "firmware must be binary: use xxd -r -p\n");
		return -EINVAL;
	}

	/*
	 * Clear any CHG edge left from before this update, then arm the handler.
	 * The reset comes before the wait, so no edge raised for this update is
	 * lost.
	 */
	/*
	 * Discard every status entry left from an earlier session (a failed
	 * update, or a probe-time read), so only states from this update can be
	 * consumed. The FIFO is shared with the handler, so clear it under the lock.
	 */
	spin_lock_irq(&data->bl_lock);
	data->bl_head = 0;
	data->bl_tail = 0;
	spin_unlock_irq(&data->bl_lock);
	reinit_completion(&data->bl_done);
	data->bl_waiting = true;

	if (data->in_bootloader) {
		/* Bootloader already running (CHG high): wait for its status */
		dev_info(data->dev, "firmware update: already in bootloader\n");
		error = fcd52_bl_check_direct(data, &unlocked);
	} else {
		if (!t6->present)
			return -ENODEV;

		dev_info(data->dev, "firmware update: T6 RESET 0xA5 at 0x%04X\n",
			 t6->start_address);
		error = fcd52_bus_write(data, t6->start_address + 0, 1, &value);
		if (error) {
			dev_err(data->dev, "T6 RESET write failed: %d\n", error);
			return error;
		}

		/* Bootloader is entering; from here T5 must not be read */
		data->in_bootloader = true;
		error = fcd52_bl_wait_cmd(data);
	}
	if (error) {
		dev_err(data->dev, "bootloader did not reach WAITING_CMD: %d\n",
			error);
		goto out;
	}

	if (!unlocked) {
		dev_info(data->dev, "firmware update: WAITING_CMD, sending unlock\n");
		error = data->bops->bl_write(data, unlock, sizeof(unlock));
		if (error)
			goto out;
	}

	/*
	 * After the unlock the device is at WAITING_FRAME_DATA and holds CHG
	 * high. The unlock's own status edge already delivered that state, so the
	 * first frame goes out at once rather than waiting for another edge.
	 */
	error = fcd52_bl_program(data, fw, len, true);
	if (error)
		goto out;

	/*
	 * The final frame resets the device into the application. Stop the CHG
	 * handler from touching the bootloader address for the whole switch-over:
	 * CHG is still low from the bootloader, and a level interrupt would
	 * otherwise keep re-firing and addressing 0x27 (the NACK storm).
	 */
	data->bl_waiting = false;
	if (data->irq)
		disable_irq(data->irq);
	reinit_completion(&data->bl_done);

	dev_info(data->dev, "firmware update: waiting for application start\n");
	{
		unsigned long due = data->bl_last_pass +
				    msecs_to_jiffies(FCD52_FW_APP_START_MS);
		long left = (long)(due - jiffies);

		if (left > 0)
			msleep(jiffies_to_msecs(left));
	}

	dev_info(data->dev, "firmware update: switch-over wait done\n");

	/*
	 * Application is up. The IRQ stays disabled until the info block is
	 * rebuilt (update_fw_store), so the handler never runs against an empty
	 * object cache. The caller re-enables it once the cache is valid.
	 */
	data->in_bootloader = false;

out:
	/*
	 * On failure the device is still in bootloader mode: keep in_bootloader
	 * and the waiting state so the CHG handler can still service it.
	 */
	return error;
}
EXPORT_SYMBOL_NS_GPL(fcd52_update_firmware, IIO_FCD52);

/* Re-apply the configuration after a firmware update; absence is not an error */
static int fcd52_update_reapply_cfg(struct fcd52_data *data)
{
	struct device *dev = data->dev;
	const struct firmware *fw = NULL;
	struct fcd52_cfg cfg;
	int error;

	error = request_firmware(&fw, data->cfg_name, dev);
	if (error == -ENOENT)
		return 0;
	if (error)
		return error;

	error = fcd52_cfg_parse(dev, (const char *)fw->data, fw->size, &cfg);
	if (error)
		goto out_release;

	cfg.records = kcalloc(cfg.num_records, sizeof(*cfg.records), GFP_KERNEL);
	cfg.blob = kmalloc(cfg.total_bytes ?: 1, GFP_KERNEL);
	if (!cfg.records || !cfg.blob) {
		error = -ENOMEM;
		goto out_free;
	}

	error = fcd52_cfg_fill((const char *)fw->data, fw->size, &cfg);
	if (error)
		goto out_free;

	error = fcd52_update_config(data, &cfg);

out_free:
	kfree(cfg.records);
	kfree(cfg.blob);
out_release:
	release_firmware(fw);
	return error;
}

/* Default firmware image, a binary converted from the vendor hex with xxd -r -p */
#define FCD52_FW_NAME			"fcd52.fw"
#define FCD52_FW_PATH_MAX		128

/*
 * Program the image named in the write (or the default fcd52.fw). Frames are
 * sent as stored; the image has no version header, so the device's info block
 * is read before and after to report the change. Upgrades and downgrades
 * proceed silently. After programming, the configuration file is re-applied
 * if request_firmware() finds it; a missing file is not an error.
 */
static ssize_t update_fw_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct fcd52_data *data = iio_priv(indio_dev);
	const struct firmware *fw = NULL;
	char path[FCD52_FW_PATH_MAX];
	const char *name;
	size_t len;
	int error;

	if (!*buf || sysfs_streq(buf, "default")) {
		name = FCD52_FW_NAME;
	} else {
		len = min(count, sizeof(path) - 1);
		memcpy(path, buf, len);
		path[len] = '\0';
		if (len && path[len - 1] == '\n')
			path[len - 1] = '\0';
		name = path;
	}

	error = request_firmware(&fw, name, dev);
	if (error) {
		dev_err(dev, "Failed to load %s: %d\n", name, error);
		return error;
	}

	if (!fcd52_fw_is_binary(fw->data, fw->size)) {
		dev_err(dev, "%s is not binary: use xxd -r -p\n", name);
		error = -EINVAL;
		goto out_release;
	}

	error = fcd52_update_firmware(data, fw->data, fw->size);
	if (error) {
		dev_err(dev, "Firmware update failed: %d\n", error);
		goto out_release;
	}

	/*
	 * The new firmware can move the objects, so reparse the information
	 * block and object table before anything reads them again.
	 */
	dev_info(dev, "firmware update: reading information block\n");
	error = fcd52_read_info_block(data);
	if (error) {
		dev_err(dev, "Information block not readable after update: %d\n",
			error);
		goto out_release;
	}
	dev_info(dev, "firmware update: information block done\n");

	if (data->irq)
		enable_irq(data->irq);
	dev_info(dev, "firmware update: IRQ re-enabled\n");

	dev_info(dev, "firmware update: re-applying configuration\n");
	error = fcd52_update_reapply_cfg(data);
	if (error)
		dev_warn(dev, "Config re-apply after update failed: %d\n", error);
	dev_info(dev, "firmware update: complete\n");

	release_firmware(fw);
	return count;

out_release:
	release_firmware(fw);
	return error;
}

static void fcd52_core_remove(void *p)
{
	fcd52_free_object_table(p);
}

static const struct iio_info fcd52_iio_info = {
	.debugfs_reg_access = fcd52_debugfs_reg_access,
};

/*
 * Probe the bootloader address after a failed application read. WAITING_CMD or
 * APP_CRC_FAIL both mean the device is in bootloader mode and is ready to
 * receive the unlock command, so the update path can recover it. Any other
 * answer is not a bootloader.
 */
static int fcd52_probe_bootloader(struct fcd52_data *data)
{
	/*
	 * A failed application read with CHG held low means the bootloader is
	 * running. Its status is not read here: the CHG handler reads it on the
	 * interrupt, and the update path waits for that.
	 */
	dev_info(data->dev, "application not answering: bootloader assumed\n");
	data->in_bootloader = true;
	return 0;
}

int fcd52_core_probe(struct device *dev, const struct fcd52_bus_ops *bops,
		     const char *name, u16 app_addr, int irq)
{
	struct iio_dev *indio_dev;
	struct fcd52_data *data;
	int error;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->dev = dev;
	data->bops = bops;
	data->irq = irq;

	/* Bootloader address is fixed by the application address (Table 5-73) */
	data->bl_addr = fcd52_bl_addr(app_addr);
	if (!data->bl_addr)
		return dev_err_probe(dev, -EINVAL,
				     "Application address 0x%02x outside 0x%02x-0x%02x\n",
				     app_addr, FCD52_APP_ADDR_MIN,
				     FCD52_APP_ADDR_MAX);
	dev_info(dev, "application 0x%02x, bootloader 0x%02x\n",
		 app_addr, data->bl_addr);

	error = devm_mutex_init(dev, &data->bus_lock);
	if (error)
		return error;

	error = devm_mutex_init(dev, &data->dbg_lock);
	if (error)
		return error;

	/* Configuration name: device tree override, else fcd52.cfg */
	error = device_property_read_string(dev, "microchip,cfg-name",
					    &data->cfg_name);
	if (error)
		data->cfg_name = FCD52_CFG_NAME_DEFAULT;

	init_completion(&data->reset_done);
	init_completion(&data->bl_done);
	spin_lock_init(&data->bl_lock);

	dev_set_drvdata(dev, indio_dev);

	error = fcd52_hw_reset(data);
	if (error)
		return error;

	error = fcd52_read_info_block(data);
	if (error) {
		/*
		 * No application answer. The device may be in bootloader mode
		 * (no firmware, or a failed CRC). Probe the bootloader address
		 * as the reference driver does; the update path can recover it.
		 */
		dev_info(dev, "No application answer, checking bootloader\n");
		error = fcd52_probe_bootloader(data);
		if (error)
			return dev_err_probe(dev, error,
					     "Failed to read information block\n");
	}

	error = devm_add_action_or_reset(dev, fcd52_core_remove, data);
	if (error)
		return error;

	error = fcd52_request_irq(data, indio_dev);
	if (error)
		return error;

	/*
	 * Bootloader only: no object table, so expose just the update interface.
	 * CHG is enabled so the update can read the bootloader status.
	 */
	if (data->in_bootloader) {
		error = devm_device_add_group(dev, &fcd52_bl_attr_group);
		if (error)
			return error;

		if (data->irq)
			enable_irq(data->irq);

		dev_info(dev, "bootloader only: update_fw is available\n");
		return 0;
	}

	/*
	 * Channels are driven by the SPI frame data interface and are added
	 * along with it. Register without them so that the object table can
	 * be inspected and configured over I2C in the meantime.
	 */
	indio_dev->name = name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &fcd52_iio_info;

	error = devm_iio_device_register(dev, indio_dev);
	if (error)
		return error;

	/* Raw access to the device map; the size is the map extent */
	sysfs_bin_attr_init(&fcd52_mem_access_attr);
	fcd52_mem_access_attr.attr.name = "mem_access";
	fcd52_mem_access_attr.attr.mode = 0644;
	fcd52_mem_access_attr.read = mem_access_read;
	fcd52_mem_access_attr.write = mem_access_write;
	fcd52_mem_access_attr.size = data->mem_size;

	/* Size the T5 feed attribute from the ring, as the bridge sizes by it */
	sysfs_bin_attr_init(&fcd52_debug_msg_attr);
	fcd52_debug_msg_attr.attr.name = "debug_msg";
	fcd52_debug_msg_attr.attr.mode = 0444;
	fcd52_debug_msg_attr.read = debug_msg_read;
	fcd52_debug_msg_attr.size = FCD52_DBG_RING_SLOTS *
				    (data->cached.t5.size - 1);

	error = devm_device_add_group(dev, &fcd52_attr_group);
	if (error)
		return error;

	/* Dump the parsed object table alongside direct_reg_access */
	debugfs_create_file("objects", 0444,
			    iio_get_debugfs_dentry(indio_dev), data,
			    &fcd52_objects_fops);

	/*
	 * Empty the queue before the interrupt is enabled. The power-up RESET,
	 * calibration and checksum messages are waiting here, and CHG stays low
	 * until they are read. The line is still masked, so the handler cannot
	 * run at the same time.
	 */
	error = fcd52_process_messages_until_empty(data);
	if (error)
		dev_warn(dev, "Power-up messages not fully read: %d\n", error);

	/*
	 * Power-up messages may not carry a checksum, for example when no reset
	 * line was pulsed. Request every reporting object to send its status,
	 * so config_crc holds the real value before any update compares it.
	 */
	if (!data->config_crc_valid) {
		error = fcd52_reportall(data);
		if (error)
			dev_warn(dev, "REPORTALL at probe failed: %d\n", error);
	}

	/*
	 * Enable CHG last. The handler needs the object table and the IIO
	 * device to be in place. A message already pending from power-up is
	 * found by the first poll of T44 once the line is enabled.
	 */
	if (data->irq)
		enable_irq(data->irq);

	return 0;
}
EXPORT_SYMBOL_NS_GPL(fcd52_core_probe, IIO_FCD52);

MODULE_AUTHOR("Michael Gong <michael.gong@microchip.com>");
MODULE_DESCRIPTION("Microchip ATFCD52 Fast Capacitive Digitizer core");
MODULE_LICENSE("GPL");
