/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Microchip ATFCD52 Fast Capacitive Digitizer
 *
 * Object-Based Protocol (OBP) definitions shared between the core driver and
 * the bus-specific front ends.
 *
 * Copyright (c) 2026 Microchip Technology Inc.
 */

#ifndef _FCD52_H_
#define _FCD52_H_

#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/types.h>

/* Information Block layout (Protocol Guide Table 1-1) */
#define FCD52_ID_INFO_SIZE		7
#define FCD52_OBJECT_START		FCD52_ID_INFO_SIZE
#define FCD52_OBJECT_ELEM_SIZE		6
#define FCD52_INFO_CRC_SIZE		3

/*
 * Extended Object Table T254 (Protocol Guide Table 3-14). On the ATFCD52 the
 * object holds a single 7-byte object table element followed by a 24-bit CRC.
 */
#define FCD52_EXT_OBJECT_ELEM_SIZE	7
#define FCD52_EXT_OBJECT_NUM		1
#define FCD52_T254_SIZE			(FCD52_EXT_OBJECT_ELEM_SIZE * \
					 FCD52_EXT_OBJECT_NUM + \
					 FCD52_INFO_CRC_SIZE)

/* Device identifiers (Protocol Guide Section 1.4) */
#define FCD52_FAMILY_ID			0xaa
#define FCD52_VARIANT_ID		0x00

/* Largest payload the device accepts in a single transfer */
#define FCD52_MAX_BLOCK_RW		255

/* Object types referenced by the driver */
#define FCD52_DEBUG_DIAGNOSTIC_T37	37
#define FCD52_GEN_MESSAGE_T5		5
#define FCD52_GEN_COMMAND_T6		6
#define FCD52_GEN_POWER_T7		7
#define FCD52_GEN_ACQUIRE_T8		8
#define FCD52_GEN_EXTOBJECTTABLE_T254	254
#define FCD52_SPT_COMMSCONFIG_T18	18
#define FCD52_SPT_GPIOPWM_T19		19
#define FCD52_SPT_USERDATA_T38		38
#define FCD52_SPT_MESSAGECOUNT_T44	44
#define FCD52_SPT_SPIDEBUGCONFIG_T17	17
#define FCD52_SPT_SERIALDATACOMMAND_T68	68
#define FCD52_SPT_FASTCAPDIGITIZER_T700	700

/* Reserved report IDs (Protocol Guide Section 1.5.6) */
#define FCD52_RPTID_RVSD		0x00
#define FCD52_RPTID_NOMSG		0xff

/*
 * Command Processor T6 message layout (Protocol Guide Table 3-7). Byte 0 is
 * the report ID, byte 1 the STATUS bits, bytes 2-4 the 24-bit configuration
 * checksum and bytes 5-7 the DATA fields used by SIGERR.
 */
#define FCD52_T6_MSG_STATUS		1
#define FCD52_T6_MSG_CHECKSUM		2
#define FCD52_T6_MSG_CHECKSUM_LEN	3

/* Report ID, STATUS, checksum and DATA, plus the T5 checksum byte */
#define FCD52_T6_MSG_MIN_SIZE		9

/* STATUS bits (Protocol Guide Section 3.3.3) */
#define FCD52_T6_STATUS_RESET		BIT(7)
#define FCD52_T6_STATUS_OFL		BIT(6)
#define FCD52_T6_STATUS_SIGERR		BIT(5)
#define FCD52_T6_STATUS_CAL		BIT(4)
#define FCD52_T6_STATUS_CFGERR		BIT(3)
#define FCD52_T6_STATUS_COMSERR		BIT(2)
#define FCD52_T6_STATUS_FROZEN		BIT(1)

/* Fast Capacitive Digitizer T700 field offsets (Protocol Guide Table 5-70) */
#define FCD52_T700_CTRL			0
#define FCD52_T700_XSIZE		1
#define FCD52_T700_YSIZE		2
#define FCD52_T700_GAIN			3
#define FCD52_T700_DATACFG		4
#define FCD52_T700_DATACFG2		5

#define FCD52_T700_CTRL_ENABLE		BIT(0)
#define FCD52_T700_DELTASEN		BIT(0)
#define FCD52_T700_SIGNALSEN		BIT(1)
#define FCD52_T700_REFSEN		BIT(2)
#define FCD52_T700_CRCEN		BIT(7)

/* Sensor matrix limits (Protocol Guide "XSIZE Field" and "YSIZE Field") */
#define FCD52_MAX_XSIZE			32
#define FCD52_MAX_YSIZE			20
#define FCD52_MAX_NODES			(FCD52_MAX_XSIZE * FCD52_MAX_YSIZE)

/*
 * Frame data interface (Data Sheet Section 5.2). Every frame starts and ends
 * with a matching Frame ID byte. With CRCs enabled a 2-byte data type and
 * 2-byte length precede the samples and a 32-bit CRC follows them.
 */
#define FCD52_FRAME_ID_SIZE		1
#define FCD52_FRAME_HDR_SIZE		4
#define FCD52_FRAME_CRC_SIZE		4
#define FCD52_FRAME_MAX_DATA		2320

/* Frame data type codes (Data Sheet Table 5-1) */
#define FCD52_DATA_DELTAS		0x0000
#define FCD52_DATA_DELTAS_CRC		0x0001
#define FCD52_DATA_SIGNALS		0x0002
#define FCD52_DATA_SIGNALS_CRC		0x0003
#define FCD52_DATA_REFERENCES		0x0004
#define FCD52_DATA_REFERENCES_CRC	0x0005

/* 32-bit frame CRC seed (Protocol Guide Section C.1) */
#define FCD52_CRC32_SEED		0xffffffff

/* Reset timings (Data Sheet Sections 7.3 and 7.4) */
#define FCD52_RESET_GPIO_TIME_US	100
#define FCD52_POWERUP_DELAY_MS		136
#define FCD52_RESET_DELAY_MS		140

/**
 * struct fcd52_info - device ID information block
 * @family_id:		device family, %FCD52_FAMILY_ID for the ATFCD52
 * @variant_id:		device variant
 * @version:		firmware version, major in the upper nibble
 * @build:		firmware build number
 * @matrix_xsize:	number of X lines the device supports
 * @matrix_ysize:	number of Y lines the device supports
 * @object_num:		number of elements in the main object table
 *
 * Mirrors the first seven bytes of the memory map. All fields are single
 * bytes, so the wire layout needs no endian conversion.
 */
struct fcd52_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

/**
 * struct fcd52_object - parsed object table element
 * @type:		object type code, 16-bit to cover extended objects
 * @start_address:	location of the object in the memory map
 * @size:		size of one instance in bytes
 * @instances:		number of instances present
 * @num_report_ids:	report IDs allocated per instance
 * @min_reportid:	first report ID, zero if the object sends no messages
 * @max_reportid:	last report ID, zero if the object sends no messages
 *
 * Decoded into host byte order at parse time. @size and @instances already
 * have the protocol's minus-one encoding removed.
 */
struct fcd52_object {
	u16 type;
	u16 start_address;
	u16 size;
	u16 instances;
	u8 num_report_ids;
	u8 min_reportid;
	u8 max_reportid;
};

/**
 * struct fcd52_obj_ref - cached location of a configured object
 * @present:		the object exists on this device
 * @start_address:	location of instance 0 in the memory map
 * @size:		size of one instance in bytes
 * @instances:		number of instances
 * @min_reportid:	first report ID, zero if the object sends no messages
 */
struct fcd52_obj_ref {
	bool present;
	u16 start_address;
	u16 size;
	u16 instances;
	u8 min_reportid;
};

struct fcd52_data;

/**
 * struct fcd52_bus_ops - bus-specific register access
 * @reg_read:	read @len bytes starting at @reg
 * @reg_write:	write @len bytes starting at @reg
 *
 * Both callbacks return 0 on success or a negative errno. Transfers longer
 * than %FCD52_MAX_BLOCK_RW are split by the callback.
 */
struct fcd52_bus_ops {
	int (*reg_read)(struct fcd52_data *data, u16 reg, u16 len, void *val);
	int (*reg_write)(struct fcd52_data *data, u16 reg, u16 len,
			 const void *val);
	/* Bootloader transfers, to the bootloader address paired with the app address */
	int (*bl_read)(struct fcd52_data *data, u8 *val, unsigned int len);
	int (*bl_write)(struct fcd52_data *data, const u8 *val, unsigned int len);
};

/*
 * Application and bootloader I2C addresses are paired (Protocol Guide Table
 * 5-73): application 0x4A-0x52 maps to bootloader 0x26-0x2E, a fixed offset of
 * 0x24. The application address comes from the device tree reg property.
 */
/* Bootloader status reads queued between the CHG handler and the waiter */
#define FCD52_BL_FIFO			8

#define FCD52_APP_ADDR_MIN		0x4a
#define FCD52_APP_ADDR_MAX		0x52
#define FCD52_BL_ADDR_OFFSET		0x24

/* Bootloader address for the application address @app, or 0 if out of range */
static inline u16 fcd52_bl_addr(u16 app)
{
	if (app < FCD52_APP_ADDR_MIN || app > FCD52_APP_ADDR_MAX)
		return 0;

	return app - FCD52_BL_ADDR_OFFSET;
}

/**
 * struct fcd52_data - driver state
 * @dev:		device the driver is bound to
 * @bops:		bus-specific register access
 * @bus_lock:		serializes multi-transfer register accesses
 * @raw_info_block:	unmodified information block as read from the device
 * @info:		ID information, points into @raw_info_block
 * @object_table:	parsed objects, main table followed by extended objects
 * @object_num:		number of entries in @object_table
 * @info_crc:		24-bit CRC read from the end of the information block
 * @max_reportid:	highest report ID allocated, zero if none
 * @mem_size:		size of the device memory map in bytes
 * @t254_address:	location of the Extended Object Table T254 object
 * @t254_size:		size of the Extended Object Table T254 object
 * @irq:		CHG line interrupt, zero if not wired
 */
struct fcd52_data {
	struct device *dev;
	const struct fcd52_bus_ops *bops;
	struct mutex bus_lock;	/* serializes address pointer + data phases */
	int irq;

	/* Bootloader I2C address, paired with the application address */
	u16 bl_addr;

	/* Configuration file name for request_firmware(), from the device tree */
	const char *cfg_name;

	/* Last STATUS byte seen in a T6 message, used to log changes only */
	u8 t6_status;
	/* Configuration checksum from the most recent T6 message */
	u32 config_crc;
	/* Set once a T6 message has supplied config_crc; zero alone is a valid value */
	bool config_crc_valid;

	/*
	 * Firmware update state. While in_bootloader is set the CHG handler does
	 * not read T5; it signals bl_done so the waiting update code can read the
	 * bootloader status (Protocol Guide Appendix D).
	 */
	bool in_bootloader;
	/* Time of the last CRC-passing firmware frame, for the switch-over wait */
	unsigned long bl_last_pass;
	/* Set while the update is waiting on bootloader states; clear to stop reads */
	bool bl_waiting;
	struct completion bl_done;
	/* Set by the CHG handler: the status it read, and the read result */
	/*
	 * Status reads from the CHG handler, in order. Each completion pushes one
	 * entry; the waiter pops it. A single shared slot lost states when the
	 * handler ran again before the waiter read it.
	 */
	struct fcd52_bl_status {
		u8 val[3];
		int error;
	} bl_fifo[FCD52_BL_FIFO];
	unsigned int bl_head;
	unsigned int bl_tail;
	spinlock_t bl_lock;

	/*
	 * Set while a software reset is in progress. Caller transfers are held
	 * off so the host stays quiet during the reset. The CHG handler is
	 * exempt because it has to read the RESET message to complete the wait.
	 */
	bool reset_busy;
	/*
	 * Set when a RESET message is seen. The reset sequence is complete
	 * once a later message reports no status bits (Protocol Guide
	 * Section 3.3.3), so it is recorded here until that message arrives.
	 */
	bool reset_pending;
	/* Signalled by the T6 decoder when the reset sequence has finished */
	struct completion reset_done;

	void *raw_info_block;
	struct fcd52_info *info;

	struct fcd52_object *object_table;
	unsigned int object_num;

	u32 info_crc;
	u16 max_reportid;
	u16 mem_size;

	u16 t254_address;
	u16 t254_size;

	/*
	 * T5 message feed for the Android bridge. Off by default; when enabled the
	 * ring holds whole messages, and when it is full the newest message is
	 * dropped and counted, so the order of what was queued is kept.
	 */
	bool debug_v2;
	u8 *dbg_ring;
	unsigned int dbg_ring_slots;	/* messages the ring holds */
	unsigned int dbg_head;		/* next slot to read */
	unsigned int dbg_count;		/* messages queued */
	unsigned int dbg_lost;		/* messages dropped while full */
	struct mutex dbg_lock;

	/*
	 * One T5 message, sized from the object table (size - 1 bytes, the
	 * checksum byte excluded). Allocated once the T5 object is known.
	 */
	u8 *t5_msg;
	unsigned int t5_msg_len;

	/* Cached locations of the objects the driver configures */
	struct fcd52_cached_objects {
		struct fcd52_obj_ref t5;
		struct fcd52_obj_ref t6;
		struct fcd52_obj_ref t7;
		struct fcd52_obj_ref t17;
		struct fcd52_obj_ref t18;
		struct fcd52_obj_ref t38;
		struct fcd52_obj_ref t37;
		struct fcd52_obj_ref t44;
		struct fcd52_obj_ref t68;
		struct fcd52_obj_ref t700;
	} cached;
};

/**
 * struct fcd52_frame_layout - decoded frame geometry
 * @xsize:	number of X lines in use
 * @ysize:	number of Y lines in use
 * @nodes:	number of enabled nodes, @xsize * @ysize
 * @data_type:	expected frame data type code
 * @crc_enabled:	frame carries a header and 32-bit CRC
 * @data_offset:	offset of the first sample within a frame
 * @frame_size:	total frame length in bytes, including both Frame IDs
 *
 * Derived from the Fast Capacitive Digitizer T700 configuration, which must
 * be read over I2C before the frame data interface can be interpreted.
 */
struct fcd52_frame_layout {
	u8 xsize;
	u8 ysize;
	unsigned int nodes;
	u16 data_type;
	bool crc_enabled;
	unsigned int data_offset;
	unsigned int frame_size;
};

int fcd52_core_probe(struct device *dev, const struct fcd52_bus_ops *bops,
		     const char *name, u16 app_addr, int irq);

int fcd52_read_frame_layout(struct fcd52_data *data,
			    struct fcd52_frame_layout *layout);
void fcd52_frame_layout_from_matrix(struct fcd52_frame_layout *layout,
				    u8 xsize, u8 ysize, u16 data_type,
				    bool crc_enabled);
int fcd52_frame_validate(const struct fcd52_frame_layout *layout,
			 const u8 *frame);

int fcd52_read_reg(struct fcd52_data *data, u16 reg, u16 len, void *val);
int fcd52_soft_reset(struct fcd52_data *data);
int fcd52_reportall(struct fcd52_data *data);
int fcd52_update_firmware(struct fcd52_data *data, const u8 *fw, size_t len);
int fcd52_backup_command(struct fcd52_data *data, u8 command);
int fcd52_debug_set_mode(struct fcd52_data *data, u8 mode);
int fcd52_debug_step_page(struct fcd52_data *data, u8 target_page);

struct fcd52_cfg;
int fcd52_update_config(struct fcd52_data *data,
			const struct fcd52_cfg *cfg);

int fcd52_write_reg(struct fcd52_data *data, u16 reg, u16 len,
		    const void *val);

struct fcd52_object *fcd52_get_object(struct fcd52_data *data, u16 type);

u32 fcd52_crc24(u32 crc, u8 firstbyte, u8 secondbyte);
u32 fcd52_calculate_crc(const u8 *base, size_t len);
u8 fcd52_crc8(u8 crc, u8 data);

#endif /* _FCD52_H_ */
