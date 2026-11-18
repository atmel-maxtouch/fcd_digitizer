/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Microchip ATFCD52 Fast Capacitive Digitizer - OBP_RAW configuration parser
 *
 * Parses the OBP_RAW text configuration files (V1 and V4). The parser does no
 * bus access, so it can be tested on its own.
 *
 * Copyright (c) 2026 Microchip Technology Inc.
 */

#ifndef _FCD52_CFG_H_
#define _FCD52_CFG_H_

#include <linux/device.h>
#include <linux/types.h>

#define FCD52_CFG_MAGIC_V1	"OBP_RAW V1"
#define FCD52_CFG_MAGIC_V4	"OBP_RAW V4"

/* Result of fcd52_cfg_decide() */
#define FCD52_CFG_WRITE		0
#define FCD52_CFG_SKIP		1

/* One object instance from the file. @data points into cfg->blob. */
struct fcd52_cfg_record {
	u16 type;
	u16 instance;
	u16 size;
	const u8 *data;
};

/*
 * Parsed configuration. fcd52_cfg_parse() fills the header and counts the
 * records. The caller then allocates @records and @blob, and passes them to
 * fcd52_cfg_fill().
 */
struct fcd52_cfg {
	unsigned int version;		/* 1 or 4 */
	u8 info[7];			/* ID information block from the file */
	u32 info_crc;			/* info block CRC24 from the file */
	u32 config_crc;			/* configuration CRC24 from the file */

	size_t records_start;		/* text offset of the first record */
	unsigned int num_records;
	size_t total_bytes;		/* sum of all record sizes */

	struct fcd52_cfg_record *records;	/* num_records entries */
	u8 *blob;			/* total_bytes of decoded data */
};

int fcd52_cfg_parse(struct device *dev, const char *raw, size_t raw_len,
		    struct fcd52_cfg *cfg);
int fcd52_cfg_fill(const char *raw, size_t raw_len, struct fcd52_cfg *cfg);
int fcd52_cfg_decide(const struct fcd52_cfg *cfg, const u8 *dev_info,
		     u32 dev_info_crc, u32 dev_cfg_crc);
bool fcd52_cfg_record_is_volatile(u16 type);

#endif /* _FCD52_CFG_H_ */
