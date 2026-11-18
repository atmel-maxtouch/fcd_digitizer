// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Microchip ATFCD52 Fast Capacitive Digitizer - OBP_RAW configuration parser
 *
 * File layout (OBP_RAW V1):
 *   OBP_RAW V1
 *   <7 byte info block, hex>
 *   <info CRC24, hex>
 *   <configuration CRC24, hex>
 *   <object records>
 *
 * OBP_RAW V4 adds three header lines before the info block, and a DEVICE_0
 * marker after the CRCs, ahead of the records:
 *   ENCRYPTION <n>
 *   MAX_ENCRYPTION_BLOCKS <n>
 *   NO_DEVICES <n>
 *
 *
 * Copyright (c) 2026 Microchip Technology Inc.
 */

#include <linux/ctype.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "fcd52-cfg.h"

/* Cursor over the raw text, always bounded by the buffer length */
struct fcd52_cfg_cursor {
	const char *buf;
	size_t len;
	size_t pos;
};

static bool cur_eof(const struct fcd52_cfg_cursor *c)
{
	return c->pos >= c->len;
}

/* Skip whitespace, then return the next token in @tok */
static size_t cur_token(struct fcd52_cfg_cursor *c, const char **tok)
{
	size_t start, n;

	while (!cur_eof(c) && isspace(c->buf[c->pos]))
		c->pos++;

	start = c->pos;
	while (!cur_eof(c) && !isspace(c->buf[c->pos]))
		c->pos++;

	n = c->pos - start;
	*tok = c->buf + start;
	return n;
}

/* Parse one hex token of 1 to 8 digits */
static int cur_hex(struct fcd52_cfg_cursor *c, unsigned int *val)
{
	const char *tok;
	size_t n = cur_token(c, &tok);
	unsigned int v = 0;
	size_t i;

	if (!n || n > 8)
		return -EINVAL;

	for (i = 0; i < n; i++) {
		char ch = tok[i];
		unsigned int d;

		if (ch >= '0' && ch <= '9')
			d = ch - '0';
		else if (ch >= 'A' && ch <= 'F')
			d = ch - 'A' + 10;
		else if (ch >= 'a' && ch <= 'f')
			d = ch - 'a' + 10;
		else
			return -EINVAL;

		v = (v << 4) | d;
	}

	*val = v;
	return 0;
}

/* Match a keyword token exactly */
static bool cur_expect(struct fcd52_cfg_cursor *c, const char *word)
{
	const char *tok;
	size_t n = cur_token(c, &tok);

	return n == strlen(word) && !strncmp(tok, word, n);
}

/* Read a "KEYWORD <decimal>" header line */
static int cur_keyword_value(struct fcd52_cfg_cursor *c, const char *word,
			     unsigned int *val)
{
	const char *tok;
	size_t n;
	unsigned int v = 0;
	size_t i;

	if (!cur_expect(c, word))
		return -EINVAL;

	n = cur_token(c, &tok);
	if (!n || n > 6)
		return -EINVAL;

	for (i = 0; i < n; i++) {
		if (tok[i] < '0' || tok[i] > '9')
			return -EINVAL;
		v = v * 10 + (tok[i] - '0');
	}

	*val = v;
	return 0;
}

/*
 * Read "OBP_RAW Vn" as the magic line. Returns false for any version we do
 * not support.
 */
static bool cur_expect_magic(struct fcd52_cfg_cursor *c, unsigned int *version)
{
	const char *tok;
	size_t n;

	if (!cur_expect(c, "OBP_RAW"))
		return false;

	n = cur_token(c, &tok);
	if (n == 2 && tok[0] == 'V' && tok[1] == '1')
		*version = 1;
	else if (n == 2 && tok[0] == 'V' && tok[1] == '4')
		*version = 4;
	else
		return false;

	return true;
}

/*
 * Read one object record header, "TTTT INST SIZE" in hex. Returns -ENODATA at
 * a clean end of text, and -EINVAL for a malformed header.
 */
static int cur_record_header(struct fcd52_cfg_cursor *c, unsigned int *type,
			     unsigned int *inst, unsigned int *size)
{
	int error;

	/* Trailing whitespace after the last record is a clean end of file */
	while (!cur_eof(c) && isspace(c->buf[c->pos]))
		c->pos++;
	if (cur_eof(c))
		return -ENODATA;

	error = cur_hex(c, type);
	if (error)
		return error;
	error = cur_hex(c, inst);
	if (error)
		return error;
	error = cur_hex(c, size);
	if (error)
		return error;

	if (*type > 0xffff || *inst > 0xffff || *size > 0xffff)
		return -EINVAL;

	return 0;
}

/*
 * Walk every record, counting them and the total data bytes, and checking that
 * each record's data is complete. Data bytes are checked but not decoded here.
 */
static int cfg_walk_records(struct fcd52_cfg_cursor *c, struct fcd52_cfg *cfg)
{
	unsigned int n = 0;
	size_t total = 0;

	while (1) {
		unsigned int type, inst, size, i;
		int error;

		error = cur_record_header(c, &type, &inst, &size);
		if (error == -ENODATA)
			break;
		if (error)
			return error;

		for (i = 0; i < size; i++) {
			unsigned int b;

			error = cur_hex(c, &b);
			if (error)
				return error;
			if (b > 0xff)
				return -EINVAL;
		}

		n++;
		total += size;
	}

	cfg->num_records = n;
	cfg->total_bytes = total;
	return 0;
}

/*
 * Parse the header and count the records. Does not allocate: the caller sizes
 * the arrays from num_records and total_bytes, then calls fcd52_cfg_fill().
 */
int fcd52_cfg_parse(struct device *dev, const char *raw, size_t raw_len,
		    struct fcd52_cfg *cfg)
{
	struct fcd52_cfg_cursor c = { raw, raw_len, 0 };
	unsigned int v, i;
	int error;

	memset(cfg, 0, sizeof(*cfg));

	if (!cur_expect_magic(&c, &cfg->version))
		return -EINVAL;

	dev_info(dev, "config file: OBP_RAW V%u\n", cfg->version);

	if (cfg->version == 4) {
		error = cur_keyword_value(&c, "ENCRYPTION", &v);
		if (error)
			return error;
		if (v)
			return -EOPNOTSUPP;	/* encrypted files not supported */

		error = cur_keyword_value(&c, "MAX_ENCRYPTION_BLOCKS", &v);
		if (error)
			return error;

		error = cur_keyword_value(&c, "NO_DEVICES", &v);
		if (error)
			return error;
		if (v != 1)
			return -EOPNOTSUPP;	/* single device only */
	}

	/* Seven bytes of ID information */
	for (i = 0; i < sizeof(cfg->info); i++) {
		error = cur_hex(&c, &v);
		if (error)
			return error;
		if (v > 0xff)
			return -EINVAL;
		cfg->info[i] = v;
	}

	error = cur_hex(&c, &v);
	if (error)
		return error;
	cfg->info_crc = v;

	error = cur_hex(&c, &v);
	if (error)
		return error;
	cfg->config_crc = v;

	/* V4 marks the start of the records with DEVICE_0 */
	if (cfg->version == 4 && !cur_expect(&c, "DEVICE_0"))
		return -EINVAL;

	cfg->records_start = c.pos;

	return cfg_walk_records(&c, cfg);
}

/*
 * Decode the records into caller-allocated arrays. They must hold num_records
 * entries and total_bytes bytes, as reported by fcd52_cfg_parse(). The text
 * was checked by parse, so this cannot fail on a file that already parsed.
 */
int fcd52_cfg_fill(const char *raw, size_t raw_len, struct fcd52_cfg *cfg)
{
	struct fcd52_cfg_cursor c = { raw, raw_len, cfg->records_start };
	size_t off = 0;
	unsigned int r;

	if (!cfg->records || !cfg->blob)
		return -EINVAL;

	for (r = 0; r < cfg->num_records; r++) {
		struct fcd52_cfg_record *rec = &cfg->records[r];
		unsigned int type, inst, size, i;
		int error;

		error = cur_record_header(&c, &type, &inst, &size);
		if (error)
			return error;

		for (i = 0; i < size; i++) {
			unsigned int b;

			error = cur_hex(&c, &b);
			if (error)
				return error;
			cfg->blob[off + i] = (u8)b;
		}

		rec->type = type;
		rec->instance = inst;
		rec->size = size;
		rec->data = cfg->blob + off;
		off += size;
	}

	return 0;
}

/*
 * Objects the device changes at run time. Their values are not stored
 * configuration, so the write path must not overwrite them.
 */
bool fcd52_cfg_record_is_volatile(u16 type)
{
	switch (type) {
	case 2:		/* Encryption status */
	case 5:		/* Message Processor */
	case 6:		/* Command Processor */
	case 37:	/* Diagnostic debug */
	case 44:	/* Message Count */
	case 68:	/* Serial data command */
		return true;
	default:
		return false;
	}
}

/*
 * Decide whether the configuration needs writing. The file's own header is the
 * reference:
 *
 *  - an info block that differs from the device's, or an info CRC that does
 *    not match, means the file was built for another chip or firmware, so it
 *    is refused;
 *  - a configuration CRC equal to the device's, when the device's is non-zero,
 *    means the configuration is already loaded, so the write is skipped.
 *
 * Returns FCD52_CFG_WRITE, FCD52_CFG_SKIP, or a negative errno.
 */
int fcd52_cfg_decide(const struct fcd52_cfg *cfg, const u8 *dev_info,
		     u32 dev_info_crc, u32 dev_cfg_crc)
{
	if (memcmp(cfg->info, dev_info, sizeof(cfg->info)))
		return -EINVAL;

	if (cfg->info_crc != dev_info_crc)
		return -EINVAL;

	if (dev_cfg_crc && cfg->config_crc == dev_cfg_crc)
		return FCD52_CFG_SKIP;

	return FCD52_CFG_WRITE;
}
