// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Microchip ATFCD52 Fast Capacitive Digitizer - SPI frame data interface
 *
 * The ATFCD52 drives the frame data interface as the SPI host: it asserts SS,
 * generates SCK and shifts sample data out on MOSI. There is no MISO line and
 * no interrupt line, so the host SoC acts as an SPI client and a cyclic DMA
 * transfer drains the receive register continuously.
 *
 * Because capture is continuous and unsolicited rather than message based,
 * this driver programs the SPI block and its DMA channel directly instead of
 * registering as an spi_driver. Everything except the few registers that put
 * the block into client mode is bus agnostic and lives in the generic paths
 * below; the block specific part is isolated in struct fcd52_spi_hw.
 *
 * Copyright (c) 2026 Microchip Technology Inc.
 */

#include <linux/bitops.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/math64.h>
#include <linux/io.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/workqueue.h>
#include <linux/slab.h>

#include "fcd52.h"

/*
 * Consecutive frames that must validate before a candidate frame boundary is
 * accepted. More than one is needed because, with the frame CRC disabled, a
 * mid-frame offset can satisfy the Frame ID check by chance.
 */
#define FCD52_SPI_SYNC_FRAMES		3

/*
 * Frames the cyclic DMA ring holds. The DMA callback runs a few periods behind
 * the engine (measured lag reached 4), and a slot is overwritten once the engine
 * wraps. Sixteen periods leaves a margin of about 11 periods over the observed
 * lag, so a late callback still copies a frame that has not been overwritten.
 */
#define FCD52_SPI_RING_FRAMES		16

/*
 * SAMA7D6 FLEXCOM SPI registers (Data Sheet 74.10, offsets from the SPI block).
 * The block sits at spi@400 under the FLEXCOM, which the FLEXCOM driver puts in
 * SPI mode from atmel,flexcom-mode. The bit positions match the Atmel SPI
 * block, so the MR and CR bit names are unchanged.
 */
#define FLEX_SPI_CR			0x0000
#define FLEX_SPI_CR_SPIEN		BIT(0)
#define FLEX_SPI_CR_SPIDIS		BIT(1)
#define FLEX_SPI_CR_SWRST		BIT(7)
#define FLEX_SPI_CR_RXFCLR		BIT(17)

#define FLEX_SPI_MR			0x0004
#define FLEX_SPI_MR_MSTR		BIT(0)
#define FLEX_SPI_MR_MODFDIS		BIT(4)
#define FLEX_SPI_MR_WDRBT		BIT(5)

#define FLEX_SPI_RDR			0x0008

#define FLEX_SPI_SR			0x0010
#define FLEX_SPI_SR_RDRF		BIT(0)
#define FLEX_SPI_SR_TDRE		BIT(1)
#define FLEX_SPI_SR_TXEMPTY		BIT(9)
#define FLEX_SPI_SR_OVRES		BIT(3)
#define FLEX_SPI_SR_MODF		BIT(4)
#define FLEX_SPI_SR_NSSR		BIT(8)
#define FLEX_SPI_SR_SPIENS		BIT(16)

#define FLEX_SPI_CSR0			0x0030
#define FLEX_SPI_CSR_CPOL		BIT(0)
#define FLEX_SPI_CSR_NCPHA		BIT(1)
#define FLEX_SPI_CSR_BITS_8		0

struct fcd52_spi;

/**
 * struct fcd52_spi_hw - SPI block specific operations
 * @name:	human readable block name
 * @rx_offset:	offset of the receive data register, used as the DMA source
 * @configure:	place the block in client mode, 8-bit, SPI mode 0
 * @enable:	start accepting clocks from the digitizer
 * @disable:	stop the block
 *
 * Only these few operations touch block specific registers. The frame
 * decoding, DMA handling and IIO plumbing are all generic.
 */
struct fcd52_spi_hw {
	const char *name;
	unsigned int rx_offset;
	int (*configure)(struct fcd52_spi *spi);
	void (*enable)(struct fcd52_spi *spi);
	void (*disable)(struct fcd52_spi *spi);
};

/**
 * struct fcd52_spi - frame data interface state
 * @dev:		the platform device
 * @hw:			block specific operations
 * @base:		mapped SPI registers
 * @phys:		physical address of the SPI registers, the DMA source
 * @pclk:		peripheral bus clock
 * @rx_chan:		cyclic DMA channel draining the receive register
 * @ring:		DMA coherent ring of @FCD52_SPI_RING_FRAMES frames
 * @ring_dma:		bus address of @ring
 * @ring_size:		size of @ring in bytes
 * @written:		bytes the DMA engine has produced since capture started
 * @next_frame:		stream position of the next frame to consume
 * @synced:		a frame boundary has been located
 * @layout:		frame geometry, matching the digitizer's T700 settings
 * @frame:		scratch copy of one frame, de-wrapped from the ring
 * @samples:		scan buffer handed to the IIO core
 * @frames:		frames pushed to the buffer
 * @dropped:		frames rejected by validation
 * @resyncs:		times frame alignment had to be recovered
 * @overflows:		frames the IIO buffer rejected because it was full
 * @dma_periods:	DMA period callbacks, one per frame of raw bytes received
 * @last_frame:		copy of the most recent raw frame period, for inspection
 * @raw_buf:		raw DMA periods captured in arrival order, for offline comparison
 * @raw_count:		periods held in @raw_buf for the current capture
 * @id_match_periods:	periods whose first and last byte match, checked before sync
 * @sync_locks:		times frame sync locked onto a boundary
 * @sync_losses:	times a locked boundary was lost to a bad frame
 * @rx_cookie:		DMA cookie of the running cyclic transfer, for residue reads
 * @lag_max:		largest lag, in frame periods, between the engine and our count
 * @lag_overruns:	callbacks where the lag reached the ring depth
 * @status_work:	periodic sampler of the block status while capturing
 * @sampling:		the sampler is running
 * @nssr_edges:		NSS rising edges seen by the sampler
 * @rdrf_seen:		samples in which a received byte was waiting
 */
struct fcd52_spi {
	struct device *dev;
	const struct fcd52_spi_hw *hw;
	void __iomem *base;
	phys_addr_t phys;
	struct clk *pclk;

	struct dma_chan *rx_chan;
	void *ring;
	dma_addr_t ring_dma;
	size_t ring_size;
	u64 written;
	u64 next_frame;
	bool synced;

	struct fcd52_frame_layout layout;
	u8 *frame;
	__le16 *samples;

	unsigned long frames;
	unsigned long dropped;
	unsigned long resyncs;
	unsigned long overflows;
	unsigned long dma_periods;
	unsigned long id_match_periods;
	unsigned long sync_locks;
	unsigned long sync_losses;
	u8 *last_frame;
	u8 *raw_buf;
	unsigned int raw_count;

	/* Periodic sampling of the block status while capturing */
	struct delayed_work status_work;
	bool sampling;
	unsigned long nssr_edges;
	dma_cookie_t rx_cookie;
	unsigned long lag_max;
	unsigned long lag_overruns;
	unsigned long rdrf_seen;
};

/*
 * Reset the block, select client mode and SPI mode 0 with 8-bit words. In
 * client mode the digitizer supplies SS and SCK, so no baud rate or chip
 * select timing needs programming.
 */
static int fcd52_spi_flex_configure(struct fcd52_spi *spi)
{
	writel(FLEX_SPI_CR_SWRST, spi->base + FLEX_SPI_CR);
	writel(FLEX_SPI_CR_SWRST, spi->base + FLEX_SPI_CR);
	writel(FLEX_SPI_CR_RXFCLR, spi->base + FLEX_SPI_CR);

	/*
	 * MSTR cleared selects client mode. MODFDIS is required because the
	 * digitizer drives SS. WDRBT holds a received byte until it has been
	 * read, so DMA cannot miss one.
	 */
	writel(FLEX_SPI_MR_MODFDIS | FLEX_SPI_MR_WDRBT,
	       spi->base + FLEX_SPI_MR);

	/* SPI mode 0: CPOL = 0, CPHA = 0 which is NCPHA set */
	writel(FLEX_SPI_CSR_NCPHA | FLEX_SPI_CSR_BITS_8,
	       spi->base + FLEX_SPI_CSR0);

	if (readl(spi->base + FLEX_SPI_MR) & FLEX_SPI_MR_MSTR) {
		dev_err(spi->dev, "SPI block did not enter client mode\n");
		return -EIO;
	}

	return 0;
}

static void fcd52_spi_flex_enable(struct fcd52_spi *spi)
{
	writel(FLEX_SPI_CR_RXFCLR, spi->base + FLEX_SPI_CR);
	writel(FLEX_SPI_CR_SPIEN, spi->base + FLEX_SPI_CR);
}

static void fcd52_spi_flex_disable(struct fcd52_spi *spi)
{
	writel(FLEX_SPI_CR_SPIDIS, spi->base + FLEX_SPI_CR);
}

static const struct fcd52_spi_hw fcd52_spi_flex_hw = {
	.name = "flexcom",
	.rx_offset = FLEX_SPI_RDR,
	.configure = fcd52_spi_flex_configure,
	.enable = fcd52_spi_flex_enable,
	.disable = fcd52_spi_flex_disable,
};

/* Ring offset of a stream position; div_u64 avoids __aeabi_uldivmod on ARM */
static unsigned int fcd52_spi_ring_offset(const struct fcd52_spi *spi, u64 pos)
{
	u32 rem;

	div_u64_rem(pos, spi->ring_size, &rem);
	return rem;
}

/* Copy a frame out of the ring, wrapping at the end */
static void fcd52_spi_ring_copy(struct fcd52_spi *spi, unsigned int offset,
				u8 *dst, unsigned int len)
{
	unsigned int head = min(len, (unsigned int)spi->ring_size - offset);

	memcpy(dst, spi->ring + offset, head);
	if (len > head)
		memcpy(dst + head, spi->ring, len - head);
}

/*
 * Capture is continuous and begins wherever the digitizer happens to be in
 * its output cycle, so the ring generally does not start on a frame boundary.
 * Search the most recently written frame period for a candidate boundary.
 *
 * A single Frame ID match is weak evidence when CRCs are disabled, since any
 * two equal bytes a frame apart look alike. The candidate is therefore only
 * adopted once the following frame validates as well, which the caller
 * arranges by deferring acceptance to the next period.
 */
static bool fcd52_spi_find_sync(struct fcd52_spi *spi, u64 base,
				unsigned int span)
{
	unsigned int frame_size = spi->layout.frame_size;
	unsigned int i, n;

	for (i = 0; i < span; i++) {
		u64 pos = base + i;

		/*
		 * Require consecutive frames from the candidate to validate.
		 * Without a CRC the only check is that the bracketing Frame
		 * ID bytes match, which a mid-frame offset can satisfy by
		 * chance; it is very unlikely to do so repeatedly.
		 */
		for (n = 0; n < FCD52_SPI_SYNC_FRAMES; n++) {
			u64 at = pos + (u64)n * frame_size;

			if (at + frame_size > spi->written)
				break;

			fcd52_spi_ring_copy(spi, fcd52_spi_ring_offset(spi, at),
					    spi->frame, frame_size);
			if (fcd52_frame_validate(&spi->layout, spi->frame))
				break;
		}

		if (n < FCD52_SPI_SYNC_FRAMES)
			continue;

		spi->next_frame = pos;
		dev_dbg(spi->dev, "Frame sync at stream position %llu\n", pos);
		return true;
	}

	return false;
}

/* Log the block's receive state, so the kernel says whether it sees SS and data */
static void fcd52_spi_log_status(struct fcd52_spi *spi, const char *when)
{
	u32 sr = readl(spi->base + FLEX_SPI_SR);

	dev_info(spi->dev,
		 "%s: SR=0x%08x SPIENS=%u NSSR=%u RDRF=%u OVRES=%u MODF=%u\n",
		 when, sr,
		 !!(sr & FLEX_SPI_SR_SPIENS), !!(sr & FLEX_SPI_SR_NSSR),
		 !!(sr & FLEX_SPI_SR_RDRF), !!(sr & FLEX_SPI_SR_OVRES),
		 !!(sr & FLEX_SPI_SR_MODF));
}

#define FCD52_SPI_STATUS_MS	100

/* Raw DMA periods kept for offline comparison: 4096 periods (about 20 s) */
#define FCD52_SPI_RAW_PERIODS	4096

/*
 * Sample the block status while capturing. NSSR and RDRF are cleared by
 * reading, so the sampler takes every reading itself and counts the NSSR
 * rising edges and RDRF events it sees.
 */
static void fcd52_spi_status_poll(struct work_struct *work)
{
	struct fcd52_spi *spi = container_of(to_delayed_work(work),
					     struct fcd52_spi, status_work);
	u32 sr = readl(spi->base + FLEX_SPI_SR);

	if (sr & FLEX_SPI_SR_NSSR)
		spi->nssr_edges++;
	if (sr & FLEX_SPI_SR_RDRF)
		spi->rdrf_seen++;

	if (sr & (FLEX_SPI_SR_OVRES | FLEX_SPI_SR_MODF))
		dev_warn_ratelimited(spi->dev,
				     "block error SR=0x%08x (OVRES %u MODF %u)\n",
				     sr, !!(sr & FLEX_SPI_SR_OVRES),
				     !!(sr & FLEX_SPI_SR_MODF));

	if (spi->sampling)
		schedule_delayed_work(&spi->status_work,
				      msecs_to_jiffies(FCD52_SPI_STATUS_MS));
}

/*
 * Called once per frame period by the DMA engine. The ring keeps filling
 * while this runs, which is what makes the capture gapless.
 *
 * Positions are tracked as monotonic byte counts since capture started and
 * reduced modulo the ring size only when the ring is actually addressed.
 * That keeps the "has this been written yet" test a plain comparison:
 * @written is how many bytes the engine has produced, and a frame starting
 * at @next_frame is complete once @next_frame + frame_size <= @written.
 */
static void fcd52_spi_dma_complete(void *param)
{
	struct iio_dev *indio_dev = param;
	struct fcd52_spi *spi = iio_priv(indio_dev);
	unsigned int frame_size = spi->layout.frame_size;
	int error;

	/* One more period has been filled */
	spi->written += frame_size;

	/*
	 * Measure how far the driver's count trails the engine. The residue is
	 * the bytes left in the current cyclic period, so the engine's position
	 * is the ring size minus the residue. The lag is in whole frame periods,
	 * from the byte position the engine is at versus the one our count
	 * assumes.
	 */
	{
		struct dma_tx_state state;
		enum dma_status status;
		u64 engine_pos;
		u64 our_pos;
		u32 lag;

		u32 rem;

		status = dmaengine_tx_status(spi->rx_chan, spi->rx_cookie, &state);
		if (status != DMA_ERROR) {
			engine_pos = (u64)spi->ring_size - state.residue;
			div_u64_rem(spi->written, spi->ring_size, &rem);
			our_pos = rem;
			div_u64_rem(engine_pos + spi->ring_size - our_pos,
				    spi->ring_size, &rem);
			lag = rem / frame_size;
			if (lag > spi->lag_max)
				spi->lag_max = lag;
			if (lag >= FCD52_SPI_RING_FRAMES)
				spi->lag_overruns++;
		}
	}

	/*
	 * Count every period, synced or not, and keep the raw bytes of the period
	 * that just completed. This is independent of frame validation, so it shows
	 * what reached the SPI block.
	 */
	spi->dma_periods++;
	fcd52_spi_ring_copy(spi, fcd52_spi_ring_offset(spi, spi->written - frame_size),
			    spi->last_frame, frame_size);
	if (spi->raw_count < FCD52_SPI_RAW_PERIODS) {
		memcpy(spi->raw_buf + (size_t)spi->raw_count * frame_size,
		       spi->last_frame, frame_size);
		spi->raw_count++;
	}
	if (spi->last_frame[0] == spi->last_frame[frame_size - FCD52_FRAME_ID_SIZE])
		spi->id_match_periods++;

	if (!spi->synced) {
		unsigned int window = (FCD52_SPI_SYNC_FRAMES + 1) * frame_size;

		/*
		 * A candidate boundary can lie anywhere in one frame, and
		 * confirming it needs FCD52_SPI_SYNC_FRAMES frames beyond
		 * that, so wait until the engine has produced both.
		 */
		if (spi->written < window)
			return;

		if (!fcd52_spi_find_sync(spi, spi->written - window,
					 frame_size))
			return;

		spi->synced = true;
		spi->sync_locks++;
		dev_info(spi->dev, "frame sync locked at stream position %llu\n",
			 spi->next_frame);
	}

	/* Never read ahead of the engine */
	while (spi->next_frame + frame_size <= spi->written) {
		fcd52_spi_ring_copy(spi, fcd52_spi_ring_offset(spi, spi->next_frame),
				    spi->frame, frame_size);

		error = fcd52_frame_validate(&spi->layout, spi->frame);
		if (error) {
			spi->dropped++;
			spi->synced = false;
			spi->sync_losses++;
			spi->resyncs++;
			dev_dbg_ratelimited(spi->dev,
					    "Frame rejected (%d), resyncing\n",
					    error);
			return;
		}

		spi->next_frame += frame_size;

		/*
		 * Samples are already signed 16-bit little endian on the
		 * wire, which is what the channels advertise, so copy them
		 * across unmodified.
		 */
		memcpy(spi->samples, spi->frame + spi->layout.data_offset,
		       spi->layout.nodes * 2);

		spi->frames++;
		/*
		 * A full buffer rejects the frame, so count it. Without this the
		 * loss is silent: dropped counts only validation failures.
		 */
		if (iio_push_to_buffers(indio_dev, spi->samples) < 0)
			spi->overflows++;
	}
}

static int fcd52_spi_start_capture(struct iio_dev *indio_dev)
{
	struct fcd52_spi *spi = iio_priv(indio_dev);
	struct dma_async_tx_descriptor *desc;
	struct dma_slave_config cfg = { };
	dma_cookie_t cookie;
	int error;

	cfg.direction = DMA_DEV_TO_MEM;
	cfg.src_addr = spi->phys + spi->hw->rx_offset;
	cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	cfg.src_maxburst = 1;
	cfg.dst_maxburst = 1;
	cfg.device_fc = false;

	error = dmaengine_slave_config(spi->rx_chan, &cfg);
	if (error)
		return error;

	/*
	 * One cyclic period per frame, so the callback runs with a whole
	 * frame available and the next frame lands in a different slot.
	 */
	desc = dmaengine_prep_dma_cyclic(spi->rx_chan, spi->ring_dma,
					 spi->ring_size,
					 spi->layout.frame_size,
					 DMA_DEV_TO_MEM,
					 DMA_PREP_INTERRUPT);
	if (!desc)
		return -ENOMEM;

	desc->callback = fcd52_spi_dma_complete;
	desc->callback_param = indio_dev;

	spi->written = 0;
	spi->next_frame = 0;
	spi->dma_periods = 0;
	spi->raw_count = 0;
	spi->lag_max = 0;
	spi->lag_overruns = 0;
	spi->id_match_periods = 0;
	spi->sync_locks = 0;
	spi->sync_losses = 0;
	spi->synced = false;

	cookie = dmaengine_submit(desc);
	spi->rx_cookie = cookie;
	error = dma_submit_error(cookie);
	if (error)
		return error;

	dma_async_issue_pending(spi->rx_chan);

	spi->hw->enable(spi);
	fcd52_spi_log_status(spi, "capture started");

	spi->nssr_edges = 0;
	spi->rdrf_seen = 0;
	spi->sampling = true;
	schedule_delayed_work(&spi->status_work,
			      msecs_to_jiffies(FCD52_SPI_STATUS_MS));

	return 0;
}

static int fcd52_spi_stop_capture(struct iio_dev *indio_dev)
{
	struct fcd52_spi *spi = iio_priv(indio_dev);

	/* Reached on every buffer disable; marks the stop path for the log */
	dev_info(spi->dev, "stop_capture entered\n");

	spi->sampling = false;
	cancel_delayed_work_sync(&spi->status_work);

	spi->hw->disable(spi);
	dmaengine_terminate_sync(spi->rx_chan);

	dev_info(spi->dev, "capture stopped: NSSR edges %lu, RDRF seen %lu\n",
		 spi->nssr_edges, spi->rdrf_seen);

	dev_dbg(spi->dev, "Capture stopped: %lu frames, %lu dropped, %lu overflowed\n",
		spi->frames, spi->dropped, spi->overflows);

	return 0;
}

static const struct iio_buffer_setup_ops fcd52_spi_setup_ops = {
	.postenable = fcd52_spi_start_capture,
	.predisable = fcd52_spi_stop_capture,
};

static ssize_t matrix_xsize_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct fcd52_spi *spi = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%u\n", spi->layout.xsize);
}
static DEVICE_ATTR_RO(matrix_xsize);

static ssize_t matrix_ysize_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct fcd52_spi *spi = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%u\n", spi->layout.ysize);
}
static DEVICE_ATTR_RO(matrix_ysize);

static ssize_t dropped_frames_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct fcd52_spi *spi = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%lu\n", spi->dropped);
}
static DEVICE_ATTR_RO(dropped_frames);

static ssize_t overflow_frames_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct fcd52_spi *spi = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%lu\n", spi->overflows);
}
static DEVICE_ATTR_RO(overflow_frames);

static ssize_t dma_periods_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fcd52_spi *spi = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%lu\n", spi->dma_periods);
}
static DEVICE_ATTR_RO(dma_periods);

static ssize_t id_match_periods_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct fcd52_spi *spi = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%lu\n", spi->id_match_periods);
}
static DEVICE_ATTR_RO(id_match_periods);

static ssize_t sync_locks_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct fcd52_spi *spi = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%lu\n", spi->sync_locks);
}
static DEVICE_ATTR_RO(sync_locks);

static ssize_t sync_losses_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fcd52_spi *spi = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%lu\n", spi->sync_losses);
}
static DEVICE_ATTR_RO(sync_losses);

static ssize_t lag_max_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct fcd52_spi *spi = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%lu\n", spi->lag_max);
}
static DEVICE_ATTR_RO(lag_max);

static ssize_t lag_overruns_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct fcd52_spi *spi = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%lu\n", spi->lag_overruns);
}
static DEVICE_ATTR_RO(lag_overruns);

/* Hex dump of the last raw frame period, 16 bytes per line */
static ssize_t last_frame_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct fcd52_spi *spi = iio_priv(dev_to_iio_dev(dev));
	ssize_t len = 0;
	unsigned int i;

	for (i = 0; i < spi->layout.frame_size; i++) {
		if (i && !(i % 16))
			len += sysfs_emit_at(buf, len, "\n");
		len += sysfs_emit_at(buf, len, "%02x ", spi->last_frame[i]);
	}
	len += sysfs_emit_at(buf, len, "\n");

	return len;
}
static DEVICE_ATTR_RO(last_frame);

/* Binary read of the captured raw periods, in arrival order, one frame each */
static ssize_t raw_capture_read(struct file *filp, struct kobject *kobj,
				struct bin_attribute *attr, char *buf,
				loff_t off, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct fcd52_spi *spi = iio_priv(dev_to_iio_dev(dev));
	size_t total = (size_t)spi->raw_count * spi->layout.frame_size;

	if (off >= total)
		return 0;
	count = min(count, total - (size_t)off);
	memcpy(buf, spi->raw_buf + off, count);

	return count;
}

static struct bin_attribute fcd52_raw_capture_attr = {
	.attr = { .name = "raw_capture", .mode = 0444 },
	.read = raw_capture_read,
};

/*
 * The same periods as text, one byte per line in the 0xNN form of the Saleae
 * decode, so the two files can be compared line by line. Each byte is five
 * characters ("0xNN\n"), and the output is generated from the offset, so a
 * large capture reads in pieces without a full copy.
 */
#define FCD52_RAW_HEX_CHARS	5

static ssize_t raw_capture_hex_read(struct file *filp, struct kobject *kobj,
				    struct bin_attribute *attr, char *buf,
				    loff_t off, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct fcd52_spi *spi = iio_priv(dev_to_iio_dev(dev));
	static const char hex[] = "0123456789ABCDEF";
	size_t bytes = (size_t)spi->raw_count * spi->layout.frame_size;
	size_t total = bytes * FCD52_RAW_HEX_CHARS;
	size_t i;

	if ((size_t)off >= total)
		return 0;
	count = min(count, total - (size_t)off);

	for (i = 0; i < count; i++) {
		size_t pos = (size_t)off + i;
		size_t byte = pos / FCD52_RAW_HEX_CHARS;
		u8 v = spi->raw_buf[byte];

		switch (pos % FCD52_RAW_HEX_CHARS) {
		case 0:
			buf[i] = '0';
			break;
		case 1:
			buf[i] = 'x';
			break;
		case 2:
			buf[i] = hex[v >> 4];
			break;
		case 3:
			buf[i] = hex[v & 0xf];
			break;
		default:
			buf[i] = '\n';
			break;
		}
	}

	return count;
}

static struct bin_attribute fcd52_raw_capture_hex_attr = {
	.attr = { .name = "raw_capture_hex", .mode = 0444 },
	.read = raw_capture_hex_read,
};

static struct attribute *fcd52_spi_attrs[] = {
	&dev_attr_matrix_xsize.attr,
	&dev_attr_matrix_ysize.attr,
	&dev_attr_dropped_frames.attr,
	&dev_attr_overflow_frames.attr,
	&dev_attr_dma_periods.attr,
	&dev_attr_id_match_periods.attr,
	&dev_attr_sync_locks.attr,
	&dev_attr_sync_losses.attr,
	&dev_attr_lag_max.attr,
	&dev_attr_lag_overruns.attr,
	&dev_attr_last_frame.attr,
	NULL
};

static struct bin_attribute *fcd52_spi_bin_attrs[] = {
	&fcd52_raw_capture_attr,
	&fcd52_raw_capture_hex_attr,
	NULL
};

static const struct attribute_group fcd52_spi_attr_group = {
	.attrs = fcd52_spi_attrs,
	.bin_attrs = fcd52_spi_bin_attrs,
};

static const struct iio_info fcd52_spi_info = {
	.attrs = &fcd52_spi_attr_group,
};

/*
 * Find the digitizer's control driver through microchip,control and read the
 * T700 frame layout from it. The control driver must be bound first, so when
 * it is not, return -EPROBE_DEFER and let the probe retry.
 */
static int fcd52_spi_read_layout(struct fcd52_spi *spi)
{
	struct device *dev = spi->dev;
	struct device_node *np;
	struct i2c_client *client;
	struct iio_dev *indio_dev;
	struct fcd52_data *data;
	int error;

	np = of_parse_phandle(dev->of_node, "microchip,control", 0);
	if (!np)
		return dev_err_probe(dev, -ENODEV,
				     "Missing microchip,control phandle\n");

	client = of_find_i2c_device_by_node(np);
	of_node_put(np);
	if (!client)
		return -EPROBE_DEFER;

	indio_dev = i2c_get_clientdata(client);
	if (!indio_dev) {
		put_device(&client->dev);
		return -EPROBE_DEFER;
	}

	data = iio_priv(indio_dev);
	error = fcd52_read_frame_layout(data, &spi->layout);
	put_device(&client->dev);

	if (error)
		return dev_err_probe(dev, error,
				     "Failed to read T700 frame layout\n");

	dev_info(dev, "T700 layout %ux%u, %u bytes/frame\n",
		 spi->layout.xsize, spi->layout.ysize, spi->layout.frame_size);
	return 0;
}

/* One channel per sensor node, numbered along the device X lines */
static int fcd52_spi_alloc_channels(struct iio_dev *indio_dev,
				    const struct fcd52_frame_layout *layout)
{
	struct iio_chan_spec *channels;
	unsigned int i;

	channels = devm_kcalloc(&indio_dev->dev, layout->nodes,
				sizeof(*channels), GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	for (i = 0; i < layout->nodes; i++) {
		channels[i].type = IIO_CAPACITANCE;
		channels[i].indexed = 1;
		channels[i].channel = i;
		channels[i].scan_index = i;
		channels[i].scan_type.sign = 's';
		channels[i].scan_type.realbits = 16;
		channels[i].scan_type.storagebits = 16;
		channels[i].scan_type.endianness = IIO_LE;
	}

	indio_dev->channels = channels;
	indio_dev->num_channels = layout->nodes;

	return 0;
}

static void fcd52_spi_release_chan(void *p)
{
	struct fcd52_spi *spi = p;

	dma_release_channel(spi->rx_chan);
}

static void fcd52_spi_free_ring(void *p)
{
	struct fcd52_spi *spi = p;

	dma_free_coherent(spi->rx_chan->device->dev, spi->ring_size,
			  spi->ring, spi->ring_dma);
}

static int fcd52_spi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct fcd52_spi *spi;
	struct resource *res;
	int error;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*spi));
	if (!indio_dev)
		return -ENOMEM;

	spi = iio_priv(indio_dev);
	spi->dev = dev;
	spi->hw = device_get_match_data(dev);
	if (!spi->hw)
		return -ENODEV;
	INIT_DELAYED_WORK(&spi->status_work, fcd52_spi_status_poll);

	/* The DMA engine needs the physical address of the receive register */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	spi->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(spi->base))
		return PTR_ERR(spi->base);
	spi->phys = res->start;

	spi->pclk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(spi->pclk))
		return dev_err_probe(dev, PTR_ERR(spi->pclk),
				     "Failed to get peripheral clock\n");

	/*
	 * The geometry is the digitizer's own T700 configuration, read by the
	 * control driver. Taking it from there keeps the frame layout in step with
	 * the device; a separate property could silently disagree.
	 */
	error = fcd52_spi_read_layout(spi);
	if (error)
		return error;

	if (spi->layout.nodes * 2 > FCD52_FRAME_MAX_DATA)
		return dev_err_probe(dev, -ERANGE,
				     "%u nodes exceed the frame limit\n",
				     spi->layout.nodes);

	spi->rx_chan = dma_request_chan(dev, "rx");
	if (IS_ERR(spi->rx_chan))
		return dev_err_probe(dev, PTR_ERR(spi->rx_chan),
				     "Failed to get RX DMA channel\n");

	/* Release the channel with the device, so a re-probe can request it again */
	error = devm_add_action_or_reset(dev, fcd52_spi_release_chan, spi);
	if (error)
		return error;

	spi->ring_size = (size_t)spi->layout.frame_size *
			 FCD52_SPI_RING_FRAMES;
	spi->ring = dma_alloc_coherent(spi->rx_chan->device->dev,
				       spi->ring_size, &spi->ring_dma,
				       GFP_KERNEL);
	if (!spi->ring)
		return -ENOMEM;

	error = devm_add_action_or_reset(dev, fcd52_spi_free_ring, spi);
	if (error)
		return error;

	spi->frame = devm_kzalloc(dev, spi->layout.frame_size, GFP_KERNEL);
	if (!spi->frame)
		return -ENOMEM;

	fcd52_raw_capture_attr.size = (size_t)spi->layout.frame_size *
				      FCD52_SPI_RAW_PERIODS;
	fcd52_raw_capture_hex_attr.size = fcd52_raw_capture_attr.size *
					  FCD52_RAW_HEX_CHARS;
	spi->last_frame = devm_kzalloc(dev, spi->layout.frame_size, GFP_KERNEL);
	if (!spi->last_frame)
		return -ENOMEM;

	spi->raw_buf = devm_kmalloc(dev,
				    (size_t)spi->layout.frame_size *
				    FCD52_SPI_RAW_PERIODS, GFP_KERNEL);
	if (!spi->raw_buf)
		return -ENOMEM;

	spi->samples = devm_kcalloc(dev, spi->layout.nodes,
				    sizeof(*spi->samples), GFP_KERNEL);
	if (!spi->samples)
		return -ENOMEM;

	error = spi->hw->configure(spi);
	if (error)
		return error;

	error = fcd52_spi_alloc_channels(indio_dev, &spi->layout);
	if (error)
		return error;

	indio_dev->name = "atfcd52";
	indio_dev->modes = INDIO_BUFFER_SOFTWARE;
	indio_dev->info = &fcd52_spi_info;

	error = devm_iio_kfifo_buffer_setup(dev, indio_dev,
					    &fcd52_spi_setup_ops);
	if (error)
		return dev_err_probe(dev, error, "Failed to set up buffer\n");

	dev_info(dev, "%s SPI client, %ux%u nodes, %u bytes/frame\n",
		 spi->hw->name, spi->layout.xsize, spi->layout.ysize,
		 spi->layout.frame_size);

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id fcd52_spi_of_match[] = {
	{ .compatible = "microchip,atfcd52-frame-data",
	  .data = &fcd52_spi_flex_hw },
	{ }
};
MODULE_DEVICE_TABLE(of, fcd52_spi_of_match);

static struct platform_driver fcd52_spi_driver = {
	.driver = {
		.name = "atfcd52-frame-data",
		.of_match_table = fcd52_spi_of_match,
	},
	.probe = fcd52_spi_probe,
};
module_platform_driver(fcd52_spi_driver);

MODULE_AUTHOR("Michael Gong <michael.gong@microchip.com>");
MODULE_DESCRIPTION("Microchip ATFCD52 SPI frame data interface");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_FCD52);
