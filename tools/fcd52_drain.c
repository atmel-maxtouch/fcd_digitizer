// SPDX-License-Identifier: GPL-2.0-only
/*
 * fcd52_drain - set the ATFCD52 frame-data IIO buffer length and read it
 * continuously, reporting throughput and the driver's loss counters.
 *
 * Usage: fcd52_drain [-l length] [-t seconds] [-o file] [-d N]
 *   -l length   buffer length in scans (frames), default 4096
 *   -t seconds  stop after this many seconds, default run until Ctrl-C
 *   -o file     also write the raw scan data to file
 *   -d N        use /dev/iio:deviceN, default: first device with matrix_xsize
 *
 * The buffer length is in scans, one frame per scan, so 4096 holds 4096 frames.
 *
 * Copyright (c) 2026 Microchip Technology Inc.
 */
#define _GNU_SOURCE
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define SYSFS_IIO	"/sys/bus/iio/devices"
#define READ_SCANS	64		/* scans fetched per read() */
#define PATH_MAX_LEN	512
#define DIR_MAX_LEN	768	/* directory plus a longer suffix, no truncation */

static volatile sig_atomic_t stop_now;
static char dev_dir[DIR_MAX_LEN];
static int dev_fd = -1;

static void on_signal(int sig)
{
	(void)sig;
	stop_now = 1;
}

/* Join a directory and a name into buf; -1 if it would not fit */
static int path_join(char *buf, size_t len, const char *dir, const char *name)
{
	int n = snprintf(buf, len, "%s/%s", dir, name);

	return (n < 0 || (size_t)n >= len) ? -1 : 0;
}

static int sysfs_write(const char *dir, const char *attr, const char *value)
{
	char path[PATH_MAX_LEN];
	FILE *f;

	if (path_join(path, sizeof(path), dir, attr))
		return -1;
	f = fopen(path, "w");
	if (!f) {
		fprintf(stderr, "open %s: %s\n", path, strerror(errno));
		return -1;
	}
	if (fprintf(f, "%s\n", value) < 0 || fclose(f) != 0) {
		fprintf(stderr, "write %s: %s\n", path, strerror(errno));
		return -1;
	}
	return 0;
}

static long sysfs_read_long(const char *dir, const char *attr)
{
	char path[DIR_MAX_LEN];
	char buf[64] = { 0 };
	FILE *f;

	if (path_join(path, sizeof(path), dir, attr))
		return -1;
	f = fopen(path, "r");
	if (!f)
		return -1;
	if (!fgets(buf, sizeof(buf), f)) {
		fclose(f);
		return -1;
	}
	fclose(f);
	return strtol(buf, NULL, 0);
}

/* Find the frame-data device: the IIO device that has a matrix_xsize file */
static int find_device(int wanted, char *out, size_t len)
{
	DIR *d;
	struct dirent *ent;
	char path[PATH_MAX_LEN];

	if (wanted >= 0) {
		snprintf(out, len, "%s/iio:device%d", SYSFS_IIO, wanted);
		return 0;
	}

	d = opendir(SYSFS_IIO);
	if (!d)
		return -1;

	while ((ent = readdir(d))) {
		if (strncmp(ent->d_name, "iio:device", 10))
			continue;
		snprintf(path, sizeof(path), "%s/%s/matrix_xsize",
			 SYSFS_IIO, ent->d_name);
		if (access(path, F_OK) == 0) {
			snprintf(out, len, "%s/%s", SYSFS_IIO, ent->d_name);
			closedir(d);
			return 0;
		}
	}

	closedir(d);
	return -1;
}

/* Enable every channel: turn on each *_en file under scan_elements */
static int enable_channels(const char *dir, int *count)
{
	char scan[DIR_MAX_LEN];
	DIR *d;
	struct dirent *ent;
	int n = 0;

	if (path_join(scan, sizeof(scan), dir, "scan_elements"))
		return -1;
	d = opendir(scan);
	if (!d)
		return -1;

	while ((ent = readdir(d))) {
		size_t l = strlen(ent->d_name);

		if (l < 4 || strcmp(ent->d_name + l - 3, "_en"))
			continue;
		if (sysfs_write(scan, ent->d_name, "1"))
			goto fail;
		n++;
	}

	closedir(d);
	*count = n;
	return 0;

fail:
	closedir(d);
	return -1;
}

static int set_buffer_enable(int on)
{
	char buf[DIR_MAX_LEN];

	if (path_join(buf, sizeof(buf), dev_dir, "buffer"))
		return -1;
	if (sysfs_write(buf, "enable", on ? "1" : "0")) {
		fprintf(stderr, "buffer %s failed\n", on ? "enable" : "disable");
		return -1;
	}
	return 0;
}

int main(int argc, char **argv)
{
	long length = 4096;
	long seconds = 0;
	int wanted_dev = -1;
	int channels = 0;
	const char *outname = NULL;
	FILE *out = NULL;
	char devnode[DIR_MAX_LEN];
	char buf[DIR_MAX_LEN];
	unsigned char *data;
	size_t scan_bytes;
	unsigned long long total = 0;
	unsigned long long reads = 0;
	struct timespec t0, t1;
	int opt;

	while ((opt = getopt(argc, argv, "l:t:o:d:h")) != -1) {
		switch (opt) {
		case 'l':
			length = strtol(optarg, NULL, 0);
			break;
		case 't':
			seconds = strtol(optarg, NULL, 0);
			break;
		case 'o':
			outname = optarg;
			break;
		case 'd':
			wanted_dev = atoi(optarg);
			break;
		default:
			fprintf(stderr,
				"usage: %s [-l length] [-t seconds] [-o file] [-d N]\n",
				argv[0]);
			return 2;
		}
	}

	if (length < 2) {
		fprintf(stderr, "length must be at least 2\n");
		return 2;
	}

	if (find_device(wanted_dev, dev_dir, sizeof(dev_dir))) {
		fprintf(stderr, "no ATFCD52 frame-data device found\n");
		return 1;
	}
	printf("device: %s\n", dev_dir);

	/* Length first: the buffer size is fixed while it is enabled */
	if (path_join(buf, sizeof(buf), dev_dir, "buffer")) {
		fprintf(stderr, "device path too long\n");
		return 1;
	}
	set_buffer_enable(0);
	{
		char l[32];

		snprintf(l, sizeof(l), "%ld", length);
		if (sysfs_write(buf, "length", l)) {
			fprintf(stderr, "could not set buffer length %ld\n", length);
			return 1;
		}
	}
	printf("buffer length: %ld scans\n", length);

	if (enable_channels(dev_dir, &channels) || channels == 0) {
		fprintf(stderr, "could not enable channels\n");
		return 1;
	}

	/* Scan size: one 16-bit sample per enabled channel */
	scan_bytes = (size_t)channels * 2;
	printf("channels: %d, %zu bytes per scan\n", channels, scan_bytes);

	snprintf(devnode, sizeof(devnode), "/dev/%s",
		 strrchr(dev_dir, '/') + 1);
	dev_fd = open(devnode, O_RDONLY);
	if (dev_fd < 0) {
		fprintf(stderr, "open %s: %s\n", devnode, strerror(errno));
		return 1;
	}

	if (outname) {
		out = fopen(outname, "wb");
		if (!out) {
			fprintf(stderr, "open %s: %s\n", outname, strerror(errno));
			goto out_close;
		}
	}

	data = malloc(scan_bytes * READ_SCANS);
	if (!data)
		goto out_close;

	/*
	 * Install the handler without SA_RESTART. signal() restarts a blocked
	 * read() after Ctrl-C, so the loop would never see the stop request while
	 * no frames arrive. Without SA_RESTART the read returns EINTR.
	 */
	{
		struct sigaction sa;

		memset(&sa, 0, sizeof(sa));
		sa.sa_handler = on_signal;
		sigemptyset(&sa.sa_mask);
		sa.sa_flags = 0;
		sigaction(SIGINT, &sa, NULL);
		sigaction(SIGTERM, &sa, NULL);
	}

	set_buffer_enable(1);
	clock_gettime(CLOCK_MONOTONIC, &t0);
	printf("reading %s, Ctrl-C to stop\n", devnode);

	while (!stop_now) {
		struct pollfd pfd = { .fd = dev_fd, .events = POLLIN };
		ssize_t n;
		int ready;

		/*
		 * Wait with a timeout, so the stop request is checked at least every
		 * 200 ms even if no frame arrives and the driver does not return EINTR.
		 */
		ready = poll(&pfd, 1, 200);
		if (ready < 0) {
			if (errno == EINTR)
				continue;
			fprintf(stderr, "poll: %s\n", strerror(errno));
			break;
		}
		if (ready == 0) {
			/* No data this interval: still honour the time limit */
			clock_gettime(CLOCK_MONOTONIC, &t1);
			if (seconds && (t1.tv_sec - t0.tv_sec) >= seconds)
				break;
			continue;
		}

		n = read(dev_fd, data, scan_bytes * READ_SCANS);

		if (n < 0) {
			if (errno == EINTR)
				continue;
			if (errno == EAGAIN)
				continue;
			fprintf(stderr, "read: %s\n", strerror(errno));
			break;
		}
		if (n == 0)
			continue;

		reads++;
		total += (unsigned long long)n;
		if (out)
			fwrite(data, 1, (size_t)n, out);

		clock_gettime(CLOCK_MONOTONIC, &t1);
		if (seconds && (t1.tv_sec - t0.tv_sec) >= seconds)
			break;
	}

	if (set_buffer_enable(0) == 0)
		printf("buffer disabled\n");
	clock_gettime(CLOCK_MONOTONIC, &t1);

	{
		double secs = (t1.tv_sec - t0.tv_sec) +
			      (t1.tv_nsec - t0.tv_nsec) / 1e9;
		unsigned long long frames = total / scan_bytes;
		long overflow = sysfs_read_long(dev_dir, "overflow_frames");
		long dropped = sysfs_read_long(dev_dir, "dropped_frames");

		printf("\n%.1f s, %llu reads, %llu frames, %.1f frames/s, %llu bytes\n",
		       secs, reads, frames,
		       secs > 0 ? frames / secs : 0.0, total);
		printf("overflow_frames: %ld (frames the buffer rejected)\n",
		       overflow);
		printf("dropped_frames:  %ld (frames failing validation)\n",
		       dropped);
	}

	free(data);
	if (out)
		fclose(out);
out_close:
	close(dev_fd);
	return 0;
}
