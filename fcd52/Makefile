# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for the Microchip ATFCD52 Fast Capacitive Digitizer
#

obj-$(CONFIG_FCD52) += fcd52-core.o
fcd52-core-y := fcd52-core-main.o fcd52-cfg.o

obj-$(CONFIG_FCD52_I2C) += fcd52-i2c.o
obj-$(CONFIG_FCD52_FRAME_DATA) += fcd52-frame-data.o

fcd52-frame-data-y := fcd52-spi.o
