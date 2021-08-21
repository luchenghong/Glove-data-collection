PROJECT := test_kalman
PREFIX = 
#for x86_x64
#CFLAGS := -lm
#for Raspberry pi
CFLAGS ?= -lwiringPi -lpthread -Woverflow -lm -O2 -pipe -mfpu=vfp -mfloat-abi=hard 
CXXFLAGS = -I. -fpermissive -lstdc++
CXXFLAGS += $(CFLAGS) 
BUILD_DIR := $(CURDIR)/build

LIBMATRIX_ROOT := $(CURDIR)/libmatrix
LIBQUAT_ROOT := $(CURDIR)/libquat

# Source files (add path to VPATH below)
SRCS += sensor.c
SRCS += fast_kalman_filter.c
SRCS += icm20948.c
SRCS +=	spi_if.c
#SRCS += test_kalman.c

SRCXXS += test_kalman.cpp
LIBMATRIX_DIR := $(LIBMATRIX_ROOT)
include $(LIBMATRIX_ROOT)/libmatrix.mk

LIBQUAT_DIR := $(LIBQUAT_ROOT)
include $(LIBQUAT_ROOT)/libquat.mk

include gcc.mk

distclean: clean
	$(MAKE) -C ${LIBMATRIX_DIR} clean
	$(MAKE) -C ${LIBQUAT_DIR} clean
