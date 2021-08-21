PROJECT = libquat

ifeq "$(BUILD_DIR)" ""
BUILD_DIR := $(CURDIR)/build
endif

ifeq "$(LIBQUAT_ROOT)" ""
LIBQUAT_ROOT := .
endif

ifeq "$(CFLAGS)" ""
CFLAGS := 
endif

CFLAGS += -lm -I $(LIBQUAT_ROOT)/include

SRCS += euler2quat_f32.c
SRCS += euler2quat_f64.c
SRCS += quat_conjugate_f32.c
SRCS += quat_conjugate_f64.c
SRCS += quat_inverse_f32.c
SRCS += quat_inverse_f64.c
SRCS += quat_mult_f32.c
SRCS += quat_mult_f64.c
SRCS += quat_normlise_f32.c
SRCS += quat_normlise_f64.c
SRCS += quat_rotate_f32.c
SRCS += quat_rotate_f64.c
SRCS += quat2euler_f32.c
SRCS += quat2euler_f64.c
SRCS += quat2rotm_f32.c
SRCS += quat2rotm_f64.c
SRCS += quat2vec_f32.c
SRCS += quat2vec_f64.c
SRCS += rotm2quat_f32.c
SRCS += rotm2quat_f64.c
SRCS += vec2quat_f32.c
SRCS += vec2quat_f64.c

include $(LIBQUAT_ROOT)/../gcc.mk

# Build this as a library
.DEFAULT_GOAL := lib
