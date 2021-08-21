PROJECT = libmatrix

ifeq "$(BUILD_DIR)" ""
BUILD_DIR := $(CURDIR)/build
endif

ifeq "$(LIBMATRIX_ROOT)" ""
LIBMATRIX_ROOT := .
endif

CFLAGS += -lm -I $(LIBMATRIX_ROOT)/include

SRCS += mat_add_f32.c
SRCS += mat_add_f64.c
SRCS += mat_duplicate_f32.c
SRCS += mat_duplicate_f64.c
SRCS += mat_init_f32.c
SRCS += mat_init_f64.c
SRCS += mat_zeros_f32.c
SRCS += mat_zeros_f64.c
SRCS += mat_identity_f32.c
SRCS += mat_identity_f64.c
SRCS += mat_inverse_f32.c
SRCS += mat_inverse_f64.c
SRCS += mat_mult_f32.c
SRCS += mat_mult_f64.c
SRCS += mat_scale_f32.c
SRCS += mat_scale_f64.c
SRCS += mat_sub_f32.c
SRCS += mat_sub_f64.c
SRCS += mat_trans_f32.c
SRCS += mat_trans_f64.c

include $(LIBMATRIX_ROOT)/../gcc.mk

# Build this as a library
.DEFAULT_GOAL := lib
