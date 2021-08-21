LIBQUAT_DIR := $(LIBQUAT_ROOT)

# Specify the build directory if not defined by the project
ifeq "$(BUILD_DIR)" ""
LIBQUAT_BUILD_DIR=$(CURDIR)/build/libquat
else
LIBQUAT_BUILD_DIR=$(BUILD_DIR)/libquat
endif

LIBS += ${LIBQUAT_BUILD_DIR}/libquat.a
AFLAGS+= -I $(LIBQUAT_ROOT)/include
CFLAGS += -I $(LIBQUAT_ROOT)/include
CXXFLAGS+= -I $(LIBQUAT_ROOT)/include

# Add rule to build the Driver Library
${LIBQUAT_BUILD_DIR}/libquat.a: FORCE
	echo $(LIBQUAT_BUILD_DIR)
	$(MAKE) -C ${LIBQUAT_DIR} lib BUILD_DIR=${LIBQUAT_BUILD_DIR}
