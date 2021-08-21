LIBMATRIX_DIR := $(LIBMATRIX_ROOT)

# Specify the build directory if not defined by the project
ifeq "$(BUILD_DIR)" ""
LIBMATRIX_BUILD_DIR=$(CURDIR)/build/libmatrix
else
LIBMATRIX_BUILD_DIR=$(BUILD_DIR)/libmatrix
endif

LIBS += ${LIBMATRIX_BUILD_DIR}/libmatrix.a
AFLAGS+= -I $(LIBMATRIX_ROOT)/include
CFLAGS += -I $(LIBMATRIX_ROOT)/include
CXXFLAGS+= -I $(LIBMATRIX_ROOT)/include

# Add rule to build the Driver Library
${LIBMATRIX_BUILD_DIR}/libmatrix.a: FORCE
	echo $(LIBMATRIX_BUILD_DIR)
	$(MAKE) -C ${LIBMATRIX_DIR} lib BUILD_DIR=${LIBMATRIX_BUILD_DIR}
