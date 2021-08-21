# The build directory
ifeq "${BUILD_DIR}" ""
BUILD_DIR=${CURDIR}/build
endif

# Create output object file names
SRCS_NOPATH := $(foreach NAME,$(SRCS),$(basename $(notdir $(NAME))).c)
SRCXXS_NOPATH := $(foreach NAME,$(SRCXXS),$(basename $(notdir $(NAME))).cpp)
OBJS_NOPATH := $(SRCS_NOPATH:.c=.o) 
OBJS_NOPATH += $(SRCXXS_NOPATH:.cpp=.o)
OBJS        := $(OBJS_NOPATH:%.o=$(BUILD_DIR)/%.o)

# The default goal, which causes the example to be built.
.DEFAULT_GOAL :=
.PHONY: all
all: mkbuildir
all: ${BUILD_DIR}/${PROJECT}

# The goal to build as a library
.PHONY: lib
lib: mkbuildir
lib: ${BUILD_DIR}/${PROJECT}.a

# The goal to create the target directory.
.PHONY: mkbuildir
mkbuildir:
	@mkdir -p ${BUILD_DIR}

.PHONY: clean
clean:
	@rm -rf ${BUILD_DIR} ${wildcard *~}

.PHONY: FORCE
FORCE:

CC = ${PREFIX}-gcc
CXX = ${PREFIX}-g++
AR = ${PREFIX}-ar
LD = ${PREFIX}-gcc

# Add the include file paths to AFLAGS and CFLAGS.

${BUILD_DIR}/${PROJECT}: ${OBJS} ${LIBS}
	@echo ${CXX} -o $@ ${OBJS} ${LIBS} ${CXXFLAGS}
	${CXX} -o $@ ${OBJS} ${LIBS} ${CXXFLAGS}
${BUILD_DIR}/${PROJECT}.a: ${OBJS}

# The rule for building the object file from each C source file.
${BUILD_DIR}/%.o: %.c
	@echo ${CC} -c $^ -o $@ ${CFLAGS}
	${CC} -c $^ -o $@ ${CFLAGS}

# The rule to build an object file from a C++ source file
${BUILD_DIR}/%.o: %.cpp
	@echo ${CXX} -c -o $@ $< ${CXXFLAGS}
	${CXX} -c -o $@ $< ${CXXFLAGS}

# The rule for building the object file from each assembly source file.
${BUILD_DIR}/%.o: %.S
	@echo ${CC} $-c -o $@  $< ${AFLAGS}
	${CC} $-c -o $@  $< ${AFLAGS}

# The rule for creating an object library.
${BUILD_DIR}/%.a:
	@echo ${AR} rcs $@ $^
	${AR} rcs $@ $^

${BUILD_DIR}/%.out:${OBJS}
	@echo ${LD} $^ -o $@
	${LD} $^ -o $@
