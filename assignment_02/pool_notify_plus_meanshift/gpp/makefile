SHELL = /bin/sh

BASE_TOOLCHAIN := /opt/bbToolChain/usr/local/share/codesourcery/

CC_C    := arm-none-linux-gnueabi-gcc
CC_CPP  := arm-none-linux-gnueabi-g++
DSPLINK := /opt/bbToolChain/usr/local/share/bbframework/platform/beagle-linux/tools/dsplink_linux_1_65_00_03

#   ----------------------------------------------------------------------------
#   General options, sources and libraries
#   ----------------------------------------------------------------------------
SRCS_C   := pool_notify.c
SRCS_CPP := meanshift.cpp main.cpp
LDFLAGS  := -lpthread -lm --sysroot=/opt/rootfs
LIBS     := -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml            \
            -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect \
            -lopencv_contrib -lopencv_legacy -lopencv_flann
BIN      := meanShiftGroup06

#   ----------------------------------------------------------------------------
#   Compiler and Linker flags for Debug
#   ----------------------------------------------------------------------------
#   OBJDIR_D := Debug
#   BINDIR_D := $(OBJDIR_D)
#   LIBS_D := $(DSPLINK)/gpp/BUILD/EXPORT/DEBUG/dsplink.lib $(LIBS)
#   # If the DSP/Link was rebuilt by the user, replace the line above 
#   # with the one below to use the updated libraries
#   #LIBS_D := $(DSPLINK)/gpp/BUILD/EXPORT/DEBUG/dsplink.lib $(LIBS)
#   OBJS_D := $(SRCS:%.c=$(OBJDIR_D)/%.o)
#   ALL_DEBUG := -g -DDDSP_DEBUG $(DEBUG) -D__DEBUG

#   ----------------------------------------------------------------------------
#   Compiler and Linker flags for Release
#   ----------------------------------------------------------------------------
OBJDIR_R := Release
BINDIR_R := $(OBJDIR_R)
#LIBS_R := $(DSPLINK)/gpp/export/BIN/Linux/OMAPL1XX/RELEASE/dsplink.lib $(LIBS)
# If the DSP/Link was rebuilt by the user, replace the line above 
# with the one below to use the updated libraries
LIBS_R     := $(LIBS) $(DSPLINK)/gpp/BUILD/EXPORT/RELEASE/dsplink.lib
OBJS_C_R   := $(SRCS_C:%.c=$(OBJDIR_R)/%.o)
OBJS_CPP_R := $(SRCS_CPP:%.cpp=$(OBJDIR_R)/%.o)

#   ----------------------------------------------------------------------------
#   Compiler include directories 
#   ----------------------------------------------------------------------------
INCLUDES     := -I$(DSPLINK)/gpp/inc                   \
                -I$(DSPLINK)/gpp/inc/usr               \
                -I$(DSPLINK)/gpp/inc/sys/Linux         \
                -I$(DSPLINK)/gpp/inc/sys/Linux/2.6.18  \
                -I$(BASE_TOOLCHAIN)/include            \
                -I./

#   ----------------------------------------------------------------------------
#   All compiler options to be passed to the command line
#   ----------------------------------------------------------------------------
ALL_CFLAGS   :=   $(INCLUDES)                   \
                  -mlittle-endian               \
                  -march=armv7-a                \
                  -mtune=cortex-a8              \
                  -mfloat-abi=softfp            \
                  -Uarm                         \
                  -Wfatal-errors                \
                  -marm -Wall                   \
                  -Wno-trigraphs                \
                  -fno-strict-aliasing          \
                  -fno-common                   \
                  -fno-omit-frame-pointer       \
                  -mapcs                        \
                  -c                            \
                  -mabi=aapcs-linux             \
                  -mfpu=neon                    \
                  -ftree-vectorize              \
                  -funsafe-math-optimizations

ALL_CFLAGS_C :=   $(ALL_CFLAGS)                 \
                  -Wdeclaration-after-statement \
                  -Wstrict-prototypes           \
                  -O3

ALL_CFLAGS_CPP := $(ALL_CFLAGS)                 \
                  --sysroot=/opt/rootfs         \
                  -O3

#   ----------------------------------------------------------------------------
#   Compiler symbol definitions 
#   ----------------------------------------------------------------------------
#	NOTE: remove -DTIMING to suppress timing information on the output
DEFS :=        -DTIMING              \
               -DDSP                 \
               -DDEBUG               \
               -DARMCC               \
               -DOS_LINUX            \
               -DMAX_DSPS=1          \
               -DMAX_PROCESSORS=2    \
               -DID_GPP=1            \
               -DOMAPL1XX            \
               -DPROC_COMPONENT      \
               -DPOOL_COMPONENT      \
               -DNOTIFY_COMPONENT    \
               -DMPCS_COMPONENT      \
               -DRINGIO_COMPONENT    \
               -DMPLIST_COMPONENT    \
               -DMSGQ_COMPONENT      \
               -DMSGQ_ZCPY_LINK      \
               -DCHNL_COMPONENT      \
               -DCHNL_ZCPY_LINK      \
               -DZCPY_LINK           \
               -DKFILE_DEFAULT       \
               -DDA8XXGEM            \
               -DDA8XXGEM_PHYINTERFACE=SHMEM_INTERFACE

#   ----------------------------------------------------------------------------
#   Compiler and Linker procedure
#   From this point and on changes are very unlikely.
#   ----------------------------------------------------------------------------
.PHONY: all
all: Release

#   ----------------------------------------------------------------------------
#   Building Debug... 
#   ----------------------------------------------------------------------------
#	.PHONY: Debug
#	Debug: $(BINDIR_D)/$(BIN)
#
#	$(BINDIR_D)/$(BIN): $(OBJS_D)
#		@echo Compiling Debug...
#		@$(BASE_TOOLCHAIN)/bin/$(CC) -o $@ $(OBJS_D) $(LIBS_D) $(LDFLAGS)
#
#	$(OBJDIR_D)/%.o : %.c
#		@$(BASE_TOOLCHAIN)/bin/$(CC) $(ALL_DEBUG) $(DEFS) $(ALL_CFLAGS) -o $@ $<

#   ----------------------------------------------------------------------------
#   Building Release... 
#   ----------------------------------------------------------------------------
.PHONY: Release
Release: $(BINDIR_R)/$(BIN)

$(BINDIR_R)/$(BIN): $(OBJS_C_R) $(OBJS_CPP_R)
	@echo Compiling Release...
	@$(BASE_TOOLCHAIN)/bin/$(CC_CPP) -o $@ $(OBJS_C_R) $(OBJS_CPP_R) $(LIBS_R) $(LDFLAGS)

# COMPILE C SOURCES
$(OBJDIR_R)/%.o : %.c
	@$(BASE_TOOLCHAIN)/bin/$(CC_C) $(DEFS) $(ALL_CFLAGS_C) -o $@ $<

# COMPILE C++ SOURCES
$(OBJDIR_R)/%.o : %.cpp
	@$(BASE_TOOLCHAIN)/bin/$(CC_CPP) $(DEFS) $(ALL_CFLAGS_CPP) -o $@ $<

.PHONY: clean
clean:
	@rm -f $(OBJDIR_R)/* *~

send: $(BINDIR_R)/$(BIN)
	scp -P 40022 $(BINDIR_R)/$(BIN) root@80.112.147.22:/home/root/esLAB/carlo/
