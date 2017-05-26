BASE_TOOLCHAIN=/opt/bbToolChain/usr/local/share/codesourcery
CC=$(BASE_TOOLCHAIN)/bin/arm-none-linux-gnueabi-g++

SRCS=meanshift.cpp main.cpp
OBJS=$(SRCS:%.cpp=%.o)
EXEC=armMeanshiftExec

LDFLAGS=-lpthread -lm --sysroot=/opt/rootfs
LIBS=-lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video \
	-lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_flann

DEFS=-DARMCC
INCLUDES=-I. -I$(BASE_TOOLCHAIN)/include
CFLAGS=$(DEFS) $(INCLUDES)          \
	  -Wall -O3 -Wfatal-errors 		\
	  --sysroot=/opt/rootfs			\
      -mlittle-endian               \
			-mcpu=cortex-a8               \
      -march=armv7-a                 \
      -mtune=cortex-a8               \
      -msoft-float                  \
      -Uarm                         \
      -marm                         \
      -Wno-trigraphs                \
      -fno-strict-aliasing          \
      -fno-common                   \
      -fno-omit-frame-pointer       \
      -mapcs                        \
      -mfloat-abi=softfp            \
      -mfpu=neon                    \
			-funsafe-math-optimizations   \
			-ftree-vectorize              \
			-0time                         \
      -mabi=aapcs-linux

all: clean $(EXEC)


$(EXEC): $(OBJS)
	$(CC) -o $@ $(OBJS) $(LIBS) $(LDFLAGS)

%.o : %.cpp
	$(CC) $(CFLAGS) -c $< -o $@

neon: CFLAGS=-O3 -march=armv7-a -mtune=cortex-a8 -mfpu=neon -ftree-vectorize -mfloat-abi=softfp -funsafe-math-optimizations
neon: clean all

.PHONY: clean all
clean:
	rm -f $(OBJS) $(EXEC) tracking_result.avi *~
