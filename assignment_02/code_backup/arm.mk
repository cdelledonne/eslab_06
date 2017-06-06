BASE_TOOLCHAIN=/opt/bbToolChain/usr/local/share/codesourcery
CC=$(BASE_TOOLCHAIN)/bin/arm-none-linux-gnueabi-g++

SRCS=meanshift.cpp main.cpp
OBJS=$(SRCS:%.cpp=%.o)
EXEC=armMeanshiftExec

LDFLAGS=-lpthread -lm --sysroot=/opt/rootfs
LIBS=-lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video \
	-lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_flann

DEFS=-DARMCC -DTIMING
INCLUDES=-I. -I$(BASE_TOOLCHAIN)/include
CFLAGS=$(DEFS) $(INCLUDES)          \
	  -Wall -O3 -Wfatal-errors 		\
	  --sysroot=/opt/rootfs			\
      -mlittle-endian               \
      -march=armv7-a                \
      -mtune=cortex-a8              \
      -mfloat-abi=softfp            \
      -mfpu=neon                    \
      -ftree-vectorize              \
      -funsafe-math-optimizations   \
      -Uarm                         \
      -marm                         \
      -Wno-trigraphs                \
      -fno-strict-aliasing          \
      -fno-common                   \
      -fno-omit-frame-pointer       \
      -mapcs                        \
      -mabi=aapcs-linux

all: clean $(EXEC)


$(EXEC): $(OBJS)
	$(CC) -o $@ $(OBJS) $(LIBS) $(LDFLAGS)

%.o : %.cpp
	$(CC) $(CFLAGS) -c $< -o $@ 

gprof: CFLAGS=$(DEFS) $(INCLUDES)     \
	  	-Wall -O2 -Wfatal-errors 	  \
	  	--sysroot=/opt/rootfs		  \
      	-mlittle-endian               \
      -march=armv7-a                \
      -mtune=cortex-a8              \
      -mfloat-abi=softfp            \
      -mfpu=neon                    \
      -ftree-vectorize              \
      -funsafe-math-optimizations   \
      	-Uarm                         \
      	-marm                         \
      	-Wno-trigraphs                \
      	-fno-strict-aliasing          \
      	-fno-common                   \
      	-fno-omit-frame-pointer       \
      	-mapcs                        \
      	-mabi=aapcs-linux			  \
      	-g -pg
gprof: LDFLAGS+= -pg
gprof: clean $(EXEC)
	# $(CMD)
	# gprof -b $(EXEC) > gprof.txt
	# $(MCPROF_DIR)/scripts/gprof2pdf.sh gprof.txt

send:
	scp -P 40022 $(EXEC) root@80.112.147.22:/home/root/esLAB/carlo/

.PHONY: clean all
clean:
	rm -f $(OBJS) $(EXEC) tracking_result.avi *~
