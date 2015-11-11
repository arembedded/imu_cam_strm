CC=gcc
INC_DIR=/home/arembedded/imu_cam_strm
CFLAGS=-I$(INC_DIR)
MPU=MPU6050

IMU_SRC_FILES=imu/imu_app.c \
          imu/inv_mpu_dmp_motion_driver.c \
          imu/inv_mpu.c \
          util/util.c

CAM_SRC_FILES=cam/cam_app.c

imu: $(IMU_SRC_FILES)
	$(CC) $(IMU_SRC_FILES) -lm -D $(MPU) -D EMPL_TARGET_LINUX $(CFLAGS) -o imu/imu_app.o
cam: $(CAM_SRC_FILES) 
	$(CC) -O2 -Wall `pkg-config --cflags --libs libv4l2` $(CAM_SRC_FILES) $(CFLAGS) -o cam/cam_app.o
all: imu cam

clean:
	rm -rf */*.o

.PHONY: imu cam all clean
