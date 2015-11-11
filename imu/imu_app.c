/* Interface with MPU6050 IMU to 
 * collect DMP quaternion data
 * 
 * Code provided is as is,
 * not all functionality has been tested
 *
 * Aadil Rizvi
 */

#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <fcntl.h>
#include <math.h>
#include <util/util.h>
#include <errno.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <imu/imu_app.h>
#include <imu/inv_mpu.h>
#include <imu/inv_mpu_dmp_motion_driver.h>

#define SLAVE_ADDR 0x68
#define IMU_SAMPLE_RATE_HZ 100

#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)

#define COMM_PORT_NUM 9930
#define COMM_SRV_IP "192.168.1.8"

static I32 file;
I8 *filename = "/dev/i2c-1";
double boot_ts_us;

static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};

struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
    struct rx_s rx;
};

typedef struct euler_t {
    F32 roll;
    F32 pitch;
    F32 yaw;
} euler; 

static struct hal_s hal = {0};

I32 s1;

void sig_handler(I32 signo)
{
    if (signo == SIGINT)
        printf("\nreceived SIGINT\n");
    exit(1);
}

void delay_ms(unsigned long num_ms) {
    usleep(num_ms*1000);
}

I32 min(I32 a, I32 b) {
    if (a <= b)
        return a;
    else
        return b;
}

void get_ms(unsigned long *count) {
    struct timeval ts;
    double total_time;
    unsigned long ms_count;

    gettimeofday(&ts, NULL);

    total_time = (double)(ts.tv_sec * 1e6) + (double)ts.tv_usec;
    total_time -= boot_ts_us;    
    ms_count = total_time/1000;

    *count = ms_count;
}

I32 i2c_write(U8 addr, U8 reg, U8 nBytes, U8* data_ptr) 
{
    I32 status;
    U8 buf[nBytes+1];
    U8 i; 

    ASSERT(nBytes >= 1);
    ASSERT(addr == SLAVE_ADDR);
    ASSERT(reg <= 0x75);

    buf[0] = reg;
    for(i=1; i<= nBytes; i++) {
        buf[i] = *data_ptr;
        data_ptr++;
    } 

    status = write(file, buf, i);
    if (status == i)
        return 0;
    else
        return -1;
}

I32 i2c_read(U8 addr, U8 reg, U8 nBytes, U8* data_ptr) 
{
    I32 status;
 
    ASSERT(nBytes >= 1);
    ASSERT(addr == SLAVE_ADDR);
    ASSERT(reg <= 0x75);

    status = write(file, &reg, 1);
    if (status != 1)
        return -1;

    status = read(file, data_ptr, nBytes); 
    if (status == nBytes)
        return 0;
    else
        return -1;
}

U8 read_imu_reg(U8 reg) {
    
    I32 status;
    U8 data;
    U8 buf; 
 
    /*Max IMU reg number check*/
    ASSERT(reg <= 0x75);   

    /*Write reg number over I2C to IMU*/
    buf = reg;
    status = write(file, &buf, 1);
    ASSERT(status == 1);

    /*Read over I2C from IMU*/
    status = read(file, &data, 1);
    ASSERT(status == 1);

    return data;
}

U8 write_imu_reg(U8 reg, U8 val) {

    U8 status;
    U8 buf[2];

    ASSERT(reg <= 0x75);
    ASSERT(val <= 0xFF);

    buf[0] = reg;
    buf[1] = val;

    status = write(file, buf, 2);

    return status;
}

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}


#define EPSILON		0.0001f
#define PI			3.14159265358979323846f
#define PI_2		1.57079632679489661923f

static void quaternionToEuler( const float* quat_wxyz, float* x, float* y, float* z )
{
	float test;
	const struct quat { float w, x, y, z; } *q = ( const struct quat* )quat_wxyz;

	float sqy = q->y * q->y;
	float sqz = q->z * q->z;
	float sqw = q->w * q->w;

	test = q->x * q->z - q->w * q->y;

	if( test > 0.5f - EPSILON )
	{
		*x = 2.f * atan2( q->y, q->w );
		*y = PI_2;
		*z = 0;
	}
	else if( test < -0.5f + EPSILON )
	{
		*x = -2.f * atan2( q->y, q->w );
		*y = -PI_2;
		*z = 0;
	}
	else
	{
		*x = atan2( 2.f * ( q->x * q->w + q->y * q->z ), 1.f - 2.f * ( sqz + sqw ) );
		*y = asin( 2.f * test );
		*z = atan2( 2.f * ( q->x * q->y - q->z * q->w ), 1.f - 2.f * ( sqy + sqz ) );
	}
}


U8 dmpGetGravity(F32 *v, long *q) {
    v[0] = 2 * (q[1]*q[3] - q[0]*q[2]);
    v[1] = 2 * (q[0]*q[1] + q[2]*q[3]);
    v[2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    return 0;
}

U8 dmpGetEuler(float *data, long *q) {
    data[0] = atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);   // psi
    data[1] = -asin(2*q[1]*q[3] + 2*q[0]*q[2]);                              // theta
    data[2] = atan2(2*q[2]*q[3] - 2*q[0]*q[1], 2*q[0]*q[0] + 2*q[3]*q[3] - 1);   // phi
    return 0;
}

U8 dmpGetYawPitchRoll(float *data, long *q, F32 *gravity) {
    // yaw: (about Z axis)
    data[0] = 2.0*atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);

    // pitch: (nose up/down, about Y axis)
    if (gravity[2] > 0) 
       data[1] = atan(gravity[0] / sqrt(gravity[1]*gravity[1] + gravity[2]*gravity[2]));
    else 
       data[1] = -1.0*PI - atan(gravity[0] / sqrt(gravity[1]*gravity[1] + gravity[2]*gravity[2]));
    if (data[1] > 0)
       data[1] = -2.0*PI + data[1];
    if (data[1] < 0)
       data[1] *= -1.0;

    // roll: (tilt left/right, about X axis)
    if (gravity[2] > 0)
       data[2] = atan(gravity[1] / sqrt(gravity[0]*gravity[0] + gravity[2]*gravity[2]));
    else 
       data[2] = -1.0*PI - atan(gravity[1] / sqrt(gravity[0]*gravity[0] + gravity[2]*gravity[2]));
    if (data[2] > 0)
       data[2] = -2.0*PI + data[2];
    if (data[2] < 0)
       data[2] *= -1.0;

    return 0;
}

int main(void) {

    I32 status;
    struct timeval boot_ts;
    static I32 gpio_file;
    fd_set read_fd;
    struct timeval ts;
    U8 val;
    euler angle;
    F32 gravity[3];
    F32 angle_data[3];

    U8 sndBuf[64];
    I32 sndBuf_idx;
    I32 data_i32;
    F32 data_f32;
    U32 data_u32;

    if (signal(SIGINT, sig_handler) == SIG_ERR) {
        printf("\ncan't catch SIG Handler\n");
        exit(1);
    }

    gettimeofday(&boot_ts, NULL);
    boot_ts_us = (double)(boot_ts.tv_sec * 1e6) + (double)boot_ts.tv_usec;

    status = file = open(filename, O_RDWR);
    if (status < 0) {
        /* ERROR HANDLING: you can check errno to see what went wrong */
        perror("Failed to open the i2c bus");
        ASSERT(status >= 0)
    }

    status = ioctl(file, I2C_TENBIT, 0);
    if (status < 0) {
        printf("Failed to set TENBIT addressing to 0\n");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        ASSERT(status >= 0);
    }

    I32 addr = SLAVE_ADDR;          // The I2C address of the IMU
    status = ioctl(file, I2C_SLAVE, addr);
    if (status < 0) {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        ASSERT(status >= 0);
    }

    if( access("/sys/class/gpio/gpio40/value", F_OK ) != -1 ) {
        // file exists
        gpio_file = open("/sys/class/gpio/gpio40/value", O_RDONLY);
        if (gpio_file < 0) {
            /* ERROR HANDLING: you can check errno to see what went wrong */
            perror("Failed to open the gpio file\n");
            ASSERT(gpio_file >= 0)
        }
    } else {
        // file doesn't exist
        printf("GPIO_40 file doesn't exist. Execute \'echo $GPIO > export\' \
                in /sys/class/gpio as root where $GPIO = 40\n");
        exit(1);
    }


    s1 = sock_init(COMM_PORT_NUM, COMM_SRV_IP);
    ASSERT(s1 != -1);
    printf("IMU COMM socket successfully initialized\n");

    status = mpu_init(NULL);
    ASSERT(status == 0);
    
    status = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    ASSERT(status == 0)

    status = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    ASSERT(status == 0);

    status = mpu_set_sample_rate(IMU_SAMPLE_RATE_HZ);
    ASSERT(status == 0);

    /* Initialize HAL state variables. */
    memset(&hal, 0, sizeof(hal));
    hal.sensors = ACCEL_ON | GYRO_ON;
    hal.report = PRINT_QUAT;

    status = dmp_load_motion_driver_firmware();
    ASSERT(status == 0);

    status = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    ASSERT(status == 0);

    //dmp_register_tap_cb(tap_cb);
    //dmp_register_android_orient_cb(android_orient_cb);

    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    status = dmp_enable_feature(hal.dmp_features);
    ASSERT(status == 0);

    status = dmp_set_fifo_rate(IMU_SAMPLE_RATE_HZ);
    ASSERT(status == 0);

    status = mpu_set_dmp_state(1);
    ASSERT(status == 0);

    hal.dmp_on = 1;

    short gyro[3], accel[3], sensors;
    unsigned char more=0;
    long quat[4];
    unsigned long sensor_timestamp=0;
    F32 quat_norm[4];
    F32 norm;

    ts.tv_sec = 0;
    ts.tv_usec = 0;

    while(1) {
        FD_ZERO(&read_fd);
        FD_SET(gpio_file, &read_fd);

        status = select(gpio_file+1, &read_fd, NULL, NULL, &ts);
        if (status > 0) {
            lseek(gpio_file, 0, SEEK_SET);
            status = read(gpio_file, &val, 1);

            if ((status > 0) && (val == '1')) {
                status = dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
                if(status == 0) {
                    //quat[0] >>= 16;
                    //quat[1] >>= 16;
                    //quat[2] >>= 16;
                    //quat[3] >>= 16;

                    norm = sqrt((long long)((long long)quat[0]*(long long)quat[0] +
                                            (long long)quat[1]*(long long)quat[1] + 
                                            (long long)quat[2]*(long long)quat[2] + 
                                            (long long)quat[3]*(long long)quat[3]));

                    quat_norm[0] = quat[0]/norm; 
                    quat_norm[1] = quat[1]/norm; 
                    quat_norm[2] = quat[2]/norm; 
                    quat_norm[3] = quat[3]/norm; 

                    //dmpGetGravity(gravity, quat_norm);      
                    //dmpGetYawPitchRoll(angle_data, quat_norm, gravity); 
                   
                    // Yaw - rotation around z-axis
                    angle_data[0] = atan2(2.0*(quat_norm[0]*quat_norm[3] + quat_norm[1]*quat_norm[2]), 1.0 - 2.0*(quat_norm[2]*quat_norm[2] + quat_norm[3]*quat_norm[3]));

                    // Pitch - rotation around y-axis
                    angle_data[1] = asin(2.0*(quat_norm[0]*quat_norm[2] - quat_norm[3]*quat_norm[1]));

                    // Roll - rotation around x-axis
                    angle_data[2] = atan2(2.0*(quat_norm[0]*quat_norm[1] + quat_norm[2]*quat_norm[3]), 1.0 - 2.0*(quat_norm[1]*quat_norm[1] + quat_norm[2]*quat_norm[2]));

                    sndBuf_idx = 0;

                    data_u32 = 0xDEADBEEF;
                    memcpy(sndBuf+sndBuf_idx, (const char*)&data_u32, sizeof(data_u32));
                    sndBuf_idx += sizeof(data_u32);

                    data_u32 = (U32)sensor_timestamp;
                    memcpy(sndBuf+sndBuf_idx, (const char*)&data_u32, sizeof(data_u32));
                    sndBuf_idx += sizeof(data_u32);

                    data_f32 = angle_data[0];
                    memcpy(sndBuf+sndBuf_idx, (const char*)&data_f32, sizeof(data_f32));
                    sndBuf_idx += sizeof(data_f32);

                    data_f32 = angle_data[1];
                    memcpy(sndBuf+sndBuf_idx, (const char*)&data_f32, sizeof(data_f32));
                    sndBuf_idx += sizeof(data_f32);

                    data_f32 = angle_data[2];
                    memcpy(sndBuf+sndBuf_idx, (const char*)&data_f32, sizeof(data_f32));
                    sndBuf_idx += sizeof(data_f32);

                    data_u32 = 0xDEADBEEF;
                    memcpy(sndBuf+sndBuf_idx, (const char*)&data_u32, sizeof(data_u32));
                    sndBuf_idx += sizeof(data_u32);

                    ASSERT(sndBuf_idx < 64);

                    status = 0;
                    while(status < sndBuf_idx) {
                        status += send_comm_pkt(s1, sndBuf, sndBuf_idx);
                    }

                    angle_data[0] *= 180.0/PI;
                    angle_data[1] *= 180.0/PI;
                    angle_data[2] *= 180.0/PI;

                    //printf("%.5f, %.5f, %.5f, %.5f\n", quat_norm[0], quat_norm[1], quat_norm[2], quat_norm[3]);

                    /*printf("%lu, %.2f, %.2f, %.2f\n", \
                            sensor_timestamp, angle_data[0], \
                            angle_data[1], angle_data[2]);*/
                }
            }
        }
    }

    return 0;
}
