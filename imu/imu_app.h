#ifndef IMU_APP_H_
#define IMU_APP_H_

#include <util/util.h>

#define log_i(...)     do {} while (0)
#define log_e(...)     do {} while (0)
#define __no_operation(...)     do {} while (0)

void delay_ms(unsigned long num_ms);
void get_ms(unsigned long *count);
I32 min(I32 a, I32 b);
I32 i2c_write(U8 addr, U8 reg, U8 nBytes, U8* data_ptr); 
I32 i2c_read(U8 addr, U8 reg, U8 nBytes, U8* data_ptr) ;

#endif
