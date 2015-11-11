/* Code provided is as is, 
 * not all functionality has been tested.
 *
 * Aadil Rizvi
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <stdio.h>
#include <stdlib.h>

typedef int I32;
typedef unsigned int U32;
typedef short int I16;
typedef unsigned short int U16;
typedef char I8;
typedef unsigned char U8;
typedef float F32;
typedef double F64;

#define ASSERT(test) (test) ? assert_good():assert_internal(__FILE__, __LINE__);

void assert_internal(const char* file, int line);
void assert_good(void);
I32 sock_init(U32 port_num, char* srv_ip);
I32 send_comm_pkt(I32 sock_fd, U8* buf_ptr, U32 buf_size);
 
#endif

