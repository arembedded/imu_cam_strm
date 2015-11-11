/* The code provided is as is. 
 * Not all functionality has been tested.
 *
 * Aadil Rizvi
 */

#include <util/util.h>
#include <sys/stat.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define MAX_BYTES_PER_SOCKET_WRITE 65000

struct sockaddr_in si_me;
struct sockaddr_in si_other; 
struct sockaddr_in si_other_out; 
struct sockaddr_in si_msg_out; 
I32 slen=sizeof(si_other);

I32 sock_init(U32 port_num, char* srv_ip) {
   I32 status;
   I32 s1;
   I32 iMode;
   struct timeval ts;
   U32 sndbuf_size;

   ASSERT(srv_ip != NULL);

    if ((s1=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) {
        printf("Failed to initialized COMM socket\n");
        return -1;
    }

   ts.tv_sec = 0;
   ts.tv_usec = 1;

   status = 0;

   /*set POLL timeout to 1 usec*/
   status = setsockopt(s1, SOL_SOCKET, SO_RCVTIMEO,(char *)&ts,sizeof(struct timeval));
   if (status == -1) {
      printf("Failed to set COMM socket read timeout to 1 usec.\n");
      return status;
   }

   memset((char *)&si_me, 0, sizeof(si_me));
   si_me.sin_family = AF_INET;
   si_me.sin_port = htons(port_num);
   si_me.sin_addr.s_addr = htonl(INADDR_ANY);
   status = bind(s1, (struct sockaddr *)&si_me, sizeof(si_me));
   if (status == -1) {
      printf("Failed to bind to socket while initializing COMM socket\n");
      return status;
   }

   /*make socket non-blocking*/ 
   status = ioctl(s1, FIONBIO, &iMode); 
   if (status == -1) {
       printf("Failed to set socket as non-blocking\n");
       return status;
   }

   /*Increase socket buffer size*/
   sndbuf_size = 163840;
   status = setsockopt(s1, SOL_SOCKET, SO_SNDBUF,(char *)&sndbuf_size,sizeof(sndbuf_size));
   if (status == -1) {
      printf("Failed to set socket sndbuf size to 10m bytes.\n");
      ASSERT(status != -1);
   }

   memset((char *)&si_other_out, 0, sizeof(si_other_out));
   si_other_out.sin_family = AF_INET;
   si_other_out.sin_port = htons(port_num);

   status = inet_aton(srv_ip, &si_other_out.sin_addr);
   if (status == 0) {
      printf("inet_aton() failed\n");
      return status;      
   }

   return s1;
}

/*write data passed in as input to COMM socket*/
/*return status of socket write*/
I32 send_comm_pkt(I32 sock_fd, U8* buf_ptr, U32 buf_size) 
{
    ASSERT(buf_ptr != NULL);
    ASSERT(buf_size > 0);
    ASSERT(sock_fd > 0);

    I32 status;
    U32 pkt_size;
     
    status = 0;    

    while(buf_size > 0) {
       if (buf_size >= MAX_BYTES_PER_SOCKET_WRITE) {
          pkt_size = MAX_BYTES_PER_SOCKET_WRITE;
       }
       else {
          pkt_size = buf_size;
       }
       status += sendto(sock_fd, (char *)buf_ptr+status, pkt_size, 0, (struct sockaddr *)&si_other_out, slen);
       if (status == -1) {
          printf("Failed to send TLM data to BIT with errno %d\n", errno);
          ASSERT(status != -1);
          return status;
       }
       buf_size -= pkt_size;
    }

    return status;
}

void assert_internal(const char* file, int line) {
   printf("Hit ASSERT in file %s, line %d. Aborting\n", file, line);
   exit(1);
}

void assert_good(void) {
   return;
}
