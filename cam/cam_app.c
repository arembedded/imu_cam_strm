/*Copyright (C) 2009 Mauro Carvalho Chehab <mchehab@infradead.org>


   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation version 2 of the License.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   11/9/2015
   Modified by Aadil Rizvi to include network transport
   of image data
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <libv4l2.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define REMOTE_IP "192.168.1.8"
#define MSG_PORT 9920 
#define MAX_TF_IMG_PKT_SIZE 50000

struct sockaddr_in s1;
struct sockaddr_in s2;
int sock;

struct buffer {
        void   *start;
        size_t length;
};

static void xioctl(int fh, int request, void *arg)
{
        int r;

        do {
                r = v4l2_ioctl(fh, request, arg);
        } while (r == -1 && ((errno == EINTR) || (errno == EAGAIN)));

        if (r == -1) {
                fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
}

int main(int argc, char **argv)
{
        struct v4l2_format              fmt;
        struct v4l2_buffer              buf;
        struct v4l2_requestbuffers      req;
        enum v4l2_buf_type              type;
        fd_set                          fds;
        struct timeval                  tv;
        int                             r, fd = -1;
        unsigned int                    i, n_buffers;
        char                            *dev_name = "/dev/video0";
        char                            out_name[256];
        FILE                            *fout;
        struct buffer                   *buffers;
        int                             status;
        unsigned int                    buf_size;
        int                             iMode;
        unsigned int                    file_size;
        unsigned char                   out_img_buf[MAX_TF_IMG_PKT_SIZE];
        unsigned int                    img_hdr;
        unsigned int                    img_ftr;

        iMode = 1;
        img_hdr = 0xDEADC0DE;
        img_ftr = 0xC0DEDEAD;

        /*Initialize UDP socket*/
        sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); 
        if (sock == -1) {
            printf("Error creating UDP socket\n");
            return sock;
        } 

        /*Increase send buffer size for socket*/
        buf_size = 64*1024; /* 64 KB */
        status = setsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char*)&buf_size, sizeof(buf_size));
        if (status == -1) {
            printf("Error increasing socket buffer size\n");
            return status;
        } 
       
        /*Bind socket*/
        memset((char *)&s1, 0, sizeof(s1));
        s1.sin_family = AF_INET;
        s1.sin_port = htons(MSG_PORT);
        s1.sin_addr.s_addr = htonl(INADDR_ANY);
        status = bind(sock, (struct sockaddr *)&s1, sizeof(s1));
        if (status == -1) {
            printf("Failed to bind to UDP socket\n");
            return status;
        }
          
        /*Make socket non-blocking*/
        status = ioctl(sock, FIONBIO, &iMode);
        if (status == -1) {
            printf("Failed to set socket as non-blocking\n");
            return status;
        }
       
        memset((char *)&s2, 0, sizeof(s2));
        s2.sin_family = AF_INET;
        s2.sin_port = htons(MSG_PORT);

        status = inet_aton(REMOTE_IP, &s2.sin_addr);
        if (status == 0) {
            printf("Socket initialization failed\n");
            return status;
        }

        

        fd = v4l2_open(dev_name, O_RDWR | O_NONBLOCK, 0);
        if (fd < 0) {
                perror("Cannot open device");
                exit(EXIT_FAILURE);
        }

        CLEAR(fmt);
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = 640;
        fmt.fmt.pix.height      = 480;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
        fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
        xioctl(fd, VIDIOC_S_FMT, &fmt);
        if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB24) {
                printf("Libv4l didn't accept RGB24 format. Can't proceed.\n");
                exit(EXIT_FAILURE);
        }
        if ((fmt.fmt.pix.width != 640) || (fmt.fmt.pix.height != 480))
                printf("Warning: driver is sending image at %dx%d\n",
                        fmt.fmt.pix.width, fmt.fmt.pix.height);

        CLEAR(req);
        req.count = 2;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        xioctl(fd, VIDIOC_REQBUFS, &req);

        buffers = calloc(req.count, sizeof(*buffers));
        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
                CLEAR(buf);

                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_MMAP;
                buf.index       = n_buffers;

                xioctl(fd, VIDIOC_QUERYBUF, &buf);

                buffers[n_buffers].length = buf.length;
                buffers[n_buffers].start = v4l2_mmap(NULL, buf.length,
                              PROT_READ | PROT_WRITE, MAP_SHARED,
                              fd, buf.m.offset);

                if (MAP_FAILED == buffers[n_buffers].start) {
                        perror("mmap");
                        exit(EXIT_FAILURE);
                }
        }

        for (i = 0; i < n_buffers; ++i) {
                CLEAR(buf);
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;
                buf.index = i;
                xioctl(fd, VIDIOC_QBUF, &buf);
        }
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        xioctl(fd, VIDIOC_STREAMON, &type);
        for (i = 0; i < 200; i++) {
                do {
                        FD_ZERO(&fds);
                        FD_SET(fd, &fds);

                        /* Timeout. */
                        tv.tv_sec = 2;
                        tv.tv_usec = 0;

                        r = select(fd + 1, &fds, NULL, NULL, &tv);
                } while ((r == -1 && (errno = EINTR)));
                if (r == -1) {
                        perror("select");
                        return errno;
                }

                CLEAR(buf);
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;
                xioctl(fd, VIDIOC_DQBUF, &buf);

                sprintf(out_name, "out.ppm");
                fout = fopen(out_name, "w");
                if (!fout) {
                        perror("Cannot open image");
                        exit(EXIT_FAILURE);
                }
                fprintf(fout, "P6\n%d %d 255\n",
                        fmt.fmt.pix.width, fmt.fmt.pix.height);
                fwrite(buffers[buf.index].start, buf.bytesused, 1, fout);
                fflush(fout);
                fclose(fout);

                sprintf(out_name, "python img_ppm_2_jpg.py out");
                status = system(out_name);
                if (status < 0) {
                    printf("Error converting PPM img to JPG\n");
                    return status;
                }

                /*Open JPG image and send it over socket*/

                sprintf(out_name, "out.jpg");
                fout = fopen(out_name, "r");
                if (!fout) {
                        perror("Cannot open image");
                        exit(EXIT_FAILURE);
                }

                fseek(fout, 0L, SEEK_END);
                file_size = ftell(fout);
                fseek(fout, 0L, SEEK_SET);
                
                if (file_size > MAX_TF_IMG_PKT_SIZE) {
                    printf("JPG Image File size > %d bytes. Cannot transfer.\n", MAX_TF_IMG_PKT_SIZE);
                    return -1;
                }

                /*Fill header data in img buffer*/
                memcpy(out_img_buf, &img_hdr, sizeof(img_hdr));

                /*Fill img size in img buffer*/
                memcpy(out_img_buf + sizeof(img_hdr), &file_size, sizeof(file_size));

                /*Read JPG image data into buffer*/
                fread(out_img_buf + sizeof(img_hdr) + sizeof(file_size), sizeof(char), file_size, fout);
                fclose(fout);

                /*Fill footer data in img buffer*/
                memcpy(out_img_buf + sizeof(img_hdr) + sizeof(file_size) + file_size, &img_ftr, sizeof(img_ftr));

                /*Send JPG image data from buffer to socket*/
                status = sendto(sock, (char *)out_img_buf, sizeof(img_hdr) + sizeof(file_size) + file_size + sizeof(img_ftr), 0, (struct sockaddr *)&s2, sizeof(s2));
                if (status == -1) {
                    printf("Failed to send .JPG image data to remote IP\n");
                    return status;
                }

                xioctl(fd, VIDIOC_QBUF, &buf);
        }

        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(fd, VIDIOC_STREAMOFF, &type);
        for (i = 0; i < n_buffers; ++i)
                v4l2_munmap(buffers[i].start, buffers[i].length);
        v4l2_close(fd);

        return 0;
}
