import sys
import os
import socket
import struct

UDP_IP = "192.168.1.8"
UDP_PORT = 9920

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
img_num = 0

while(1):
    #Read IMG HDR
    data = sock.recv(1000000)
    hdr = struct.unpack('<I', data[:4])

    if (hdr[0] == 0xDEADC0DE):
        size = struct.unpack('<I', data[4:8])
        print str(size[0])

        file_name = 'img_' + str(img_num) + '.jpg'
        fp = open(file_name, 'wb')

                
        fp.write(data[8:8+size[0]])
        fp.close()

        ftr = struct.unpack('<I', data[8+size[0]:8+size[0]+4])

        if (ftr[0] != 0xC0DEDEAD):
           print 'Failed to find footer'
           exit(1)
        
        img_num += 1 
