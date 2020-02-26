/**************************************************
 *
 * Copyright (C) ACCREA 2020, All rights reserved
 * Authors: Piotr Jagiełło, Damian Muszyński
 * email: p.jagiello@accrea.com, d.muszynski@accrea.com
 * website: engineering.accrea.com
 *
 *************************************************/

#ifndef UDP_SOCKET_H
#define UDP_SOCKET_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

class UDPSocket {
public:
    UDPSocket(void);

    ~UDPSocket(void);

    void configureNetwork(int local_port, std::string remote_ip_address, int remote_port);

    bool open(void);

    int sendData(const void *dataFrameToBeSend, int size);

    int receiveData(const void *dataFrameToBeReceived, int size);

    uint32_t crc32_for_byte(uint32_t r);

    void CRC32_calculate(void *data, uint32_t n_bytes, uint32_t *crc);

private:
    int local_port_, remote_port_;
    std::string remote_ip_address_;
    int socket_;
    struct sockaddr_in remote_addr_;
    fd_set readfds_;
    struct timeval tv_;

    void close(void);

};


#endif //UDP_SOCKET_H
