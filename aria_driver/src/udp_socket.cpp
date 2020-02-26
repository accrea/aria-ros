/**************************************************
 *
 * Copyright (C) ACCREA 2020, All rights reserved
 * Author: Piotr Jagiełło, Damian Muszyński
 * email: p.jagiello@accrea.com, d.muszynski@accrea.com
 * website: engineering.accrea.com
 *
 *************************************************/

#include "aria_driver/udp_socket.h"

UDPSocket::UDPSocket():
  local_port_(0), remote_port_(0), remote_ip_address_(""), socket_(0)
{
}

UDPSocket::~UDPSocket()
{
//    close UDP socket while destruction object
  this->close();
}

void UDPSocket::configureNetwork(int local_port, std::string remote_ip_address, int remote_port)
{
//    set ports and IP Addres from rosparam server
  local_port_ = local_port;
  remote_ip_address_ = remote_ip_address;
  remote_port_ = remote_port;
  std::cout << "UDP configured:\n" << local_port_ << "\n" << remote_ip_address << "\n" << remote_port_ << std::endl;
}

bool UDPSocket::open(void)
{
  memset(&this->remote_addr_, 0, sizeof(this->remote_addr_));

//  create UDP socket
  socket_ = socket(PF_INET, SOCK_DGRAM, 0);

  if (socket_ < 0)
  {
    perror("socket");
    exit(1);
  }
  /*
  //time out after there was no data for 0.001 seconds.
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 300;
  setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  */
  // IP address of the Arm
  remote_addr_.sin_family = AF_INET;
  if (inet_aton(remote_ip_address_.c_str(), &this->remote_addr_.sin_addr) == 0)
  {
    perror("inet_aton");
    exit(1);
  }
  remote_addr_.sin_port = htons(remote_port_);

  // local address
  struct sockaddr_in local_addr;
  memset(&local_addr, 0, sizeof(local_addr));
  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = INADDR_ANY;
  local_addr.sin_port = htons(local_port_);

  int res = bind(socket_, reinterpret_cast<sockaddr*>(&local_addr), sizeof(local_addr));
  if (res == -1)
  {
    perror("bind");
    exit(1);
  }

  FD_ZERO(&readfds_);
  FD_SET(socket_, &readfds_);
  // blocking timeout
  tv_.tv_sec = 0;
  tv_.tv_usec = 3000;

  return true;
}

uint32_t UDPSocket::crc32_for_byte(uint32_t r) {
    for(int j = 0; j < 8; ++j)
        r = (r & 1? 0: (uint32_t)0xEDB88320L) ^ r >> 1;
    return r ^ (uint32_t)0xFF000000L;
}

void UDPSocket::CRC32_calculate(void *data, uint32_t n_bytes, uint32_t* crc) {
//    calculate cyclic redundancy code
    *crc = 0;
    static uint32_t table[0x100];
    if(!*table)
        for(uint32_t i = 0; i < 0x100; ++i)
            table[i] = crc32_for_byte(i);
    for(uint32_t i = 0; i < n_bytes; ++i)
        *crc = table[(uint8_t)*crc ^ ((uint8_t*)data)[i]] ^ *crc >> 8;
}

void UDPSocket::close(void)
{
  if (socket_ >= 0)
  {
    ::close(socket_);
  }
  socket_ = -1;
  std::cout << "-- Socket closed.\n";
  return;
}

int UDPSocket::sendData(const void *dataFrameToBeSend, int size)
{
  int ret;
  // send packet
  ret = sendto(socket_, const_cast<char *>(reinterpret_cast<const char *>(dataFrameToBeSend)), size, 0,
               (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
  if (ret == -1)
  {
    perror("sendto");
    exit(1);
  }
  // DEBUG
  // printf("sendto: %d\n",ret);
  return ret;
}

int UDPSocket::receiveData(const void *dataFrameToBeReceived, int size)
{
  struct sockaddr_in local_addr;
  socklen_t local_addr_len = sizeof(local_addr);
  int nr_of_recv_bytes = 0;

  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 100000; // 100ms timeout
  FD_ZERO(&readfds_);
  FD_SET(socket_, &readfds_);

  int rv = select(socket_+1, &readfds_, NULL, NULL, &tv);
  if (rv == 0)
  {
      nr_of_recv_bytes = -10; // Data not received - timeout of waiting for new frame
//     printf("No data received!\n");
  }
  else
  {
    // descriptor has data
    if (FD_ISSET(socket_, &readfds_))
    {
        nr_of_recv_bytes = recvfrom(socket_, const_cast<char *>(reinterpret_cast<const char *>(dataFrameToBeReceived)),
                                    size, 0, (struct sockaddr *)&local_addr, &local_addr_len);
//         printf("Recv bytes: %d\n",nr_of_recv_bytes);
    }
  }
  //  nr_of_recv_bytes = recvfrom(socket_, (char *) dataFrameToBeReceived, size, 0,
  //                              (struct sockaddr *)&local_addr, &local_addr_len);
  if (nr_of_recv_bytes == -1)
  {
    perror("recvfrom");
    // exit(1);
  }
  // printf("Recv bytes: %d\n",nr_of_recv_bytes);
  return nr_of_recv_bytes;
}
