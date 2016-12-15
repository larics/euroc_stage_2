// Definition of the ServerSocket class

#ifndef INCLUDE_SOCKET_TEST_SERVER_SOCKET_H_
#define INCLUDE_SOCKET_TEST_SERVER_SOCKET_H_

#include "euroc_vicon_transmission/socket.h"

namespace euroc_vicon_transmission {
const int kMaxRecvByte = 1e+5;  // 0.1 MByte

class ServerSocket : private Socket {
 public:
  ServerSocket(const int port);
  ServerSocket(){};
  virtual ~ServerSocket();

  const ServerSocket& operator>>(std::string& data) const;

 private:
  int receive(std::string* data) const;
};
}  // ns: euroc_vicon_transmission

#endif
