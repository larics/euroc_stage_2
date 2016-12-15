// Definition of the Socket class

#ifndef INCLUDE_SOCKET_TEST_SOCKET_H_
#define INCLUDE_SOCKET_TEST_SOCKET_H_

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>

namespace euroc_vicon_transmission {
class Socket {
 public:
  Socket();
  virtual ~Socket();

  // Initialization
  bool create();
  bool bind(const int port);

  virtual bool setSockOpt();

  bool is_valid() const { return sock_ != -1; }

 protected:
  int sock_;
  sockaddr_in addr_;
};
}  // ns: euroc_vicon_transmission

#endif
