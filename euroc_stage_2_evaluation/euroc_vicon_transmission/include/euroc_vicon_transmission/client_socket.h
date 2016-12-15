// Definition of the ClientSocket class

#ifndef INCLUDE_SOCKET_TEST_CLIENT_SOCKET_H_
#define INCLUDE_SOCKET_TEST_CLIENT_SOCKET_H_

#include "euroc_vicon_transmission/socket.h"

namespace euroc_vicon_transmission {
class ClientSocket : private Socket {
 public:
  ClientSocket(){};
  ClientSocket(const std::string& broadcast_addr, const int port);
  virtual ~ClientSocket(){};

  const ClientSocket& operator<<(const std::string& data) const;

 private:
  bool setSockOpt();
  bool broadcast(const std::string& data, const std::string& broadcast_addr,
                 const int port) const;

  std::string broadcast_addr_;
  int port_;
};
}  // ns: euroc_vicon_transmission

#endif
