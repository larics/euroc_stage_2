// Implementation of the ServerSocket class

#include "euroc_vicon_transmission/server_socket.h"
#include "euroc_vicon_transmission/socket_exception.h"

namespace euroc_vicon_transmission {
ServerSocket::ServerSocket(const int port) {
  if (!Socket::create()) {
    throw SocketException("Could not create server socket.");
  }

  if (!Socket::bind(port)) {
    throw SocketException("Could not bind to port.");
  }
}

ServerSocket::~ServerSocket() {}

const ServerSocket& ServerSocket::operator>>(std::string& data) const {
  if (!receive(&data)) {
    throw SocketException("Could not read from socket.");
  }

  return *this;
}

int ServerSocket::receive(std::string* data) const {
  char buf[kMaxRecvByte + 1];
  memset(buf, 0, kMaxRecvByte + 1);

  sockaddr_in from_addr;
  unsigned int from_addr_len = sizeof(from_addr);
  memset(&from_addr, 0, from_addr_len);

  int status = ::recvfrom(sock_, buf, kMaxRecvByte, 0,
                          (struct sockaddr*)&from_addr, &from_addr_len);

  if (status == -1) {
    std::cerr << "status == -1   errno == " << errno
              << "  in ServerSocket::receive\n";
    return 0;
  } else if (status == 0) {
    *data = "";
    return 0;
  } else {
    *data = std::string(buf, status);
    return status;
  }
}
}  // ns: euroc_vicon_transmission
