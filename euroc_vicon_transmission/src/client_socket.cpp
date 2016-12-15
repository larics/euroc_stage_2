// Implementation of the ClientSocket class

#include "euroc_vicon_transmission/client_socket.h"
#include "euroc_vicon_transmission/socket_exception.h"

namespace euroc_vicon_transmission {
ClientSocket::ClientSocket(const std::string& broadcast_addr, const int port)
    : broadcast_addr_(broadcast_addr), port_(port) {
  if (!Socket::create()) {
    throw SocketException("Could not create client socket.");
  }
}

bool ClientSocket::setSockOpt() {
  int on = 1;
  if (setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, (const char*)&on,
                 sizeof(on)) == -1)
    return false;
  on = 1;
  if (setsockopt(sock_, SOL_SOCKET, SO_BROADCAST, (const char*)&on,
                 sizeof(on)) == -1)
    return false;
  return true;
}

const ClientSocket& ClientSocket::operator<<(const std::string& data) const {
  if (!broadcast(data, broadcast_addr_, port_)) {
    throw SocketException("Could not write to socket.");
  }
  return *this;
}

bool ClientSocket::broadcast(const std::string& data,
                             const std::string& broadcast_addr,
                             const int port) const {
  struct sockaddr_in sock_in;
  unsigned int s_in_len = sizeof(struct sockaddr_in);
  memset(&sock_in, 0, s_in_len);

  sock_in.sin_family = AF_INET;
  sock_in.sin_addr.s_addr = inet_addr(broadcast_addr.c_str());
  sock_in.sin_port = htons(port);

  int status = ::sendto(sock_, data.c_str(), data.size(), 0,
                        (struct sockaddr*)&sock_in, s_in_len);
  if (status == -1) {
    std::cerr << "status == -1   errno == " << errno
              << "  in ClientSocket::broadcast\n";
    return false;
  } else {
    return true;
  }
}
}  // ns: euroc_vicon_transmission
