// Implementation of the Socket class.

#include "euroc_vicon_transmission/socket.h"

namespace euroc_vicon_transmission {
Socket::Socket() : sock_(-1) { memset(&addr_, 0, sizeof(addr_)); }

Socket::~Socket() {
  if (is_valid()) ::close(sock_);
}

bool Socket::create() {
  sock_ = socket(AF_INET, SOCK_DGRAM, 0);

  if (!is_valid()) return false;

  // TIME_WAIT - argh
  int on = 1;
  if (!setSockOpt()) return false;

  return true;
}

bool Socket::setSockOpt() {
  int on = 1;
  if (setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, (const char*)&on,
                 sizeof(on)) == -1)
    return false;
  return true;
}

bool Socket::bind(const int port) {
  if (!is_valid()) {
    return false;
  }

  addr_.sin_family = AF_INET;
  addr_.sin_addr.s_addr = INADDR_ANY;
  addr_.sin_port = htons(port);

  int bind_return = ::bind(sock_, (struct sockaddr*)&addr_, sizeof(addr_));

  if (bind_return == -1) {
    return false;
  }

  return true;
}
}  // ns: euroc_vicon_transmission
