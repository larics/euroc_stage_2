// SocketException class

#ifndef INCLUDE_SOCKET_TEST_SOCKET_EXCEPTION_H_
#define INCLUDE_SOCKET_TEST_SOCKET_EXCEPTION_H_

#include <string>

namespace euroc_vicon_transmission {
class SocketException {
 public:
  SocketException(const std::string& s) : s_(s){};
  ~SocketException(){};

  std::string description() { return s_; }

 private:
  std::string s_;
};
}  // ns: euroc_vicon_transmission

#endif
