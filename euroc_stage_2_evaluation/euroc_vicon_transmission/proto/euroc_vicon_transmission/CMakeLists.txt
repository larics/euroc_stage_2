FIND_PACKAGE(Protobuf REQUIRED)
PROTOBUF_GENERATE_CPP(PROTO_SRC PROTO_HEADER odometry_msg.proto)
CS_ADD_LIBRARY(PROTO_LIB ${PROTO_HEADER} ${PROTO_SRC})
target_link_libraries(PROTO_LIB ${PROTOBUF_LIBRARIES})
