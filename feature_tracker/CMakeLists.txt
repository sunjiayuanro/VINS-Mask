cmake_minimum_required(VERSION 2.8.3)
project(feature_tracker)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(PkgConfig REQUIRED)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Protobuf REQUIRED)
find_package(GRPC REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    camera_model
    )

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROTOBUF_LIBRARIES}
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

set(PROTO_SRC_DIR ${PROJECT_SOURCE_DIR}/include/feature_tracker)
file(MAKE_DIRECTORY ${PROTO_SRC_DIR})
include_directories(${PROTO_SRC_DIR})

set(PROTOS
  ${CMAKE_CURRENT_SOURCE_DIR}/protos/common.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/protos/service.proto
)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_SRC_DIR} ${PROTOS})
grpc_generate_cpp(GRPC_SRCS GRPC_HDRS ${PROTO_SRC_DIR} ${PROTOS})

find_package(Eigen3)

add_library(CppVIOApi SHARED
      src/service_api.cpp
      ${PROTO_SRCS}
      ${GRPC_SRCS})
target_link_libraries(CppVIOApi protobuf::libprotobuf gRPC::grpc++_reflection
      ${PROTOBUF_LIBRARIES}
      ${catkin_LIBRARIES})

add_executable(feature_tracker
    src/feature_tracker_node.cpp
    src/parameters.cpp
    src/feature_tracker.cpp
    )
target_link_libraries(feature_tracker CppVIOApi ${catkin_LIBRARIES} ${OpenCV_LIBS})

