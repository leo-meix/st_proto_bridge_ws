cmake_minimum_required(VERSION 3.0.2)
project(st_proto_bridge)

## 使用C++14标准 (ROS Noetic / Ubuntu 20.04)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


## 查找依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
)

## catkin 包描述
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES st_proto_bridge_lib
  CATKIN_DEPENDS
    roscpp
    std_msgs
    nav_msgs
    geometry_msgs
)

## 头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 查找 OpenSSL (AES加解密)
find_package(OpenSSL REQUIRED)


## 查找 nlohmann-json (Ubuntu 18.04: apt install nlohmann-json-dev)
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  find_package(PkgConfig)
  pkg_check_modules(NLOHMANN_JSON nlohmann_json)
endif()
if(NOT nlohmann_json_FOUND AND NOT NLOHMANN_JSON_FOUND)
  message(FATAL_ERROR "nlohmann_json not found. Install: sudo apt install nlohmann-json-dev")
endif()

## 共享库: 协议编解码 + TCP客户端 + JSON工具
add_library(st_proto_bridge_lib
  src/protocol_codec.cpp
  src/json_helper.cpp
  src/tcp_client.cpp
)

target_include_directories(st_proto_bridge_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(st_proto_bridge_lib
  ${catkin_LIBRARIES}
  OpenSSL::Crypto
  OpenSSL::SSL
)

if(nlohmann_json_FOUND)
  target_link_libraries(st_proto_bridge_lib nlohmann_json::nlohmann_json)
elseif(NLOHMANN_JSON_FOUND)
  target_include_directories(st_proto_bridge_lib PRIVATE ${NLOHMANN_JSON_INCLUDE_DIRS})
endif()

## 可执行文件: protocol_parsing_node (协议解析节点)
add_executable(protocol_parsing_node
  src/protocol_parsing/protocol_parsing_node.cpp
  src/protocol_parsing/main.cpp
)

target_link_libraries(protocol_parsing_node
  st_proto_bridge_lib
  ${catkin_LIBRARIES}
)

## 可执行文件: can_bridge_node (CAN 桥接节点)
add_executable(can_bridge_node
  src/can_bridge/can_codec.cpp
  src/can_bridge/can_bridge_node.cpp
)

target_link_libraries(can_bridge_node
  st_proto_bridge_lib
  ${catkin_LIBRARIES}
)

## 可执行文件: path_bridge_node (路径桥接节点)
add_executable(path_bridge_node
  src/path_bridge/path_bridge_node.cpp
)

target_link_libraries(path_bridge_node
  ${catkin_LIBRARIES}
)

## 可执行文件: charger_cover_node (充电口盖板 Modbus TCP 控制节点)
## 独立节点: 手动构造 Modbus TCP 帧，零第三方依赖
add_executable(charger_cover_node
  src/charger_cover/charger_cover_node.cpp
)

target_link_libraries(charger_cover_node
  ${catkin_LIBRARIES}
)

if(nlohmann_json_FOUND)
  target_link_libraries(charger_cover_node nlohmann_json::nlohmann_json)
elseif(NLOHMANN_JSON_FOUND)
  target_include_directories(charger_cover_node PRIVATE ${NLOHMANN_JSON_INCLUDE_DIRS})
endif()

## 安装
install(TARGETS protocol_parsing_node can_bridge_node path_bridge_node charger_cover_node st_proto_bridge_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)