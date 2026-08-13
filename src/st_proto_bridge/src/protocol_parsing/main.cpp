#include <ros/ros.h>
#include "st_proto_bridge/protocol_parsing_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "st_proto_bridge");

    st_proto::ProtocolParsingNode node;
    node.spin();

    return 0;
}