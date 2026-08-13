#ifndef ST_PROTO_BRIDGE_PROTOCOL_PARSING_NODE_H
#define ST_PROTO_BRIDGE_PROTOCOL_PARSING_NODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Trigger.h>
#include <nlohmann/json.hpp>

#include "st_proto_bridge/types.h"

#include <memory>
#include <atomic>
#include <vector>
#include <cstdint>

namespace st_proto {

using json = nlohmann::json;

// 前置声明
class ProtocolCodec;
class TcpClient;

/**
 * @brief 协议解析ROS节点
 *
 * 负责：
 * 1. 与云平台服务器TCP长连接（登录→心跳→业务）
 * 2. 上行：ROS话题 → JSON业务数据 → AES加密 → 协议帧 → TCP发送
 * 3. 下行：TCP接收 → 协议帧解包 → AES解密 → JSON解析 → ROS话题发布
 * 4. 状态机管理（断线重连、登录超时、心跳超时）
 */
class ProtocolParsingNode {
public:
    ProtocolParsingNode();
    ~ProtocolParsingNode();

    void spin();

private:
    void loadParams();

    // TCP回调
    void onFrameReceived(const std::vector<uint8_t>& frame);
    void onStateChanged(ConnectionState state);

    // 登录/心跳
    void sendLoginRequest();
    void handleLoginResponse(const std::vector<uint8_t>& body);
    void sendHeartbeat();
    void handleHeartbeat();
    void handleHeartbeatResponse();

    // JSON业务处理
    void handleJsonBusiness(const std::vector<uint8_t>& body);
    void sendBusinessJson(const json& j);

    // 上行消息（ROS话题 → TCP）
    void onRobotStatus(const std_msgs::String::ConstPtr& msg);
    void onFaultAlarm(const std_msgs::String::ConstPtr& msg);
    void onTaskState(const std_msgs::String::ConstPtr& msg);
    void onLocalPath(const nav_msgs::Path::ConstPtr& msg);
    void onSendResp(const std_msgs::String::ConstPtr& msg);

    // 服务回调
    bool onSendJsonService(std_srvs::Trigger::Request& req,
                           std_srvs::Trigger::Response& res);

    // 定时任务
    void processTimers();

    // ============================================================
    // 成员变量
    // ============================================================

    ros::NodeHandle nh_;

    // 协议层
    std::unique_ptr<ProtocolCodec> codec_;
    std::unique_ptr<TcpClient> tcp_;

    // 配置
    std::string serverHost_;
    int serverPort_;
    std::string robotUuid_;
    std::string aesKey_;
    std::string aesIv_;
    int vehicleType_;
    int fwVersion_;
    int heartbeatMs_;
    int reconnectMs_;
    ros::Duration heartbeatInterval_;

    // 状态
    std::atomic<bool> loggedIn_;
    ros::Time loginTime_;
    ros::Time lastHbSent_;
    ros::Time lastHbRecv_;

    // 业务上下文
    int64_t currentTaskId_ = 0;
    std::atomic<int64_t> msgIdCounter_{1};

    // 订阅器
    ros::Subscriber statusSub_;
    ros::Subscriber faultSub_;
    ros::Subscriber taskStateSub_;
    ros::Subscriber localPathSub_;
    ros::Subscriber respSub_;

    // 发布器
    ros::Publisher ctrlPub_;
    ros::Publisher taskPub_;
    ros::Publisher rawDownPub_;
    ros::Publisher batteryLevelDownPub_;
    ros::Publisher taskCtrlPub_;

    // 服务
    ros::ServiceServer sendJsonSrv_;
};

} // namespace st_proto

#endif // ST_PROTO_BRIDGE_PROTOCOL_PARSING_NODE_H