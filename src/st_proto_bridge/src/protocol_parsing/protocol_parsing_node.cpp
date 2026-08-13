#include "st_proto_bridge/protocol_parsing_node.h"
#include "st_proto_bridge/json_helper.h"
#include "st_proto_bridge/protocol_codec.h"
#include "st_proto_bridge/tcp_client.h"
#include "st_proto_bridge/types.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <memory>
#include <mutex>
#include <queue>
#include <cstdint>

namespace st_proto {

// ============================================================
// 构造/析构
// ============================================================

ProtocolParsingNode::ProtocolParsingNode()
    : nh_("")
    , codec_(std::make_unique<ProtocolCodec>())
    , tcp_(std::make_unique<TcpClient>())
    , heartbeatMs_(DEFAULT_HEARTBEAT_MS)
    , reconnectMs_(DEFAULT_RECONNECT_MS)
    , fwVersion_(100)
    , vehicleType_(0)
    , loggedIn_(false)
{
    loadParams();

    // 设置TCP回调
    tcp_->setFrameCallback(
        [this](const std::vector<uint8_t>& frame) { onFrameReceived(frame); });
    tcp_->setStateCallback(
        [this](ConnectionState state) { onStateChanged(state); });

    // 订阅器：接收ROS话题并编码为协议帧上行
    statusSub_ = nh_.subscribe("robot_status", 10,
        &ProtocolParsingNode::onRobotStatus, this);
    faultSub_ = nh_.subscribe("robot_fault", 10,
        &ProtocolParsingNode::onFaultAlarm, this);
    taskStateSub_ = nh_.subscribe("task_state", 10,
        &ProtocolParsingNode::onTaskState, this);
    localPathSub_ = nh_.subscribe("local_path", 10,
        &ProtocolParsingNode::onLocalPath, this);
    respSub_ = nh_.subscribe("send_resp", 10,
        &ProtocolParsingNode::onSendResp, this);

    // 发布器：解析下行JSON后发布到ROS话题
    ctrlPub_ = nh_.advertise<std_msgs::String>("ctrl_cmd", 10);
    taskPub_ = nh_.advertise<std_msgs::String>("task_cmd", 10);
    rawDownPub_ = nh_.advertise<std_msgs::String>("raw_down", 10);
    batteryLevelDownPub_ = nh_.advertise<std_msgs::String>("battery_level_down", 10);
    taskCtrlPub_ = nh_.advertise<std_msgs::String>("task_ctrl", 10);

    // 服务：外部可主动请求发送JSON上行
    sendJsonSrv_ = nh_.advertiseService("send_business_json",
        &ProtocolParsingNode::onSendJsonService, this);

    // 启动TCP
    tcp_->configure(serverHost_, serverPort_, reconnectMs_);
    tcp_->start();

    ROS_INFO("[st_proto_bridge] Node started, connecting to %s:%d",
             serverHost_.c_str(), serverPort_);
}

ProtocolParsingNode::~ProtocolParsingNode() {
    tcp_->stop();
}

void ProtocolParsingNode::spin() {
    ros::Rate rate(20);  // 20Hz 处理定时任务
    while (ros::ok()) {
        ros::spinOnce();
        processTimers();
        rate.sleep();
    }
}

// ============================================================
// 参数加载
// ============================================================

void ProtocolParsingNode::loadParams() {
    // 参数从私有命名空间加载 (nh_("~") → /st_robot/st_proto_bridge/)
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("server_host", serverHost_, "192.168.1.100");
    nh_private.param<int>("server_port", serverPort_, 6000);
    nh_private.param<std::string>("robot_uuid", robotUuid_, "robot_001");
    nh_private.param<std::string>("aes_key", aesKey_, "f7b4f15012de4e37");
    nh_private.param<std::string>("aes_iv",  aesIv_,  "ad2fdc88772c0e95");
    nh_private.param<int>("vehicle_type", vehicleType_, 1);
    nh_private.param<int>("fw_version", fwVersion_, 100);
    nh_private.param<int>("heartbeat_ms", heartbeatMs_, DEFAULT_HEARTBEAT_MS);
    nh_private.param<int>("reconnect_ms", reconnectMs_, DEFAULT_RECONNECT_MS);

    // 配置AES密钥（必须在加解密操作之前调用）
    codec_->configure(aesKey_, aesIv_);
}

// ============================================================
// TCP回调
// ============================================================

void ProtocolParsingNode::onFrameReceived(const std::vector<uint8_t>& frame) {
    if (frame.size() < HEADER_SIZE) return;

    MessageHeader hdr = MessageHeader::fromBytes(frame.data());
    auto packetType = extractPacketType(hdr.msgType);
    bool encrypted = isEncrypted(hdr.msgType);

    // 提取消息体
    std::vector<uint8_t> body(frame.begin() + HEADER_SIZE, frame.end());

    // AES解密
    if (encrypted) {
        try {
            body = codec_->aesDecrypt(body);
        } catch (const std::exception& e) {
            ROS_WARN("[st_proto_bridge] AES decrypt failed: %s", e.what());
            return;
        }
    }

    switch (packetType) {
    case PacketType::LOGIN_RESP:
        handleLoginResponse(body);
        break;
    case PacketType::HEARTBEAT:
        handleHeartbeat();
        break;
    case PacketType::HEARTBEAT_RESP:
        handleHeartbeatResponse();
        break;
    case PacketType::JSON_BUSINESS:
        handleJsonBusiness(body);
        break;
    default:
        ROS_DEBUG("[st_proto_bridge] Unknown packet type: 0x%02X",
                  static_cast<int>(packetType));
        break;
    }
}

void ProtocolParsingNode::onStateChanged(ConnectionState state) {
    switch (state) {
    case ConnectionState::CONNECTED:
        ROS_INFO("[st_proto_bridge] TCP connected, sending login request...");
        sendLoginRequest();
        break;
    case ConnectionState::DISCONNECTED:
        loggedIn_ = false;
        ROS_WARN("[st_proto_bridge] TCP disconnected");
        break;
    default:
        break;
    }
}

// ============================================================
// 登录/心跳
// ============================================================

void ProtocolParsingNode::sendLoginRequest() {
    auto frame = codec_->packLoginRequest(
        static_cast<uint8_t>(vehicleType_),
        static_cast<uint32_t>(fwVersion_),
        robotUuid_);
    tcp_->send(frame);
    loginTime_ = ros::Time::now();
    ROS_INFO("[st_proto_bridge] Login request sent, uuid=%s", robotUuid_.c_str());
}

void ProtocolParsingNode::handleLoginResponse(const std::vector<uint8_t>& body) {
    try {
        auto resp = codec_->parseLoginResponse(body);
        if (resp.result == 0x01) {
            loggedIn_ = true;
            heartbeatMs_ = static_cast<int>(resp.heartbeatMs);
            heartbeatInterval_ = ros::Duration(heartbeatMs_ / 1000.0);
            lastHbSent_ = ros::Time::now();
            lastHbRecv_ = ros::Time::now();
            ROS_INFO("[st_proto_bridge] Login success, heartbeat interval=%dms",
                     heartbeatMs_);
        } else {
            ROS_ERROR("[st_proto_bridge] Login rejected (result=0x%02X)",
                      resp.result);
        }
    } catch (const std::exception& e) {
        ROS_ERROR("[st_proto_bridge] Failed to parse login response: %s", e.what());
    }
}

void ProtocolParsingNode::sendHeartbeat() {
    auto frame = codec_->packHeartbeat();
    tcp_->send(frame);
    lastHbSent_ = ros::Time::now();
}

void ProtocolParsingNode::handleHeartbeat() {
    // 收到服务端心跳，回复心跳响应
    auto frame = codec_->packHeartbeatResponse();
    tcp_->send(frame);
    lastHbRecv_ = ros::Time::now();
}

void ProtocolParsingNode::handleHeartbeatResponse() {
    lastHbRecv_ = ros::Time::now();
}

// ============================================================
// JSON业务处理
// ============================================================

void ProtocolParsingNode::handleJsonBusiness(const std::vector<uint8_t>& body) {
    std::string jsonStr(body.begin(), body.end());
    ROS_DEBUG("[st_proto_bridge] Received business JSON: %s", jsonStr.c_str());

    try {
        json j = JsonHelper::deserialize(jsonStr);
        std::string cmd = j.value("cmd", "");

        std::string mid = JsonHelper::extractMid(j);

        if (cmd == BusinessCmd::ROBOT_CTRL) {
            CtrlCmdCode ctrlCode;
            int walkDir = 0;
            double param = 0.0;
            LocPose loc;
            if (JsonHelper::parseCtrlCmd(j, ctrlCode, walkDir, param, loc)) {
                std_msgs::String msg;
                msg.data = jsonStr;
                ctrlPub_.publish(msg);
                // 自动回复ACK（V1.0.2 使用 mid）
                auto resp = JsonHelper::buildResp(mid, cmd, 0, "ok");
                sendBusinessJson(resp);
            }
        } else if (cmd == BusinessCmd::TASK_CREATE) {
            TaskCreateData data;
            if (JsonHelper::parseTaskCreate(j, data)) {
                currentTaskId_ = data.taskId;  // 记录当前任务ID
                std_msgs::String msg;
                msg.data = jsonStr;
                taskPub_.publish(msg);
                auto resp = JsonHelper::buildResp(mid, cmd, 0, "task received");
                sendBusinessJson(resp);
            }
        } else if (cmd == BusinessCmd::BATTERY_LEVEL_DOWN) {
            BatteryLevelDownData data;
            if (JsonHelper::parseBatteryLevelDown(j, data)) {
                std_msgs::String msg;
                msg.data = jsonStr;
                batteryLevelDownPub_.publish(msg);
                auto resp = JsonHelper::buildBatteryLevelDownAck(mid);
                sendBusinessJson(resp);
            }
        } else if (cmd == BusinessCmd::TASK_CTRL) {
            TaskCtrlData data;
            if (JsonHelper::parseTaskCtrl(j, data)) {
                std_msgs::String msg;
                msg.data = jsonStr;
                taskCtrlPub_.publish(msg);
                auto resp = JsonHelper::buildTaskCtrlAck(mid);
                sendBusinessJson(resp);
            }
        }

        // 转发原始下行JSON
        std_msgs::String rawMsg;
        rawMsg.data = jsonStr;
        rawDownPub_.publish(rawMsg);

    } catch (const std::exception& e) {
        ROS_WARN("[st_proto_bridge] JSON parse error: %s", e.what());
    }
}

void ProtocolParsingNode::sendBusinessJson(const json& j) {
    std::string jsonStr = JsonHelper::serialize(j);
    auto frame = codec_->packJsonBusiness(jsonStr);
    tcp_->send(frame);
}

// ============================================================
// 上行消息（ROS话题 → TCP）
// ============================================================

void ProtocolParsingNode::onRobotStatus(const std_msgs::String::ConstPtr& msg) {
    if (!loggedIn_) return;
    try {
        json j = JsonHelper::deserialize(msg->data);
        sendBusinessJson(j);
    } catch (const std::exception& e) {
        ROS_WARN("[st_proto_bridge] Invalid robot_status JSON: %s", e.what());
    }
}

void ProtocolParsingNode::onFaultAlarm(const std_msgs::String::ConstPtr& msg) {
    if (!loggedIn_) return;
    try {
        json j = JsonHelper::deserialize(msg->data);
        sendBusinessJson(j);
    } catch (const std::exception& e) {
        ROS_WARN("[st_proto_bridge] Invalid robot_fault JSON: %s", e.what());
    }
}

void ProtocolParsingNode::onTaskState(const std_msgs::String::ConstPtr& msg) {
    if (!loggedIn_) return;
    try {
        json j = JsonHelper::deserialize(msg->data);
        sendBusinessJson(j);
    } catch (const std::exception& e) {
        ROS_WARN("[st_proto_bridge] Invalid task_state JSON: %s", e.what());
    }
}

void ProtocolParsingNode::onLocalPath(const nav_msgs::Path::ConstPtr& msg) {
    if (!loggedIn_) return;
    std::vector<std::pair<double, double>> points;
    for (const auto& pose : msg->poses) {
        points.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }
    int64_t mid = msgIdCounter_.fetch_add(1);
    auto j = JsonHelper::buildLocalPath(mid, currentTaskId_, points);
    sendBusinessJson(j);
}

void ProtocolParsingNode::onSendResp(const std_msgs::String::ConstPtr& msg) {
    if (!loggedIn_) return;
    try {
        json j = JsonHelper::deserialize(msg->data);
        sendBusinessJson(j);
    } catch (const std::exception& e) {
        ROS_WARN("[st_proto_bridge] Invalid send_resp JSON: %s", e.what());
    }
}

// ============================================================
// 服务回调
// ============================================================

bool ProtocolParsingNode::onSendJsonService(std_srvs::Trigger::Request& req,
                                             std_srvs::Trigger::Response& res) {
    (void)req;
    res.success = loggedIn_;
    res.message = loggedIn_ ? "connected" : "not logged in";
    return true;
}

// ============================================================
// 定时任务
// ============================================================

void ProtocolParsingNode::processTimers() {
    if (!loggedIn_) {
        // 登录超时检查
        if (tcp_->getState() == ConnectionState::CONNECTED) {
            ros::Duration loginTimeout(10.0);  // 10秒登录超时
            if (ros::Time::now() - loginTime_ > loginTimeout) {
                ROS_WARN("[st_proto_bridge] Login timeout, reconnecting...");
                tcp_->stop();
                tcp_->start();
            }
        }
        return;
    }

    auto now = ros::Time::now();

    // 发送心跳
    if (now - lastHbSent_ >= heartbeatInterval_) {
        sendHeartbeat();
    }

    // 心跳超时检查
    ros::Duration hbTimeout(heartbeatMs_ * HEARTBEAT_TIMEOUT_MULTIPLIER / 1000.0);
    if (now - lastHbRecv_ > hbTimeout) {
        ROS_WARN("[st_proto_bridge] Heartbeat timeout, disconnecting...");
        loggedIn_ = false;
        tcp_->stop();
        tcp_->start();
    }
}

} // namespace st_proto