#include "st_proto_bridge/charger_cover_node.h"

#include <nlohmann/json.hpp>

#include <string>
#include <cstring>
#include <atomic>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

namespace st_proto {

using json = nlohmann::json;

// ============================================================
// 构造
// ============================================================
ChargerCoverNode::ChargerCoverNode()
    : nh_("")
    , modbusPort_(502)
    , modbusUnitId_(1)
    , coilStartAddr_(0)
    , coilOpenValue_(0x01)
    , coilCloseValue_(0x02)
    , timeoutMs_(5000)
    , dryRun_(false)
{
    loadParams();

    // 订阅盖板命令
    cmdSub_ = nh_.subscribe("cover_cmd", 10,
        &ChargerCoverNode::onCoverCmd, this);

    // 发布执行结果
    resultPub_ = nh_.advertise<std_msgs::String>("cover_result", 10);

    ROS_INFO("[charger_cover] Node started. modbus=%s:%d unit=%d coil_start=%d dry_run=%s",
             modbusIp_.c_str(), modbusPort_, modbusUnitId_, coilStartAddr_,
             dryRun_ ? "true" : "false");
}

// ============================================================
// 参数加载
// ============================================================
void ChargerCoverNode::loadParams() {
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("modbus_ip",       modbusIp_,       "192.168.1.100");
    nh_private.param<int>("modbus_port",             modbusPort_,     502);
    nh_private.param<int>("modbus_unit_id",          modbusUnitId_,   1);
    nh_private.param<int>("coil_start_address",      coilStartAddr_,  0);
    nh_private.param<int>("coil_open_value",         coilOpenValue_,  0x01);
    nh_private.param<int>("coil_close_value",        coilCloseValue_, 0x02);
    nh_private.param<int>("request_timeout_ms",      timeoutMs_,      5000);
    nh_private.param<bool>("dry_run",                dryRun_,         false);
}

// ============================================================
// 主循环
// ============================================================
void ChargerCoverNode::spin() {
    ros::spin();
}

// ============================================================
// 命令回调
// ============================================================
void ChargerCoverNode::onCoverCmd(const std_msgs::String::ConstPtr& msg) {
    std::string action;
    try {
        json j = json::parse(msg->data);
        action = j.value("action", "");
    } catch (const std::exception& e) {
        ROS_WARN("[charger_cover] cover_cmd JSON parse error: %s, raw=%s",
                 e.what(), msg->data.c_str());
        return;
    }

    bool open;
    if (action == "open") {
        open = true;
    } else if (action == "close") {
        open = false;
    } else {
        ROS_WARN("[charger_cover] unknown action: '%s' (expect open/close)",
                 action.c_str());
        return;
    }

    bool ok = setCover(open);
    publishResult(action, ok);
}

// ============================================================
// 控制盖板 (Modbus TCP 0x0F Write Multiple Coils)
//
// 手动构造 Modbus TCP 帧并发送（不依赖 libmodbus 的 write_bits），
// 确保发出的字节与文档完全一致。
// 帧结构 (MBAP 7B + PDU 6B = 13B):
//   TID(2) PID(2) LEN(2) UID(1) FUNC(1) ADDR(2) N(2) BYTES(1) DATA(1)
// ============================================================
bool ChargerCoverNode::setCover(bool open) {
    uint8_t dataByte = static_cast<uint8_t>(open ? coilOpenValue_ : coilCloseValue_);

    // L0 dry-run: 只打印, 不发请求
    if (dryRun_) {
        ROS_INFO("[charger_cover][DRY-RUN] Modbus TCP %s %s:%d unit=%d addr=%d coils=2 data=0x%02X",
                 open ? "open" : "close",
                 modbusIp_.c_str(), modbusPort_, modbusUnitId_,
                 coilStartAddr_, dataByte);
        return true;
    }

    // 1. 创建 TCP 连接
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        ROS_ERROR("[charger_cover] socket() failed: %s", strerror(errno));
        return false;
    }

    struct timeval tv;
    tv.tv_sec  = timeoutMs_ / 1000;
    tv.tv_usec = (timeoutMs_ % 1000) * 1000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(static_cast<uint16_t>(modbusPort_));
    inet_pton(AF_INET, modbusIp_.c_str(), &addr.sin_addr);

    if (connect(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        ROS_ERROR("[charger_cover] connect %s:%d failed: %s",
                  modbusIp_.c_str(), modbusPort_, strerror(errno));
        close(sock);
        return false;
    }

    // 2. 组装 Modbus TCP 请求帧
    // MBAP: 事务ID=1, 协议ID=0, 长度=UID(1)+PDU(6)=7, 单元ID
    // PDU:  功能码0x0F + 起始地址 + 线圈数 + 字节数 + 数据
    static std::atomic<uint16_t> tid{1};
    uint16_t transactionId = tid.fetch_add(1);
    uint16_t startAddr     = static_cast<uint16_t>(coilStartAddr_);

    uint8_t frame[14];
    size_t frameLen = 0;

    // MBAP 头
    frame[frameLen++] = static_cast<uint8_t>((transactionId >> 8) & 0xFF);
    frame[frameLen++] = static_cast<uint8_t>(transactionId & 0xFF);
    frame[frameLen++] = 0x00;  // 协议ID 高字节
    frame[frameLen++] = 0x00;  // 协议ID 低字节
    frame[frameLen++] = 0x00;  // 长度 高字节
    frame[frameLen++] = 0x08;  // 长度 = UID(1) + PDU(7) = 8
    frame[frameLen++] = static_cast<uint8_t>(modbusUnitId_);

    // PDU: 功能码 0x0F
    frame[frameLen++] = 0x0F;
    // 起始地址
    frame[frameLen++] = static_cast<uint8_t>((startAddr >> 8) & 0xFF);
    frame[frameLen++] = static_cast<uint8_t>(startAddr & 0xFF);
    // 线圈数
    frame[frameLen++] = 0x00;
    frame[frameLen++] = 0x02;
    // 字节数 + 数据
    frame[frameLen++] = 0x01;
    frame[frameLen++] = dataByte;

    // 3. 发送
    ssize_t sent = send(sock, frame, frameLen, 0);
    if (sent != static_cast<ssize_t>(frameLen)) {
        ROS_ERROR("[charger_cover] send failed: sent=%zd expected=%zu err=%s",
                  sent, frameLen, strerror(errno));
        close(sock);
        return false;
    }

    // 4. 接收响应 (12字节)
    uint8_t resp[12];
    ssize_t recvd = recv(sock, resp, sizeof(resp), 0);
    close(sock);

    if (recvd < 12) {
        ROS_ERROR("[charger_cover] recv response failed: recvd=%zd err=%s",
                  recvd, strerror(errno));
        return false;
    }

    // 检查响应功能码是否为 0x0F
    if (resp[7] != 0x0F) {
        ROS_ERROR("[charger_cover] unexpected response func=0x%02X (expected 0x0F)", resp[7]);
        return false;
    }

    ROS_INFO("[charger_cover] %s OK (tid=%u data=0x%02X)",
             open ? "open" : "close", transactionId, dataByte);
    return true;
}

// ============================================================
// 发布结果
// ============================================================
void ChargerCoverNode::publishResult(const std::string& action, bool success) {
    json j;
    j["action"]  = action;
    j["success"] = success;

    std_msgs::String msg;
    msg.data = j.dump();
    resultPub_.publish(msg);
}

}  // namespace st_proto

// ============================================================
// main
// ============================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "charger_cover_node");
    st_proto::ChargerCoverNode node;
    node.spin();
    return 0;
}
