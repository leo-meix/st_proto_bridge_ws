#ifndef ST_PROTO_BRIDGE_TYPES_H
#define ST_PROTO_BRIDGE_TYPES_H

#include <cstdint>
#include <string>
#include <vector>
#include <stdexcept>

namespace st_proto {

// ============================================================
// 协议常量
// ============================================================

/// 前导码 - 固定 0xAA
constexpr uint8_t PREAMBLE = 0xAA;

/// 协议类型 - 机器人业务 0xB1
constexpr uint8_t PROTOCOL_TYPE = 0xB1;

/// AES密钥/偏移量长度（16字节 = 128位）
constexpr int AES_BLOCK_SIZE = 16;

/// 消息头长度：前导码(1) + 协议类型(1) + 消息类型(1) + 消息长度(4) = 7 字节
constexpr int HEADER_SIZE = 7;

/// 登录申请包中鉴权标识（固定 "ST"）
constexpr uint8_t AUTH_ID0 = 0x53;  // 'S'
constexpr uint8_t AUTH_ID1 = 0x54;  // 'T'

/// 默认心跳间隔 (ms)
constexpr int DEFAULT_HEARTBEAT_MS = 1000;

/// 默认重连间隔 (ms)
constexpr int DEFAULT_RECONNECT_MS = 5000;

/// 心跳超时倍数（服务端3个周期收不到心跳判定断开）
constexpr int HEARTBEAT_TIMEOUT_MULTIPLIER = 3;

// ============================================================
// 枚举定义
// ============================================================

/// 包类型 (消息类型 bit[0:5])
enum class PacketType : uint8_t {
    LOGIN_REQ       = 0x01,  // 登录申请包
    LOGIN_RESP      = 0x02,  // 登录响应包
    HEARTBEAT       = 0x03,  // 心跳包
    HEARTBEAT_RESP  = 0x04,  // 心跳响应包
    JSON_BUSINESS   = 0x05,  // JSON业务数据包
    BINARY_BUSINESS = 0x06,  // 二进制业务数据包
    KICK_OUT        = 0x07,  // 踢出登录（预留）
    DISCONNECT      = 0x08,  // 主动断开（预留）
    SUBSCRIBE       = 0x09,  // 订阅主题（预留）
    PUBLISH         = 0x0A,  // 发布消息（预留）
};

/// 消息类型字节 bit7 掩码（加密标志）
constexpr uint8_t MSG_ENCRYPT_MASK = 0x80;

/// 从消息类型字节提取包类型
inline PacketType extractPacketType(uint8_t msgType) {
    return static_cast<PacketType>(msgType & 0x3F);
}

/// 判断消息体是否加密
inline bool isEncrypted(uint8_t msgType) {
    return (msgType & MSG_ENCRYPT_MASK) != 0;
}

/// TCP连接状态
enum class ConnectionState {
    DISCONNECTED = 0,
    CONNECTING   = 1,
    CONNECTED    = 2,
    LOGGED_IN    = 3,
};

/// 任务状态码（V1.0.2）
enum class TaskStateCode : int {
    CANNOT_EXECUTE  = 0,  // 无法执行
    IDLE            = 1,  // 空闲中
    NAVIGATING      = 2,  // 导航中
    CHARGING        = 3,  // 充电中
    UNPLUGGING      = 4,  // 拔枪中
    EMERGENCY_STOP  = 5,  // 急停中
    REMOTE_TAKEOVER = 6,  // 远程接管中
    FINISHED        = 7,  // 已结束
};

/// 控制命令码（V1.0.3）
enum class CtrlCmdCode : int {
    LIGHT          = 1,  // 三色灯光控制
    EMERGENCY      = 2,  // 急停
    DRIVE_ENABLE   = 3,  // 驾驶使能 (远程控制)
    STOP_WORK      = 4,  // 终止工作
    WALK           = 5,  // 行走（0=前进/1=后退/2=左转/3=右转）
    GOTO_TARGET    = 6,  // 前往目标点 (V1.0.3 新增)
};

/// 业务命令类型 (JSON中的 "cmd" 字段)（V1.0.2）
namespace BusinessCmd {
    constexpr const char* ROBOT_STATUS        = "report";            // 机器人状态上报
    constexpr const char* ROBOT_FAULT         = "robotFaultAlarm";   // 故障告警上报
    constexpr const char* ROBOT_CTRL          = "robotCtrl";         // 机器人控制命令
    constexpr const char* LOCAL_PATH          = "localPath";         // 局部路径上报
    constexpr const char* TASK_CREATE         = "taskCreate";        // 任务下发
    constexpr const char* TASK_STATE          = "taskState";         // 任务状态上报
    constexpr const char* BATTERY_LEVEL_DOWN  = "batteryLevelDown";  // 回归充电电量下发（server→client）
    constexpr const char* TASK_CTRL           = "taskCtrl";          // 任务控制（server→client）
    constexpr const char* RESP                = "resp";              // 通用回复
}

/// 消息方向
enum class MessageDirection {
    CLIENT_TO_SERVER,  // 上行
    SERVER_TO_CLIENT,  // 下行
};

// ============================================================
// 数据结构
// ============================================================

/// 消息头（7字节，大端序）
#pragma pack(push, 1)
struct MessageHeader {
    uint8_t  preamble;      // 前导码 0xAA
    uint8_t  protocolType;  // 协议类型 0xB1
    uint8_t  msgType;       // bit7=加密, bit[0:5]=包类型
    uint32_t bodyLength;    // 消息体长度（大端序）
    // 消息体后跟1字节校验和

    /// 从字节数组解析（大端序）
    static MessageHeader fromBytes(const uint8_t* data) {
        MessageHeader h;
        h.preamble     = data[0];
        h.protocolType = data[1];
        h.msgType      = data[2];
        h.bodyLength   = (static_cast<uint32_t>(data[3]) << 24) |
                         (static_cast<uint32_t>(data[4]) << 16) |
                         (static_cast<uint32_t>(data[5]) << 8)  |
                         (static_cast<uint32_t>(data[6]));
        return h;
    }

    /// 序列化为字节数组（大端序）
    std::vector<uint8_t> toBytes() const {
        std::vector<uint8_t> buf(HEADER_SIZE);
        buf[0] = preamble;
        buf[1] = protocolType;
        buf[2] = msgType;
        buf[3] = static_cast<uint8_t>((bodyLength >> 24) & 0xFF);
        buf[4] = static_cast<uint8_t>((bodyLength >> 16) & 0xFF);
        buf[5] = static_cast<uint8_t>((bodyLength >> 8)  & 0xFF);
        buf[6] = static_cast<uint8_t>(bodyLength & 0xFF);
        return buf;
    }
};
#pragma pack(pop)

/// 登录申请包消息体
struct LoginRequestBody {
    uint8_t  authId0;        // 0x53 'S'
    uint8_t  authId1;        // 0x54 'T'
    uint8_t  vehicleType;    // 车辆类型
    uint32_t fwVersion;      // 固件版本（大端序）
    uint8_t  uuidLength;     // 唯一标识长度
    std::string uuid;        // 唯一标识字符串
};

/// 登录响应包消息体
struct LoginResponseBody {
    uint8_t  result;         // 0x01=成功
    uint32_t heartbeatMs;    // 心跳间隔ms（大端序）
};

/// 位姿（用于终止工作命令的充电基站坐标）
struct LocPose {
    double x;
    double y;
    double yaw;
};

/// 充电口坐标（用于任务下发的充电位坐标）
struct ChargingPortLoc {
    double x;
    double y;
};

/// 单个故障/告警条目
struct FaultAlarmItem {
    int code;    // 故障/告警码
    int level;   // 故障级别

    bool operator==(const FaultAlarmItem& o) const {
        return code == o.code && level == o.level;
    }
};

/// 故障告警上报数据
struct FaultAlarmReport {
    std::vector<FaultAlarmItem> faults;
    std::vector<FaultAlarmItem> alarms;
};

/// 回归充电电量下发数据
struct BatteryLevelDownData {
    double batteryLevel;
};

/// 任务控制数据
struct TaskCtrlData {
    int64_t taskId;
    int ctrlCmd;  // 1=暂停, 2=恢复, 3=取消
};

/// 任务创建数据（V1.0.2）
struct TaskCreateData {
    int64_t taskId;
    int taskType;        // 0=自动任务, 1=临时任务
    int operationType;   // 0=充电, 1=拔枪
    std::string chargingEquipment;
    std::string chargingPort;
    ChargingPortLoc chargingPortLoc;
};

} // namespace st_proto

#endif // ST_PROTO_BRIDGE_TYPES_H