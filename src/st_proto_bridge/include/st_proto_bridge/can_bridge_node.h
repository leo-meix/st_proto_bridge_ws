#ifndef ST_PROTO_BRIDGE_CAN_BRIDGE_NODE_H
#define ST_PROTO_BRIDGE_CAN_BRIDGE_NODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nlohmann/json.hpp>

#include "st_proto_bridge/can_types.h"
#include "st_proto_bridge/types.h"
#include "st_proto_bridge/json_helper.h"

#include <string>
#include <vector>
#include <memory>
#include <atomic>
#include <array>

namespace st_proto {

using json = nlohmann::json;
using CanFrame = std::array<uint8_t, 8>;

/**
 * @brief CAN 桥接 ROS 节点
 *
 * 数据流:
 *   上行: SocketCAN → CanCodec 解码 → 缓存 → 定时组装 JSON → /can/robot_status 等话题
 *   下行: ctrl_cmd 等话题 → 提取 CAN 命令 → CanCodec 编码 → SocketCAN 发送
 *
 * 与 protocol_parsing_node 通过 ROS 话题松耦合:
 *   can_bridge 发布完整 JSON → protocol_parsing_node 订阅并 TCP 上行
 */
class CanBridgeNode {
public:
    CanBridgeNode();
    ~CanBridgeNode();

    void spin();

private:
    void loadParams();

    // ============================================================
    // SocketCAN
    // ============================================================
    bool initSocketCan();
    void closeSocketCan();
    void receiveCanFrames();
    void sendCanFrame(uint32_t canId, const CanFrame& data);

    // ============================================================
    // CAN 帧处理
    // ============================================================
    void onCanFrame(uint32_t canId, const CanFrame& frame);

    // ============================================================
    // 上行: CAN → JSON → ROS Topic
    // ============================================================
    void publishRobotStatus(const ros::TimerEvent&);
    void publishFaultAlarm(const ros::TimerEvent&);
    void publishTaskState(const ros::TimerEvent&);

    // 收集所有故障
    void collectFaults(std::vector<FaultAlarmItem>& faults,
                       std::vector<FaultAlarmItem>& alarms);
    void collectMotorFaults(uint32_t bitmap,
                            std::vector<FaultAlarmItem>& faults);

    // VCU 超时检测
    void checkVcuTimeout(const ros::TimerEvent&);

    // ============================================================
    // 下行: ROS Topic → CAN
    // ============================================================
    void onCtrlCmd(const std_msgs::String::ConstPtr& msg);
    void onCmdVelHighfreq(const geometry_msgs::Twist::ConstPtr& msg);
    void onTaskCmd(const std_msgs::String::ConstPtr& msg);
    void onBatteryLevelDown(const std_msgs::String::ConstPtr& msg);
    void onTaskCtrl(const std_msgs::String::ConstPtr& msg);

    // 下行 CAN 定时发送
    void sendControlFrame(const ros::TimerEvent&);   // 0x200 100ms
    void sendMotionFrame(const ros::TimerEvent&);    // 0x201 10ms

    // ============================================================
    // 成员变量
    // ============================================================

    ros::NodeHandle nh_;

    // ---- CAN 配置 ----
    std::string canInterface_;
    int canBitrate_;

    // ---- 上报配置 ----
    double robotStatusHz_;
    double faultCheckHz_;
    double taskStateHz_;
    int vcuMotionTimeoutMs_;
    int vcuBasicTimeoutMs_;

    // ---- 行走速度配置 ----
    double walkLinearSpeed_   = 1.0;   // m/s
    double walkAngularSpeed_  = 0.5;   // rad/s

    // ---- SocketCAN ----
    int sockFd_ = -1;

    // ---- 数据缓存 (VCU 上行帧解码结果) ----
    BmsBasicData      bmsBasic_;
    BmsCellData       bmsCell_;
    BmsTempData       bmsTemp_;
    MotionData        motion_;
    MotorPositionData motorPos_;
    VcuFaultFlags     vcuFlags_;
    BmsFaultData      bmsFault_;
    MotorFaultBitmap  motorFaults_;
    MotorSpeedData    motorSpeed_;

    // ---- 故障状态缓存 (用于变化检测，避免重复上报) ----
    std::vector<FaultAlarmItem> lastFaults_;
    std::vector<FaultAlarmItem> lastAlarms_;

    // ---- VCU 通信时间戳 ----
    ros::Time lastMotionTime_;   // 0x18FF5321 (10ms 周期)
    ros::Time lastBasicTime_;    // 0x18FF5021 (100ms 周期)
    bool vcuOnline_ = false;

    // ---- 外部定位数据 (非 CAN) ----
    double reportYaw_ = 0.0;     // 航向角 (由定位模块提供)

    // ---- 上行定时器 ----
    ros::Timer statusTimer_;
    ros::Timer faultTimer_;
    ros::Timer taskStateTimer_;
    ros::Timer vcuTimeoutTimer_;

    // ---- 上行发布器 ----
    ros::Publisher statusPub_;
    ros::Publisher faultPub_;
    ros::Publisher taskStatePub_;

    // ---- 下行订阅器 ----
    ros::Subscriber ctrlSub_;
    ros::Subscriber cmdVelHighfreqSub_;
    ros::Subscriber taskSub_;
    ros::Subscriber batteryLevelSub_;
    ros::Subscriber taskCtrlSub_;

    // ---- 下行控制命令缓存 ----
    IpcControlCmd  currentCtrlCmd_;    // 当前寄存的控制命令位
    IpcMotionCmd   currentMotionCmd_;  // 当前速度命令
    std::mutex     cmdMutex_;          // 线程安全保护

    // ---- 下行 CAN 发送定时器 ----
    ros::Timer ctrlSendTimer_;   // 0x200 100ms
    ros::Timer motionSendTimer_; // 0x201 10ms

    // ---- 任务状态 (V1.0.3) ----
    int64_t currentTaskId_ = 0;          // 当前任务ID (V1.0.3: Int)
    bool taskActive_   = false;          // 是否有任务在执行中
    bool taskFinished_ = false;          // 任务是否已结束

    // ---- 状态 ----
    std::atomic<bool> running_{false};
    std::atomic<int64_t> msgIdCounter_{1};
};

} // namespace st_proto

#endif // ST_PROTO_BRIDGE_CAN_BRIDGE_NODE_H
