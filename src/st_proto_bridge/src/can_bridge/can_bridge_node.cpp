#include "st_proto_bridge/can_bridge_node.h"
#include "st_proto_bridge/can_codec.h"
#include "st_proto_bridge/json_helper.h"
#include "st_proto_bridge/types.h"

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <mutex>
#include <algorithm>

namespace st_proto {

// ============================================================
// 构造 / 析构
// ============================================================

CanBridgeNode::CanBridgeNode()
    : nh_("")
{
    loadParams();

    if (!initSocketCan()) {
        ROS_ERROR("[can_bridge] Failed to initialize SocketCAN on %s", canInterface_.c_str());
        ros::shutdown();
        return;
    }

    // ---- 上行发布器 ----
    statusPub_    = nh_.advertise<std_msgs::String>("robot_status", 10);
    faultPub_     = nh_.advertise<std_msgs::String>("robot_fault", 10);
    taskStatePub_ = nh_.advertise<std_msgs::String>("task_state", 10);

    // ---- 下行订阅器 ----
    ctrlSub_          = nh_.subscribe("ctrl_cmd", 10, &CanBridgeNode::onCtrlCmd, this);
    taskSub_          = nh_.subscribe("task_cmd", 10, &CanBridgeNode::onTaskCmd, this);
    batteryLevelSub_  = nh_.subscribe("battery_level_down", 10, &CanBridgeNode::onBatteryLevelDown, this);
    taskCtrlSub_      = nh_.subscribe("task_ctrl", 10, &CanBridgeNode::onTaskCtrl, this);

    // ---- 上行定时器 ----
    double statusPeriod = (robotStatusHz_ > 0) ? (1.0 / robotStatusHz_) : 1.0;
    double faultPeriod  = (faultCheckHz_ > 0)  ? (1.0 / faultCheckHz_)  : 1.0;
    double taskPeriod   = (taskStateHz_ > 0)   ? (1.0 / taskStateHz_)   : 1.0;

    statusTimer_ = nh_.createTimer(ros::Duration(statusPeriod),
        &CanBridgeNode::publishRobotStatus, this);
    faultTimer_  = nh_.createTimer(ros::Duration(faultPeriod),
        &CanBridgeNode::publishFaultAlarm, this);
    taskStateTimer_ = nh_.createTimer(ros::Duration(taskPeriod),
        &CanBridgeNode::publishTaskState, this);

    // ---- VCU 超时检测 (500ms) ----
    vcuTimeoutTimer_ = nh_.createTimer(ros::Duration(0.5),
        &CanBridgeNode::checkVcuTimeout, this);

    // ---- 下行 CAN 发送定时器 ----
    ctrlSendTimer_ = nh_.createTimer(ros::Duration(0.1),  // 100ms = 0x200 周期
        &CanBridgeNode::sendControlFrame, this);
    motionSendTimer_ = nh_.createTimer(ros::Duration(0.01),  // 10ms = 0x201 周期
        &CanBridgeNode::sendMotionFrame, this);

    running_ = true;
    lastMotionTime_ = ros::Time::now();
    lastBasicTime_  = ros::Time::now();

    ROS_INFO("[can_bridge] Node started on %s, bitrate=%d", canInterface_.c_str(), canBitrate_);
}

CanBridgeNode::~CanBridgeNode() {
    running_ = false;
    closeSocketCan();
}

// ============================================================
// 参数加载
// ============================================================

void CanBridgeNode::loadParams() {
    // 参数从私有命名空间加载 (nh_("~") → /st_robot/st_can_bridge/)
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("can_interface", canInterface_, "can0");
    nh_private.param<int>("can_bitrate", canBitrate_, 500000);
    nh_private.param<double>("robot_status_hz", robotStatusHz_, 1.0);
    nh_private.param<double>("fault_check_hz", faultCheckHz_, 1.0);
    nh_private.param<double>("task_state_hz", taskStateHz_, 1.0);
    nh_private.param<int>("vcu_motion_timeout_ms", vcuMotionTimeoutMs_, 30);
    nh_private.param<int>("vcu_basic_timeout_ms", vcuBasicTimeoutMs_, 300);
    nh_private.param<double>("walk_linear_speed", walkLinearSpeed_, 1.0);
    nh_private.param<double>("walk_angular_speed", walkAngularSpeed_, 0.5);
}

// ============================================================
// 主循环
// ============================================================

void CanBridgeNode::spin() {
    ros::Rate rate(200);  // 200Hz = 5ms, 快于最快 CAN 帧 (10ms)

    while (ros::ok() && running_) {
        ros::spinOnce();
        receiveCanFrames();
        rate.sleep();
    }
}

// ============================================================
// SocketCAN 初始化
// ============================================================

bool CanBridgeNode::initSocketCan() {
    sockFd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockFd_ < 0) {
        ROS_ERROR("[can_bridge] socket(PF_CAN) failed: %s", strerror(errno));
        return false;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, canInterface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(sockFd_, SIOCGIFINDEX, &ifr) < 0) {
        ROS_ERROR("[can_bridge] ioctl(SIOCGIFINDEX) failed for %s: %s",
                  canInterface_.c_str(), strerror(errno));
        close(sockFd_);
        sockFd_ = -1;
        return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sockFd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        ROS_ERROR("[can_bridge] bind() failed for %s: %s",
                  canInterface_.c_str(), strerror(errno));
        close(sockFd_);
        sockFd_ = -1;
        return false;
    }

    // 设为非阻塞模式，避免 read() 阻塞 ROS 事件循环
    int flags = fcntl(sockFd_, F_GETFL, 0);
    fcntl(sockFd_, F_SETFL, flags | O_NONBLOCK);

    ROS_INFO("[can_bridge] SocketCAN %s initialized successfully", canInterface_.c_str());
    return true;
}

void CanBridgeNode::closeSocketCan() {
    if (sockFd_ >= 0) {
        close(sockFd_);
        sockFd_ = -1;
    }
}

// ============================================================
// CAN 收发
// ============================================================

void CanBridgeNode::receiveCanFrames() {
    if (sockFd_ < 0) return;

    struct can_frame frame;
    while (true) {
        ssize_t n = read(sockFd_, &frame, sizeof(frame));
        if (n <= 0) break;  // 无更多帧或错误

        uint32_t canId = frame.can_id & CAN_EFF_MASK;  // 29位扩展帧
        if (!CanCodec::isVcuFrame(canId)) continue;

        CanFrame data;
        size_t copyLen = std::min(static_cast<size_t>(frame.can_dlc), data.size());
        std::copy(frame.data, frame.data + copyLen, data.begin());
        onCanFrame(canId, data);
    }
}

void CanBridgeNode::sendCanFrame(uint32_t canId, const CanFrame& data) {
    if (sockFd_ < 0) return;

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id  = canId | CAN_EFF_FLAG;  // 29位扩展帧
    frame.can_dlc = static_cast<uint8_t>(data.size());
    std::copy(data.begin(), data.end(), frame.data);

    write(sockFd_, &frame, sizeof(frame));
}

// ============================================================
// CAN 帧处理
// ============================================================

void CanBridgeNode::onCanFrame(uint32_t canId, const CanFrame& frame) {
    switch (canId) {
    case 0x18FF5021:
        bmsBasic_ = CanCodec::decodeBmsBasic(frame);
        lastBasicTime_ = ros::Time::now();
        break;
    case 0x18FF5121:
        bmsCell_ = CanCodec::decodeBmsCell(frame);
        break;
    case 0x18FF5221:
        bmsTemp_ = CanCodec::decodeBmsTemp(frame);
        break;
    case 0x18FF5321:
        motion_ = CanCodec::decodeMotion(frame);
        lastMotionTime_ = ros::Time::now();
        break;
    case 0x18FF5421:
        motorPos_ = CanCodec::decodeMotorPosition(frame);
        break;
    case 0x18FF5521: {
        VcuFaultFlags old = vcuFlags_;
        vcuFlags_ = CanCodec::decodeVcuFaultFlags(frame);
        // 故障位变化 → 立即上报
        if (old.fresh && std::memcmp(&old, &vcuFlags_, sizeof(VcuFaultFlags) - sizeof(bool)) != 0) {
            publishFaultAlarm(ros::TimerEvent());
        }
        break;
    }
    case 0x18FF5621:
        bmsFault_ = CanCodec::decodeBmsFault(frame);
        break;
    case 0x18FF5721:
        motorFaults_ = CanCodec::decodeMotorFaultBitmap(frame);
        break;
    case 0x18FF5821:
        motorSpeed_ = CanCodec::decodeMotorSpeed(frame);
        break;
    default:
        break;
    }
}

// ============================================================
// 上行: CAN → JSON → ROS Topic
// ============================================================

void CanBridgeNode::publishRobotStatus(const ros::TimerEvent&) {
    if (!bmsBasic_.fresh || !bmsCell_.fresh || !bmsTemp_.fresh
        || !motion_.fresh || !motorSpeed_.fresh || !vcuFlags_.fresh) {
        return;  // 关键帧未就绪
    }

    int64_t mid = msgIdCounter_.fetch_add(1);
    json j = JsonHelper::buildRobotStatus(
        mid,
        0.0, 0.0,           // reportLocation {x, y} — 由定位模块提供 (V1.0.3)
        reportYaw_,          // reportLocation yaw (V1.0.3 新增) — 由定位模块提供
        0.0,                 // estimatedRange — 上层计算
        bmsBasic_.totalVoltage,
        bmsBasic_.totalCurrent,
        bmsBasic_.soc,
        bmsCell_.maxCellVoltage,
        std::to_string(bmsCell_.maxCellVoltageSerialNumber),
        bmsCell_.minCellVoltage,
        std::to_string(bmsCell_.minCellVoltageSerialNumber),
        bmsTemp_.maxTemperature,
        std::to_string(bmsTemp_.maxTemperatureId),
        bmsTemp_.minTemperature,
        std::to_string(bmsTemp_.minTemperatureId),
        motion_.linearSpeed,
        motion_.angularSpeed,
        static_cast<int>(vcuFlags_.vehStopSwt),
        static_cast<int>(motorSpeed_.leftRequested),
        static_cast<int>(motorSpeed_.rightRequested),
        static_cast<int>(motorSpeed_.leftActual),
        static_cast<int>(motorSpeed_.rightActual)
    );

    std_msgs::String msg;
    msg.data = JsonHelper::serialize(j);
    statusPub_.publish(msg);
}

void CanBridgeNode::publishFaultAlarm(const ros::TimerEvent&) {
    std::vector<FaultAlarmItem> faults, alarms;
    collectFaults(faults, alarms);

    // 仅当有故障时才上报
    if (faults.empty() && alarms.empty()) {
        lastFaults_.clear();
        lastAlarms_.clear();
        return;
    }

    // 变化检测：仅故障/告警集合变化时才上报，避免重复
    if (faults == lastFaults_ && alarms == lastAlarms_) return;

    lastFaults_ = faults;
    lastAlarms_ = alarms;

    FaultAlarmReport report;
    report.faults = faults;
    report.alarms = alarms;

    int64_t mid = msgIdCounter_.fetch_add(1);
    json j = JsonHelper::buildFaultAlarm(mid, report);

    std_msgs::String msg;
    msg.data = JsonHelper::serialize(j);
    faultPub_.publish(msg);
}

void CanBridgeNode::publishTaskState(const ros::TimerEvent&) {
    // ============================================================
    // V1.0.3 任务状态上报逻辑 (文档 9 条规则)
    // ============================================================
    TaskStateCode state = TaskStateCode::IDLE;  // 规则2: 默认空闲中

    // 规则5: 收到急停指令 → 急停中
    bool emergencyActive = vcuFlags_.fresh && vcuFlags_.vehStopSwt;
    // 规则7: 遥控手柄使能 → 远程接管中
    bool remoteTakeoverActive = vcuFlags_.fresh && vcuFlags_.handleCtrlEn;
    // 检查 CAN 数据是否有效
    bool vcuOnline = vcuFlags_.fresh;
    bool bmsOnline = bmsFault_.fresh;

    if (emergencyActive) {
        // 规则5: 急停中
        state = TaskStateCode::EMERGENCY_STOP;
    } else if (remoteTakeoverActive) {
        // 规则7: 远程接管中
        state = TaskStateCode::REMOTE_TAKEOVER;
    } else if (bmsOnline && bmsFault_.chargeState == 1) {
        // 规则8: 充电中
        state = TaskStateCode::CHARGING;
    } else if (bmsOnline && bmsFault_.gunConnected && bmsFault_.chargeState == 0) {
        // 规则8: 拔枪中
        state = TaskStateCode::UNPLUGGING;
    } else if (taskActive_ && !taskFinished_) {
        // 规则3: 收到任务后上报导航中 (onTaskCmd 设置 taskActive_=true)
        // 规则9: 结束远程控制后，有任务→导航中
        state = TaskStateCode::NAVIGATING;
    } else if (taskFinished_) {
        // 规则4: 完成任务→已结束
        state = TaskStateCode::FINISHED;
    } else if (!vcuOnline) {
        // 规则1: VCU 未就绪→无法执行
        state = TaskStateCode::CANNOT_EXECUTE;
    }
    // 否则保持规则2: 空闲中 (IDLE)

    int64_t taskId = 0;
    if (state == TaskStateCode::NAVIGATING ||
        state == TaskStateCode::CHARGING ||
        state == TaskStateCode::UNPLUGGING ||
        state == TaskStateCode::EMERGENCY_STOP ||
        state == TaskStateCode::FINISHED) {
        // 规则3/4/5/8: 这些状态需附带任务编号
        taskId = currentTaskId_;
    }

    int64_t mid = msgIdCounter_.fetch_add(1);
    json j = JsonHelper::buildTaskState(mid, taskId, state);

    std_msgs::String msg;
    msg.data = JsonHelper::serialize(j);
    taskStatePub_.publish(msg);
}

// ============================================================
// 故障码收集
// ============================================================

void CanBridgeNode::collectMotorFaults(uint32_t bitmap,
                                        std::vector<FaultAlarmItem>& faults) {
    // 左电机: baseCode=1000, 右电机: baseCode=2000 (此函数只遍历一个位图)
    // 调用方需传入正确的 bitmap (左/右)
    if (bitmap == 0) return;

    // 前13个故障位有明确定义
    static const int definedFaultCodes[13] = {
        1000, 1001, 1002, 1003, 1004, 1005, 1006,
        1007, 1008, 1009, 1010, 1011, 1012
    };

    for (int bit = 0; bit < 32; ++bit) {
        if (bitmap & (1u << bit)) {
            int code;
            if (bit < 13) {
                code = definedFaultCodes[bit];
            } else {
                code = 1013 + (bit - 13);  // 扩展故障码
            }
            faults.push_back({code, 1});
        }
    }
}

void CanBridgeNode::collectFaults(std::vector<FaultAlarmItem>& faults,
                                   std::vector<FaultAlarmItem>& alarms) {
    // 1. 电机故障位图
    if (motorFaults_.fresh) {
        // 左电机 (1000-1031)
        for (int bit = 0; bit < 32; ++bit) {
            if (motorFaults_.leftFaults & (1u << bit)) {
                int code = (bit < 13) ?
                    (MotorFaultLeft::CRC_FAIL + bit) : (MotorFaultLeft::EXT_BASE + (bit - 13));
                faults.push_back({code, 1});
            }
        }
        // 右电机 (2000-2031)
        for (int bit = 0; bit < 32; ++bit) {
            if (motorFaults_.rightFaults & (1u << bit)) {
                int code = (bit < 13) ?
                    (MotorFaultRight::CRC_FAIL + bit) : (MotorFaultRight::EXT_BASE + (bit - 13));
                faults.push_back({code, 1});
            }
        }
    }

    // 2. BMS 报警码 (0x18FF5621)
    if (bmsFault_.fresh && bmsFault_.errorCode != 0xFF && bmsFault_.errorCode != 0) {
        alarms.push_back({static_cast<int>(bmsFault_.errorCode),
                          static_cast<int>(bmsFault_.errorLevel)});
    }

    // 3. 0x18FF5521 故障标志位 (3001-3023)
    if (!vcuFlags_.fresh) return;

    // 辅助宏: 如果 flag 为 true 则添加到 faults
    auto addFault = [&](int code, bool flag) {
        if (flag) faults.push_back({code, 1});
    };

    // 机械臂故障
    addFault(3001, vcuFlags_.robotArmErr);
    // 通讯故障
    addFault(3002, vcuFlags_.mcuOffLine);
    addFault(3003, vcuFlags_.bmsOffLine);
    // 插枪信号诊断
    addFault(3004, vcuFlags_.gunInShrtGnd);
    addFault(3005, vcuFlags_.gunInShrtPwr);
    addFault(3006, vcuFlags_.gunInOpen);
    // 拔枪信号诊断
    addFault(3007, vcuFlags_.gunOutShrtGnd);
    addFault(3008, vcuFlags_.gunOutShrtPwr);
    addFault(3009, vcuFlags_.gunOutOpen);
    // DCDC 掉线
    addFault(3010, vcuFlags_.dcdc1OffLine);
    addFault(3011, vcuFlags_.dcdc2OffLine);
    // 电机控制故障
    addFault(3012, vcuFlags_.mtSetCtrlModErr);
    addFault(3013, vcuFlags_.mtSetProtectErr);
    addFault(3014, vcuFlags_.mtEnErr);
    addFault(3015, vcuFlags_.mtDisEnErr);
    // 机械臂信号诊断
    addFault(3016, vcuFlags_.armStartShrtGnd);
    addFault(3017, vcuFlags_.armStartShrtPwr);
    addFault(3018, vcuFlags_.armStartOpen);
    addFault(3019, vcuFlags_.armResetShrtGnd);
    addFault(3020, vcuFlags_.armResetShrtPwr);
    addFault(3021, vcuFlags_.armResetOpen);
    // 蓄电池
    addFault(3022, vcuFlags_.battLowVolt);
    addFault(3023, vcuFlags_.battHightVolt);
}

// ============================================================
// VCU 通信超时检测
// ============================================================

void CanBridgeNode::checkVcuTimeout(const ros::TimerEvent&) {
    auto now = ros::Time::now();

    bool motionOk = (now - lastMotionTime_).toSec() * 1000.0 < vcuMotionTimeoutMs_;
    bool basicOk  = (now - lastBasicTime_).toSec() * 1000.0 < vcuBasicTimeoutMs_;

    bool online = motionOk && basicOk;

    if (online != vcuOnline_) {
        vcuOnline_ = online;
        if (!online) {
            ROS_WARN("[can_bridge] VCU communication timeout detected");
        } else {
            ROS_INFO("[can_bridge] VCU communication restored");
        }
    }
}

// ============================================================
// 下行: ROS Topic → CAN
// ============================================================

void CanBridgeNode::onCtrlCmd(const std_msgs::String::ConstPtr& msg) {
    try {
        json j = JsonHelper::deserialize(msg->data);
        CtrlCmdCode cmdCode;
        int walkDir = 0;
        double param = 0.0;
        LocPose loc;
        if (!JsonHelper::parseCtrlCmd(j, cmdCode, walkDir, param, loc)) return;

        std::lock_guard<std::mutex> lock(cmdMutex_);

        switch (cmdCode) {
        case CtrlCmdCode::LIGHT:
            currentCtrlCmd_.lightCtrl = static_cast<uint8_t>(param);  // 0=关,1=红,2=绿,3=黄
            break;
        case CtrlCmdCode::EMERGENCY:
            currentCtrlCmd_.emergencyStop = (param != 0);
            break;
        case CtrlCmdCode::DRIVE_ENABLE:
            currentCtrlCmd_.driveEnable = (param != 0);
            break;
        case CtrlCmdCode::STOP_WORK:
            // V1.0.3 规则6: 终止工作, 完成当前任务后上报空闲中
            currentCtrlCmd_.driveEnable = false;
            taskActive_   = false;
            taskFinished_ = false;
            break;
        case CtrlCmdCode::WALK:
            // V1.0.3 行走控制: walkDir(ctrlValue) 0=前进 1=后退 2=左转 3=右转
            // 当前协议 ctrlValue 仅承载方向，不承载速度；速度由 yaml 配置提供默认值。
            // TODO: 协议升级后在 data 新增 speed 字段，届时由云平台实时下发速度。
            {
                double linSpeed  = walkLinearSpeed_;
                double angSpeed  = walkAngularSpeed_;
                currentMotionCmd_.linearSpeed  = 0;
                currentMotionCmd_.angularSpeed = 0;
                switch (walkDir) {
                    case 0: currentMotionCmd_.linearSpeed  = static_cast<int16_t>(linSpeed * 1000); break;
                    case 1: currentMotionCmd_.linearSpeed  = static_cast<int16_t>(-linSpeed * 1000); break;
                    case 2: currentMotionCmd_.angularSpeed = static_cast<int16_t>(angSpeed * 1000); break;
                    case 3: currentMotionCmd_.angularSpeed = static_cast<int16_t>(-angSpeed * 1000); break;
                }
            }
            break;
        case CtrlCmdCode::GOTO_TARGET:  // V1.0.3 新增: 前往目标点
            // loc 坐标已在 parseCtrlCmd 中解析，终止当前运动，由导航层接管
            currentMotionCmd_.linearSpeed  = 0;
            currentMotionCmd_.angularSpeed = 0;
            // loc 坐标 (loc.x, loc.y, loc.yaw) 由上层导航模块读取并规划路径
            ROS_INFO("[can_bridge] GOTO_TARGET: x=%.2f, y=%.2f, yaw=%.2f", loc.x, loc.y, loc.yaw);
            break;
        }
    } catch (const std::exception& e) {
        ROS_WARN("[can_bridge] ctrl_cmd parse error: %s", e.what());
    }
}

void CanBridgeNode::onTaskCmd(const std_msgs::String::ConstPtr& msg) {
    // V1.0.3 规则3: 接收到任务后上报导航中，附带 taskId
    try {
        json j = JsonHelper::deserialize(msg->data);
        const auto& data = j["data"];
        if (data.contains("taskId")) {
            currentTaskId_ = data["taskId"].get<int64_t>();
        }
        taskActive_   = true;
        taskFinished_ = false;
        ROS_INFO("[can_bridge] Task received: taskId=%ld, switching to NAVIGATING", currentTaskId_);
    } catch (const std::exception& e) {
        ROS_WARN("[can_bridge] task_cmd parse error: %s", e.what());
    }
}

void CanBridgeNode::onBatteryLevelDown(const std_msgs::String::ConstPtr& msg) {
    // 回归充电电量下发 — 由上层决策模块处理, CAN 层仅转发
    ROS_DEBUG("[can_bridge] Received battery_level_down: %s", msg->data.c_str());
}

void CanBridgeNode::onTaskCtrl(const std_msgs::String::ConstPtr& msg) {
    // V1.0.3 规则4: 任务控制命令 → 更新 taskActive_/taskFinished_
    try {
        json j = JsonHelper::deserialize(msg->data);
        const auto& data = j["data"];
        int ctrlCmd = data.value("ctrlCmd", 0);

        switch (ctrlCmd) {
        case 1:  // 暂停任务
            taskActive_   = false;
            taskFinished_ = false;
            ROS_INFO("[can_bridge] Task paused, switching to IDLE");
            break;
        case 2:  // 恢复任务
            taskActive_   = true;
            taskFinished_ = false;
            ROS_INFO("[can_bridge] Task resumed, switching to NAVIGATING");
            break;
        case 3:  // 取消任务
            taskFinished_ = true;
            taskActive_   = false;
            ROS_INFO("[can_bridge] Task cancelled, switching to FINISHED");
            break;
        default:
            ROS_WARN("[can_bridge] Unknown task ctrlCmd: %d", ctrlCmd);
            break;
        }
    } catch (const std::exception& e) {
        ROS_WARN("[can_bridge] task_ctrl parse error: %s", e.what());
    }
}

// ============================================================
// 下行 CAN 定时发送
// ============================================================

void CanBridgeNode::sendControlFrame(const ros::TimerEvent&) {
    IpcControlCmd cmd;
    {
        std::lock_guard<std::mutex> lock(cmdMutex_);
        cmd = currentCtrlCmd_;
    }
    CanFrame frame = CanCodec::encodeControlCmd(cmd);
    sendCanFrame(0x200, frame);
}

void CanBridgeNode::sendMotionFrame(const ros::TimerEvent&) {
    IpcMotionCmd cmd;
    {
        std::lock_guard<std::mutex> lock(cmdMutex_);
        cmd = currentMotionCmd_;
        cmd.heartbeat++;  // 工控机心跳递增
    }
    CanFrame frame = CanCodec::encodeMotionCmd(cmd);
    sendCanFrame(0x201, frame);
}

} // namespace st_proto

// ============================================================
// main
// ============================================================

int main(int argc, char** argv) {
    ros::init(argc, argv, "st_can_bridge");

    st_proto::CanBridgeNode node;
    node.spin();

    return 0;
}
