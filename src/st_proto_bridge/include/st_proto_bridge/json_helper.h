#ifndef ST_PROTO_BRIDGE_JSON_HELPER_H
#define ST_PROTO_BRIDGE_JSON_HELPER_H

#include "types.h"
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace st_proto {

using json = nlohmann::json;

/**
 * @brief JSON 业务数据构建/解析工具（V1.0.2）
 *
 * 负责：
 * - 构建上行业务JSON（机器人状态上报report、故障告警、任务状态、局部路径、通用回复）
 * - 解析下行业务JSON（机器人控制、任务下发、回归充电电量下发、任务控制）
 */
class JsonHelper {
public:
    // ============================================================
    // 上行消息构建 (Pub → Topic)
    // ============================================================

    /// 构建机器人状态上报JSON（V1.0.2 "report"）
    /// @param reportLocation 位置 {x, y, yaw}（机器人在地图中的平面坐标及航向角）
    /// @param estimatedRange 预估剩余里程（km）
    /// @param batteryPackTotalVoltage 电池包总电压（V）
    /// @param batteryPackTotalCurrent 电池包总电流（A）
    /// @param batterySoc 电池SOC（%）
    /// @param maxCellVoltage 单体最高电压（V）
    /// @param maxCellVoltageSerialNumber 单体最高电压序号
    /// @param minCellVoltage 单体最低电压（V）
    /// @param minCellVoltageSerialNumber 单体最低电压序号
    /// @param batteryMaxTemperature 电池最高温度（℃）
    /// @param batteryMaxTemperatureId 电池最高温度ID
    /// @param batteryMinTemperature 电池最低温度（℃）
    /// @param batteryMinTemperatureId 电池最低温度ID
    /// @param centroidLinearSpeed 质心线速度（m/s）
    /// @param centroidAngularSpeed 质心角速度（rad/s）
    /// @param vehicleEmergencyStopSwitchStatus 急停开关状态
    /// @param vcuRequestedLeftMotorSpeed VCU请求左电机转速（rpm）
    /// @param vcuRequestedRightMotorSpeed VCU请求右电机转速（rpm）
    /// @param leftMotorFeedbackActualSpeed 左电机反馈实际转速（rpm）
    /// @param rightMotorFeedbackActualSpeed 右电机反馈实际转速（rpm）
    static json buildRobotStatus(
        int64_t mid,
        double reportLocationX,
        double reportLocationY,
        double reportLocationYaw,
        double estimatedRange,
        double batteryPackTotalVoltage,
        double batteryPackTotalCurrent,
        double batterySoc,
        double maxCellVoltage,
        const std::string& maxCellVoltageSerialNumber,
        double minCellVoltage,
        const std::string& minCellVoltageSerialNumber,
        double batteryMaxTemperature,
        const std::string& batteryMaxTemperatureId,
        double batteryMinTemperature,
        const std::string& batteryMinTemperatureId,
        double centroidLinearSpeed,
        double centroidAngularSpeed,
        int vehicleEmergencyStopSwitchStatus,
        int vcuRequestedLeftMotorSpeed,
        int vcuRequestedRightMotorSpeed,
        int leftMotorFeedbackActualSpeed,
        int rightMotorFeedbackActualSpeed
    );

    /// 构建故障告警上报JSON（V1.0.2 批量结构）
    static json buildFaultAlarm(int64_t mid, const FaultAlarmReport& report);

    /// 构建任务状态上报JSON（V1.0.2 简化版）
    static json buildTaskState(
        int64_t mid,
        int64_t taskId,
        TaskStateCode state
    );

    /// 构建局部路径上报JSON（V1.0.2 带序号）
    static json buildLocalPath(
        int64_t mid,
        int64_t taskId,
        const std::vector<std::pair<double, double>>& points
    );

    /// 构建通用回复JSON（V1.0.2 带mid）
    static json buildResp(
        const std::string& mid,
        const std::string& ackCmd,
        int retCode,
        const std::string& msg = ""
    );

    /// 构建回归充电电量回复ACK
    static json buildBatteryLevelDownAck(const std::string& mid);

    /// 构建任务控制回复ACK
    static json buildTaskCtrlAck(const std::string& mid);

    // ============================================================
    // 下行消息解析
    // ============================================================

    /// 从JSON提取业务命令类型和mid
    static std::string extractCmd(const json& j);
    static std::string extractMid(const json& j);

    /// 解析控制命令（V1.0.2 CtrlCmdCode + 可选loc）
    static bool parseCtrlCmd(const json& j, CtrlCmdCode& cmdCode, int& walkDir, double& param, LocPose& loc);

    /// 解析任务下发（V1.0.2 新字段）
    static bool parseTaskCreate(const json& j, TaskCreateData& data);

    /// 解析回归充电电量下发
    static bool parseBatteryLevelDown(const json& j, BatteryLevelDownData& data);

    /// 解析任务控制命令
    static bool parseTaskCtrl(const json& j, TaskCtrlData& data);

    // ============================================================
    // 工具方法
    // ============================================================

    static std::string serialize(const json& j);
    static json deserialize(const std::string& str);
    static bool validate(const json& j, const std::vector<std::string>& requiredFields);
};

} // namespace st_proto

#endif // ST_PROTO_BRIDGE_JSON_HELPER_H
