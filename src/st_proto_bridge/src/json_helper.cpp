#include "st_proto_bridge/json_helper.h"
#include <stdexcept>

namespace st_proto {

// ============================================================
// 上行消息构建
// ============================================================

json JsonHelper::buildRobotStatus(
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
    int rightMotorFeedbackActualSpeed)
{
    json j;
    j["cmd"] = BusinessCmd::ROBOT_STATUS;
    j["mid"] = mid;

    // data 包裹层 — 所有业务字段按需求文档 V1.0.2 置于 data 内
    json& data = j["data"];

    // 位置信息
    data["reportLocation"]["x"]   = reportLocationX;
    data["reportLocation"]["y"]   = reportLocationY;
    data["reportLocation"]["yaw"] = reportLocationYaw;

    // 电池状态
    data["estimatedRange"]               = estimatedRange;
    data["batteryPackTotalVoltage"]       = batteryPackTotalVoltage;
    data["batteryPackTotalCurrent"]       = batteryPackTotalCurrent;
    data["batterySoc"]                    = batterySoc;

    // 单体电压最高/最低
    data["maxCellVoltage"]              = maxCellVoltage;
    data["maxCellVoltageSerialNumber"]  = maxCellVoltageSerialNumber;
    data["minCellVoltage"]              = minCellVoltage;
    data["minCellVoltageSerialNumber"]  = minCellVoltageSerialNumber;

    // 电池温度最高/最低
    data["batteryMaxTemperature"]       = batteryMaxTemperature;
    data["batteryMaxTemperatureId"]     = batteryMaxTemperatureId;
    data["batteryMinTemperature"]       = batteryMinTemperature;
    data["batteryMinTemperatureId"]     = batteryMinTemperatureId;

    // 质心速度
    data["centroidLinearSpeed"]  = centroidLinearSpeed;
    data["centroidAngularSpeed"] = centroidAngularSpeed;

    // 急停开关状态
    data["vehicleEmergencyStopSwitchStatus"] = vehicleEmergencyStopSwitchStatus;

    // 电机速度
    data["vcuRequestedLeftMotorSpeed"]       = vcuRequestedLeftMotorSpeed;
    data["vcuRequestedRightMotorSpeed"]      = vcuRequestedRightMotorSpeed;
    data["leftMotorFeedbackActualSpeed"]     = leftMotorFeedbackActualSpeed;
    data["rightMotorFeedbackActualSpeed"]    = rightMotorFeedbackActualSpeed;

    return j;
}

json JsonHelper::buildFaultAlarm(int64_t mid, const FaultAlarmReport& report)
{
    json j;
    j["cmd"] = BusinessCmd::ROBOT_FAULT;
    j["mid"] = mid;

    // data 包裹层 — 所有业务字段按需求文档 V1.0.2 置于 data 内
    json& data = j["data"];

    // 故障列表
    json faultArray = json::array();
    for (const auto& f : report.faults) {
        json item;
        item["code"]  = f.code;
        item["level"] = f.level;
        faultArray.push_back(item);
    }
    data["faults"] = faultArray;

    // 告警列表
    json alarmArray = json::array();
    for (const auto& a : report.alarms) {
        json item;
        item["code"]  = a.code;
        item["level"] = a.level;
        alarmArray.push_back(item);
    }
    data["alarms"] = alarmArray;

    return j;
}

json JsonHelper::buildTaskState(
    int64_t mid,
    int64_t taskId,
    TaskStateCode state)
{
    json j;
    j["cmd"] = BusinessCmd::TASK_STATE;
    j["mid"] = mid;

    json& data = j["data"];
    data["taskId"] = taskId;
    data["state"]  = static_cast<int>(state);
    return j;
}

json JsonHelper::buildLocalPath(
    int64_t mid,
    int64_t taskId,
    const std::vector<std::pair<double, double>>& points)
{
    json j;
    j["cmd"] = BusinessCmd::LOCAL_PATH;
    j["mid"] = mid;

    json& data = j["data"];
    data["taskId"] = taskId;

    json pathArray = json::array();
    int index = 1;  // V1.0.2: 点序号从1开始
    for (const auto& pt : points) {
        json point;
        point["i"] = index++;
        point["x"] = pt.first;
        point["y"] = pt.second;
        point["z"] = 0.0;  // V1.0.3: z 字段(可选,默认0)
        pathArray.push_back(point);
    }
    data["points"] = pathArray;
    return j;
}

json JsonHelper::buildResp(
    const std::string& mid,
    const std::string& ackCmd,
    int retCode,
    const std::string& msg)
{
    // V1.0.3 格式: {"cmd":"resp","mid":N,"result":1}(方向: server→client)
    //      或: {"cmd":"resp","mid":N,"ackCmd":"...","retCode":0,"msg":"ok"}(方向: client→server)
    json j;
    j["cmd"]    = BusinessCmd::RESP;
    j["mid"]    = std::stoll(mid);  // 转换为数值类型
    j["result"] = (retCode == 0) ? 1 : 0;
    if (!ackCmd.empty()) j["ackCmd"] = ackCmd;
    if (!msg.empty())    j["msg"]   = msg;
    return j;
}

json JsonHelper::buildBatteryLevelDownAck(const std::string& mid)
{
    json j;
    j["cmd"]    = BusinessCmd::RESP;
    j["mid"]    = std::stoll(mid);
    j["result"] = 1;
    return j;
}

json JsonHelper::buildTaskCtrlAck(const std::string& mid)
{
    json j;
    j["cmd"]    = BusinessCmd::RESP;
    j["mid"]    = std::stoll(mid);
    j["result"] = 1;
    return j;
}

// ============================================================
// 下行消息解析
// ============================================================

std::string JsonHelper::extractCmd(const json& j) {
    if (!j.contains("cmd")) {
        throw std::runtime_error("JSON missing 'cmd' field");
    }
    return j["cmd"].get<std::string>();
}

std::string JsonHelper::extractMid(const json& j) {
    return std::to_string(j.value("mid", 0));
}

bool JsonHelper::parseCtrlCmd(const json& j, CtrlCmdCode& cmdCode, int& walkDir, double& param, LocPose& loc) {
    if (j.value("cmd", "") != BusinessCmd::ROBOT_CTRL) {
        return false;
    }
    const auto& data = j["data"];
    cmdCode = static_cast<CtrlCmdCode>(data.value("ctrlCmd", 0));
    int ctrlValue = data.value("ctrlValue", 0);
    param   = static_cast<double>(ctrlValue);
    walkDir = ctrlValue;  // ctrlCmd==5 时 ctrlValue 即行走方向

    // 解析可选的 loc 字段（终止工作命令附带充电基站坐标）
    loc.x   = 0.0;
    loc.y   = 0.0;
    loc.yaw = 0.0;
    if (data.contains("loc") && data["loc"].is_object()) {
        loc.x   = data["loc"].value("x", 0.0);
        loc.y   = data["loc"].value("y", 0.0);
        loc.yaw = data["loc"].value("yaw", 0.0);
    }
    return true;
}

bool JsonHelper::parseTaskCreate(const json& j, TaskCreateData& data) {
    if (j.value("cmd", "") != BusinessCmd::TASK_CREATE) {
        return false;
    }
    const auto& body = j["data"];
    data.taskId          = body.value("taskId", 0);
    data.taskType        = body.value("taskType", 0);
    data.operationType   = body.value("operationType", 0);
    data.chargingEquipment = body.value("chargingEquipment", "");
    data.chargingPort    = body.value("chargingPort", "");

    data.chargingPortLoc.x = 0.0;
    data.chargingPortLoc.y = 0.0;
    if (body.contains("chargingPortLoc") && body["chargingPortLoc"].is_object()) {
        data.chargingPortLoc.x = body["chargingPortLoc"].value("x", 0.0);
        data.chargingPortLoc.y = body["chargingPortLoc"].value("y", 0.0);
    }
    return true;
}

bool JsonHelper::parseBatteryLevelDown(const json& j, BatteryLevelDownData& data) {
    if (j.value("cmd", "") != BusinessCmd::BATTERY_LEVEL_DOWN) {
        return false;
    }
    data.batteryLevel = j["data"].value("batteryLevel", 0.0);
    return true;
}

bool JsonHelper::parseTaskCtrl(const json& j, TaskCtrlData& data) {
    if (j.value("cmd", "") != BusinessCmd::TASK_CTRL) {
        return false;
    }
    const auto& body = j["data"];
    data.taskId  = body.value("taskId", 0);
    data.ctrlCmd = body.value("ctrlCmd", 0);
    return true;
}

// ============================================================
// 工具方法
// ============================================================

std::string JsonHelper::serialize(const json& j) {
    return j.dump();
}

json JsonHelper::deserialize(const std::string& str) {
    return json::parse(str);
}

bool JsonHelper::validate(const json& j, const std::vector<std::string>& requiredFields) {
    for (const auto& field : requiredFields) {
        if (!j.contains(field)) {
            return false;
        }
    }
    return true;
}

} // namespace st_proto