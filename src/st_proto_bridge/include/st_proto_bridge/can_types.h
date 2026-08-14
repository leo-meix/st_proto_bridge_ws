#ifndef ST_PROTO_BRIDGE_CAN_TYPES_H
#define ST_PROTO_BRIDGE_CAN_TYPES_H

#include <cstdint>

namespace st_proto {

// ============================================================
// VCU → 工控机 CAN 帧解码数据 (12 路)
// 基于: 机器人项目与工控机通讯协议 V1.7
// ============================================================

/// 0x18FF5021: BMS 综合信息 (100ms)
struct BmsBasicData {
    double totalVoltage   = 0.0;  // 总电压 (V), 精度 ×0.1
    double totalCurrent   = 0.0;  // 总电流 (A), 精度 ×0.1, 偏移 -1000
    double soc            = 0.0;  // SOC (%), 精度 ×0.1
    bool   fresh          = false;
};

/// 0x18FF5121: 单体电压 (100ms)
struct BmsCellData {
    double   maxCellVoltage             = 0.0;  // 最高单体电压 (V), ×0.001
    uint16_t maxCellVoltageSerialNumber = 0;    // 最高单体电压串号
    double   minCellVoltage             = 0.0;  // 最低单体电压 (V), ×0.001
    uint16_t minCellVoltageSerialNumber = 0;    // 最低单体电压串号
    bool     fresh = false;
};

/// 0x18FF5221: 温度信息 (100ms)
struct BmsTempData {
    double   maxTemperature   = 0.0;  // 最高温度 (℃), 偏移 -50
    uint16_t maxTemperatureId = 0;    // 最高温度编号
    double   minTemperature   = 0.0;  // 最低温度 (℃), 偏移 -50
    uint16_t minTemperatureId = 0;    // 最低温度编号
    bool     fresh = false;
};

/// 0x18FF5321: 速度 + VCU 心跳 (10ms)
struct MotionData {
    double  linearSpeed   = 0.0;  // 质心线速度 (m/s), ×0.001, sint16
    double  angularSpeed  = 0.0;  // 质心角速度 (rad/s), ×0.001, sint16
    uint8_t vcuHeartbeat  = 0;    // VCU 心跳计数
    bool    fresh = false;
};

/// 0x18FF5421: 电机反馈位置 (10ms)
struct MotorPositionData {
    uint32_t leftRotation  = 0;   // 左电机反馈位置 (÷10000=转数)
    uint32_t rightRotation = 0;   // 右电机反馈位置
    bool     fresh = false;
};

/// 0x18FF5521: VCU 故障/状态标志位 (100ms) — 56 bit
struct VcuFaultFlags {
    // Byte0 (bits 0-7)
    bool gunIn         : 1;  // bit0:  插枪
    bool gunOut        : 1;  // bit1:  拔枪
    bool leftMotorEn   : 1;  // bit2:  左电机使能
    bool rightMotorEn  : 1;  // bit3:  右电机使能
    bool robotArmErr   : 1;  // bit4:  机械臂故障
    bool robotArmReady : 1;  // bit5:  机械臂准备完成
    bool mcuOffLine    : 1;  // bit6:  电机控制器通讯故障
    bool bmsOffLine    : 1;  // bit7:  电池通讯故障

    // Byte1 (bits 8-15)
    bool gunInShrtGnd  : 1;  // bit8:  插枪信号对地短路
    bool gunInShrtPwr  : 1;  // bit9:  插枪信号对电源短路
    bool gunInOpen     : 1;  // bit10: 插枪信号开路
    bool gunOutShrtGnd : 1;  // bit11: 拔枪信号对地短路
    bool gunOutShrtPwr : 1;  // bit12: 拔枪信号对电源短路
    bool gunOutOpen    : 1;  // bit13: 拔枪信号开路
    bool dcdc1OffLine  : 1;  // bit14: 48→24V DCDC 掉线
    bool dcdc2OffLine  : 1;  // bit15: 48→12V DCDC 掉线

    // Byte2 (bits 16-23)
    bool mtSetCtrlModErr : 1;  // bit16: 电机速度控制模式故障
    bool mtSetProtectErr : 1;  // bit17: 电机节点超时故障
    bool mtEnErr         : 1;  // bit18: 电机使能故障
    bool mtDisEnErr      : 1;  // bit19: 电机断开使能故障
    bool vehStopSwt      : 1;  // bit20: 车上急停开关状态
    bool robotArmWork    : 1;  // bit21: 机械臂工作完成
    bool armStartFb      : 1;  // bit22: 机械臂启动反馈
    bool handleCtrlEn    : 1;  // bit23: 遥控手柄使能

    // Byte3 (bits 24-31)
    bool armFbWorkOverErr   : 1;  // bit24: 机械臂反馈超时报警
    bool handleCtrlArmOff   : 1;  // bit25: 遥控手柄机械臂下电指令
    bool handleCtrlArmReset : 1;  // bit26: 遥控手柄机械臂复位指令
    bool armReserve1        : 1;  // bit27: 机械臂预留1
    bool armReserve2        : 1;  // bit28: 机械臂预留2
    bool armStartShrtGnd    : 1;  // bit29: 机械臂启动对地短路
    bool armStartShrtPwr    : 1;  // bit30: 机械臂启动对电源短路
    bool armStartOpen       : 1;  // bit31: 机械臂启动开路

    // Byte4 (bits 32-39)
    bool armResetShrtGnd : 1;  // bit32: 机械臂复位对地短路
    bool armResetShrtPwr : 1;  // bit33: 机械臂复位对电源短路
    bool armResetOpen    : 1;  // bit34: 机械臂复位开路
    bool battLowVolt     : 1;  // bit35: 蓄电池电压低
    bool battHightVolt   : 1;  // bit36: 蓄电池电压高
    // bits 37-39: 预留

    bool fresh = false;
};

/// 0x18FF5621: BMS 故障码 (100ms)
struct BmsFaultData {
    uint8_t errorCode    = 0xFF;  // BMS 报警码 (0xFF=无故障)
    uint8_t errorLevel   = 0;     // 报警等级
    uint8_t chargeState  = 0;     // 充电状态: 0=未充电, 1=充电中, 2=充电完成
    bool    gunConnected = false; // 充电枪连接状态
    bool    fresh = false;
};

/// 0x18FF5721: 电机故障位图 (100ms)
struct MotorFaultBitmap {
    uint32_t leftFaults  = 0;     // 左电机 32 位故障位图
    uint32_t rightFaults = 0;     // 右电机 32 位故障位图
    bool     fresh = false;
};

/// 0x18FF5821: 电机转速 (10ms)
struct MotorSpeedData {
    int16_t leftRequested  = 0;   // VCU 请求左电机转速 (rpm), sint16
    int16_t rightRequested = 0;   // VCU 请求右电机转速 (rpm), sint16
    int16_t leftActual     = 0;   // 左电机反馈实际转速 (rpm), sint16
    int16_t rightActual    = 0;   // 右电机反馈实际转速 (rpm), sint16
    bool    fresh = false;
};

// ============================================================
// 工控机 → VCU CAN 编码数据 (2 路)
// ============================================================

/// 0x200: 控制命令 (100ms)
struct IpcControlCmd {
    bool     gunIn          = false;  // bit0: 插枪使能
    bool     gunOut         = false;  // bit1: 拔枪使能
    uint8_t  lightCtrl      = 0;      // bit2-3: 0=关闭,1=红(喇叭),2=绿,3=黄
    bool     emergencyStop  = false;  // bit4: 急停使能
    bool     driveEnable    = true;   // bit5: 驾驶使能，默认使能
    bool     armStart       = false;  // bit6: 机械臂启动(上电)
    bool     armReset       = false;  // bit7: 机械臂复位
};

/// 0x201: 速度命令 (10ms)
struct IpcMotionCmd {
    int16_t linearSpeed   = 100;    // 物理线速度 ×1000；默认 1 m/s，编码时叠加 -5 偏移
    int16_t angularSpeed  = 0;       // 物理角速度 ×1000；编码时叠加 -5 偏移
    uint8_t heartbeat     = 0;       // 工控机心跳计数
};

// ============================================================
// 故障码映射常量
// ============================================================

/// 左电机故障位 → TCP 故障码 (1000-1031)
namespace MotorFaultLeft {
    constexpr int CRC_FAIL          = 1000;  // bit0
    constexpr int DRIVER_INNER      = 1001;  // bit1
    constexpr int SHORT             = 1002;  // bit2
    constexpr int DRIVER_OVER_TEMP  = 1003;  // bit3
    constexpr int MOTOR_OVER_TEMP   = 1004;  // bit4
    constexpr int OVER_VOLT         = 1005;  // bit5
    constexpr int UNDER_VOLT        = 1006;  // bit6
    constexpr int FEEDBACK_ERR      = 1007;  // bit7
    constexpr int PHASE_ERR         = 1008;  // bit8
    constexpr int FOLLOW_ERR        = 1009;  // bit9
    constexpr int OVER_CURRENT      = 1010;  // bit10
    constexpr int FPGA_ERR          = 1011;  // bit11
    constexpr int INPUT_CMD_ERR     = 1012;  // bit12
    constexpr int EXT_BASE          = 1013;  // bit13-31 扩展起始
}

/// 右电机故障位 → TCP 故障码 (2000-2031)
namespace MotorFaultRight {
    constexpr int CRC_FAIL          = 2000;
    constexpr int DRIVER_INNER      = 2001;
    constexpr int SHORT             = 2002;
    constexpr int DRIVER_OVER_TEMP  = 2003;
    constexpr int MOTOR_OVER_TEMP   = 2004;
    constexpr int OVER_VOLT         = 2005;
    constexpr int UNDER_VOLT        = 2006;
    constexpr int FEEDBACK_ERR      = 2007;
    constexpr int PHASE_ERR         = 2008;
    constexpr int FOLLOW_ERR        = 2009;
    constexpr int OVER_CURRENT      = 2010;
    constexpr int FPGA_ERR          = 2011;
    constexpr int INPUT_CMD_ERR     = 2012;
    constexpr int EXT_BASE          = 2013;
}

} // namespace st_proto

#endif // ST_PROTO_BRIDGE_CAN_TYPES_H
