#include "st_proto_bridge/can_codec.h"
#include <cstring>

namespace st_proto {

// ============================================================
// Intel 小端序读取辅助
// ============================================================

uint16_t CanCodec::readU16Le(const uint8_t* data, int offset) {
    return static_cast<uint16_t>(data[offset]) |
           (static_cast<uint16_t>(data[offset + 1]) << 8);
}

int16_t CanCodec::readS16Le(const uint8_t* data, int offset) {
    uint16_t raw = readU16Le(data, offset);
    return static_cast<int16_t>(raw);
}

uint32_t CanCodec::readU32Le(const uint8_t* data, int offset) {
    return static_cast<uint32_t>(data[offset]) |
           (static_cast<uint32_t>(data[offset + 1]) << 8) |
           (static_cast<uint32_t>(data[offset + 2]) << 16) |
           (static_cast<uint32_t>(data[offset + 3]) << 24);
}

bool CanCodec::readBit(const uint8_t* data, int bitIndex) {
    int byteIdx = bitIndex / 8;
    int bitPos  = bitIndex % 8;
    return (data[byteIdx] >> bitPos) & 0x01;
}

// ============================================================
// 上行解码: VCU → 工控机
// ============================================================

BmsBasicData CanCodec::decodeBmsBasic(const CanFrame& frame) {
    BmsBasicData d;
    uint16_t rawVolt = readU16Le(frame.data(), 0);  // Byte0-1
    int16_t  rawCurr = static_cast<int16_t>(readU16Le(frame.data(), 2));  // Byte2-3
    uint16_t rawSoc  = readU16Le(frame.data(), 4);  // Byte4-5

    d.totalVoltage = rawVolt * 0.1;
    d.totalCurrent = (rawCurr - 10000) * 0.1;  // 偏移 -1000, 精度 0.1 → raw = (phys+1000)*10
    // 实际上 CAN 定义: rawCurr 为 unsigned, offset=-1000, resolution=0.1
    // 物理值 = raw × 0.1 + (-1000) = raw*0.1 - 1000
    // 我们直接用 unsigned 读, 物理值 = raw * 0.1 - 1000
    {
        uint16_t rawCurrU = readU16Le(frame.data(), 2);
        d.totalCurrent = rawCurrU * 0.1 - 1000.0;
    }
    d.soc  = rawSoc * 0.1;
    d.fresh = true;
    return d;
}

BmsCellData CanCodec::decodeBmsCell(const CanFrame& frame) {
    BmsCellData d;
    uint16_t rawMax = readU16Le(frame.data(), 0);  // Byte0-1
    d.maxCellVoltage = rawMax * 0.001;
    d.maxCellVoltageSerialNumber = readU16Le(frame.data(), 2);  // Byte2-3

    uint16_t rawMin = readU16Le(frame.data(), 4);  // Byte4-5
    d.minCellVoltage = rawMin * 0.001;
    d.minCellVoltageSerialNumber = readU16Le(frame.data(), 6);  // Byte6-7

    d.fresh = true;
    return d;
}

BmsTempData CanCodec::decodeBmsTemp(const CanFrame& frame) {
    BmsTempData d;
    d.maxTemperature   = static_cast<double>(frame[0]) - 50.0;  // Byte0, offset -50
    d.maxTemperatureId = readU16Le(frame.data(), 1);            // Byte1-2

    d.minTemperature   = static_cast<double>(frame[3]) - 50.0;  // Byte3, offset -50
    d.minTemperatureId = readU16Le(frame.data(), 4);            // Byte4-5

    d.fresh = true;
    return d;
}

MotionData CanCodec::decodeMotion(const CanFrame& frame) {
    MotionData d;
    int16_t rawLinear  = readS16Le(frame.data(), 0);  // Byte0-1, sint16
    int16_t rawAngular = readS16Le(frame.data(), 2);  // Byte2-3, sint16

    d.linearSpeed  = rawLinear * 0.001;
    d.angularSpeed = rawAngular * 0.001;
    d.vcuHeartbeat = frame[4];  // Byte4
    d.fresh = true;
    return d;
}

MotorPositionData CanCodec::decodeMotorPosition(const CanFrame& frame) {
    MotorPositionData d;
    d.leftRotation  = readU32Le(frame.data(), 0);  // Byte0-3
    d.rightRotation = readU32Le(frame.data(), 4);  // Byte4-7
    d.fresh = true;
    return d;
}

VcuFaultFlags CanCodec::decodeVcuFaultFlags(const CanFrame& frame) {
    VcuFaultFlags f;
    std::memset(&f, 0, sizeof(f));

    // Byte0 (bits 0-7)
    f.gunIn         = readBit(frame.data(), 0);
    f.gunOut        = readBit(frame.data(), 1);
    f.leftMotorEn   = readBit(frame.data(), 2);
    f.rightMotorEn  = readBit(frame.data(), 3);
    f.robotArmErr   = readBit(frame.data(), 4);
    f.robotArmReady = readBit(frame.data(), 5);
    f.mcuOffLine    = readBit(frame.data(), 6);
    f.bmsOffLine    = readBit(frame.data(), 7);

    // Byte1 (bits 8-15)
    f.gunInShrtGnd  = readBit(frame.data(), 8);
    f.gunInShrtPwr  = readBit(frame.data(), 9);
    f.gunInOpen     = readBit(frame.data(), 10);
    f.gunOutShrtGnd = readBit(frame.data(), 11);
    f.gunOutShrtPwr = readBit(frame.data(), 12);
    f.gunOutOpen    = readBit(frame.data(), 13);
    f.dcdc1OffLine  = readBit(frame.data(), 14);
    f.dcdc2OffLine  = readBit(frame.data(), 15);

    // Byte2 (bits 16-23)
    f.mtSetCtrlModErr = readBit(frame.data(), 16);
    f.mtSetProtectErr = readBit(frame.data(), 17);
    f.mtEnErr         = readBit(frame.data(), 18);
    f.mtDisEnErr      = readBit(frame.data(), 19);
    f.vehStopSwt      = readBit(frame.data(), 20);
    f.robotArmWork    = readBit(frame.data(), 21);
    f.armStartFb      = readBit(frame.data(), 22);
    f.handleCtrlEn    = readBit(frame.data(), 23);

    // Byte3 (bits 24-31)
    f.armFbWorkOverErr    = readBit(frame.data(), 24);
    f.handleCtrlArmOff    = readBit(frame.data(), 25);
    f.handleCtrlArmReset  = readBit(frame.data(), 26);
    f.armReserve1         = readBit(frame.data(), 27);
    f.armReserve2         = readBit(frame.data(), 28);
    f.armStartShrtGnd     = readBit(frame.data(), 29);
    f.armStartShrtPwr     = readBit(frame.data(), 30);
    f.armStartOpen        = readBit(frame.data(), 31);

    // Byte4 (bits 32-39)
    f.armResetShrtGnd = readBit(frame.data(), 32);
    f.armResetShrtPwr = readBit(frame.data(), 33);
    f.armResetOpen    = readBit(frame.data(), 34);
    f.battLowVolt     = readBit(frame.data(), 35);
    f.battHightVolt   = readBit(frame.data(), 36);

    f.fresh = true;
    return f;
}

BmsFaultData CanCodec::decodeBmsFault(const CanFrame& frame) {
    BmsFaultData d;
    d.errorCode  = frame[0];                          // Byte0: BMS 报警码
    d.errorLevel = frame[1];                          // Byte1: 报警等级
    d.chargeState = static_cast<uint8_t>(frame[2] & 0x03);  // Byte2, bit0-1: 充电状态
    d.gunConnected = (frame[3] & 0x01) != 0;           // Byte3, bit0: 充电枪连接
    d.fresh = true;
    return d;
}

MotorFaultBitmap CanCodec::decodeMotorFaultBitmap(const CanFrame& frame) {
    MotorFaultBitmap d;
    d.leftFaults  = readU32Le(frame.data(), 0);  // Byte0-3: 左电机 32 位故障位图
    d.rightFaults = readU32Le(frame.data(), 4);  // Byte4-7: 右电机 32 位故障位图
    d.fresh = true;
    return d;
}

MotorSpeedData CanCodec::decodeMotorSpeed(const CanFrame& frame) {
    MotorSpeedData d;
    d.leftRequested  = readS16Le(frame.data(), 0);  // Byte0-1
    d.rightRequested = readS16Le(frame.data(), 2);  // Byte2-3
    d.leftActual     = readS16Le(frame.data(), 4);  // Byte4-5
    d.rightActual    = readS16Le(frame.data(), 6);  // Byte6-7
    d.fresh = true;
    return d;
}

// ============================================================
// 下行编码: 工控机 → VCU
// ============================================================

CanFrame CanCodec::encodeControlCmd(const IpcControlCmd& cmd) {
    CanFrame frame;
    frame.fill(0);

    // Byte0: 构建控制命令位
    uint8_t byte0 = 0;
    if (cmd.gunIn)         byte0 |= (1 << 0);
    if (cmd.gunOut)        byte0 |= (1 << 1);
    byte0 |= ((cmd.lightCtrl & 0x03) << 2);  // bit2-3: 灯光控制 (2bit)
    if (cmd.emergencyStop) byte0 |= (1 << 4);
    if (cmd.driveEnable)   byte0 |= (1 << 5);
    if (cmd.armStart)      byte0 |= (1 << 6);
    if (cmd.armReset)      byte0 |= (1 << 7);
    frame[0] = byte0;

    // Byte1-7: 预留 (机械臂预留信号等)
    // 当前不使用

    return frame;
}

CanFrame CanCodec::encodeMotionCmd(const IpcMotionCmd& cmd) {
    CanFrame frame;
    frame.fill(0);

    // Byte0-1: 质心线速度 (sint16, Intel 小端)
    uint16_t linearRaw = static_cast<uint16_t>(cmd.linearSpeed);
    frame[0] = static_cast<uint8_t>(linearRaw & 0xFF);
    frame[1] = static_cast<uint8_t>((linearRaw >> 8) & 0xFF);

    // Byte2-3: 质心角速度 (sint16, Intel 小端)
    uint16_t angularRaw = static_cast<uint16_t>(cmd.angularSpeed);
    frame[2] = static_cast<uint8_t>(angularRaw & 0xFF);
    frame[3] = static_cast<uint8_t>((angularRaw >> 8) & 0xFF);

    // Byte4: 工控机心跳
    frame[4] = cmd.heartbeat;

    return frame;
}

// ============================================================
// 工具方法
// ============================================================

bool CanCodec::isVcuFrame(uint32_t canId) {
    // 29位扩展帧白名单
    switch (canId) {
    case 0x18FF5021:  // BMS 综合信息
    case 0x18FF5121:  // 单体电压
    case 0x18FF5221:  // 温度信息
    case 0x18FF5321:  // 速度+VCU心跳
    case 0x18FF5421:  // 电机反馈位置
    case 0x18FF5521:  // 故障/状态标志位
    case 0x18FF5621:  // BMS 故障码
    case 0x18FF5721:  // 电机故障位图
    case 0x18FF5821:  // 电机转速
        return true;
    default:
        return false;
    }
}

CanFrameCallback CanCodec::getDecoder(uint32_t canId) {
    switch (canId) {
    case 0x18FF5021: return [](uint32_t id, const CanFrame& f) { (void)id; decodeBmsBasic(f); };
    case 0x18FF5121: return [](uint32_t id, const CanFrame& f) { (void)id; decodeBmsCell(f); };
    case 0x18FF5221: return [](uint32_t id, const CanFrame& f) { (void)id; decodeBmsTemp(f); };
    case 0x18FF5321: return [](uint32_t id, const CanFrame& f) { (void)id; decodeMotion(f); };
    case 0x18FF5421: return [](uint32_t id, const CanFrame& f) { (void)id; decodeMotorPosition(f); };
    case 0x18FF5521: return [](uint32_t id, const CanFrame& f) { (void)id; decodeVcuFaultFlags(f); };
    case 0x18FF5621: return [](uint32_t id, const CanFrame& f) { (void)id; decodeBmsFault(f); };
    case 0x18FF5721: return [](uint32_t id, const CanFrame& f) { (void)id; decodeMotorFaultBitmap(f); };
    case 0x18FF5821: return [](uint32_t id, const CanFrame& f) { (void)id; decodeMotorSpeed(f); };
    default: return nullptr;
    }
}

} // namespace st_proto
