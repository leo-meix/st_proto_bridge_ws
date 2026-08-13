#ifndef ST_PROTO_BRIDGE_CAN_CODEC_H
#define ST_PROTO_BRIDGE_CAN_CODEC_H

#include "st_proto_bridge/can_types.h"
#include <cstdint>
#include <array>
#include <functional>

namespace st_proto {

/// CAN 帧原始数据 (max 8 bytes)
using CanFrame = std::array<uint8_t, 8>;

/// CAN 帧回调: (can_id, data)
using CanFrameCallback = std::function<void(uint32_t canId, const CanFrame& data)>;

/**
 * @brief CAN 信号编解码器 (V1.7)
 *
 * 负责:
 * - VCU → 工控机: 12 路 CAN 帧信号解码 (Intel 小端序 + 精度/偏移)
 * - 工控机 → VCU: 2 路 CAN 帧编码
 */
class CanCodec {
public:
    // ============================================================
    // 上行解码: VCU → 工控机
    // ============================================================

    static BmsBasicData      decodeBmsBasic(const CanFrame& frame);
    static BmsCellData       decodeBmsCell(const CanFrame& frame);
    static BmsTempData       decodeBmsTemp(const CanFrame& frame);
    static MotionData        decodeMotion(const CanFrame& frame);
    static MotorPositionData decodeMotorPosition(const CanFrame& frame);
    static VcuFaultFlags     decodeVcuFaultFlags(const CanFrame& frame);
    static BmsFaultData      decodeBmsFault(const CanFrame& frame);
    static MotorFaultBitmap  decodeMotorFaultBitmap(const CanFrame& frame);
    static MotorSpeedData    decodeMotorSpeed(const CanFrame& frame);

    // ============================================================
    // 下行编码: 工控机 → VCU
    // ============================================================

    /// 0x200: 控制命令 (100ms)
    /// 注意: 应在发送前将当前寄存的命令位合并后调用
    static CanFrame encodeControlCmd(const IpcControlCmd& cmd);

    /// 0x201: 速度命令 (10ms)
    static CanFrame encodeMotionCmd(const IpcMotionCmd& cmd);

    // ============================================================
    // 工具方法
    // ============================================================

    /// 判断是否为 VCU → 工控机 上行帧 (29位扩展帧白名单)
    static bool isVcuFrame(uint32_t canId);

    /// 根据 CAN ID 返回解码回调 (不在白名单返回 nullptr)
    static CanFrameCallback getDecoder(uint32_t canId);

private:
    // Intel 小端序读取辅助
    static uint16_t readU16Le(const uint8_t* data, int offset);
    static int16_t  readS16Le(const uint8_t* data, int offset);
    static uint32_t readU32Le(const uint8_t* data, int offset);

    /// 从 8 字节数据提取指定 bit (0-63)
    static bool readBit(const uint8_t* data, int bitIndex);
};

} // namespace st_proto

#endif // ST_PROTO_BRIDGE_CAN_CODEC_H
