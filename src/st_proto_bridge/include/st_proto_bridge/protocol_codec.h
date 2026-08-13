#ifndef ST_PROTO_BRIDGE_PROTOCOL_CODEC_H
#define ST_PROTO_BRIDGE_PROTOCOL_CODEC_H

#include "types.h"
#include <string>
#include <vector>
#include <cstdint>

namespace st_proto {

/**
 * @brief 协议编解码器
 *
 * 负责：
 * - 消息帧的打包/解包（前导码+头+体+校验）
 * - 各类消息的序列化/反序列化
 * - AES128 CBC 加解密
 * - 校验和计算
 */
class ProtocolCodec {
public:
    ProtocolCodec();
    ~ProtocolCodec();

    /**
     * @brief 配置AES密钥和偏移量
     * @param key 16字节密钥字符串
     * @param iv  16字节偏移量字符串
     * @note 必须在加解密操作之前调用
     */
    void configure(const std::string& key, const std::string& iv);

    // ============================================================
    // 帧级操作
    // ============================================================

    /**
     * @brief 打包完整消息帧
     * @param msgType   消息类型（bit7=1表示加密）
     * @param body      消息体明文
     * @return 完整帧字节数组（可直接发送到TCP）
     */
    std::vector<uint8_t> packFrame(uint8_t msgType, const std::vector<uint8_t>& body);

    /**
     * @brief 打包JSON业务帧（自动加密+设置加密标志位）
     * @param jsonStr JSON字符串
     * @return 完整帧字节数组
     */
    std::vector<uint8_t> packJsonBusiness(const std::string& jsonStr);

    /**
     * @brief 计算校验和：所有字节相加的低8位
     */
    static uint8_t calcChecksum(const std::vector<uint8_t>& data);

    /**
     * @brief 计算校验和（指针版本）
     */
    static uint8_t calcChecksum(const uint8_t* data, size_t len);

    // ============================================================
    // AES128 CBC 加解密
    // ============================================================

    /**
     * @brief AES加密
     * @param plaintext 明文
     * @return 密文（PKCS5填充）
     */
    std::vector<uint8_t> aesEncrypt(const std::vector<uint8_t>& plaintext);

    /**
     * @brief AES解密
     * @param ciphertext 密文（PKCS5填充）
     * @return 明文
     */
    std::vector<uint8_t> aesDecrypt(const std::vector<uint8_t>& ciphertext);

    /**
     * @brief AES加密（字符串版本）
     */
    std::string aesEncryptString(const std::string& plaintext);

    /**
     * @brief AES解密（字符串版本）
     */
    std::string aesDecryptString(const std::string& ciphertext);

    // ============================================================
    // 各类消息的序列化
    // ============================================================

    /// 打包登录申请包帧
    std::vector<uint8_t> packLoginRequest(
        uint8_t vehicleType,
        uint32_t fwVersion,
        const std::string& uuid
    );

    /// 打包心跳包帧
    std::vector<uint8_t> packHeartbeat();

    /// 打包心跳响应包帧
    std::vector<uint8_t> packHeartbeatResponse();

    /// 解析登录响应包消息体
    LoginResponseBody parseLoginResponse(const std::vector<uint8_t>& body);

private:
    /**
     * @brief PKCS5填充
     */
    std::vector<uint8_t> pkcs5Pad(const std::vector<uint8_t>& data, size_t blockSize);

    /**
     * @brief 去除PKCS5填充
     */
    std::vector<uint8_t> pkcs5Unpad(const std::vector<uint8_t>& data);

    // AES密钥和IV
    std::vector<uint8_t> aesKey_;
    std::vector<uint8_t> aesIv_;
};

} // namespace st_proto

#endif // ST_PROTO_BRIDGE_PROTOCOL_CODEC_H