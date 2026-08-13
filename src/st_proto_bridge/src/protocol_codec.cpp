#include "st_proto_bridge/protocol_codec.h"
#include <cstring>
#include <stdexcept>
#include <algorithm>
#include <openssl/evp.h>
#include <openssl/err.h>

namespace st_proto {

// ============================================================
// 构造 / 析构
// ============================================================

ProtocolCodec::ProtocolCodec() {
    // 初始化OpenSSL
    OpenSSL_add_all_algorithms();
    ERR_load_crypto_strings();
}

ProtocolCodec::~ProtocolCodec() {
    EVP_cleanup();
    ERR_free_strings();
}

void ProtocolCodec::configure(const std::string& key, const std::string& iv) {
    if (key.size() != AES_BLOCK_SIZE) {
        throw std::runtime_error("AES key must be exactly 16 characters");
    }
    if (iv.size() != AES_BLOCK_SIZE) {
        throw std::runtime_error("AES IV must be exactly 16 characters");
    }
    aesKey_.assign(key.begin(), key.end());
    aesIv_.assign(iv.begin(), iv.end());
}

// ============================================================
// 校验和
// ============================================================

uint8_t ProtocolCodec::calcChecksum(const std::vector<uint8_t>& data) {
    return calcChecksum(data.data(), data.size());
}

uint8_t ProtocolCodec::calcChecksum(const uint8_t* data, size_t len) {
    uint32_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum += data[i];
    }
    return static_cast<uint8_t>(sum & 0xFF);
}

// ============================================================
// PKCS5填充
// ============================================================

std::vector<uint8_t> ProtocolCodec::pkcs5Pad(const std::vector<uint8_t>& data, size_t blockSize) {
    size_t padLen = blockSize - (data.size() % blockSize);
    std::vector<uint8_t> padded(data);
    padded.insert(padded.end(), padLen, static_cast<uint8_t>(padLen));
    return padded;
}

std::vector<uint8_t> ProtocolCodec::pkcs5Unpad(const std::vector<uint8_t>& data) {
    if (data.empty()) {
        throw std::runtime_error("pkcs5Unpad: empty data");
    }
    uint8_t padLen = data.back();
    if (padLen == 0 || padLen > AES_BLOCK_SIZE) {
        throw std::runtime_error("pkcs5Unpad: invalid padding length");
    }
    // 验证所有填充字节
    for (size_t i = data.size() - padLen; i < data.size(); ++i) {
        if (data[i] != padLen) {
            throw std::runtime_error("pkcs5Unpad: invalid padding values");
        }
    }
    return std::vector<uint8_t>(data.begin(), data.end() - padLen);
}

// ============================================================
// AES128 CBC 加解密
// ============================================================

std::vector<uint8_t> ProtocolCodec::aesEncrypt(const std::vector<uint8_t>& plaintext) {
    auto padded = pkcs5Pad(plaintext, AES_BLOCK_SIZE);

    EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
    if (!ctx) throw std::runtime_error("EVP_CIPHER_CTX_new failed");

    int len = 0;
    std::vector<uint8_t> ciphertext(padded.size() + AES_BLOCK_SIZE);

    EVP_EncryptInit_ex(ctx, EVP_aes_128_cbc(), nullptr, aesKey_.data(), aesIv_.data());
    EVP_CIPHER_CTX_set_padding(ctx, 0);  // 禁用OpenSSL自动PKCS填充，已手动PKCS5
    EVP_EncryptUpdate(ctx, ciphertext.data(), &len, padded.data(), static_cast<int>(padded.size()));
    int totalLen = len;
    EVP_EncryptFinal_ex(ctx, ciphertext.data() + len, &len);
    totalLen += len;

    EVP_CIPHER_CTX_free(ctx);
    ciphertext.resize(totalLen);
    return ciphertext;
}

std::vector<uint8_t> ProtocolCodec::aesDecrypt(const std::vector<uint8_t>& ciphertext) {
    if (ciphertext.empty()) {
        throw std::runtime_error("aesDecrypt: empty ciphertext");
    }
    if (ciphertext.size() % AES_BLOCK_SIZE != 0) {
        throw std::runtime_error("aesDecrypt: ciphertext length not multiple of block size");
    }

    EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
    if (!ctx) throw std::runtime_error("EVP_CIPHER_CTX_new failed");

    int len = 0;
    std::vector<uint8_t> plaintext(ciphertext.size());

    EVP_DecryptInit_ex(ctx, EVP_aes_128_cbc(), nullptr, aesKey_.data(), aesIv_.data());
    EVP_CIPHER_CTX_set_padding(ctx, 0);  // 禁用OpenSSL自动PKCS填充，已手动PKCS5
    EVP_DecryptUpdate(ctx, plaintext.data(), &len, ciphertext.data(), static_cast<int>(ciphertext.size()));
    int totalLen = len;
    EVP_DecryptFinal_ex(ctx, plaintext.data() + len, &len);
    totalLen += len;

    EVP_CIPHER_CTX_free(ctx);
    plaintext.resize(totalLen);

    return pkcs5Unpad(plaintext);
}

std::string ProtocolCodec::aesEncryptString(const std::string& plaintext) {
    std::vector<uint8_t> input(plaintext.begin(), plaintext.end());
    auto encrypted = aesEncrypt(input);
    return std::string(encrypted.begin(), encrypted.end());
}

std::string ProtocolCodec::aesDecryptString(const std::string& ciphertext) {
    std::vector<uint8_t> input(ciphertext.begin(), ciphertext.end());
    auto decrypted = aesDecrypt(input);
    return std::string(decrypted.begin(), decrypted.end());
}

// ============================================================
// 帧级打包/解包
// ============================================================

std::vector<uint8_t> ProtocolCodec::packFrame(uint8_t msgType, const std::vector<uint8_t>& body) {
    std::vector<uint8_t> frame;
    frame.reserve(HEADER_SIZE + body.size() + 1); // +1 校验和

    // 1. 前导码 + 协议类型 + 消息类型
    frame.push_back(PREAMBLE);
    frame.push_back(PROTOCOL_TYPE);
    frame.push_back(msgType);

    // 2. 消息体长度（大端序）
    uint32_t len = static_cast<uint32_t>(body.size());
    frame.push_back(static_cast<uint8_t>((len >> 24) & 0xFF));
    frame.push_back(static_cast<uint8_t>((len >> 16) & 0xFF));
    frame.push_back(static_cast<uint8_t>((len >> 8)  & 0xFF));
    frame.push_back(static_cast<uint8_t>(len & 0xFF));

    // 3. 消息体
    frame.insert(frame.end(), body.begin(), body.end());

    // 4. 校验和（对前导码到消息体末尾的所有字节）
    uint8_t checksum = calcChecksum(frame);
    frame.push_back(checksum);

    return frame;
}

std::vector<uint8_t> ProtocolCodec::packJsonBusiness(const std::string& jsonStr) {
    // 1. 序列化JSON为字节
    std::vector<uint8_t> plainBody(jsonStr.begin(), jsonStr.end());

    // 2. AES加密
    auto encrypted = aesEncrypt(plainBody);

    // 3. 打包帧（加密标志位=1）
    uint8_t msgType = static_cast<uint8_t>(PacketType::JSON_BUSINESS) | MSG_ENCRYPT_MASK;
    return packFrame(msgType, encrypted);
}

// ============================================================
// 登录/心跳消息打包
// ============================================================

std::vector<uint8_t> ProtocolCodec::packLoginRequest(
    uint8_t vehicleType,
    uint32_t fwVersion,
    const std::string& uuid)
{
    // 构建登录请求体
    std::vector<uint8_t> body;
    body.reserve(8 + uuid.size());  // 鉴权(2) + 车型(1) + 版本(4) + uuidLen(1) + uuid

    body.push_back(AUTH_ID0);       // 'S'
    body.push_back(AUTH_ID1);       // 'T'
    body.push_back(vehicleType);

    // 固件版本（大端序）
    body.push_back(static_cast<uint8_t>((fwVersion >> 24) & 0xFF));
    body.push_back(static_cast<uint8_t>((fwVersion >> 16) & 0xFF));
    body.push_back(static_cast<uint8_t>((fwVersion >> 8)  & 0xFF));
    body.push_back(static_cast<uint8_t>(fwVersion & 0xFF));

    // UUID
    if (uuid.size() > 255) {
        throw std::runtime_error("UUID too long (max 255 bytes)");
    }
    body.push_back(static_cast<uint8_t>(uuid.size()));
    body.insert(body.end(), uuid.begin(), uuid.end());

    return packFrame(static_cast<uint8_t>(PacketType::LOGIN_REQ), body);
}

std::vector<uint8_t> ProtocolCodec::packHeartbeat() {
    // 心跳包消息体为空
    return packFrame(static_cast<uint8_t>(PacketType::HEARTBEAT), {});
}

std::vector<uint8_t> ProtocolCodec::packHeartbeatResponse() {
    return packFrame(static_cast<uint8_t>(PacketType::HEARTBEAT_RESP), {});
}

LoginResponseBody ProtocolCodec::parseLoginResponse(const std::vector<uint8_t>& body) {
    if (body.size() < 5) {
        throw std::runtime_error("Login response body too short");
    }
    LoginResponseBody resp;
    resp.result = body[0];
    resp.heartbeatMs = (static_cast<uint32_t>(body[1]) << 24) |
                       (static_cast<uint32_t>(body[2]) << 16) |
                       (static_cast<uint32_t>(body[3]) << 8)  |
                       (static_cast<uint32_t>(body[4]));
    return resp;
}

} // namespace st_proto