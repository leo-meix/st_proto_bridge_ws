#ifndef ST_PROTO_BRIDGE_TCP_CLIENT_H
#define ST_PROTO_BRIDGE_TCP_CLIENT_H

#include "types.h"
#include <string>
#include <vector>
#include <cstdint>
#include <functional>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <queue>

namespace st_proto {

/**
 * @brief 异步TCP客户端
 *
 * 负责：
 * - TCP连接/重连（非阻塞模式）
 * - 异步读写（后台线程）
 * - 粘包/半包处理（基于前导码0xAA+长度字段的帧解析）
 * - 完整的帧回调通知
 */
class TcpClient {
public:
    /// 帧到达回调 (完整帧字节流)
    using FrameCallback = std::function<void(const std::vector<uint8_t>& frame)>;

    /// 连接状态变更回调
    using StateCallback = std::function<void(ConnectionState state)>;

    TcpClient();
    ~TcpClient();

    /**
     * @brief 配置连接参数
     */
    void configure(const std::string& host, int port, int reconnectMs = DEFAULT_RECONNECT_MS);

    /**
     * @brief 设置帧回调（当完整帧解析完成时调用）
     */
    void setFrameCallback(FrameCallback cb);

    /**
     * @brief 设置连接状态回调
     */
    void setStateCallback(StateCallback cb);

    /**
     * @brief 启动连接（异步，后台线程自动重连）
     */
    void start();

    /**
     * @brief 停止连接
     */
    void stop();

    /**
     * @brief 发送数据
     */
    bool send(const std::vector<uint8_t>& data);

    /**
     * @brief 获取当前连接状态
     */
    ConnectionState getState() const;

    /**
     * @brief 更新连接状态（由上层在登录成功后调用）
     */
    void setState(ConnectionState state);

private:
    /// 后台IO线程主循环
    void ioLoop();

    /// 尝试连接服务器
    bool tryConnect();

    /// 接收数据并解析帧
    void receiveAndParse();

    /// 从缓冲区解析帧：找到前导码+头+消息体+校验和的一个完整帧
    /// @return 完整帧（包含头和体，不含校验和），如果未解析完成则返回空
    std::vector<uint8_t> parseFrame();

    // ---- 配置 ----
    std::string host_;
    int port_ = 0;
    int reconnectMs_ = DEFAULT_RECONNECT_MS;

    // ---- 套接字 ----
    int sockFd_ = -1;

    // ---- 回调 ----
    FrameCallback frameCallback_;
    StateCallback stateCallback_;

    // ---- 状态 ----
    std::atomic<ConnectionState> state_{ConnectionState::DISCONNECTED};
    std::atomic<bool> running_{false};

    // ---- IO线程 ----
    std::unique_ptr<std::thread> ioThread_;

    // ---- 接收缓冲区 ----
    std::vector<uint8_t> recvBuf_;

    // ---- 发送队列（线程安全） ----
    std::queue<std::vector<uint8_t>> sendQueue_;
    std::mutex sendMutex_;

    // ---- 状态变更通知（线程安全） ----
    void notifyState(ConnectionState newState);
};

} // namespace st_proto

#endif // ST_PROTO_BRIDGE_TCP_CLIENT_H