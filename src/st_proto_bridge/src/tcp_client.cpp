#include "st_proto_bridge/tcp_client.h"
#include "st_proto_bridge/protocol_codec.h"
#include <cstring>
#include <stdexcept>
#include <algorithm>
#include <chrono>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

namespace st_proto {

// ============================================================
// 构造 / 析构
// ============================================================

TcpClient::TcpClient() = default;

TcpClient::~TcpClient() {
    stop();
}

// ============================================================
// 配置
// ============================================================

void TcpClient::configure(const std::string& host, int port, int reconnectMs) {
    host_ = host;
    port_ = port;
    reconnectMs_ = reconnectMs;
}

void TcpClient::setFrameCallback(FrameCallback cb) {
    frameCallback_ = std::move(cb);
}

void TcpClient::setStateCallback(StateCallback cb) {
    stateCallback_ = std::move(cb);
}

// ============================================================
// 启动 / 停止
// ============================================================

void TcpClient::start() {
    if (running_) return;
    running_ = true;
    ioThread_ = std::make_unique<std::thread>(&TcpClient::ioLoop, this);
}

void TcpClient::stop() {
    running_ = false;
    if (ioThread_ && ioThread_->joinable()) {
        ioThread_->join();
    }
    ioThread_.reset();
    if (sockFd_ >= 0) {
        close(sockFd_);
        sockFd_ = -1;
    }
    notifyState(ConnectionState::DISCONNECTED);
}

// ============================================================
// 状态
// ============================================================

ConnectionState TcpClient::getState() const {
    return state_.load();
}

void TcpClient::setState(ConnectionState state) {
    notifyState(state);
}

void TcpClient::notifyState(ConnectionState newState) {
    ConnectionState old = state_.exchange(newState);
    if (old != newState && stateCallback_) {
        stateCallback_(newState);
    }
}

// ============================================================
// 发送
// ============================================================

bool TcpClient::send(const std::vector<uint8_t>& data) {
    if (sockFd_ < 0) return false;

    std::lock_guard<std::mutex> lock(sendMutex_);
    sendQueue_.push(data);

    bool allSent = true;
    while (!sendQueue_.empty()) {
        auto& front = sendQueue_.front();
        ssize_t sent = ::send(sockFd_, reinterpret_cast<const char*>(front.data()),
                              static_cast<int>(front.size()), 0);
        if (sent < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                allSent = false;
                break;
            }
            close(sockFd_);
            sockFd_ = -1;
            notifyState(ConnectionState::DISCONNECTED);
            return false;
        }
        if (static_cast<size_t>(sent) < front.size()) {
            front.erase(front.begin(), front.begin() + sent);
            allSent = false;
            break;
        }
        sendQueue_.pop();
    }
    return allSent;
}

// ============================================================
// IO线程
// ============================================================

void TcpClient::ioLoop() {
    recvBuf_.reserve(65536);

    while (running_) {
        if (sockFd_ < 0) {
            notifyState(ConnectionState::CONNECTING);
            if (!tryConnect()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(reconnectMs_));
                continue;
            }
            notifyState(ConnectionState::CONNECTED);
            recvBuf_.clear();
        }

        receiveAndParse();

        if (sockFd_ < 0) {
            notifyState(ConnectionState::DISCONNECTED);
        }
    }
}

bool TcpClient::tryConnect() {
    sockFd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sockFd_ < 0) return false;

    int flags = fcntl(sockFd_, F_GETFL, 0);
    fcntl(sockFd_, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port_));
    inet_pton(AF_INET, host_.c_str(), &addr.sin_addr);

    int ret = connect(sockFd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
    if (ret < 0) {
        if (errno != EINPROGRESS) {
            close(sockFd_);
            sockFd_ = -1;
            return false;
        }

        fd_set writeSet;
        FD_ZERO(&writeSet);
        FD_SET(sockFd_, &writeSet);
        struct timeval tv = {5, 0};
        ret = select(sockFd_ + 1, nullptr, &writeSet, nullptr, &tv);
        if (ret <= 0) {
            close(sockFd_);
            sockFd_ = -1;
            return false;
        }
    }
    return true;
}

void TcpClient::receiveAndParse() {
    uint8_t temp[4096];
    ssize_t n = recv(sockFd_, reinterpret_cast<char*>(temp), sizeof(temp), 0);
    if (n > 0) {
        recvBuf_.insert(recvBuf_.end(), temp, temp + n);
    } else if (n == 0) {
        close(sockFd_);
        sockFd_ = -1;
        return;
    } else {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            close(sockFd_);
            sockFd_ = -1;
        }
        return;
    }

    while (!recvBuf_.empty()) {
        auto frame = parseFrame();
        if (frame.empty()) break;
        if (frameCallback_) {
            frameCallback_(frame);
        }
    }
}

std::vector<uint8_t> TcpClient::parseFrame() {
    auto it = std::find(recvBuf_.begin(), recvBuf_.end(), PREAMBLE);
    if (it == recvBuf_.end()) {
        recvBuf_.clear();
        return {};
    }
    if (it != recvBuf_.begin()) {
        recvBuf_.erase(recvBuf_.begin(), it);
    }

    if (recvBuf_.size() < HEADER_SIZE) {
        return {};
    }

    uint32_t bodyLen = (static_cast<uint32_t>(recvBuf_[3]) << 24) |
                       (static_cast<uint32_t>(recvBuf_[4]) << 16) |
                       (static_cast<uint32_t>(recvBuf_[5]) << 8)  |
                       (static_cast<uint32_t>(recvBuf_[6]));

    size_t totalLen = HEADER_SIZE + bodyLen + 1;
    if (recvBuf_.size() < totalLen) {
        return {};
    }

    uint8_t expectedChecksum = recvBuf_[totalLen - 1];
    uint8_t calculatedChecksum = ProtocolCodec::calcChecksum(recvBuf_.data(), totalLen - 1);
    if (expectedChecksum != calculatedChecksum) {
        recvBuf_.erase(recvBuf_.begin());
        return {};
    }

    std::vector<uint8_t> frame(recvBuf_.begin(), recvBuf_.begin() + totalLen - 1);
    recvBuf_.erase(recvBuf_.begin(), recvBuf_.begin() + totalLen);

    return frame;
}

} // namespace st_proto