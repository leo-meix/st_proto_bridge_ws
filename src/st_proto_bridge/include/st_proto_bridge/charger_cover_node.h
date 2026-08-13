#ifndef ST_PROTO_BRIDGE_CHARGER_COVER_NODE_H
#define ST_PROTO_BRIDGE_CHARGER_COVER_NODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <string>

namespace st_proto {

/**
 * @brief 充电口盖板控制节点 (Modbus TCP)
 *
 * 通过 Modbus TCP 协议（功能码 0x0F Write Multiple Coils）控制 LoRa 网关，
 * 进而驱动充电口盖板开合。
 *
 * 物理链路: 工控机 → 车载网关 → LoRa网关 → 充电板 Q0.0/Q0.1
 *
 * ROS 接口:
 *   订阅 /st_robot/cover_cmd    (std_msgs/String): {"action":"open"|"close"}
 *   发布 /st_robot/cover_result (std_msgs/String): {"action":"open","success":true}
 *
 * 设计为独立节点: 不依赖 CAN / TCP / AES，仅依赖 libmodbus。
 * 调用方 (云平台 / CAN / 命令行) 后期再定，通过 cover_cmd 触发即可。
 */
class ChargerCoverNode {
public:
    ChargerCoverNode();
    ~ChargerCoverNode() = default;

    void spin();

private:
    /// 加载 yaml 参数
    void loadParams();

    /// 收到盖板命令回调
    void onCoverCmd(const std_msgs::String::ConstPtr& msg);

    /// 控制盖板开/关 (Modbus TCP 0x0F 写线圈 Q0.0/Q0.1)
    /// @param open true=开门, false=关门
    /// @return true=成功
    bool setCover(bool open);

    /// 发布执行结果
    void publishResult(const std::string& action, bool success);

    // ---- ROS ----
    ros::NodeHandle nh_;
    ros::Subscriber cmdSub_;
    ros::Publisher  resultPub_;

    // ---- Modbus TCP 配置 ----
    std::string modbusIp_;       ///< LoRa 网关 IP
    int         modbusPort_;     ///< Modbus TCP 端口 (默认 502)
    int         modbusUnitId_;   ///< Modbus 单元 ID (默认 1)
    int         coilStartAddr_;  ///< 线圈起始地址 0x0000

    /// 线圈值: 开门 = Q0.0=1,Q0.1=0 → 0x01; 关门 = Q0.0=0,Q0.1=1 → 0x02
    int         coilOpenValue_;
    int         coilCloseValue_;

    int         timeoutMs_;      ///< Modbus 超时 (毫秒)
    bool        dryRun_;         ///< true=只打印不发请求 (L0 测试)
};

}  // namespace st_proto

#endif  // ST_PROTO_BRIDGE_CHARGER_COVER_NODE_H
