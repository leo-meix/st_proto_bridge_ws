#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include <string>

/**
 * @brief path_bridge — 导航路径桥接节点
 *
 * 接收导航栈（move_base/自定义规划器）发布的 nav_msgs::Path，
 * 去重后转发到 /st_robot/local_path，供 protocol_parsing_node
 * 订阅并编码为 4.5 localPath JSON → TCP 上行。
 *
 * 运行方式: roslaunch st_proto_bridge path_bridge_node.launch
 */

class PathBridgeNode {
public:
    PathBridgeNode()
        : nh_("~")
        , changeThreshold_(0.5)
    {
        loadParams();

        // 订阅导航路径
        pathSub_ = nh_.subscribe("input_path", 10,
            &PathBridgeNode::onPath, this);

        // 转发到 protocol_parsing_node 订阅的话题
        pathPub_ = nh_.advertise<nav_msgs::Path>("output_path", 10);

        ROS_INFO("[path_bridge] Node started, input=%s, output=%s, threshold=%.2fm",
                 inputTopic_.c_str(), outputTopic_.c_str(), changeThreshold_);
    }

    void spin() {
        ros::spin();
    }

private:
    void loadParams() {
        nh_.param<std::string>("input_path_topic", inputTopic_,
                               "/move_base/GlobalPlanner/plan");
        nh_.param<std::string>("output_path_topic", outputTopic_,
                               "/st_robot/local_path");
        nh_.param<double>("change_threshold", changeThreshold_, 0.5);
    }

    void onPath(const nav_msgs::Path::ConstPtr& msg) {
        if (msg->poses.empty()) return;

        // 去重：只有路径变化时才转发（避免 TCP 高频重复上行）
        if (isPathChanged(*msg)) {
            lastPath_ = *msg;
            pathPub_.publish(*msg);
        }
    }

    /// 判定路径是否变化：点数变化 或 任意关键点偏移超过阈值
    bool isPathChanged(const nav_msgs::Path& current) {
        // 首次收到，记作变化
        if (lastPath_.poses.empty()) return true;

        // 点数变化 → 路径结构变了
        if (current.poses.size() != lastPath_.poses.size()) return true;

        // 采样首、中、末三个关键点比对位移
        const double sqThreshold = changeThreshold_ * changeThreshold_;
        size_t n = current.poses.size();

        // 首点
        if (sqDist(current.poses[0], lastPath_.poses[0]) > sqThreshold)
            return true;
        // 中点
        if (n > 1 && sqDist(current.poses[n / 2], lastPath_.poses[n / 2]) > sqThreshold)
            return true;
        // 末点
        if (sqDist(current.poses[n - 1], lastPath_.poses[n - 1]) > sqThreshold)
            return true;

        return false;
    }

    static double sqDist(const geometry_msgs::PoseStamped& a,
                         const geometry_msgs::PoseStamped& b) {
        double dx = a.pose.position.x - b.pose.position.x;
        double dy = a.pose.position.y - b.pose.position.y;
        return dx * dx + dy * dy;
    }

    ros::NodeHandle nh_;
    ros::Subscriber pathSub_;
    ros::Publisher  pathPub_;

    std::string inputTopic_;
    std::string outputTopic_;
    double      changeThreshold_;
    nav_msgs::Path lastPath_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "st_path_bridge");

    PathBridgeNode node;
    node.spin();

    return 0;
}
