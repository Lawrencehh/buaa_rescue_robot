#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"  // 导入用于订阅的消息类型

class Listener : public rclcpp::Node  // 继承自 rclcpp::Node 类
{
public:
  Listener() : Node("listener")  // 初始化节点名为 "listener"
  {
    // 创建一个名为 "chatter" 的订阅器，订阅 talker 发布的话题
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "chatter", 
      10, 
      std::bind(&Listener::callback, this, std::placeholders::_1));
  }

private:
  void callback(const std_msgs::msg::String::SharedPtr msg)  // 回调函数
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());  // 在终端输出收到的消息
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;  // 声明订阅器
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);  // 初始化 ROS
  auto node = std::make_shared<Listener>();  // 创建 Listener 节点
  RCLCPP_INFO(node->get_logger(), "Listener node has been started.");  // 在终端输出节点已启动
  rclcpp::spin(node);  // 保持节点运行，直到被关闭
  rclcpp::shutdown();  // 关闭 ROS
  return 0;
}
