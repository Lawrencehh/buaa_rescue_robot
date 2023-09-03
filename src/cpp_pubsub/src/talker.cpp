#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"  // 导入用于发布的消息类型
#include <string>  // 导入 C++ 标准库中的字符串处理功能

class Talker : public rclcpp::Node  // 继承自 rclcpp::Node 类
{
public:
  Talker() : Node("talker"), count_(0)  // 初始化节点名为 "talker"，并初始化计数为0
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("motor_control", 10);  // 创建一个名为 "chatter" 的发布器

    // 使用定时器来触发消息发布
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),  // 设置时间间隔为 1000 毫秒，即 1 秒
      std::bind(&Talker::timer_callback, this));  // 绑定定时器回调函数
  }

private:
  void timer_callback()  // 定时器回调函数
  {
    auto message = std_msgs::msg::String();  // 创建一个 std_msgs/String 类型的消息

    message.data = std::to_string(count_);  // 将计数转换为字符串并设置消息内容

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());  // 在终端输出即将发布的消息

    publisher_->publish(message);  // 发布消息

    ++count_;  // 递增计数
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  // 声明发布器
  rclcpp::TimerBase::SharedPtr timer_;  // 声明定时器
  int count_;  // 声明一个用于跟踪已发布消息数量的计数器
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);  // 初始化 ROS
  auto node = std::make_shared<Talker>();  // 创建 Talker 节点
  RCLCPP_INFO(node->get_logger(), "Talker node has been started.");  // 在终端输出节点已启动
  rclcpp::spin(node);  // 保持节点运行，直到被关闭
  rclcpp::shutdown();  // 关闭 ROS
  return 0;
}

