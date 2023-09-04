#include "rclcpp/rclcpp.hpp"  // 引入ROS 2的C++库
#include "keyboard_controller/msg/control_message.hpp"  // 引入自定义消息类型

#include <termios.h>  // 用于键盘输入
#include <unistd.h>  // Unix标准库

// 获取键盘输入，不需要按Enter
int getch() {
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);  // 初始化ROS 2节点
  auto node = std::make_shared<rclcpp::Node>("keyboard_controller");  // 创建节点
  auto publisher = node->create_publisher<keyboard_controller::msg::ControlMessage>("control_topic", 10);  // 创建发布器

  auto msg = std::make_shared<keyboard_controller::msg::ControlMessage>();  // 创建消息对象

  while (rclcpp::ok()) {  // 主循环
    int key = getch();  // 获取键盘输入

    if (key == '8') {  // 检查是否按下了'8'
      msg->controller_addr = 3;
      msg->dev_addr = 1;
      msg->func_code = 0x33;
      msg->control_data = 0x010001;
      publisher->publish(*msg);  // 发布消息
      RCLCPP_INFO(node->get_logger(), "Published control message with func_code: 0x33");  // 打印日志
    }
    else if (key == '5') {  // 检查是否按下了'5'
      msg->controller_addr = 3;
      msg->dev_addr = 1;
      msg->func_code = 0x4F;
      msg->control_data = 0x014F4F;
      publisher->publish(*msg);  // 发布消息
      RCLCPP_INFO(node->get_logger(), "Published control message with func_code: 0x4F");  // 打印日志
    }
    else if (key == '2') {  // 检查是否按下了'2'
      msg->controller_addr = 3;
      msg->dev_addr = 1;
      msg->func_code = 0x33;
      msg->control_data = 0x01FFFF;
      publisher->publish(*msg);  // 发布消息
      RCLCPP_INFO(node->get_logger(), "Published control message with func_code: 0x33");  // 打印日志
    }

    rclcpp::spin_some(node);  // 处理回调
  }

  rclcpp::shutdown();  // 关闭ROS 2节点
  return 0;
}
