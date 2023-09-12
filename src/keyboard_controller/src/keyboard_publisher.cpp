#include "rclcpp/rclcpp.hpp"  // 引入ROS 2的C++库
#include "buaa_rescue_robot_msgs/msg/control_message.hpp"  // 引入自定义消息类型

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
  auto publisher = node->create_publisher<buaa_rescue_robot_msgs::msg::ControlMessage>("control_topic", 10);  // 创建发布器

  auto msg = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessage>();  // 创建消息对象

  while (rclcpp::ok()) {  // 主循环
    int key = getch();  // 获取键盘输入

    if (key == '8') {  // 检查是否按下了'8'
      msg->elevator_control = 1;
      publisher->publish(*msg);  // 发布消息
      RCLCPP_INFO(node->get_logger(), "Published control message with elevator_control: 1");  // 打印日志
    }
    else if (key == '5') {  // 检查是否按下了'5'
      msg->elevator_control = 0;
      msg->below_linear_module_control = 0;
      msg->upper_linear_module_control = 0;
      publisher->publish(*msg);  // 发布消息
      RCLCPP_INFO(node->get_logger(), "Published control message with elevator_control: 0");  // 打印日志
    }
    else if (key == '2') {  // 检查是否按下了'2'
      msg->elevator_control = -1;
      publisher->publish(*msg);  // 发布消息
      RCLCPP_INFO(node->get_logger(), "Published control message with elevator_control: -1");  // 打印日志
    }
    else if (key == '0') {  // 检查是否按下了'0'
      msg->elevator_control = 666;
      msg->below_linear_module_control = 666;
      msg->upper_linear_module_control = 666;
      publisher->publish(*msg);  // 发布消息
      RCLCPP_INFO(node->get_logger(), "Published control message to reset the counter with elevator_control: 666");  // 打印日志
    }else if (key == '4') {  // 检查是否按下了'4'
      msg->below_linear_module_control = 1; // forward direction
      publisher->publish(*msg);  // 发布消息
      RCLCPP_INFO(node->get_logger(), "Published control message with below_linear_module_control: 1");  // 打印日志
    }else if (key == '6') {  // 检查是否按下了'6'
      msg->below_linear_module_control = -1; // backward direction
      publisher->publish(*msg);  // 发布消息
      RCLCPP_INFO(node->get_logger(), "Published control message with below_linear_module_control: -1");  // 打印日志
    }else if (key == '7') {  // 检查是否按下了'7'
      msg->upper_linear_module_control = 1; // forward direction
      publisher->publish(*msg);  // 发布消息
      RCLCPP_INFO(node->get_logger(), "Published control message with upper_linear_module_control: 1");  // 打印日志
    }else if (key == '9') {  // 检查是否按下了'9'
      msg->upper_linear_module_control = -1; // backward direction
      publisher->publish(*msg);  // 发布消息
      RCLCPP_INFO(node->get_logger(), "Published control message with upper_linear_module_control: -1");  // 打印日志
    }


    rclcpp::spin_some(node);  // 处理回调
  }

  rclcpp::shutdown();  // 关闭ROS 2节点
  return 0;
}
