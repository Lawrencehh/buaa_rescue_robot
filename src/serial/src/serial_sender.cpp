#include <rclcpp/rclcpp.hpp>
#include <asio.hpp>
#include <std_msgs/msg/string.hpp>
#include "buaa_rescue_robot_msgs/msg/control_message.hpp"  // 引入自定义消息类型

#include <thread>   // delay
#include <chrono>   // delay

using namespace std;
using asio::ip::tcp;

class SerialSender : public rclcpp::Node
{
public:
    SerialSender() : Node("serial_sender")
    {
        // 初始化串口
        try {
            serial_port_ = std::make_shared<asio::serial_port>(io_, "/dev/ttyUSB0");  // 这里的路径需要根据您的设备进行更改
            serial_port_->set_option(asio::serial_port::baud_rate(57600));  // 设置波特率
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Could not open serial port: %s", e.what());
            return;
        }

        // 创建定时器，定时发送消息
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SerialSender::timer_callback, this));
        
        // 创建订阅器，订阅名为"control_topic"的话题
          subscription_ = this->create_subscription<buaa_rescue_robot_msgs::msg::ControlMessage>(
            "control_topic",
            10,
            std::bind(&SerialSender::callback, this, std::placeholders::_1));
    }


private:
    void callback(const buaa_rescue_robot_msgs::msg::ControlMessage::SharedPtr msg)
    {
        // 直接使用类成员变量
        if (msg->elevator_control == 1) //elavator going upwards
        {
            modbus_frame_ = {0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xCA};   // turn off J1
            asio::write(*serial_port_, asio::buffer(modbus_frame_, modbus_frame_.size()));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 延迟10毫秒            
            modbus_frame_ = {0x01, 0x05, 0x00, 0x01, 0x00, 0x00, 0x9C, 0x0A};   // turn off J2
            asio::write(*serial_port_, asio::buffer(modbus_frame_, modbus_frame_.size()));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 延迟10毫秒 
            modbus_frame_ = {0x01, 0x05, 0x00, 0x00, 0xFF, 0x00, 0x8C, 0x3A};
            asio::write(*serial_port_, asio::buffer(modbus_frame_, modbus_frame_.size()));
        }
        else if (msg->elevator_control == 0)    //elavator stop
        {
            modbus_frame_ = {0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xCA};   // turn off J1
            asio::write(*serial_port_, asio::buffer(modbus_frame_, modbus_frame_.size()));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 延迟10毫秒            
            modbus_frame_ = {0x01, 0x05, 0x00, 0x01, 0x00, 0x00, 0x9C, 0x0A};   // turn off J2
            asio::write(*serial_port_, asio::buffer(modbus_frame_, modbus_frame_.size()));

        }
        else if (msg->elevator_control == -1)   //elavator going downwards
        {
            modbus_frame_ = {0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xCA};   // turn off J1
            asio::write(*serial_port_, asio::buffer(modbus_frame_, modbus_frame_.size()));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 延迟10毫秒            
            modbus_frame_ = {0x01, 0x05, 0x00, 0x01, 0x00, 0x00, 0x9C, 0x0A};   // turn off J2
            asio::write(*serial_port_, asio::buffer(modbus_frame_, modbus_frame_.size()));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 延迟10毫秒 
            modbus_frame_ = {0x01, 0x05, 0x00, 0x01, 0xFF, 0x00, 0xDD, 0xFA};
            asio::write(*serial_port_, asio::buffer(modbus_frame_, modbus_frame_.size()));
        }
        else if (msg->elevator_control == 666)  //elavator stop and reset the counter
        {
            modbus_frame_ = {0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xCA};   // turn off J1
            asio::write(*serial_port_, asio::buffer(modbus_frame_, modbus_frame_.size()));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 延迟10毫秒            
            modbus_frame_ = {0x01, 0x05, 0x00, 0x01, 0x00, 0x00, 0x9C, 0x0A};   // turn off J2
            asio::write(*serial_port_, asio::buffer(modbus_frame_, modbus_frame_.size()));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 延迟10毫秒 
            modbus_frame_ = {0x01, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0xF3, 0xAF}; //reset the counter 01 10 00 00 00 02 04 00 00 00 00 F3 AF
            asio::write(*serial_port_, asio::buffer(modbus_frame_, modbus_frame_.size()));
        }

        
    }

    void timer_callback()
    {
        // 打印vector的内容
        std::string msg_str = "";
        for (auto &byte : modbus_frame_) {
            msg_str += "0x" + to_string(static_cast<int>(byte)) + " ";
        }
        RCLCPP_INFO(this->get_logger(), "Sent message: %s", msg_str.c_str());
    }


    rclcpp::TimerBase::SharedPtr timer_;    // 定时器
    rclcpp::Subscription<buaa_rescue_robot_msgs::msg::ControlMessage>::SharedPtr subscription_;  //订阅器
    std::shared_ptr<asio::serial_port> serial_port_;    // 串口对象
    asio::io_service io_;   // ASIO I/O服务
    std::vector<uint8_t> modbus_frame_; // 存储Modbus协议帧

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);   // 初始化ROS 2
    rclcpp::spin(std::make_shared<SerialSender>()); // 串口对象
    rclcpp::shutdown(); // ASIO I/O服务
    return 0;
}
