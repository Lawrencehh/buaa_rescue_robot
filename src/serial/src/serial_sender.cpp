#include <rclcpp/rclcpp.hpp>
#include <asio.hpp>
#include <std_msgs/msg/string.hpp>

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
            serial_port_->set_option(asio::serial_port::baud_rate(115200));  // 设置波特率
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Could not open serial port: %s", e.what());
            return;
        }

        // 创建定时器，定时发送消息
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SerialSender::timer_callback, this));
        
        // 创建订阅器，订阅名为"motor_control"的话题
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "motor_control",
            10,
            std::bind(&SerialSender::callback, this, std::placeholders::_1));
    }


private:
    void callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // 解析接收到的消息，并生成Modbus协议帧
        // 这里简单地将接收到的消息存储为待发送的Modbus帧
        modbus_frame_ = msg->data;
    }
    void timer_callback()   // 定时器回调函数
    {
        const std::string msg = modbus_frame_;
        asio::write(*serial_port_, asio::buffer(msg, msg.size()));  // 发送消息
        RCLCPP_INFO(this->get_logger(), "Sent message: '%s'", msg.c_str());
    }

    rclcpp::TimerBase::SharedPtr timer_;    // 定时器
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;  //订阅器
    std::shared_ptr<asio::serial_port> serial_port_;    // 串口对象
    asio::io_service io_;   // ASIO I/O服务
    std::string modbus_frame_;  // 存储Modbus协议帧
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);   // 初始化ROS 2
    rclcpp::spin(std::make_shared<SerialSender>()); // 串口对象
    rclcpp::shutdown(); // ASIO I/O服务
    return 0;
}
