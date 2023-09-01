#include <rclcpp/rclcpp.hpp>
#include <asio.hpp>

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
    }

private:
    void timer_callback()
    {
        const std::string msg = "Hello, Serial Port!\n";
        asio::write(*serial_port_, asio::buffer(msg, msg.size()));  // 发送消息
        RCLCPP_INFO(this->get_logger(), "Sent message: '%s'", msg.c_str());
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<asio::serial_port> serial_port_;
    asio::io_service io_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialSender>());
    rclcpp::shutdown();
    return 0;
}
