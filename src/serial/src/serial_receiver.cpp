#include "rclcpp/rclcpp.hpp"
#include <asio.hpp>  // 包含 ASIO 库
#include <iostream>
#include <string>

class SerialReceiver : public rclcpp::Node
{
public:
    SerialReceiver() : Node("serial_receiver")
    {
        // 创建 ASIO I/O 服务
        asio::io_service io;

        // 创建串口对象，并打开串口
        asio::serial_port serial(io, "/dev/ttyUSB1");

        // 设置串口参数
        serial.set_option(asio::serial_port_base::baud_rate(115200));
        serial.set_option(asio::serial_port_base::character_size(8));
        serial.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
        serial.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
        serial.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));

        // 无限循环，持续接收数据
        while (rclcpp::ok())
        {
            // 创建缓冲区存储数据
            asio::streambuf buffer;
            
            // 读取一行数据
            asio::read_until(serial, buffer, "\n");

            // 转换为字符串
            std::istream input_stream(&buffer);
            std::string line;
            std::getline(input_stream, line);

            // 打印接收到的数据
            RCLCPP_INFO(this->get_logger(), "Received: '%s'", line.c_str());

            // 处理数据...

            // 使用 rclcpp 的 spin_some 以便处理其他 ROS 回调
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }
};

int main(int argc, char **argv)
{
    // 初始化 ROS 2
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = std::make_shared<SerialReceiver>();

    // 该节点永远不会退出，因为所有操作都在构造函数中完成
    rclcpp::spin(node);

    return 0;
}
