// 头文件引入
#include <rclcpp/rclcpp.hpp>    // 引入ROS 2的核心库
#include <asio.hpp> // 引入ASIO库，用于串口通信
#include <std_msgs/msg/string.hpp>  // 引入标准消息类型
#include "buaa_rescue_robot_msgs/msg/control_message.hpp"  // 引入自定义消息类型
#include "buaa_rescue_robot_msgs/msg/sensors_message.hpp"   // 引入自定义消息类型
#include <thread>   // 用于线程中的sleep_for函数
#include <chrono>   // 用于时间表示
#include <iostream>
#include <string>
#include <vector>
#include <cstdint>
#include <termios.h>
// 命名空间
using namespace std;
using asio::ip::tcp;

// CRC-16 Modbus查表法
const uint16_t  crc16_table[256] = {
0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

// 计算CRC-16 Modbus校验码
uint16_t calculate_crc16(const std::vector<uint8_t>& data, size_t start, size_t end) {
    uint16_t crc = 0xFFFF;
    for (size_t i = start; i < end; ++i) {
        uint8_t pos = crc ^ data[i];
        crc = crc16_table[pos] ^ (crc >> 8);
    }
    return crc;
}

// swap the Hi_8 and the low_8
uint16_t swap_endian(uint16_t value) {
    uint16_t low_byte = value & 0xFF;  // 获取低8位
    uint16_t high_byte = (value >> 8) & 0xFF;  // 获取高8位
    return (low_byte << 8) | high_byte;  // 交换高低8位并返回
}

// 类定义
// 创建一个继承自rclcpp::Node的serial_robomaster_1类，并在构造函数中进行初始化。
class serial_robomaster_1 : public rclcpp::Node
{
public:
    serial_robomaster_1() : Node("serial_robomaster_1")
    {
        // 尝试初始化串口，如果出错，记录错误信息。
        try {
            serial_port_ = std::make_shared<asio::serial_port>(io_, "/dev/ttyUSB0");  // 这里的路径需要根据您的设备进行更改
            serial_port_->set_option(asio::serial_port::baud_rate(115200));  // 设置波特率
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Could not open serial port: %s", e.what());
            return;
        }

        // 创建定时器，定时发送消息
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),  // 100毫秒，即10 Hz
            std::bind(&serial_robomaster_1::timer_callback, this));
        
        // 创建订阅器，订阅名为"control_topic"的话题
          subscription_ = this->create_subscription<buaa_rescue_robot_msgs::msg::ControlMessage>("control_topic", 10, 
          std::bind(&serial_robomaster_1::callback, this, std::placeholders::_1));

        // 在serial_robomaster_1的构造函数中初始化这个发布器
        publisher_ = this->create_publisher<buaa_rescue_robot_msgs::msg::SensorsMessage>("sensors_data", 10);

        // 在构造函数中启动接收
        start_receive();
       
    }

    // 获取io_service的引用，用于在main函数中运行
    asio::io_service& get_io_service() {
    return io_;
    }


private:    // 私有成员函数和变量
    // 在私有成员变量区域添加
    asio::streambuf read_buffer_;
    asio::streambuf write_buffer_;  // 新添加的写缓冲区
    std::vector<uint8_t> last_received_message_;  // 添加一个新的私有成员变量来存储最后接收到的消息
    rclcpp::Publisher<buaa_rescue_robot_msgs::msg::SensorsMessage>::SharedPtr publisher_;   // add a publisher of SensorsMessage

    // ROS 2 Humble版本的异步写入封装函数
    void async_write_to_serial(const std::vector<uint8_t>& data_to_write)
    {
        asio::async_write(*serial_port_, asio::buffer(data_to_write),
            [this](const asio::error_code& error, std::size_t bytes_transferred)
            {
                // 检查是否有错误
                if (!error) 
                {
                    // RCLCPP_INFO(this->get_logger(), "Successfully wrote %zu bytes", bytes_transferred);
                } 
                else 
                {
                    RCLCPP_ERROR(this->get_logger(), "Write Error: %s", error.message().c_str());
                }
            }
        );
    }

    // 处理接收到的Modbus帧
    int32_t process_modbus_frame_for_robomaster_1 (const std::vector<uint8_t>& frame) {
      
    }

    // 重构后的 start_receive 函数
    std::vector<uint8_t> received_modbus_frame_; 
    // ROS 2 Humble版本的start_receive函数
    void start_receive() 
    {
        // 清空received_modbus_frame_并初始化大小
        received_modbus_frame_.clear();
        received_modbus_frame_.resize(256);

        // 异步读取串口数据
        serial_port_->async_read_some(asio::buffer(received_modbus_frame_, 256),
            [this](const asio::error_code& error, std::size_t bytes_transferred)
            {
                // 调整received_modbus_frame_的大小以匹配实际接收到的字节数
                received_modbus_frame_.resize(bytes_transferred);

                // 检查是否有错误
                if (!error) 
                {
                    // 打印发送的Modbus帧内容
                    std::string msg_str = "";
                    for (auto &byte : received_modbus_frame_) {
                        msg_str += "0x" + to_string(static_cast<int>(byte)) + " ";
                    }
                    RCLCPP_INFO(this->get_logger(), "Receive message: %s", msg_str.c_str());  


                    

                    

                    // 递归调用以持续接收
                    start_receive();
                }
                else 
                {
                    RCLCPP_ERROR(this->get_logger(), "Receive Error: %s", error.message().c_str());
                }
            }
        );
    }

 
    void callback(const buaa_rescue_robot_msgs::msg::ControlMessage::SharedPtr msg){        
        // 1. 准备数据帧的头部, robomaster_1, snake control
        std::vector<uint8_t> frame = {0xAA, 0x55, 0x01};

        // 数据长度 = 1字节功能码 + 1字节电机数量 + (12电机 * 5字节/电机) + 2字节校验码
        frame.push_back(1 + 1 + 12 * 5 + 2);

        // 其他固定字段
        frame.push_back(0x31);  // 功能码
        frame.push_back(0x0C);  // 电机数量（12个）

        // 2. 提取snake_control_1_array的值，并添加到数据帧中
        for (int i = 0; i < 12; ++i)  // 12个电机
        {
            int32_t speed = msg->snake_control_1_array[i];  // 提取速度值
            uint8_t motor_address = 0x01 + i;  // 电机地址从0x01开始

            if (i == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Speed%d:%d",i,speed);
            }
            
            

            frame.push_back(motor_address);  // 添加电机地址

            uint8_t bytes[4];  // 用于存储4个字节的数组

            // 分解 int32_t 变量为4个字节
            bytes[0] = (speed >> 24) & 0xFF;  // 最高有效字节 (MSB)
            bytes[1] = (speed >> 16) & 0xFF;  // 次高有效字节
            bytes[2] = (speed >> 8) & 0xFF;   // 次低有效字节
            bytes[3] = speed & 0xFF;          // 最低有效字节 (LSB)

            frame.push_back(bytes[0]);  
            frame.push_back(bytes[1]);   
            frame.push_back(bytes[2]);
            frame.push_back(bytes[3]);
        }

        // 3. 计算CRC-16 Modbus校验码
        uint16_t crc = calculate_crc16(frame, 0, frame.size());

        // 将校验码分解为高字节和低字节
        uint8_t crc_low_byte = crc & 0xFF;
        uint8_t crc_high_byte = (crc >> 8) & 0xFF;

        // 4. 添加校验码到数据帧, first low byte, and then high byte
        frame.push_back(crc_high_byte);
        frame.push_back(crc_low_byte);
        
        // 打印发送的Modbus帧内容
        std::string frame_str = "";
        for (auto &byte : frame) {
            frame_str += "0x" + to_string(static_cast<int>(byte)) + " ";
        }
        RCLCPP_INFO(this->get_logger(), "Sending frame: %s", frame_str.c_str());

        // 5. 通过串口发送数据帧
        async_write_to_serial(frame);
    }

    void timer_callback()
    {
             
    }


    rclcpp::TimerBase::SharedPtr timer_;    // 定时器

    rclcpp::Subscription<buaa_rescue_robot_msgs::msg::ControlMessage>::SharedPtr subscription_;  //订阅器
    std::shared_ptr<asio::serial_port> serial_port_;    // 串口对象
    asio::io_service io_;   // ASIO I/O服务
    std::vector<uint8_t> modbus_frame_; // 存储Modbus协议帧
    
    std::vector<uint8_t> frame;  // Modbus协议帧
};

int main(int argc, char **argv) // 主函数
{
    rclcpp::init(argc, argv);  // 初始化ROS 2

    auto serial_robomaster_1_node = std::make_shared<serial_robomaster_1>();   // 创建serial_robomaster_1对象

    // 创建一个新线程来运行ASIO的io_service
    std::thread asio_thread([&]() {
        serial_robomaster_1_node->get_io_service().run();
    });

    rclcpp::spin(serial_robomaster_1_node);  // 开始ROS的事件循环

    rclcpp::shutdown();  // 关闭ROS 2

    // 等待ASIO的io_service线程完成
    asio_thread.join();

    return 0;
}
