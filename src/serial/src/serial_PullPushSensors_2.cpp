// 头文件引入
#include <rclcpp/rclcpp.hpp>    // 引入ROS 2的核心库
#include <asio.hpp> // 引入ASIO库，用于串口通信
#include <std_msgs/msg/string.hpp>  // 引入标准消息类型
#include "buaa_rescue_robot_msgs/msg/control_message_master.hpp"  // 引入自定义消息类型
#include "buaa_rescue_robot_msgs/msg/sensors_message_master_device_pull_push_sensors.hpp"   // 引入自定义消息类型
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
class serial_PullPushSensors_2 : public rclcpp::Node
{
public:
    serial_PullPushSensors_2() : Node("serial_PullPushSensors_2")
    {
        // 尝试初始化串口，如果出错，记录错误信息。
        try {
            serial_port_1 = std::make_shared<asio::serial_port>(io_, "/dev/ttyPullPushSensors3");  // 这里的路径需要根据您的设备进行更改
            serial_port_1->set_option(asio::serial_port::baud_rate(115200));  // 设置波特率
            serial_port_2 = std::make_shared<asio::serial_port>(io_, "/dev/ttyPullPushSensors4");  // 这里的路径需要根据您的设备进行更改
            serial_port_2->set_option(asio::serial_port::baud_rate(115200));  // 设置波特率
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Could not open serial port: %s", e.what());
            return;
        }

        // 创建定时器，定时发送消息
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),  // 100毫秒，即10 Hz
            std::bind(&serial_PullPushSensors_2::timer_callback, this));
        
        // 创建订阅器，订阅名为"master_control_topic"的话题
          subscription_ = this->create_subscription<buaa_rescue_robot_msgs::msg::ControlMessageMaster>("master_control_topic", 10, 
          std::bind(&serial_PullPushSensors_2::callback, this, std::placeholders::_1));

        // 在serial_PullPushSensors_2的构造函数中初始化这个发布器
        publisher_ = this->create_publisher<buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors>("Sensors_Pull_Push_Sensors_2", 10);


        // 在构造函数中启动接收
        start_receive_1();
        start_receive_2();
       
    }

    // 获取io_service的引用，用于在main函数中运行
    asio::io_service& get_io_service() {
    return io_;
    }


private:    // 私有成员函数和变量
    // 在私有成员变量区域添加
    asio::streambuf read_buffer_;
    asio::streambuf write_buffer_;  // 新添加的写缓冲区
    rclcpp::Publisher<buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors>::SharedPtr publisher_;   // add a publisher of SensorsMessage

    // ROS 2 Humble版本的异步写入封装函数,serial_port_1
    void async_write_to_serial_1(const std::vector<uint8_t>& data_to_write)
    {
        asio::async_write(*serial_port_1, asio::buffer(data_to_write),
            [this](const asio::error_code& error, std::size_t /*bytes_transferred*/)
            {
                // 检查是否有错误
                if (!error) 
                {

                } 
                else 
                {
                    RCLCPP_ERROR(this->get_logger(), "Write Error: %s", error.message().c_str());
                }
            }
        );
    }

    // ROS 2 Humble版本的异步写入封装函数,serial_port_2
    void async_write_to_serial_2(const std::vector<uint8_t>& data_to_write)
    {
        asio::async_write(*serial_port_2, asio::buffer(data_to_write),
            [this](const asio::error_code& error, std::size_t /*bytes_transferred*/)
            {
                // 检查是否有错误
                if (!error) 
                {

                } 
                else 
                {
                    RCLCPP_ERROR(this->get_logger(), "Write Error: %s", error.message().c_str());
                }
            }
        );
    }

    // 处理接收到的Modbus帧
    std::array<int32_t, 6> process_modbus_frame_for_pull_push_sensors (const std::vector<uint8_t>& frame) {
       // 数据头
        const std::vector<uint8_t> header = {0x01,0x50,0xFF};
        std::array<int32_t, 6>  pull_push_six_sensors_value = {0,0,0,0,0,0};  // 初始化编码器数据为0

        // 检查数据头
        if (std::equal(header.begin(), header.end(), frame.begin())) {
            // 提取数据和CRC校验码
            std::vector<uint8_t> data(frame.begin() + header.size(), frame.begin() + header.size() + 24);  // 提取24个字节的数据
            uint16_t received_crc = (frame[frame.size() - 2] << 8) | frame[frame.size() - 1];

            // 计算CRC校验码
            uint16_t calculated_crc = calculate_crc16(frame, 0, frame.size() - 2); 
            // calculated_crc = swap_endian(calculated_crc);

            // 检查CRC校验码是否匹配
            if (received_crc == calculated_crc) {

                // 解析数据段
                if (data.size() == 24) {
                    for (size_t i = 0; i < 6; i++)
                    {
                        pull_push_six_sensors_value[i] = -1 * static_cast<int32_t>(     //product -1
                        static_cast<uint64_t>(data[4*i])    << 24 |
                        static_cast<uint64_t>(data[4*i+1])  << 16 |
                        static_cast<uint64_t>(data[4*i+2])  << 8  |
                        static_cast<uint64_t>(data[4*i+3])  
                    );
                    }
                    
                    
                } else {
                    std::cout << "Insufficient data size for encoder." << std::endl;
                }
            } else {
                std::cout << "CRC check failed for encoder." << std::endl;
            }
        } else {
            std::cout << "Invalid header for encoder." << std::endl;
        }

        return pull_push_six_sensors_value;   
    }

    // 重构后的 start_receive 函数
    std::vector<uint8_t> received_modbus_frame_1; 
    // ROS 2 Humble版本的start_receive函数
    void start_receive_1() 
    {
        // received_modbus_frame_初始化大小
        received_modbus_frame_1.clear();
        received_modbus_frame_1.resize(256);

        // 异步读取串口数据serial_port_1
        serial_port_1->async_read_some(asio::buffer(received_modbus_frame_1, 256),
            [this](const asio::error_code& error, std::size_t bytes_transferred)
            {
                // 调整received_modbus_frame_的大小以匹配实际接收到的字节数
                received_modbus_frame_1.resize(bytes_transferred);

                // 检查是否有错误
                if (!error) 
                {
                    // 将接收到的数据添加到数据缓存区
                    data_buffer_1.insert(data_buffer_1.end(), received_modbus_frame_1.begin(), received_modbus_frame_1.end());

                    // 打印data_buffer_内容
                    // std::string msg_str = "";
                    // for (auto &byte : data_buffer_1) {
                    //     msg_str += "0x" + to_string(static_cast<int>(byte)) + " ";
                    // }
                    // RCLCPP_INFO(this->get_logger(), "Receive message: %s", msg_str.c_str());  


                    // 创建一个包含数据头的vector
                    std::vector<uint8_t> pull_push_sensors_header = {0xFE,0x01,0x50,0xFF};
                    // 搜索数据头
                    auto it_pull_push_encorder = std::search(data_buffer_1.begin(), data_buffer_1.end(), pull_push_sensors_header.begin(), pull_push_sensors_header.end());

                    // 如果找到了数据头，并且有足够的字节用于完整的34字节帧 (excluding the ending 0xCFFCCCFF)
                    if (it_pull_push_encorder != data_buffer_1.end() && std::distance(it_pull_push_encorder, data_buffer_1.end()) >= 34)
                    {
                        // 提取30字节帧
                        std::vector<uint8_t> frame(it_pull_push_encorder+1, it_pull_push_encorder + 30);

                        // 处理Modbus帧并获取elevator_counter
                        pull_push_sensors_2_part = process_modbus_frame_for_pull_push_sensors(frame);

                        for (size_t i = 0; i < 6; i++)
                        {
                            pull_push_sensors_2[2*i] = pull_push_sensors_2_part[i];
                        }

                        // 移除这34字节(including the ending 0xCFFCCCFF)及之前的字节
                        data_buffer_1.erase(data_buffer_1.begin(), it_pull_push_encorder + 34);
                    }
                    received_modbus_frame_1.clear();
                    auto msg = buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors();       
                    msg.pull_push_sensors = pull_push_sensors_2;
                    publisher_->publish(msg);

                    // 递归调用以持续接收
                    start_receive_1();
                }
                else 
                {
                    RCLCPP_ERROR(this->get_logger(), "Receive Error: %s", error.message().c_str());
                }
            }
        );
    }

    // 重构后的 start_receive 函数
    std::vector<uint8_t> received_modbus_frame_2; 
    void start_receive_2() 
    {
        // received_modbus_frame_初始化大小
        received_modbus_frame_2.clear();
        received_modbus_frame_2.resize(256);

        // 异步读取串口数据serial_port_1
        serial_port_2->async_read_some(asio::buffer(received_modbus_frame_2, 256),
            [this](const asio::error_code& error, std::size_t bytes_transferred)
            {
                // 调整received_modbus_frame_2的大小以匹配实际接收到的字节数
                received_modbus_frame_2.resize(bytes_transferred);

                // 检查是否有错误
                if (!error) 
                {
                    // 将接收到的数据添加到数据缓存区
                    data_buffer_2.insert(data_buffer_2.end(), received_modbus_frame_2.begin(), received_modbus_frame_2.end());

                    // 打印data_buffer_2内容
                    // std::string msg_str = "";
                    // for (auto &byte : data_buffer_2) {
                    //     msg_str += "0x" + to_string(static_cast<int>(byte)) + " ";
                    // }
                    // RCLCPP_INFO(this->get_logger(), "Receive message: %s", msg_str.c_str());  


                    // 创建一个包含数据头的vector
                    std::vector<uint8_t> pull_push_sensors_header = {0xFE,0x01,0x50,0xFF};
                    // 搜索数据头
                    auto it_pull_push_encorder = std::search(data_buffer_2.begin(), data_buffer_2.end(), pull_push_sensors_header.begin(), pull_push_sensors_header.end());

                    // 如果找到了数据头，并且有足够的字节用于完整的34字节帧 (excluding the ending 0xCFFCCCFF)
                    if (it_pull_push_encorder != data_buffer_2.end() && std::distance(it_pull_push_encorder, data_buffer_2.end()) >= 34)
                    {
                        // 提取30字节帧
                        std::vector<uint8_t> frame(it_pull_push_encorder+1, it_pull_push_encorder + 30);

                        // 处理Modbus帧并获取elevator_counter
                        pull_push_sensors_2_part = process_modbus_frame_for_pull_push_sensors(frame);

                        for (size_t i = 0; i < 6; i++)
                        {
                            pull_push_sensors_2[2*i+1] = pull_push_sensors_2_part[i];
                        }

                        // 移除这34字节(including the ending 0xCFFCCCFF)及之前的字节
                        data_buffer_2.erase(data_buffer_2.begin(), it_pull_push_encorder + 34);
                    }
                    received_modbus_frame_2.clear();
                    auto msg = buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors();       
                    msg.pull_push_sensors = pull_push_sensors_2;
                    publisher_->publish(msg);

                    // 递归调用以持续接收
                    start_receive_2();
                }
                else 
                {
                    RCLCPP_ERROR(this->get_logger(), "Receive Error: %s", error.message().c_str());
                }
            }
        );
    }

 
    void callback(const buaa_rescue_robot_msgs::msg::ControlMessageMaster::SharedPtr msg){        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 延迟10毫秒   
        // reset the pull push sensors
        if (msg->pull_push_sensors_reset == 1)
        {
            // robomaster_1, snake pull push sensors reset control
            std::vector<uint8_t> frame = {0xFE,0x01,0x56,0xFF,0xE0,0x5F,0xCF,0xFC,0xCC,0xFF};
            // 打印发送的Modbus帧内容
            // std::string frame_str = "";
            // for (auto &byte : frame) {
            //     frame_str += "0x" + to_string(static_cast<int>(byte)) + " ";
            // }
            // RCLCPP_INFO(this->get_logger(), "Sending frame: %s", frame_str.c_str());
            // 通过串口发送数据帧
            async_write_to_serial_1(frame);
            async_write_to_serial_2(frame); 
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 延迟10毫秒   
    }

    void timer_callback()
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 延迟100毫秒   
        // robomaster_1, reading snake pull push sensors
        std::vector<uint8_t> frame = {0xFE,0x01,0x50,0xFF,0x40,0x5C,0xCF,0xFC,0xCC,0xFF};
        // 打印发送的Modbus帧内容
        // std::string frame_str = "";
        // for (auto &byte : frame) {
        //     frame_str += "0x" + to_string(static_cast<int>(byte)) + " ";
        // }
        // RCLCPP_INFO(this->get_logger(), "Sending frame: %s", frame_str.c_str());
        // 通过串口发送数据帧
        async_write_to_serial_1(frame);     
        async_write_to_serial_2(frame); 
    }


    rclcpp::TimerBase::SharedPtr timer_;    // 定时器

    rclcpp::Subscription<buaa_rescue_robot_msgs::msg::ControlMessageMaster>::SharedPtr subscription_;  //订阅器
    std::shared_ptr<asio::serial_port> serial_port_1;    // 串口对象
    std::shared_ptr<asio::serial_port> serial_port_2;    // 串口对象
    asio::io_service io_;   // ASIO I/O服务
    std::vector<uint8_t> modbus_frame_; // 存储Modbus协议帧
    
    std::vector<uint8_t> frame;  // Modbus协议帧
    std::array<int32_t, 12> pull_push_sensors_2;
    std::array<int32_t, 6> pull_push_sensors_2_part;

    std::vector<uint8_t> data_buffer_1;  // 数据缓存区
    std::vector<uint8_t> data_buffer_2;  // 数据缓存区
};

int main(int argc, char **argv) // 主函数
{
    rclcpp::init(argc, argv);  // 初始化ROS 2

    auto serial_PullPushSensors_2_node = std::make_shared<serial_PullPushSensors_2>();   // 创建pull_push_sensors_2对象

    // 创建一个新线程来运行ASIO的io_service
    std::thread asio_thread([&]() {
        serial_PullPushSensors_2_node->get_io_service().run();
    });

    rclcpp::spin(serial_PullPushSensors_2_node);  // 开始ROS的事件循环

    rclcpp::shutdown();  // 关闭ROS 2

    // 等待ASIO的io_service线程完成
    asio_thread.join();

    return 0;
}
