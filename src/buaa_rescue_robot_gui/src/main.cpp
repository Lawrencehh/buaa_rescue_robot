#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <QApplication>
#include <QLocale>
#include <QTranslator>


#include <rclcpp/rclcpp.hpp>
#include <QStringListModel>  

#include <QPushButton>
#include <QTimer>

#include <std_msgs/msg/string.hpp>
#include "buaa_rescue_robot_msgs/msg/control_message_master.hpp"  // 引入自定义消息类型
#include "buaa_rescue_robot_msgs/msg/control_message_slave.hpp"  // 引入自定义消息类型
#include "buaa_rescue_robot_msgs/msg/sensors_message_robomaster.hpp"  // 引入自定义消息类型
#include "buaa_rescue_robot_msgs/msg/sensors_message_master_device_elevator.hpp"   // 引入自定义消息类型
#include "buaa_rescue_robot_msgs/msg/sensors_message_master_device_pull_push_sensors.hpp"   // 引入自定义消息类型
#include "std_msgs/msg/float64_multi_array.hpp"



int main(int argc, char **argv) {
    // 初始化ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("buaa_rescue_robot_gui_node");
    

    // 初始化Qt
    qputenv("QT_QPA_PLATFORM", "xcb");
    QApplication app(argc, argv);

    MainWindow mainWindow;
    mainWindow.show();  // 显示主窗口

    // show 
    auto subscription_slave_control_topic = node->create_subscription<buaa_rescue_robot_msgs::msg::ControlMessageSlave>(
        "slave_control_topic",
        100,
        [&](const buaa_rescue_robot_msgs::msg::ControlMessageSlave::SharedPtr msg) {

            // 在这里更新QSpinBox的值
            mainWindow.updateSlaveControlIndicator(msg);     
        }
    );

    // show 
    auto subscription_master_control_topic = node->create_subscription<buaa_rescue_robot_msgs::msg::ControlMessageMaster>(
        "master_control_topic",
        100,
        [&](const buaa_rescue_robot_msgs::msg::ControlMessageMaster::SharedPtr msg) {

            // 在这里更新QSpinBox的值
            mainWindow.updateMasterControlIndicator(msg);     
        }
    );

    auto subscription_joint_space_topic = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "joint_space_topic",
        10,
        [&](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {

            // 在这里更新QSpinBox的值
            mainWindow.updateJointSpaceIndicator(msg);     

        }
    );

    auto subscription_sensors_topic_1 = node->create_subscription<buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster>(
        "Sensors_Robomaster_1",
        10,
        [&](const buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster::SharedPtr msg) {
            mainWindow.updateSensorsMessageDisplay_1(msg);  // 通过MainWindow实例来更新 
            mainWindow.updateSensorsMessageDisplay_6(msg);  // 通过MainWindow实例来更新 
        }
    );

    auto subscription_sensors_topic_2 = node->create_subscription<buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster>(
        "Sensors_Robomaster_2",
        10,
        [&](const buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster::SharedPtr msg) {
            mainWindow.updateSensorsMessageDisplay_2(msg);  // 通过MainWindow实例来更新 
            mainWindow.updateSensorsMessageDisplay_7(msg);  // 通过MainWindow实例来更新 
        }
    );

    auto subscription_sensors_topic_3 = node->create_subscription<buaa_rescue_robot_msgs::msg::SensorsMessageMasterDeviceElevator>(
        "Sensors_Elevator_LinearModules",
        10,
        [&](const buaa_rescue_robot_msgs::msg::SensorsMessageMasterDeviceElevator::SharedPtr msg) {
            mainWindow.updateSensorsMessageDisplay_3(msg);  // 通过MainWindow实例来更新 
        }
    );

    auto subscription_sensors_topic_4 = node->create_subscription<buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors>(
        "Sensors_Pull_Push_Sensors_1",
        10,
        [&](const buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors::SharedPtr msg) {
            mainWindow.updateSensorsMessageDisplay_4(msg);  // 通过MainWindow实例来更新 
        }
    );

    auto subscription_sensors_topic_5 = node->create_subscription<buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors>(
        "Sensors_Pull_Push_Sensors_2",
        10,
        [&](const buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors::SharedPtr msg) {
            mainWindow.updateSensorsMessageDisplay_4(msg);  // 通过MainWindow实例来更新 
        }
    );

    // 在Qt事件循环中添加ROS 2 spin
    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&]() {
        rclcpp::spin_some(node);
    });
    timer.start(100);


    return app.exec();
}
