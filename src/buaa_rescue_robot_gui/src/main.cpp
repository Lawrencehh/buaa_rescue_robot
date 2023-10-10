#include "mainwindow.h"

#include <QApplication>
#include <QLocale>
#include <QTranslator>

#include <QApplication>
#include <rclcpp/rclcpp.hpp>

#include <QPushButton>
#include <QTimer>

#include <std_msgs/msg/string.hpp>



int main(int argc, char **argv) {
    // 初始化ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("buaa_rescue_robot_gui_node");

    // 初始化Qt
    QApplication app(argc, argv);

    MainWindow mainWindow;
    mainWindow.show();  // 显示主窗口


    // show 
    auto subscription_control_topic = node->create_subscription<buaa_rescue_robot_msgs::msg::ControlMessage>(
        "control_topic",
        10,
        [&](const buaa_rescue_robot_msgs::msg::ControlMessage::SharedPtr msg) {
            QStringList list;
            for (int i = 0; i < 12; ++i) {
                list << QString::number(msg->snake_control_1_array[i]);
            }
            mainWindow.updateSnakePositionControlModel1(list);  // 通过MainWindow实例来更新
        }
    );


    // 在Qt事件循环中添加ROS 2 spin（可选）
    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&]() {
        rclcpp::spin_some(node);
    });
    timer.start(100);

    return app.exec();
}
