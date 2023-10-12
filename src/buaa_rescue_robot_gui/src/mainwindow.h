#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <opencv2/opencv.hpp>
#include <QMainWindow>
#include <rclcpp/rclcpp.hpp>  
#include <QStringListModel>  
#include "buaa_rescue_robot_msgs/msg/control_message.hpp"  // 引入自定义消息类型
#include "buaa_rescue_robot_msgs/msg/sensors_message_robomaster1.hpp"  // 引入自定义消息类型
#include "buaa_rescue_robot_msgs/msg/sensors_message_master_device_elevator.hpp"   // 引入自定义消息类型
#include "buaa_rescue_robot_msgs/msg/sensors_message_master_device_pull_push_sensors.hpp"   // 引入自定义消息类型
#include <QTimer>
#include "./ui_mainwindow.h"
#include <QDebug>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void updateControlMessageDisplay(const buaa_rescue_robot_msgs::msg::ControlMessage::SharedPtr msg) {
        QStringList list;
        for (int i = 0; i < 12; ++i) {
            list << QString::number(msg->snake_control_1_array[i]);
        }
        list << QString::number(msg->gripper_gm6020_position_1);
        list << QString::number(msg->gripper_c610_position_1);
        list << QString::number(msg->gripper_sts3032_position_1);
        list << QString::number(msg->robomaster_1_reset);
        list << "";
        for (int i = 0; i < 12; ++i) {
            list << QString::number(msg->snake_control_2_array[i]);
        }
        list << QString::number(msg->gripper_gm6020_position_2);
        list << QString::number(msg->gripper_c610_position_2);
        list << QString::number(msg->gripper_sts3032_position_2);
        list << QString::number(msg->robomaster_2_reset);
        list << "";
        list << QString::number(msg->elevator_control);
        list << QString::number(msg->lower_linear_module_control);
        list << QString::number(msg->upper_linear_module_control);
        list << QString::number(msg->pull_push_sensors_reset);
        list << QString::number(msg->elevator_counter_reset);
        list << QString::number(msg->lower_linear_module_encorder_reset);
        list << QString::number(msg->upper_linear_module_encorder_reset);
        ControlMessageDisplay->setStringList(list);
    }

    void updateSensorsMessageDisplay_1(const buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster1::SharedPtr msg) {
        QStringList list;

        for (int i = 0; i < 12; ++i) {
            list << QString::number(msg->snake_motor_encorder_position_1[i]);
        }
        list << QString::number(msg->gripper_gm6020_position_1);
        list << QString::number(msg->gripper_c610_position_1);
        list << QString::number(msg->gripper_sts3032_position_1);
        list << QString::number(msg->robomaster_1_reset);
        SensorsMessageDisplay_1->setStringList(list);
    }

    void updateSensorsMessageDisplay_3(const buaa_rescue_robot_msgs::msg::SensorsMessageMasterDeviceElevator::SharedPtr msg) {
        QStringList list;
        list << QString::number(msg->elevator_counter);
        list << QString::number(msg->lower_encorder);
        list << QString::number(msg->upper_encorder);
        SensorsMessageDisplay_3->setStringList(list);
    }

    void updateSensorsMessageDisplay_4(const buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors::SharedPtr msg) {
        QStringList list;

        for (int i = 0; i < 12; ++i) {
            list << QString::number(msg->pull_push_sensors_1[i]);
        }
        SensorsMessageDisplay_4->setStringList(list);
    }

    void updateControlIndicator(const buaa_rescue_robot_msgs::msg::ControlMessage::SharedPtr msg) {
        // 在这里更新QSpinBox的值
        // robomaster 1
        ui->robomaster1_snake_motor_position_control_1->setValue(msg->snake_control_1_array[0]);  
        ui->robomaster1_snake_motor_position_control_2->setValue(msg->snake_control_1_array[1]);
        ui->robomaster1_snake_motor_position_control_3->setValue(msg->snake_control_1_array[2]);
        ui->robomaster1_snake_motor_position_control_4->setValue(msg->snake_control_1_array[3]);
        ui->robomaster1_snake_motor_position_control_5->setValue(msg->snake_control_1_array[4]);
        ui->robomaster1_snake_motor_position_control_6->setValue(msg->snake_control_1_array[5]);
        ui->robomaster1_snake_motor_position_control_7->setValue(msg->snake_control_1_array[6]);
        ui->robomaster1_snake_motor_position_control_8->setValue(msg->snake_control_1_array[7]);
        ui->robomaster1_snake_motor_position_control_9->setValue(msg->snake_control_1_array[8]);
        ui->robomaster1_snake_motor_position_control_10->setValue(msg->snake_control_1_array[9]);
        ui->robomaster1_snake_motor_position_control_11->setValue(msg->snake_control_1_array[10]);
        ui->robomaster1_snake_motor_position_control_12->setValue(msg->snake_control_1_array[11]);
        ui->gripper_gm6020_position_1_control->setValue(msg->gripper_gm6020_position_1);
        ui->gripper_c610_position_1_control->setValue(msg->gripper_c610_position_1);
        ui->gripper_sts3032_position_1_control->setValue(msg->gripper_sts3032_position_1);
        ui->robomaster1_sensors_reset->setValue(msg->robomaster_1_reset);
        // robomaster 2
        ui->robomaster2_snake_motor_position_control_1->setValue(msg->snake_control_2_array[0]);  
        ui->robomaster2_snake_motor_position_control_2->setValue(msg->snake_control_2_array[1]);
        ui->robomaster2_snake_motor_position_control_3->setValue(msg->snake_control_2_array[2]);
        ui->robomaster2_snake_motor_position_control_4->setValue(msg->snake_control_2_array[3]);
        ui->robomaster2_snake_motor_position_control_5->setValue(msg->snake_control_2_array[4]);
        ui->robomaster2_snake_motor_position_control_6->setValue(msg->snake_control_2_array[5]);
        ui->robomaster2_snake_motor_position_control_7->setValue(msg->snake_control_2_array[6]);
        ui->robomaster2_snake_motor_position_control_8->setValue(msg->snake_control_2_array[7]);
        ui->robomaster2_snake_motor_position_control_9->setValue(msg->snake_control_2_array[8]);
        ui->robomaster2_snake_motor_position_control_10->setValue(msg->snake_control_2_array[9]);
        ui->robomaster2_snake_motor_position_control_11->setValue(msg->snake_control_2_array[10]);
        ui->robomaster2_snake_motor_position_control_12->setValue(msg->snake_control_2_array[11]);
        ui->gripper_gm6020_position_2_control->setValue(msg->gripper_gm6020_position_2);
        ui->gripper_c610_position_2_control->setValue(msg->gripper_c610_position_2);
        ui->gripper_sts3032_position_2_control->setValue(msg->gripper_sts3032_position_2);
        ui->robomaster2_sensors_reset->setValue(msg->robomaster_2_reset);
        // master devices
        ui->elevator_speed_control->setValue(msg->elevator_control);
        ui->lower_LM_speed_control->setValue(msg->lower_linear_module_control);
        ui->upper_LM_speed_control->setValue(msg->upper_linear_module_control);
        ui->PP_sensors_reset->setValue(msg->pull_push_sensors_reset);
        ui->elevator_counter_reset->setValue(msg->elevator_counter_reset);
        ui->lower_LM_encorder_reset->setValue(msg->lower_linear_module_encorder_reset);
        ui->upper_LM_encorder_reset->setValue(msg->upper_linear_module_encorder_reset);
    }

public slots:
    void on_publishButton_clicked();
    void updateCameraFrame();

private:
    
    rclcpp::Node::SharedPtr node;  // 添加这一行，声明一个ROS 2节点
    rclcpp::Subscription<buaa_rescue_robot_msgs::msg::ControlMessage>::SharedPtr ControlMessageSubscription_;
    rclcpp::Subscription<buaa_rescue_robot_msgs::msg::ControlMessage>::SharedPtr SensorsMessageSubscription_1;
    rclcpp::Subscription<buaa_rescue_robot_msgs::msg::ControlMessage>::SharedPtr SensorsMessageSubscription_2;
    rclcpp::Subscription<buaa_rescue_robot_msgs::msg::SensorsMessageMasterDeviceElevator>::SharedPtr SensorsMessageSubscription_3;
    rclcpp::Subscription<buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors>::SharedPtr SensorsMessageSubscription_4;
    rclcpp::Subscription<buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors>::SharedPtr SensorsMessageSubscription_5;
    rclcpp::Publisher<buaa_rescue_robot_msgs::msg::ControlMessage>::SharedPtr control_topic_publisher;  // 添加这一行，声明一个ROS 2发布器
    rclcpp::Subscription<buaa_rescue_robot_msgs::msg::ControlMessage>::SharedPtr control_topic_subscription; // ROS 2 subscriber
    QStringListModel *ControlMessageDisplay;  
    QStringListModel *SensorsMessageDisplay_1;
    QStringListModel *SensorsMessageDisplay_2;
    QStringListModel *SensorsMessageDisplay_3;
    QStringListModel *SensorsMessageDisplay_4;
    QStringListModel *SensorsMessageDisplay_5;
    Ui::MainWindow *ui;

    cv::VideoCapture cap;
    QGraphicsScene *scene;
    QTimer *timer;


    
};
#endif // MAINWINDOW_H
