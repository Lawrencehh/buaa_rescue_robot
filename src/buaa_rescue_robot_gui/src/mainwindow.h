#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <opencv2/opencv.hpp>
#include <QMainWindow>
#include <rclcpp/rclcpp.hpp>  
#include <QStringListModel>  
#include "buaa_rescue_robot_msgs/msg/control_message_master.hpp"  // 引入自定义消息类型
#include "buaa_rescue_robot_msgs/msg/control_message_slave.hpp"  // 引入自定义消息类型
#include "buaa_rescue_robot_msgs/msg/sensors_message_robomaster.hpp"  // 引入自定义消息类型
#include "buaa_rescue_robot_msgs/msg/sensors_message_master_device_elevator.hpp"   // 引入自定义消息类型
#include "buaa_rescue_robot_msgs/msg/sensors_message_master_device_pull_push_sensors.hpp"   // 引入自定义消息类型
#include "std_msgs/msg/float64_multi_array.hpp"
#include <QTimer>
#include "./ui_mainwindow.h"
#include <QDebug>
#include <cmath> // 为了使用 fabs 和 M_PI

#include <iostream>
#include <array>


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void updateSensorsMessageDisplay_1(const buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster::SharedPtr msg) {
        QStringList list;

        for (int i = 0; i < 12; ++i) {
            list << QString::number(msg->snake_motor_encorder_position[i]);
        }
        list << QString::number(msg->gripper_gm6020_position);
        list << QString::number(msg->gripper_c610_position);
        list << QString::number(msg->gripper_sts3032_position);
        list << QString::number(msg->robomaster_mode);
        SensorsMessageDisplay_1->setStringList(list);
    }

    void updateSensorsMessageDisplay_2(const buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster::SharedPtr msg) {
        QStringList list;

        for (int i = 0; i < 12; ++i) {
            list << QString::number(msg->snake_motor_encorder_position[i]);
        }
        list << QString::number(msg->gripper_gm6020_position);
        list << QString::number(msg->gripper_c610_position);
        list << QString::number(msg->gripper_sts3032_position);
        list << QString::number(msg->robomaster_mode);
        SensorsMessageDisplay_1->setStringList(list);
    }

    void updateSensorsMessageDisplay_3(const buaa_rescue_robot_msgs::msg::SensorsMessageMasterDeviceElevator::SharedPtr msg) {
        QStringList list;
        list << QString::number(msg->elevator_counter);
        list << QString::number(msg->lower_encorder, 'f', 2);
        list << QString::number(msg->upper_encorder, 'f', 2);
        SensorsMessageDisplay_3->setStringList(list);
    }

    void updateSensorsMessageDisplay_4(const buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors::SharedPtr msg) {
        QStringList list;

        for (int i = 0; i < 12; ++i) {
            list << QString::number(msg->pull_push_sensors[i]);
        }    
        SensorsMessageDisplay_4->setStringList(list);
    }

    void updateSensorsMessageDisplay_5(const buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors::SharedPtr msg) {
        QStringList list;

        for (int i = 0; i < 12; ++i) {
            list << QString::number(msg->pull_push_sensors[i]);
        }    
        SensorsMessageDisplay_4->setStringList(list);
    }

    void updateSensorsMessageDisplay_6(const buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster::SharedPtr msg) {
        QStringList list;

        for (int i = 0; i < 12; ++i) {
            list << QString::number(msg->snake_motor_encorder_speed[i]);
        }    
        SensorsMessageDisplay_6->setStringList(list);
    }

    void updateSensorsMessageDisplay_7(const buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster::SharedPtr msg) {
        QStringList list;

        for (int i = 0; i < 12; ++i) {
            list << QString::number(msg->snake_motor_encorder_speed[i]);
        }    
        SensorsMessageDisplay_6->setStringList(list);
    }



    void updateSlaveControlIndicator(const buaa_rescue_robot_msgs::msg::ControlMessageSlave::SharedPtr msg) {
        // 在这里更新QSpinBox的值
        // robomaster 1
        ui->robomaster1_snake_motor_speed_control_1->setValue(msg->snake_speed_control_1_array[0]);  
        ui->robomaster1_snake_motor_speed_control_2->setValue(msg->snake_speed_control_1_array[1]);
        ui->robomaster1_snake_motor_speed_control_3->setValue(msg->snake_speed_control_1_array[2]);
        ui->robomaster1_snake_motor_speed_control_4->setValue(msg->snake_speed_control_1_array[3]);
        ui->robomaster1_snake_motor_speed_control_5->setValue(msg->snake_speed_control_1_array[4]);
        ui->robomaster1_snake_motor_speed_control_6->setValue(msg->snake_speed_control_1_array[5]);
        ui->robomaster1_snake_motor_speed_control_7->setValue(msg->snake_speed_control_1_array[6]);
        ui->robomaster1_snake_motor_speed_control_8->setValue(msg->snake_speed_control_1_array[7]);
        ui->robomaster1_snake_motor_speed_control_9->setValue(msg->snake_speed_control_1_array[8]);
        ui->robomaster1_snake_motor_speed_control_10->setValue(msg->snake_speed_control_1_array[9]);
        ui->robomaster1_snake_motor_speed_control_11->setValue(msg->snake_speed_control_1_array[10]);
        ui->robomaster1_snake_motor_speed_control_12->setValue(msg->snake_speed_control_1_array[11]);
        ui->robomaster1_snake_motor_position_control_1->setValue(msg->snake_position_control_1_array[0]);  
        ui->robomaster1_snake_motor_position_control_2->setValue(msg->snake_position_control_1_array[1]);
        ui->robomaster1_snake_motor_position_control_3->setValue(msg->snake_position_control_1_array[2]);
        ui->robomaster1_snake_motor_position_control_4->setValue(msg->snake_position_control_1_array[3]);
        ui->robomaster1_snake_motor_position_control_5->setValue(msg->snake_position_control_1_array[4]);
        ui->robomaster1_snake_motor_position_control_6->setValue(msg->snake_position_control_1_array[5]);
        ui->robomaster1_snake_motor_position_control_7->setValue(msg->snake_position_control_1_array[6]);
        ui->robomaster1_snake_motor_position_control_8->setValue(msg->snake_position_control_1_array[7]);
        ui->robomaster1_snake_motor_position_control_9->setValue(msg->snake_position_control_1_array[8]);
        ui->robomaster1_snake_motor_position_control_10->setValue(msg->snake_position_control_1_array[9]);
        ui->robomaster1_snake_motor_position_control_11->setValue(msg->snake_position_control_1_array[10]);
        ui->robomaster1_snake_motor_position_control_12->setValue(msg->snake_position_control_1_array[11]);
        ui->gripper_gm6020_position_1_control->setValue(msg->gripper_gm6020_position_1);
        ui->gripper_c610_position_1_control->setValue(msg->gripper_c610_position_1);
        ui->gripper_sts3032_position_1_control->setValue(msg->gripper_sts3032_position_1);
        ui->gripper1_gm6020->setValue(msg->gripper_gm6020_position_1);
        ui->gripper1_c610->setValue(msg->gripper_c610_position_1);
        ui->gripper1_sts3032->setValue(msg->gripper_sts3032_position_1);
        ui->robomaster1_mode->setValue(msg->robomaster_1_mode);
        
        // robomaster 2
        ui->robomaster2_snake_motor_speed_control_1->setValue(msg->snake_speed_control_2_array[0]);  
        ui->robomaster2_snake_motor_speed_control_2->setValue(msg->snake_speed_control_2_array[1]);
        ui->robomaster2_snake_motor_speed_control_3->setValue(msg->snake_speed_control_2_array[2]);
        ui->robomaster2_snake_motor_speed_control_4->setValue(msg->snake_speed_control_2_array[3]);
        ui->robomaster2_snake_motor_speed_control_5->setValue(msg->snake_speed_control_2_array[4]);
        ui->robomaster2_snake_motor_speed_control_6->setValue(msg->snake_speed_control_2_array[5]);
        ui->robomaster2_snake_motor_speed_control_7->setValue(msg->snake_speed_control_2_array[6]);
        ui->robomaster2_snake_motor_speed_control_8->setValue(msg->snake_speed_control_2_array[7]);
        ui->robomaster2_snake_motor_speed_control_9->setValue(msg->snake_speed_control_2_array[8]);
        ui->robomaster2_snake_motor_speed_control_10->setValue(msg->snake_speed_control_2_array[9]);
        ui->robomaster2_snake_motor_speed_control_11->setValue(msg->snake_speed_control_2_array[10]);
        ui->robomaster2_snake_motor_speed_control_12->setValue(msg->snake_speed_control_2_array[11]);
        ui->robomaster2_snake_motor_position_control_1->setValue(msg->snake_position_control_2_array[0]);  
        ui->robomaster2_snake_motor_position_control_2->setValue(msg->snake_position_control_2_array[1]);
        ui->robomaster2_snake_motor_position_control_3->setValue(msg->snake_position_control_2_array[2]);
        ui->robomaster2_snake_motor_position_control_4->setValue(msg->snake_position_control_2_array[3]);
        ui->robomaster2_snake_motor_position_control_5->setValue(msg->snake_position_control_2_array[4]);
        ui->robomaster2_snake_motor_position_control_6->setValue(msg->snake_position_control_2_array[5]);
        ui->robomaster2_snake_motor_position_control_7->setValue(msg->snake_position_control_2_array[6]);
        ui->robomaster2_snake_motor_position_control_8->setValue(msg->snake_position_control_2_array[7]);
        ui->robomaster2_snake_motor_position_control_9->setValue(msg->snake_position_control_2_array[8]);
        ui->robomaster2_snake_motor_position_control_10->setValue(msg->snake_position_control_2_array[9]);
        ui->robomaster2_snake_motor_position_control_11->setValue(msg->snake_position_control_2_array[10]);
        ui->robomaster2_snake_motor_position_control_12->setValue(msg->snake_position_control_2_array[11]);
        ui->gripper_gm6020_position_2_control->setValue(msg->gripper_gm6020_position_2);
        ui->gripper_c610_position_2_control->setValue(msg->gripper_c610_position_2);
        ui->gripper_sts3032_position_2_control->setValue(msg->gripper_sts3032_position_2);
        ui->gripper2_gm6020->setValue(msg->gripper_gm6020_position_2);
        ui->gripper2_c610->setValue(msg->gripper_c610_position_2);
        ui->gripper2_sts3032->setValue(msg->gripper_sts3032_position_2);
        ui->robomaster2_mode->setValue(msg->robomaster_2_mode);
    }

    void updateMasterControlIndicator(const buaa_rescue_robot_msgs::msg::ControlMessageMaster::SharedPtr msg) {
        // 在这里更新QSpinBox的值
        // master devices
        ui->elevator_speed_control->setValue(msg->elevator_control);
        ui->lower_LM_speed_control->setValue(msg->lower_linear_module_control);
        ui->upper_LM_speed_control->setValue(msg->upper_linear_module_control);
        ui->PP_sensors_reset->setValue(msg->pull_push_sensors_reset);
        ui->elevator_counter_reset->setValue(msg->elevator_counter_reset);
        ui->lower_LM_encorder_reset->setValue(msg->lower_linear_module_encorder_reset);
        ui->upper_LM_encorder_reset->setValue(msg->upper_linear_module_encorder_reset);
    }

    void updateJointSpaceIndicator(const std_msgs::msg::Float64MultiArray::SharedPtr theta_msg) {
        // 在这里更新QSpinBox的值, rad to degree
        ui->robomaster1_theta_1->setValue(theta_msg->data[0] * 180 / M_PI);  
        ui->robomaster1_theta_2->setValue(theta_msg->data[1] * 180 / M_PI);
        ui->robomaster1_theta_3->setValue(theta_msg->data[2] * 180 / M_PI);
        ui->robomaster1_theta_4->setValue(theta_msg->data[3] * 180 / M_PI);
        ui->robomaster1_theta_5->setValue(theta_msg->data[4] * 180 / M_PI);
        ui->robomaster1_theta_6->setValue(theta_msg->data[5] * 180 / M_PI);
        ui->robomaster2_theta_1->setValue(theta_msg->data[6] * 180 / M_PI);
        ui->robomaster2_theta_2->setValue(theta_msg->data[7] * 180 / M_PI);
        ui->robomaster2_theta_3->setValue(theta_msg->data[8] * 180 / M_PI);
        ui->robomaster2_theta_4->setValue(theta_msg->data[9] * 180 / M_PI);
        ui->robomaster2_theta_5->setValue(theta_msg->data[10] * 180 / M_PI);
        ui->robomaster2_theta_6->setValue(theta_msg->data[11] * 180 / M_PI);
    }


public slots:
    void on_publishButton_clicked();
    void on_transButton_clicked();
    void updateCameraFrame();
    void dialValueChanged(int value);

private:
    
    rclcpp::Node::SharedPtr node;  // 添加这一行，声明一个ROS 2节点
    rclcpp::Publisher<buaa_rescue_robot_msgs::msg::ControlMessageSlave>::SharedPtr slave_control_topic_publisher;  // 添加这一行，声明一个ROS 2发布器
    rclcpp::Publisher<buaa_rescue_robot_msgs::msg::ControlMessageMaster>::SharedPtr master_control_topic_publisher;  // 添加这一行，声明一个ROS 2发布器
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_space_topic_publisher;  // 添加这一行，声明一个ROS 2发布器

    QStringListModel *ControlMessageDisplay;  
    QStringListModel *SensorsMessageDisplay_1;
    QStringListModel *SensorsMessageDisplay_2;
    QStringListModel *SensorsMessageDisplay_3;
    QStringListModel *SensorsMessageDisplay_4;
    QStringListModel *SensorsMessageDisplay_5;
    QStringListModel *SensorsMessageDisplay_6;
    QStringListModel *SensorsMessageDisplay_7;
    Ui::MainWindow *ui;

    cv::VideoCapture cap;
    QGraphicsScene *scene;
    QTimer *timer;

protected:
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;
    
};
#endif // MAINWINDOW_H
