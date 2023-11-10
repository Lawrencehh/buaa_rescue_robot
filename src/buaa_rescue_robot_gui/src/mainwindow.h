#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <opencv2/opencv.hpp>
#include <QMainWindow>
#include <rclcpp/rclcpp.hpp>  
#include <QStringListModel>  
#include "buaa_rescue_robot_msgs/msg/control_message.hpp"  // 引入自定义消息类型
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
        ui->gripper1_gm6020->setValue(msg->gripper_gm6020_position_1);
        ui->gripper1_c610->setValue(msg->gripper_c610_position_1);
        ui->gripper1_sts3032->setValue(msg->gripper_sts3032_position_1);
        ui->robomaster1_mode->setValue(msg->robomaster_1_mode);
        
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
        ui->gripper2_gm6020->setValue(msg->gripper_gm6020_position_2);
        ui->gripper2_c610->setValue(msg->gripper_c610_position_2);
        ui->gripper2_sts3032->setValue(msg->gripper_sts3032_position_2);
        ui->robomaster2_mode->setValue(msg->robomaster_2_mode);
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

        std::array<double, 6> theta_1; // left hand omega7 
        std::array<double, 6> theta_2; // right hand omega7 
        std::array<double, 6> theta_initial = {0,0,0,0,0,0};
        for (size_t i = 0; i < 6; i++)
        {
            theta_1[i] = theta_msg->data[i];
        }
        for (size_t i = 6; i < 12; i++)
        {
            theta_2[i-6] = theta_msg->data[i];
        }
        std::array<double, 12> rope_1 = theta2rope(theta_1);
        std::array<double, 12> rope_2 = theta2rope(theta_2);
        std::array<double, 12> rope_initial = theta2rope(theta_initial);

        auto msg = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessage>();
        // snake motors control for robomaster 1
        for (size_t i = 0; i < 12; i++)
        {
            msg-> snake_control_1_array[i]  = (rope_initial[i] - rope_1[i]) * 65536/4; // mm transformed to pulse
        }

        // gripper control for robomaster 1
        msg->gripper_gm6020_position_1 = ui->gripper_gm6020_position_1_control->value();
        msg->gripper_c610_position_1 = ui->gripper_c610_position_1_control->value();
        msg->gripper_sts3032_position_1 = ui->gripper_sts3032_position_1_control->value();
        msg->robomaster_1_mode = ui->robomaster1_mode->value();
        // snake motors control for robomaster 2
        for (size_t i = 0; i < 12; i++)
        {
            msg-> snake_control_2_array[i]  = (rope_initial[i] - rope_2[i]) * 65536/4;
        }
        // gripper control for robomaster 2
        msg->gripper_gm6020_position_2 = ui->gripper_gm6020_position_2_control->value();
        msg->gripper_c610_position_2 = ui->gripper_c610_position_2_control->value();
        msg->gripper_sts3032_position_2 = ui->gripper_sts3032_position_2_control->value();
        msg->robomaster_2_mode = ui->robomaster2_mode->value();
        // master devices control
        msg->elevator_control = ui->elevator_speed_control->value();
        msg->lower_linear_module_control = ui->lower_LM_speed_control->value();
        msg->upper_linear_module_control = ui->upper_LM_speed_control->value();
        msg->pull_push_sensors_reset = ui->PP_sensors_reset->value();
        msg->elevator_counter_reset = ui->elevator_counter_reset->value();
        msg->lower_linear_module_encorder_reset = ui->lower_LM_encorder_reset->value();
        msg->upper_linear_module_encorder_reset = ui->upper_LM_encorder_reset->value();

        // 发布消息
        control_topic_publisher->publish(*msg);

    }

// theta to rope
std::array<double, 12> theta2rope(const std::array<double, 6>& theta) {
    // 定义各间隙变量，并赋予固定值
    std::array<double, 12> Gap_1 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::array<double, 12> Gap_2 = {29.63, 29.63, 29.09, 29.09, 29.63, 29.63, 29.63, 29.63, 29.09, 29.09, 29.63, 29.63};
    std::array<double, 12> Gap_3 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::array<double, 12> Gap_4 = {29.63, 29.63, 29.09, 29.09, 29.63, 29.63, 29.63, 29.63, 29.09, 29.09, 29.63, 29.63};
    std::array<double, 12> Gap_base = {24.57, 24.57, 26.11, 26.11, 28.12, 28.12, 28.12, 28.12, 26.11, 26.11, 24.57, 24.57};
    
    std::array<double, 12> Rope_length = {0}; // 初始化为0
    double beta = M_PI * 50 / 180; // 初始角度
    std::array<double, 12> R_Gap_1 = {23.03, 23.03, 18.82, 18.82, 13.33, 13.33, 13.33, 13.33, 18.82, 18.82, 23.03, 23.03};
    std::array<double, 12> R_Gap_3 = {13.33, 13.33, 18.82, 18.82, 23.03, 23.03, 23.03, 23.03, 18.82, 18.82, 13.33, 13.33};

    for (int N = 0; N < 5; ++N) {
        for (int i = 0; i < 12; ++i) {
            // 计算第一节的Gap1和Gap3
            if (i > 5) {
                Gap_1[i] = sin((beta - theta[0]) / 2) * 2 * R_Gap_1[i];
            } else {
                Gap_1[i] = sin((beta + theta[0]) / 2) * 2 * R_Gap_1[i];
            }

            if (i % 2 == 0) {
                Gap_3[i] = sin((beta + theta[1]) / 2) * 2 * R_Gap_3[i];
            } else {
                Gap_3[i] = sin((beta - theta[1]) / 2) * 2 * R_Gap_3[i];
            }

            Rope_length[i] += Gap_1[i] + Gap_2[i] + Gap_3[i] + Gap_4[i] + Gap_base[i];

            // 计算第二节的Gap1和Gap3，除了1,2,11,12
            if (i != 0 && i != 1 && i != 10 && i != 11) {
                if (i > 5) {
                    Gap_1[i] = sin((beta - theta[2]) / 2) * 2 * R_Gap_1[i];
                } else {
                    Gap_1[i] = sin((beta + theta[2]) / 2) * 2 * R_Gap_1[i];
                }

                if (i % 2 == 0) {
                    Gap_3[i] = sin((beta + theta[3]) / 2) * 2 * R_Gap_3[i];
                } else {
                    Gap_3[i] = sin((beta - theta[3]) / 2) * 2 * R_Gap_3[i];
                }

                Rope_length[i] += Gap_1[i] + Gap_2[i] + Gap_3[i] + Gap_4[i];
            }

            // 计算第三节的Gap1和Gap3，除了1,2,11,12,3,4,9,10
            if (i != 0 && i != 1 && i != 10 && i != 11 && i != 2 && i != 3 && i != 8 && i != 9) {
                if (i > 5) {
                    Gap_1[i] = sin((beta - theta[4]) / 2) * 2 * R_Gap_1[i];
                } else {
                    Gap_1[i] = sin((beta + theta[4]) / 2) * 2 * R_Gap_1[i];
                }

                if (i % 2 == 0) {
                    Gap_3[i] = sin((beta + theta[5]) / 2) * 2 * R_Gap_3[i];
                } else {
                    Gap_3[i] = sin((beta - theta[5]) / 2) * 2 * R_Gap_3[i];
                }

                Rope_length[i] += Gap_1[i] + Gap_2[i] + Gap_3[i] + Gap_4[i];
            }
        }
    }

    // 所有关节都计算完之后，每个关节和下一个关节连接处的圆盘内绳长需减掉一截Gap_4
    for (int i = 0; i < 12; ++i) {
        Rope_length[i] -= Gap_4[i];
    }

    return Rope_length;
}

public slots:
    void on_publishButton_clicked();
    void updateCameraFrame();
    void dialValueChanged(int value);

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

protected:
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;
    
};
#endif // MAINWINDOW_H
