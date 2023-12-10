#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <opencv2/opencv.hpp>
#include <QMainWindow>
#include <rclcpp/rclcpp.hpp>  
#include <QStringListModel>  
#include "buaa_rescue_robot_msgs/msg/control_message_master.hpp"  // 引入自定义消息类型
#include "buaa_rescue_robot_msgs/msg/control_message_slave.hpp"  // 引入自定义消息类型
#include "buaa_rescue_robot_msgs/msg/control_message_slave_gripper.hpp"  // 引入自定义消息类型
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

#include <thread>
#include <atomic>


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    // 摄像头
    void startCameraThreads();
    void stopCameraThreads();
    void initializeCameras(); 
    void initializeCamera(int cameraIndex, cv::VideoCapture &camera, const char *cameraName);
    void updateSensorsMessageDisplay_1(const buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster::SharedPtr msg);
    void updateSensorsMessageDisplay_2(const buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster::SharedPtr msg);
    void updateSensorsMessageDisplay_3(const buaa_rescue_robot_msgs::msg::SensorsMessageMasterDeviceElevator::SharedPtr msg);
    void updateSensorsMessageDisplay_4(const buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors::SharedPtr msg);
    void updateSensorsMessageDisplay_5(const buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors::SharedPtr msg);
    void updateSensorsMessageDisplay_6(const buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster::SharedPtr msg);
    void updateSensorsMessageDisplay_7(const buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster::SharedPtr msg);
    void updateSlaveControlIndicator1(const buaa_rescue_robot_msgs::msg::ControlMessageSlave::SharedPtr msg);
    void updateSlaveControlIndicator2(const buaa_rescue_robot_msgs::msg::ControlMessageSlave::SharedPtr msg);
    void updateSlaveGripperControlIndicator1(const buaa_rescue_robot_msgs::msg::ControlMessageSlaveGripper::SharedPtr msg);
    void updateSlaveGripperControlIndicator2(const buaa_rescue_robot_msgs::msg::ControlMessageSlaveGripper::SharedPtr msg);
    void updateMasterControlIndicator(const buaa_rescue_robot_msgs::msg::ControlMessageMaster::SharedPtr msg);
    void updateJointSpaceIndicator(const std_msgs::msg::Float64MultiArray::SharedPtr theta_msg);

public slots:
    void on_publishButton_clicked();
    void on_transButton_clicked();
    void sliderValueChanged(int value);
    void updateCameraFrame(cv::VideoCapture &cap, QGraphicsScene *scene, QGraphicsView *view);
    void updateGraphicsView(QGraphicsScene* scene, QPixmap pixmap);

// 新的信号定义
signals:
    void signalUpdateGraphicsView(QGraphicsScene* scene, QPixmap pixmap, QGraphicsView* view);

private:
    
    rclcpp::Node::SharedPtr node;  // 添加这一行，声明一个ROS 2节点
    rclcpp::Publisher<buaa_rescue_robot_msgs::msg::ControlMessageSlave>::SharedPtr slave_control_topic_publisher_1;  // 添加这一行，声明一个ROS 2发布器
    rclcpp::Publisher<buaa_rescue_robot_msgs::msg::ControlMessageSlave>::SharedPtr slave_control_topic_publisher_2;  // 添加这一行，声明一个ROS 2发布器
    rclcpp::Publisher<buaa_rescue_robot_msgs::msg::ControlMessageSlaveGripper>::SharedPtr gripper_control_topic_publisher_1;  // 添加这一行，声明一个ROS 2发布器
    rclcpp::Publisher<buaa_rescue_robot_msgs::msg::ControlMessageSlaveGripper>::SharedPtr gripper_control_topic_publisher_2;  // 添加这一行，声明一个ROS 2发布器
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
    cv::VideoCapture cap1, cap2, cap3; // 三个摄像头
    QGraphicsScene *scene1, *scene2, *scene3; // 三个场景
    QTimer *timer;

    // 新的线程成员
    std::thread cameraThread1, cameraThread2, cameraThread3;
    std::atomic<bool> cameraThreadRunning;

    // 生成控件的指针数组
    std::vector<QDoubleSpinBox*> thetaControls;
    std::vector<QSpinBox*> speedControls_robomaster1;
    std::vector<QSpinBox*> positionControls_robomaster1;
    std::vector<QSpinBox*> speedControls_robomaster2;
    std::vector<QSpinBox*> positionControls_robomaster2;

protected:
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;
    
};
#endif // MAINWINDOW_H
