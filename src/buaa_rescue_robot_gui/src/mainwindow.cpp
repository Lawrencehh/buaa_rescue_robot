#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <opencv2/opencv.hpp>
#include <QKeyEvent>  // 引入QKeyEvent头文件
#include <thread>
// #include <chrono>
#include <QTime>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

QTime lastTime = QTime::currentTime(); // 防抖动
bool reset_flag = false;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    cap.open(0);  // 打开默认摄像头
    scene = new QGraphicsScene(this);
    ui->camera1->setScene(scene);

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateCameraFrame()));
    timer->start(100); // 10 fps

    // 连接 QDial 的 valueChanged 信号到自定义的槽函数
    connect(ui->gripper1_gm6020, SIGNAL(valueChanged(int)), this, SLOT(dialValueChanged(int)));
    connect(ui->gripper1_c610, SIGNAL(valueChanged(int)), this, SLOT(dialValueChanged(int)));
    connect(ui->gripper1_sts3032, SIGNAL(valueChanged(int)), this, SLOT(dialValueChanged(int)));
    connect(ui->gripper2_gm6020, SIGNAL(valueChanged(int)), this, SLOT(dialValueChanged(int)));
    connect(ui->gripper2_c610, SIGNAL(valueChanged(int)), this, SLOT(dialValueChanged(int)));
    connect(ui->gripper2_sts3032, SIGNAL(valueChanged(int)), this, SLOT(dialValueChanged(int)));

    // 初始化 SensorsMessageDisplay_1
    SensorsMessageDisplay_1 = new QStringListModel(this);
    // 将模型与视图关联
    ui->SensorsMessageDisplay_1->setModel(SensorsMessageDisplay_1);

    // 初始化 SensorsMessageDisplay_2
    SensorsMessageDisplay_2 = new QStringListModel(this);
    // 将模型与视图关联
    ui->SensorsMessageDisplay_2->setModel(SensorsMessageDisplay_2);

    // 初始化 SensorsMessageDisplay_3
    SensorsMessageDisplay_3 = new QStringListModel(this);
    // 将模型与视图关联
    ui->SensorsMessageDisplay_3->setModel(SensorsMessageDisplay_3);

    // 初始化 SensorsMessageDisplay_4
    SensorsMessageDisplay_4 = new QStringListModel(this);
    // 将模型与视图关联
    ui->SensorsMessageDisplay_4->setModel(SensorsMessageDisplay_4);

    // 初始化 SensorsMessageDisplay_5
    SensorsMessageDisplay_5 = new QStringListModel(this);
    // 将模型与视图关联
    ui->SensorsMessageDisplay_5->setModel(SensorsMessageDisplay_5);

    // 初始化 SensorsMessageDisplay_6
    SensorsMessageDisplay_6 = new QStringListModel(this);
    // 将模型与视图关联
    ui->SensorsMessageDisplay_6->setModel(SensorsMessageDisplay_6);

    // 初始化 SensorsMessageDisplay_7
    SensorsMessageDisplay_7 = new QStringListModel(this);
    // 将模型与视图关联
    ui->SensorsMessageDisplay_7->setModel(SensorsMessageDisplay_7);

    node = std::make_shared<rclcpp::Node>("mainwindow_node");

    // 初始化ROS 2发布器
    control_topic_publisher = node->create_publisher<buaa_rescue_robot_msgs::msg::ControlMessage>("control_topic", 10);
    joint_space_topic_publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("joint_space_topic", 10);
    // 连接Qt信号和槽
    connect(ui->publishButton, SIGNAL(clicked()), this, SLOT(on_publishButton_clicked()));
    connect(ui->transButton, SIGNAL(clicked()), this, SLOT(on_transButton_clicked()));
}



// 重载keyPressEvent方法, control the robot by keyboard
void MainWindow::keyPressEvent(QKeyEvent *event)
{
    auto msg = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessage>();
    // snake motors control for robomaster 1
    msg-> snake_speed_control_1_array[0]  = ui->robomaster1_snake_motor_speed_control_1->value();
    msg-> snake_speed_control_1_array[1]  = ui->robomaster1_snake_motor_speed_control_2->value();
    msg-> snake_speed_control_1_array[2]  = ui->robomaster1_snake_motor_speed_control_3->value();
    msg-> snake_speed_control_1_array[3]  = ui->robomaster1_snake_motor_speed_control_4->value();
    msg-> snake_speed_control_1_array[4]  = ui->robomaster1_snake_motor_speed_control_5->value();
    msg-> snake_speed_control_1_array[5]  = ui->robomaster1_snake_motor_speed_control_6->value();
    msg-> snake_speed_control_1_array[6]  = ui->robomaster1_snake_motor_speed_control_7->value();
    msg-> snake_speed_control_1_array[7]  = ui->robomaster1_snake_motor_speed_control_8->value();
    msg-> snake_speed_control_1_array[8]  = ui->robomaster1_snake_motor_speed_control_9->value();
    msg-> snake_speed_control_1_array[9]  = ui->robomaster1_snake_motor_speed_control_10->value();
    msg-> snake_speed_control_1_array[10] = ui->robomaster1_snake_motor_speed_control_11->value();
    msg-> snake_speed_control_1_array[11] = ui->robomaster1_snake_motor_speed_control_12->value();
    msg-> snake_position_control_1_array[0]  = ui->robomaster1_snake_motor_position_control_1->value();
    msg-> snake_position_control_1_array[1]  = ui->robomaster1_snake_motor_position_control_2->value();
    msg-> snake_position_control_1_array[2]  = ui->robomaster1_snake_motor_position_control_3->value();
    msg-> snake_position_control_1_array[3]  = ui->robomaster1_snake_motor_position_control_4->value();
    msg-> snake_position_control_1_array[4]  = ui->robomaster1_snake_motor_position_control_5->value();
    msg-> snake_position_control_1_array[5]  = ui->robomaster1_snake_motor_position_control_6->value();
    msg-> snake_position_control_1_array[6]  = ui->robomaster1_snake_motor_position_control_7->value();
    msg-> snake_position_control_1_array[7]  = ui->robomaster1_snake_motor_position_control_8->value();
    msg-> snake_position_control_1_array[8]  = ui->robomaster1_snake_motor_position_control_9->value();
    msg-> snake_position_control_1_array[9]  = ui->robomaster1_snake_motor_position_control_10->value();
    msg-> snake_position_control_1_array[10] = ui->robomaster1_snake_motor_position_control_11->value();
    msg-> snake_position_control_1_array[11] = ui->robomaster1_snake_motor_position_control_12->value();
    // gripper control for robomaster 1
    msg->gripper_gm6020_position_1 = ui->gripper_gm6020_position_1_control->value();
    msg->gripper_c610_position_1 = ui->gripper_c610_position_1_control->value();
    msg->gripper_sts3032_position_1 = ui->gripper_sts3032_position_1_control->value();
    msg->robomaster_1_mode = ui->robomaster1_mode->value();
    // snake motors control for robomaster 2
    msg-> snake_speed_control_2_array[0]  = ui->robomaster2_snake_motor_speed_control_1->value();
    msg-> snake_speed_control_2_array[1]  = ui->robomaster2_snake_motor_speed_control_2->value();
    msg-> snake_speed_control_2_array[2]  = ui->robomaster2_snake_motor_speed_control_3->value();
    msg-> snake_speed_control_2_array[3]  = ui->robomaster2_snake_motor_speed_control_4->value();
    msg-> snake_speed_control_2_array[4]  = ui->robomaster2_snake_motor_speed_control_5->value();
    msg-> snake_speed_control_2_array[5]  = ui->robomaster2_snake_motor_speed_control_6->value();
    msg-> snake_speed_control_2_array[6]  = ui->robomaster2_snake_motor_speed_control_7->value();
    msg-> snake_speed_control_2_array[7]  = ui->robomaster2_snake_motor_speed_control_8->value();
    msg-> snake_speed_control_2_array[8]  = ui->robomaster2_snake_motor_speed_control_9->value();
    msg-> snake_speed_control_2_array[9]  = ui->robomaster2_snake_motor_speed_control_10->value();
    msg-> snake_speed_control_2_array[10] = ui->robomaster2_snake_motor_speed_control_11->value();
    msg-> snake_speed_control_2_array[11] = ui->robomaster2_snake_motor_speed_control_12->value();
    msg-> snake_position_control_2_array[0]  = ui->robomaster2_snake_motor_position_control_1->value();
    msg-> snake_position_control_2_array[1]  = ui->robomaster2_snake_motor_position_control_2->value();
    msg-> snake_position_control_2_array[2]  = ui->robomaster2_snake_motor_position_control_3->value();
    msg-> snake_position_control_2_array[3]  = ui->robomaster2_snake_motor_position_control_4->value();
    msg-> snake_position_control_2_array[4]  = ui->robomaster2_snake_motor_position_control_5->value();
    msg-> snake_position_control_2_array[5]  = ui->robomaster2_snake_motor_position_control_6->value();
    msg-> snake_position_control_2_array[6]  = ui->robomaster2_snake_motor_position_control_7->value();
    msg-> snake_position_control_2_array[7]  = ui->robomaster2_snake_motor_position_control_8->value();
    msg-> snake_position_control_2_array[8]  = ui->robomaster2_snake_motor_position_control_9->value();
    msg-> snake_position_control_2_array[9]  = ui->robomaster2_snake_motor_position_control_10->value();
    msg-> snake_position_control_2_array[10] = ui->robomaster2_snake_motor_position_control_11->value();
    msg-> snake_position_control_2_array[11] = ui->robomaster2_snake_motor_position_control_12->value();
    // gripper control for robomaster 1
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

    // 创建消息
    auto theta_msg = std_msgs::msg::Float64MultiArray();
    theta_msg.data = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    QString keyText = event->text();  // 获取按下键的文本表示
    QByteArray byteArray = keyText.toLocal8Bit();  // 转换为 QByteArray
    char *keyChar = byteArray.data();  // 获取字符指针
    // 如果键入的是单个字符，则进行处理
    if (byteArray.size() == 1) {
        // 判断按下的键
        switch(event->key()) {
            
            case '8':
                msg->elevator_control = 1;  // 举例，设定电梯控制为1
                control_topic_publisher->publish(*msg);
                break;
            case '2':
                msg->elevator_control = -1;  // 举例，设定电梯控制为-1
                control_topic_publisher->publish(*msg);
                break;
            case '4':
                msg->lower_linear_module_control = 1; // forward direction
                control_topic_publisher->publish(*msg);
                break;
            case '6':
                msg->lower_linear_module_control = -1; // backward direction
                control_topic_publisher->publish(*msg);
                break;
            case '7':
                msg->upper_linear_module_control = 1; // forward direction
                control_topic_publisher->publish(*msg);
                break;
            case '9':
                msg->upper_linear_module_control = -1; // backward direction
                control_topic_publisher->publish(*msg);
                break;
            case '5': // stop the master devices motors
                reset_flag = false;
                msg->elevator_control = 0;
                msg->lower_linear_module_control = 0;
                msg->upper_linear_module_control = 0;
                control_topic_publisher->publish(*msg);
                break;
             case Qt::Key_C: // Calibration, Mode 2
                msg->robomaster_1_mode = 2; // Calibration
                msg->robomaster_2_mode = 2; // Calibration
                control_topic_publisher->publish(*msg);  // 发布消息
                break;
            case Qt::Key_E: // Calibration, Mode 12
                msg->robomaster_1_mode = 12; // Calibration at zero
                msg->robomaster_2_mode = 12; // Calibration at zero
                control_topic_publisher->publish(*msg);  // 发布消息
                break;
            case Qt::Key_O: // Calibration, Mode 5
                control_topic_publisher->publish(*msg);  // 发布消息
                theta_msg.data[0] = ui->robomaster1_theta_1->value() * M_PI / 180;
                theta_msg.data[1] = ui->robomaster1_theta_2->value() * M_PI / 180;
                theta_msg.data[2] = ui->robomaster1_theta_3->value() * M_PI / 180;
                theta_msg.data[3] = ui->robomaster1_theta_4->value() * M_PI / 180;
                theta_msg.data[4] = ui->robomaster1_theta_5->value() * M_PI / 180;
                theta_msg.data[5] = ui->robomaster1_theta_6->value() * M_PI / 180;
                theta_msg.data[6] = ui->robomaster2_theta_1->value() * M_PI / 180;
                theta_msg.data[7] = ui->robomaster2_theta_2->value() * M_PI / 180;
                theta_msg.data[8] = ui->robomaster2_theta_3->value() * M_PI / 180;
                theta_msg.data[9] = ui->robomaster2_theta_4->value() * M_PI / 180;
                theta_msg.data[10] = ui->robomaster2_theta_5->value() * M_PI / 180;
                theta_msg.data[11] = ui->robomaster2_theta_6->value() * M_PI / 180;
                // 发布消息
                joint_space_topic_publisher->publish(theta_msg);
                // 然后发布新的控制消息
                std::this_thread::sleep_for(std::chrono::milliseconds(10)); // sleep for 10ms
                msg->robomaster_1_mode = 5; 
                msg->robomaster_2_mode = 5;
                control_topic_publisher->publish(*msg);  // 发布消息 
                break;
            case Qt::Key_R: // Release, Mode 0
                for (size_t i = 0; i < 12; i++)
                {
                     msg->snake_position_control_1_array[i] = -200000;
                     msg->snake_position_control_2_array[i] = -200000;
                }                
                msg->robomaster_1_mode = 0; // Release
                msg->robomaster_2_mode = 0; // Release
                control_topic_publisher->publish(*msg);  // 发布消息
                break;
            case Qt::Key_A: // enable, Mode 9
                msg->robomaster_1_mode = 9; // enable
                msg->robomaster_2_mode = 9; // enable
                msg->snake_speed_control_1_array = {10,10,10,10,10,10,10,10,10,10,10,10};
                msg->snake_speed_control_2_array = {10,10,10,10,10,10,10,10,10,10,10,10};
                control_topic_publisher->publish(*msg);  // 发布消息
                break;
            case Qt::Key_Q: // quit, Mode 6
                msg->robomaster_1_mode = 6; // quit
                msg->robomaster_2_mode = 6; // quit
                control_topic_publisher->publish(*msg);  // 发布消息
                break;
            case Qt::Key_Z: // reset the snake sensors, Mode 1
                reset_flag = true;
                msg->snake_position_control_1_array = {0,0,0,0,0,0,0,0,0,0,0,0};
                msg->snake_position_control_2_array = {0,0,0,0,0,0,0,0,0,0,0,0};            

                msg->robomaster_1_mode = 1; // robomaster 1 motor encorders to be 1
                msg->robomaster_2_mode = 1; // robomaster 2 motor encorders to be 1
                control_topic_publisher->publish(*msg);  // 发布消息
                break;
            case Qt::Key_P: // reset the Pull-Push force sensors
                reset_flag = true;           
                msg->pull_push_sensors_reset = 1;
                control_topic_publisher->publish(*msg);  // 发布消息
                break;
            case Qt::Key_Left:
                // 左箭头键被按下
                // TODO: 你的代码
                qDebug() << "Left Arrow Pressed!";
                break;
            case Qt::Key_Right:
                // 右箭头键被按下
                // TODO: 你的代码
                qDebug() << "Right Arrow Pressed!";
                break;
            default:
                QMainWindow::keyPressEvent(event);  // 如果其他键被按下，调用默认处理
        }
    } else {
    // 处理其他按键或忽略
    QMainWindow::keyPressEvent(event);
    }
}

// 重载keyReleaseEvent方法来检测按键松开事件
void MainWindow::keyReleaseEvent(QKeyEvent *event){
    auto msg = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessage>();
    // snake motors control for robomaster 1
    msg-> snake_speed_control_1_array[0]  = ui->robomaster1_snake_motor_speed_control_1->value();
    msg-> snake_speed_control_1_array[1]  = ui->robomaster1_snake_motor_speed_control_2->value();
    msg-> snake_speed_control_1_array[2]  = ui->robomaster1_snake_motor_speed_control_3->value();
    msg-> snake_speed_control_1_array[3]  = ui->robomaster1_snake_motor_speed_control_4->value();
    msg-> snake_speed_control_1_array[4]  = ui->robomaster1_snake_motor_speed_control_5->value();
    msg-> snake_speed_control_1_array[5]  = ui->robomaster1_snake_motor_speed_control_6->value();
    msg-> snake_speed_control_1_array[6]  = ui->robomaster1_snake_motor_speed_control_7->value();
    msg-> snake_speed_control_1_array[7]  = ui->robomaster1_snake_motor_speed_control_8->value();
    msg-> snake_speed_control_1_array[8]  = ui->robomaster1_snake_motor_speed_control_9->value();
    msg-> snake_speed_control_1_array[9]  = ui->robomaster1_snake_motor_speed_control_10->value();
    msg-> snake_speed_control_1_array[10] = ui->robomaster1_snake_motor_speed_control_11->value();
    msg-> snake_speed_control_1_array[11] = ui->robomaster1_snake_motor_speed_control_12->value();
    msg-> snake_position_control_1_array[0]  = ui->robomaster1_snake_motor_position_control_1->value();
    msg-> snake_position_control_1_array[1]  = ui->robomaster1_snake_motor_position_control_2->value();
    msg-> snake_position_control_1_array[2]  = ui->robomaster1_snake_motor_position_control_3->value();
    msg-> snake_position_control_1_array[3]  = ui->robomaster1_snake_motor_position_control_4->value();
    msg-> snake_position_control_1_array[4]  = ui->robomaster1_snake_motor_position_control_5->value();
    msg-> snake_position_control_1_array[5]  = ui->robomaster1_snake_motor_position_control_6->value();
    msg-> snake_position_control_1_array[6]  = ui->robomaster1_snake_motor_position_control_7->value();
    msg-> snake_position_control_1_array[7]  = ui->robomaster1_snake_motor_position_control_8->value();
    msg-> snake_position_control_1_array[8]  = ui->robomaster1_snake_motor_position_control_9->value();
    msg-> snake_position_control_1_array[9]  = ui->robomaster1_snake_motor_position_control_10->value();
    msg-> snake_position_control_1_array[10] = ui->robomaster1_snake_motor_position_control_11->value();
    msg-> snake_position_control_1_array[11] = ui->robomaster1_snake_motor_position_control_12->value();
    // gripper control for robomaster 1
    msg->gripper_gm6020_position_1 = ui->gripper_gm6020_position_1_control->value();
    msg->gripper_c610_position_1 = ui->gripper_c610_position_1_control->value();
    msg->gripper_sts3032_position_1 = ui->gripper_sts3032_position_1_control->value();
    msg->robomaster_1_mode = ui->robomaster1_mode->value();
    // snake motors control for robomaster 2
    msg-> snake_speed_control_2_array[0]  = ui->robomaster2_snake_motor_speed_control_1->value();
    msg-> snake_speed_control_2_array[1]  = ui->robomaster2_snake_motor_speed_control_2->value();
    msg-> snake_speed_control_2_array[2]  = ui->robomaster2_snake_motor_speed_control_3->value();
    msg-> snake_speed_control_2_array[3]  = ui->robomaster2_snake_motor_speed_control_4->value();
    msg-> snake_speed_control_2_array[4]  = ui->robomaster2_snake_motor_speed_control_5->value();
    msg-> snake_speed_control_2_array[5]  = ui->robomaster2_snake_motor_speed_control_6->value();
    msg-> snake_speed_control_2_array[6]  = ui->robomaster2_snake_motor_speed_control_7->value();
    msg-> snake_speed_control_2_array[7]  = ui->robomaster2_snake_motor_speed_control_8->value();
    msg-> snake_speed_control_2_array[8]  = ui->robomaster2_snake_motor_speed_control_9->value();
    msg-> snake_speed_control_2_array[9]  = ui->robomaster2_snake_motor_speed_control_10->value();
    msg-> snake_speed_control_2_array[10] = ui->robomaster2_snake_motor_speed_control_11->value();
    msg-> snake_speed_control_2_array[11] = ui->robomaster2_snake_motor_speed_control_12->value();
    msg-> snake_position_control_2_array[0]  = ui->robomaster2_snake_motor_position_control_1->value();
    msg-> snake_position_control_2_array[1]  = ui->robomaster2_snake_motor_position_control_2->value();
    msg-> snake_position_control_2_array[2]  = ui->robomaster2_snake_motor_position_control_3->value();
    msg-> snake_position_control_2_array[3]  = ui->robomaster2_snake_motor_position_control_4->value();
    msg-> snake_position_control_2_array[4]  = ui->robomaster2_snake_motor_position_control_5->value();
    msg-> snake_position_control_2_array[5]  = ui->robomaster2_snake_motor_position_control_6->value();
    msg-> snake_position_control_2_array[6]  = ui->robomaster2_snake_motor_position_control_7->value();
    msg-> snake_position_control_2_array[7]  = ui->robomaster2_snake_motor_position_control_8->value();
    msg-> snake_position_control_2_array[8]  = ui->robomaster2_snake_motor_position_control_9->value();
    msg-> snake_position_control_2_array[9]  = ui->robomaster2_snake_motor_position_control_10->value();
    msg-> snake_position_control_2_array[10] = ui->robomaster2_snake_motor_position_control_11->value();
    msg-> snake_position_control_2_array[11] = ui->robomaster2_snake_motor_position_control_12->value();
    // gripper control for robomaster 1
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

    // 创建消息
    auto theta_msg = std_msgs::msg::Float64MultiArray();
    theta_msg.data = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    switch (event->key()) {
        case Qt::Key_Z: // reset the snake sensors, Mode be reset back to 0
            reset_flag = false;
            msg->snake_position_control_1_array = {0,0,0,0,0,0,0,0,0,0,0,0};
            msg->snake_position_control_2_array = {0,0,0,0,0,0,0,0,0,0,0,0};

            msg->robomaster_1_mode = 0; 
            msg->robomaster_2_mode = 0; 
            control_topic_publisher->publish(*msg);  // 发布消息
            break;
        case Qt::Key_A: // enable the snake sensors, Mode be reset back to 0
            reset_flag = false;
            msg->robomaster_1_mode = 0; 
            msg->robomaster_2_mode = 0; 
            control_topic_publisher->publish(*msg);  // 发布消息
            break;
        case Qt::Key_O: // enable the snake sensors, Mode be reset back to 0
            reset_flag = false;
            
            theta_msg.data[0] = ui->robomaster1_theta_1->value() * M_PI / 180;
            theta_msg.data[1] = ui->robomaster1_theta_2->value() * M_PI / 180;
            theta_msg.data[2] = ui->robomaster1_theta_3->value() * M_PI / 180;
            theta_msg.data[3] = ui->robomaster1_theta_4->value() * M_PI / 180;
            theta_msg.data[4] = ui->robomaster1_theta_5->value() * M_PI / 180;
            theta_msg.data[5] = ui->robomaster1_theta_6->value() * M_PI / 180;
            theta_msg.data[6] = ui->robomaster2_theta_1->value() * M_PI / 180;
            theta_msg.data[7] = ui->robomaster2_theta_2->value() * M_PI / 180;
            theta_msg.data[8] = ui->robomaster2_theta_3->value() * M_PI / 180;
            theta_msg.data[9] = ui->robomaster2_theta_4->value() * M_PI / 180;
            theta_msg.data[10] = ui->robomaster2_theta_5->value() * M_PI / 180;
            theta_msg.data[11] = ui->robomaster2_theta_6->value() * M_PI / 180;
            // 发布消息
            joint_space_topic_publisher->publish(theta_msg); 
            // 然后发布新的控制消息
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // sleep for 10ms
            msg->robomaster_1_mode = 5; 
            msg->robomaster_2_mode = 5;
            control_topic_publisher->publish(*msg);  // 发布消息
            break;
        case Qt::Key_P: // reset the Pull-Push force sensors, Mode 0
            reset_flag = false;        
            msg->pull_push_sensors_reset = 0;
            control_topic_publisher->publish(*msg);  // 发布消息
            break;
        // ... 其他按键
        default:
        QMainWindow::keyReleaseEvent(event);
    }
}

void MainWindow::updateCameraFrame()
{
    cv::Mat frame;
    cap >> frame;
    if(!frame.empty())
    {
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
        QImage qimg((uchar*)frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
        QPixmap pixmap = QPixmap::fromImage(qimg);
        scene->clear();
        scene->addPixmap(pixmap);
        ui->camera1->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
    }
}

// 自定义槽函数：当 QDial 的值改变时会被调用
void MainWindow::dialValueChanged(int value)
{
    QTime currentTime = QTime::currentTime();
    int elapsed = lastTime.msecsTo(currentTime);

    if (elapsed < 100) {  // 100 毫秒的防抖动时间
        return;
    }

    if (reset_flag == true) {
        return;
    }

    lastTime = currentTime;

    auto msg = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessage>();
    // snake motors control for robomaster 1
    msg-> snake_speed_control_1_array[0]  = ui->robomaster1_snake_motor_speed_control_1->value();
    msg-> snake_speed_control_1_array[1]  = ui->robomaster1_snake_motor_speed_control_2->value();
    msg-> snake_speed_control_1_array[2]  = ui->robomaster1_snake_motor_speed_control_3->value();
    msg-> snake_speed_control_1_array[3]  = ui->robomaster1_snake_motor_speed_control_4->value();
    msg-> snake_speed_control_1_array[4]  = ui->robomaster1_snake_motor_speed_control_5->value();
    msg-> snake_speed_control_1_array[5]  = ui->robomaster1_snake_motor_speed_control_6->value();
    msg-> snake_speed_control_1_array[6]  = ui->robomaster1_snake_motor_speed_control_7->value();
    msg-> snake_speed_control_1_array[7]  = ui->robomaster1_snake_motor_speed_control_8->value();
    msg-> snake_speed_control_1_array[8]  = ui->robomaster1_snake_motor_speed_control_9->value();
    msg-> snake_speed_control_1_array[9]  = ui->robomaster1_snake_motor_speed_control_10->value();
    msg-> snake_speed_control_1_array[10] = ui->robomaster1_snake_motor_speed_control_11->value();
    msg-> snake_speed_control_1_array[11] = ui->robomaster1_snake_motor_speed_control_12->value();
    msg-> snake_position_control_1_array[0]  = ui->robomaster1_snake_motor_position_control_1->value();
    msg-> snake_position_control_1_array[1]  = ui->robomaster1_snake_motor_position_control_2->value();
    msg-> snake_position_control_1_array[2]  = ui->robomaster1_snake_motor_position_control_3->value();
    msg-> snake_position_control_1_array[3]  = ui->robomaster1_snake_motor_position_control_4->value();
    msg-> snake_position_control_1_array[4]  = ui->robomaster1_snake_motor_position_control_5->value();
    msg-> snake_position_control_1_array[5]  = ui->robomaster1_snake_motor_position_control_6->value();
    msg-> snake_position_control_1_array[6]  = ui->robomaster1_snake_motor_position_control_7->value();
    msg-> snake_position_control_1_array[7]  = ui->robomaster1_snake_motor_position_control_8->value();
    msg-> snake_position_control_1_array[8]  = ui->robomaster1_snake_motor_position_control_9->value();
    msg-> snake_position_control_1_array[9]  = ui->robomaster1_snake_motor_position_control_10->value();
    msg-> snake_position_control_1_array[10] = ui->robomaster1_snake_motor_position_control_11->value();
    msg-> snake_position_control_1_array[11] = ui->robomaster1_snake_motor_position_control_12->value();
    // gripper control for robomaster 1
    // msg->gripper_gm6020_position_1 = ui->gripper_gm6020_position_1_control->value();
    // msg->gripper_c610_position_1 = ui->gripper_c610_position_1_control->value();
    // msg->gripper_sts3032_position_1 = ui->gripper_sts3032_position_1_control->value();
    msg->robomaster_1_mode = ui->robomaster1_mode->value();
    // snake motors control for robomaster 2
    msg-> snake_speed_control_2_array[0]  = ui->robomaster2_snake_motor_speed_control_1->value();
    msg-> snake_speed_control_2_array[1]  = ui->robomaster2_snake_motor_speed_control_2->value();
    msg-> snake_speed_control_2_array[2]  = ui->robomaster2_snake_motor_speed_control_3->value();
    msg-> snake_speed_control_2_array[3]  = ui->robomaster2_snake_motor_speed_control_4->value();
    msg-> snake_speed_control_2_array[4]  = ui->robomaster2_snake_motor_speed_control_5->value();
    msg-> snake_speed_control_2_array[5]  = ui->robomaster2_snake_motor_speed_control_6->value();
    msg-> snake_speed_control_2_array[6]  = ui->robomaster2_snake_motor_speed_control_7->value();
    msg-> snake_speed_control_2_array[7]  = ui->robomaster2_snake_motor_speed_control_8->value();
    msg-> snake_speed_control_2_array[8]  = ui->robomaster2_snake_motor_speed_control_9->value();
    msg-> snake_speed_control_2_array[9]  = ui->robomaster2_snake_motor_speed_control_10->value();
    msg-> snake_speed_control_2_array[10] = ui->robomaster2_snake_motor_speed_control_11->value();
    msg-> snake_speed_control_2_array[11] = ui->robomaster2_snake_motor_speed_control_12->value();
    msg-> snake_position_control_2_array[0]  = ui->robomaster2_snake_motor_position_control_1->value();
    msg-> snake_position_control_2_array[1]  = ui->robomaster2_snake_motor_position_control_2->value();
    msg-> snake_position_control_2_array[2]  = ui->robomaster2_snake_motor_position_control_3->value();
    msg-> snake_position_control_2_array[3]  = ui->robomaster2_snake_motor_position_control_4->value();
    msg-> snake_position_control_2_array[4]  = ui->robomaster2_snake_motor_position_control_5->value();
    msg-> snake_position_control_2_array[5]  = ui->robomaster2_snake_motor_position_control_6->value();
    msg-> snake_position_control_2_array[6]  = ui->robomaster2_snake_motor_position_control_7->value();
    msg-> snake_position_control_2_array[7]  = ui->robomaster2_snake_motor_position_control_8->value();
    msg-> snake_position_control_2_array[8]  = ui->robomaster2_snake_motor_position_control_9->value();
    msg-> snake_position_control_2_array[9]  = ui->robomaster2_snake_motor_position_control_10->value();
    msg-> snake_position_control_2_array[10] = ui->robomaster2_snake_motor_position_control_11->value();
    msg-> snake_position_control_2_array[11] = ui->robomaster2_snake_motor_position_control_12->value();
    // gripper control for robomaster 1
    // msg->gripper_gm6020_position_2 = ui->gripper_gm6020_position_2_control->value();
    // msg->gripper_c610_position_2 = ui->gripper_c610_position_2_control->value();
    // msg->gripper_sts3032_position_2 = ui->gripper_sts3032_position_2_control->value();
    msg->robomaster_2_mode = ui->robomaster2_mode->value();
    // master devices control
    msg->elevator_control = ui->elevator_speed_control->value();
    msg->lower_linear_module_control = ui->lower_LM_speed_control->value();
    msg->upper_linear_module_control = ui->upper_LM_speed_control->value();
    msg->pull_push_sensors_reset = ui->PP_sensors_reset->value();
    msg->elevator_counter_reset = ui->elevator_counter_reset->value();
    msg->lower_linear_module_encorder_reset = ui->lower_LM_encorder_reset->value();
    msg->upper_linear_module_encorder_reset = ui->upper_LM_encorder_reset->value();
    // 从 QDial 读取值并设置到 ROS2 消息中
    // gripper control for robomaster 1
    msg->gripper_gm6020_position_1 = ui->gripper1_gm6020->value();
    msg->gripper_c610_position_1 = ui->gripper1_c610->value();
    msg->gripper_sts3032_position_1 = ui->gripper1_sts3032->value();
    // gripper control for robomaster 2
    msg->gripper_gm6020_position_2 = ui->gripper2_gm6020->value();
    msg->gripper_c610_position_2 = ui->gripper2_c610->value();
    msg->gripper_sts3032_position_2 = ui->gripper2_sts3032->value();

    // 发布 ROS2 消息
    control_topic_publisher->publish(*msg);
}

void MainWindow::on_transButton_clicked(){
    // 创建消息
    auto theta_msg = std_msgs::msg::Float64MultiArray();
    theta_msg.data = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    theta_msg.data[0] = ui->robomaster1_theta_1->value() * M_PI / 180;
    theta_msg.data[1] = ui->robomaster1_theta_2->value() * M_PI / 180;
    theta_msg.data[2] = ui->robomaster1_theta_3->value() * M_PI / 180;
    theta_msg.data[3] = ui->robomaster1_theta_4->value() * M_PI / 180;
    theta_msg.data[4] = ui->robomaster1_theta_5->value() * M_PI / 180;
    theta_msg.data[5] = ui->robomaster1_theta_6->value() * M_PI / 180;
    theta_msg.data[6] = ui->robomaster2_theta_1->value() * M_PI / 180;
    theta_msg.data[7] = ui->robomaster2_theta_2->value() * M_PI / 180;
    theta_msg.data[8] = ui->robomaster2_theta_3->value() * M_PI / 180;
    theta_msg.data[9] = ui->robomaster2_theta_4->value() * M_PI / 180;
    theta_msg.data[10] = ui->robomaster2_theta_5->value() * M_PI / 180;
    theta_msg.data[11] = ui->robomaster2_theta_6->value() * M_PI / 180;
    // 发布消息
    joint_space_topic_publisher->publish(theta_msg);

    auto msg = buaa_rescue_robot_msgs::msg::ControlMessage();
    msg.snake_speed_control_1_array = {0,0,0,0,0,0,0,0,0,0,0,0};
    msg.snake_position_control_1_array = {0,0,0,0,0,0,0,0,0,0,0,0};
    msg.gripper_gm6020_position_1 = 0;
    msg.gripper_c610_position_1 = 0;
    msg.gripper_sts3032_position_1 = 0;

    msg.snake_speed_control_2_array = {0,0,0,0,0,0,0,0,0,0,0,0};
    msg.snake_position_control_2_array = {0,0,0,0,0,0,0,0,0,0,0,0};
    msg.gripper_gm6020_position_2 = 0;
    msg.gripper_c610_position_2 = 0;
    msg.gripper_sts3032_position_2 = 0;

    msg.elevator_control = 0;
    msg.lower_linear_module_control = 0;
    msg.upper_linear_module_control = 0;


    msg.pull_push_sensors_reset = 0;
    msg.elevator_counter_reset = 0; // reset to be 0
    msg.lower_linear_module_encorder_reset = 0; // reset to be 0
    msg.upper_linear_module_encorder_reset = 0; // reset to be 0
    msg.robomaster_1_mode = 5; // mode 5, control by omega7
    msg.robomaster_2_mode = 5; // mode 5, control by omega7
    // 然后发布新的控制消息
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // sleep for 10ms
    control_topic_publisher->publish(msg);

}

void MainWindow::on_publishButton_clicked()
{
    auto msg = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessage>();
    for (size_t i = 0; i < 12; i++)
    {
        msg->snake_position_control_1_array[i] = 10;
        msg->snake_position_control_2_array[i] = 10;
    }    
    // snake motors control for robomaster 1
    msg-> snake_speed_control_1_array[0]  = ui->robomaster1_snake_motor_speed_control_1->value();
    msg-> snake_speed_control_1_array[1]  = ui->robomaster1_snake_motor_speed_control_2->value();
    msg-> snake_speed_control_1_array[2]  = ui->robomaster1_snake_motor_speed_control_3->value();
    msg-> snake_speed_control_1_array[3]  = ui->robomaster1_snake_motor_speed_control_4->value();
    msg-> snake_speed_control_1_array[4]  = ui->robomaster1_snake_motor_speed_control_5->value();
    msg-> snake_speed_control_1_array[5]  = ui->robomaster1_snake_motor_speed_control_6->value();
    msg-> snake_speed_control_1_array[6]  = ui->robomaster1_snake_motor_speed_control_7->value();
    msg-> snake_speed_control_1_array[7]  = ui->robomaster1_snake_motor_speed_control_8->value();
    msg-> snake_speed_control_1_array[8]  = ui->robomaster1_snake_motor_speed_control_9->value();
    msg-> snake_speed_control_1_array[9]  = ui->robomaster1_snake_motor_speed_control_10->value();
    msg-> snake_speed_control_1_array[10] = ui->robomaster1_snake_motor_speed_control_11->value();
    msg-> snake_speed_control_1_array[11] = ui->robomaster1_snake_motor_speed_control_12->value();
    msg-> snake_position_control_1_array[0]  = ui->robomaster1_snake_motor_position_control_1->value();
    msg-> snake_position_control_1_array[1]  = ui->robomaster1_snake_motor_position_control_2->value();
    msg-> snake_position_control_1_array[2]  = ui->robomaster1_snake_motor_position_control_3->value();
    msg-> snake_position_control_1_array[3]  = ui->robomaster1_snake_motor_position_control_4->value();
    msg-> snake_position_control_1_array[4]  = ui->robomaster1_snake_motor_position_control_5->value();
    msg-> snake_position_control_1_array[5]  = ui->robomaster1_snake_motor_position_control_6->value();
    msg-> snake_position_control_1_array[6]  = ui->robomaster1_snake_motor_position_control_7->value();
    msg-> snake_position_control_1_array[7]  = ui->robomaster1_snake_motor_position_control_8->value();
    msg-> snake_position_control_1_array[8]  = ui->robomaster1_snake_motor_position_control_9->value();
    msg-> snake_position_control_1_array[9]  = ui->robomaster1_snake_motor_position_control_10->value();
    msg-> snake_position_control_1_array[10] = ui->robomaster1_snake_motor_position_control_11->value();
    msg-> snake_position_control_1_array[11] = ui->robomaster1_snake_motor_position_control_12->value();
    // gripper control for robomaster 1
    msg->gripper_gm6020_position_1 = ui->gripper_gm6020_position_1_control->value();
    msg->gripper_c610_position_1 = ui->gripper_c610_position_1_control->value();
    msg->gripper_sts3032_position_1 = ui->gripper_sts3032_position_1_control->value();
    msg->robomaster_1_mode = ui->robomaster1_mode->value();
    // snake motors control for robomaster 2
    msg-> snake_speed_control_2_array[0]  = ui->robomaster2_snake_motor_speed_control_1->value();
    msg-> snake_speed_control_2_array[1]  = ui->robomaster2_snake_motor_speed_control_2->value();
    msg-> snake_speed_control_2_array[2]  = ui->robomaster2_snake_motor_speed_control_3->value();
    msg-> snake_speed_control_2_array[3]  = ui->robomaster2_snake_motor_speed_control_4->value();
    msg-> snake_speed_control_2_array[4]  = ui->robomaster2_snake_motor_speed_control_5->value();
    msg-> snake_speed_control_2_array[5]  = ui->robomaster2_snake_motor_speed_control_6->value();
    msg-> snake_speed_control_2_array[6]  = ui->robomaster2_snake_motor_speed_control_7->value();
    msg-> snake_speed_control_2_array[7]  = ui->robomaster2_snake_motor_speed_control_8->value();
    msg-> snake_speed_control_2_array[8]  = ui->robomaster2_snake_motor_speed_control_9->value();
    msg-> snake_speed_control_2_array[9]  = ui->robomaster2_snake_motor_speed_control_10->value();
    msg-> snake_speed_control_2_array[10] = ui->robomaster2_snake_motor_speed_control_11->value();
    msg-> snake_speed_control_2_array[11] = ui->robomaster2_snake_motor_speed_control_12->value();
    msg-> snake_position_control_2_array[0]  = ui->robomaster2_snake_motor_position_control_1->value();
    msg-> snake_position_control_2_array[1]  = ui->robomaster2_snake_motor_position_control_2->value();
    msg-> snake_position_control_2_array[2]  = ui->robomaster2_snake_motor_position_control_3->value();
    msg-> snake_position_control_2_array[3]  = ui->robomaster2_snake_motor_position_control_4->value();
    msg-> snake_position_control_2_array[4]  = ui->robomaster2_snake_motor_position_control_5->value();
    msg-> snake_position_control_2_array[5]  = ui->robomaster2_snake_motor_position_control_6->value();
    msg-> snake_position_control_2_array[6]  = ui->robomaster2_snake_motor_position_control_7->value();
    msg-> snake_position_control_2_array[7]  = ui->robomaster2_snake_motor_position_control_8->value();
    msg-> snake_position_control_2_array[8]  = ui->robomaster2_snake_motor_position_control_9->value();
    msg-> snake_position_control_2_array[9]  = ui->robomaster2_snake_motor_position_control_10->value();
    msg-> snake_position_control_2_array[10] = ui->robomaster2_snake_motor_position_control_11->value();
    msg-> snake_position_control_2_array[11] = ui->robomaster2_snake_motor_position_control_12->value();
    // gripper control for robomaster 1
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

MainWindow::~MainWindow()
{
    delete ui;
}

