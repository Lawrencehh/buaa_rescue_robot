#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <opencv2/opencv.hpp>
#include <QKeyEvent>  // 引入QKeyEvent头文件
// #include <thread>
// #include <chrono>
#include <QTime>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

QTime lastTime = QTime::currentTime(); // 防抖动
bool reset_flag = false;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , cameraThreadRunning(false) // 初始化为 false
{
    ui->setupUi(this);
    // 初始化三个摄像头
    initializeCameras();
    // 为每个摄像头创建场景
    scene1 = new QGraphicsScene(this);
    scene2 = new QGraphicsScene(this);
    scene3 = new QGraphicsScene(this);
    // 将场景关联到UI控件
    ui->camera1->setScene(scene1);
    ui->camera2->setScene(scene2);
    ui->camera3->setScene(scene3);
    // 连接新的槽函数
    connect(this, &MainWindow::signalUpdateGraphicsView, this, &MainWindow::updateGraphicsView, Qt::QueuedConnection);
    startCameraThreads(); // 启动摄像头线程

    // 手部电机的控制旋钮相关
    // 连接 QSlider 的 valueChanged 信号到自定义的槽函数
    connect(ui->gripper1_gm6020, SIGNAL(valueChanged(int)), this, SLOT(sliderValueChanged(int)));
    connect(ui->gripper1_c610, SIGNAL(valueChanged(int)), this, SLOT(sliderValueChanged(int)));
    connect(ui->gripper1_sts3032, SIGNAL(valueChanged(int)), this, SLOT(sliderValueChanged(int)));
    connect(ui->gripper2_gm6020, SIGNAL(valueChanged(int)), this, SLOT(sliderValueChanged(int)));
    connect(ui->gripper2_c610, SIGNAL(valueChanged(int)), this, SLOT(sliderValueChanged(int)));
    connect(ui->gripper2_sts3032, SIGNAL(valueChanged(int)), this, SLOT(sliderValueChanged(int)));
    // 初始化 SensorsMessageDisplay_X，将模型与视图关联
    SensorsMessageDisplay_1 = new QStringListModel(this);    ui->SensorsMessageDisplay_1->setModel(SensorsMessageDisplay_1);
    SensorsMessageDisplay_2 = new QStringListModel(this);    ui->SensorsMessageDisplay_2->setModel(SensorsMessageDisplay_2);
    SensorsMessageDisplay_3 = new QStringListModel(this);    ui->SensorsMessageDisplay_3->setModel(SensorsMessageDisplay_3);
    SensorsMessageDisplay_4 = new QStringListModel(this);    ui->SensorsMessageDisplay_4->setModel(SensorsMessageDisplay_4);
    SensorsMessageDisplay_5 = new QStringListModel(this);    ui->SensorsMessageDisplay_5->setModel(SensorsMessageDisplay_5);
    SensorsMessageDisplay_6 = new QStringListModel(this);    ui->SensorsMessageDisplay_6->setModel(SensorsMessageDisplay_6);
    SensorsMessageDisplay_7 = new QStringListModel(this);    ui->SensorsMessageDisplay_7->setModel(SensorsMessageDisplay_7);
    // 初始化ROS 2发布器
    node = std::make_shared<rclcpp::Node>("mainwindow_node");
    slave_control_topic_publisher_1 = node->create_publisher<buaa_rescue_robot_msgs::msg::ControlMessageSlave>("slave_control_topic_1", 10);
    slave_control_topic_publisher_2 = node->create_publisher<buaa_rescue_robot_msgs::msg::ControlMessageSlave>("slave_control_topic_2", 10);
    gripper_control_topic_publisher_1 = node->create_publisher<buaa_rescue_robot_msgs::msg::ControlMessageSlaveGripper>("gripper_control_topic_1", 10);
    gripper_control_topic_publisher_2 = node->create_publisher<buaa_rescue_robot_msgs::msg::ControlMessageSlaveGripper>("gripper_control_topic_2", 10);
    master_control_topic_publisher = node->create_publisher<buaa_rescue_robot_msgs::msg::ControlMessageMaster>("master_control_topic", 10);
    joint_space_topic_publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("joint_space_topic", 10);
    // 连接Qt信号和槽
    connect(ui->publishButton, SIGNAL(clicked()), this, SLOT(on_publishButton_clicked()));
    connect(ui->transButton, SIGNAL(clicked()), this, SLOT(on_transButton_clicked()));
    // 初始化控件的指针数组
    thetaControls = {
        ui->robomaster1_theta_1, ui->robomaster1_theta_2, ui->robomaster1_theta_3,
        ui->robomaster1_theta_4, ui->robomaster1_theta_5, ui->robomaster1_theta_6,
        ui->robomaster2_theta_1, ui->robomaster2_theta_2, ui->robomaster2_theta_3,
        ui->robomaster2_theta_4, ui->robomaster2_theta_5, ui->robomaster2_theta_6
    };
    speedControls_robomaster1 = {
        ui->robomaster1_snake_motor_speed_control_1, ui->robomaster1_snake_motor_speed_control_2, ui->robomaster1_snake_motor_speed_control_3,
        ui->robomaster1_snake_motor_speed_control_4, ui->robomaster1_snake_motor_speed_control_5, ui->robomaster1_snake_motor_speed_control_6,
        ui->robomaster1_snake_motor_speed_control_7, ui->robomaster1_snake_motor_speed_control_8, ui->robomaster1_snake_motor_speed_control_9,
        ui->robomaster1_snake_motor_speed_control_10, ui->robomaster1_snake_motor_speed_control_11, ui->robomaster1_snake_motor_speed_control_12
    };
    positionControls_robomaster1 = {
        ui->robomaster1_snake_motor_position_control_1, ui->robomaster1_snake_motor_position_control_2, ui->robomaster1_snake_motor_position_control_3,
        ui->robomaster1_snake_motor_position_control_4, ui->robomaster1_snake_motor_position_control_5, ui->robomaster1_snake_motor_position_control_6,
        ui->robomaster1_snake_motor_position_control_7, ui->robomaster1_snake_motor_position_control_8, ui->robomaster1_snake_motor_position_control_9,
        ui->robomaster1_snake_motor_position_control_10, ui->robomaster1_snake_motor_position_control_11, ui->robomaster1_snake_motor_position_control_12
    };
    speedControls_robomaster2 = {
        ui->robomaster2_snake_motor_speed_control_1, ui->robomaster2_snake_motor_speed_control_2, ui->robomaster2_snake_motor_speed_control_3,
        ui->robomaster2_snake_motor_speed_control_4, ui->robomaster2_snake_motor_speed_control_5, ui->robomaster2_snake_motor_speed_control_6,
        ui->robomaster2_snake_motor_speed_control_7, ui->robomaster2_snake_motor_speed_control_8, ui->robomaster2_snake_motor_speed_control_9,
        ui->robomaster2_snake_motor_speed_control_10, ui->robomaster2_snake_motor_speed_control_11, ui->robomaster2_snake_motor_speed_control_12
    };
    positionControls_robomaster2 = {
        ui->robomaster2_snake_motor_position_control_1, ui->robomaster2_snake_motor_position_control_2, ui->robomaster2_snake_motor_position_control_3,
        ui->robomaster2_snake_motor_position_control_4, ui->robomaster2_snake_motor_position_control_5, ui->robomaster2_snake_motor_position_control_6,
        ui->robomaster2_snake_motor_position_control_7, ui->robomaster2_snake_motor_position_control_8, ui->robomaster2_snake_motor_position_control_9,
        ui->robomaster2_snake_motor_position_control_10, ui->robomaster2_snake_motor_position_control_11, ui->robomaster2_snake_motor_position_control_12
    };
}

/******************************************摄像头相关函数***********************************************/
void MainWindow::initializeCamera(int cameraIndex, cv::VideoCapture &camera, const char *cameraName) 
{
    if (!camera.open(cameraIndex)) {
        qDebug() << "无法打开摄像头" << cameraName;
    }
}

void MainWindow::initializeCameras() {
    initializeCamera(3, cap1, "/dev/my_camera1");
    initializeCamera(0, cap2, "/dev/my_camera2");
    initializeCamera(4, cap3, "/dev/my_camera3");
}

void MainWindow::startCameraThreads() {
    cameraThreadRunning = true;
    cameraThread1 = std::thread(&MainWindow::updateCameraFrame, this, std::ref(cap1), scene1, ui->camera1);
    cameraThread2 = std::thread(&MainWindow::updateCameraFrame, this, std::ref(cap2), scene2, ui->camera2);
    cameraThread3 = std::thread(&MainWindow::updateCameraFrame, this, std::ref(cap3), scene3, ui->camera3);
}

void MainWindow::stopCameraThreads() {
    cameraThreadRunning = false;
    if (cameraThread1.joinable()) cameraThread1.join();
    if (cameraThread2.joinable()) cameraThread2.join();
    if (cameraThread3.joinable()) cameraThread3.join();
    // 正确调用 release 方法
    cap1.release();
    cap2.release();
    cap3.release();
}

void MainWindow::updateCameraFrame(cv::VideoCapture &cap, QGraphicsScene *scene, QGraphicsView *view) {
    while (cameraThreadRunning) {
        cv::Mat frame;
        if (!cap.isOpened() || !cap.read(frame)) {
            qDebug() << "摄像头读取失败或未打开";
            break; // 或其他错误处理
        }
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
        QImage qimg((uchar*)frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
        QPixmap pixmap = QPixmap::fromImage(qimg);

        // 使用信号发送 pixmap 给 GUI 线程
        emit signalUpdateGraphicsView(scene, pixmap, view);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 控制帧率
    }
}

void MainWindow::updateGraphicsView(QGraphicsScene* scene, QPixmap pixmap) {
    scene->clear();
    scene->addPixmap(pixmap);
    ui->camera1->fitInView(scene->sceneRect(), Qt::KeepAspectRatio); // 根据需要选择相应的视图
    ui->camera2->fitInView(scene->sceneRect(), Qt::KeepAspectRatio); // 根据需要选择相应的视图
    ui->camera3->fitInView(scene->sceneRect(), Qt::KeepAspectRatio); // 根据需要选择相应的视图
}
/******************************************摄像头相关函数***********************************************/

/*****************************************************传感反馈显示控件******************************************************/
void MainWindow::updateSensorsMessageDisplay_1(const buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster::SharedPtr msg) {
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

void MainWindow::updateSensorsMessageDisplay_2(const buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster::SharedPtr msg) {
    QStringList list;
    for (int i = 0; i < 12; ++i) {
        list << QString::number(msg->snake_motor_encorder_position[i]);
    }
    list << QString::number(msg->gripper_gm6020_position);
    list << QString::number(msg->gripper_c610_position);
    list << QString::number(msg->gripper_sts3032_position); 
    list << QString::number(msg->robomaster_mode);
    SensorsMessageDisplay_2->setStringList(list);
}

void MainWindow::updateSensorsMessageDisplay_3(const buaa_rescue_robot_msgs::msg::SensorsMessageMasterDeviceElevator::SharedPtr msg) {
    QStringList list;
    list << QString::number(msg->elevator_counter);
    list << QString::number(msg->lower_encorder, 'f', 2);
    list << QString::number(msg->upper_encorder, 'f', 2);
    SensorsMessageDisplay_3->setStringList(list);
}

void MainWindow::updateSensorsMessageDisplay_4(const buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors::SharedPtr msg) {
    QStringList list;

    for (int i = 0; i < 12; ++i) {
        list << QString::number(msg->pull_push_sensors[i]);
    }    
    SensorsMessageDisplay_4->setStringList(list);
}

void MainWindow::updateSensorsMessageDisplay_5(const buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors::SharedPtr msg) {
    QStringList list;

    for (int i = 0; i < 12; ++i) {
        list << QString::number(msg->pull_push_sensors[i]);
    }    
    SensorsMessageDisplay_5->setStringList(list);
}

void MainWindow::updateSensorsMessageDisplay_6(const buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster::SharedPtr msg) {
    QStringList list;

    for (int i = 0; i < 12; ++i) {
        list << QString::number(msg->snake_motor_encorder_speed[i]);
    }    
    SensorsMessageDisplay_6->setStringList(list);
}

void MainWindow::updateSensorsMessageDisplay_7(const buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster::SharedPtr msg) {
    QStringList list;

    for (int i = 0; i < 12; ++i) {
        list << QString::number(msg->snake_motor_encorder_speed[i]);
    }    
    SensorsMessageDisplay_7->setStringList(list);
}
/************************************************************************************************************************/

/*****************************************************所有控制指令控件******************************************************/
void MainWindow::updateSlaveControlIndicator1(const buaa_rescue_robot_msgs::msg::ControlMessageSlave::SharedPtr msg) {        
    for (size_t i = 0; i < 12; i++)
    {
        speedControls_robomaster1[i]->setValue(msg->snake_speed_control_array[i]);
        positionControls_robomaster1[i]->setValue(msg->snake_position_control_array[i]);
    }        
    ui->robomaster1_mode->setValue(msg->robomaster_mode);
}

void MainWindow::updateSlaveGripperControlIndicator1(const buaa_rescue_robot_msgs::msg::ControlMessageSlaveGripper::SharedPtr msg) {             
    ui->gripper_gm6020_position_1_control->setValue(msg->gripper_gm6020_position);
    ui->gripper_c610_position_1_control->setValue(msg->gripper_c610_position);
    ui->gripper_sts3032_position_1_control->setValue(msg->gripper_sts3032_position);
    ui->gripper1_gm6020->setValue(msg->gripper_gm6020_position);
    ui->gripper1_c610->setValue(msg->gripper_c610_position);
    ui->gripper1_sts3032->setValue(msg->gripper_sts3032_position);
}
    
void MainWindow::updateSlaveControlIndicator2(const buaa_rescue_robot_msgs::msg::ControlMessageSlave::SharedPtr msg) {       
    for (size_t i = 0; i < 12; i++)
    {
        speedControls_robomaster2[i]->setValue(msg->snake_speed_control_array[i]);
        positionControls_robomaster2[i]->setValue(msg->snake_position_control_array[i]);
    }    
    ui->robomaster2_mode->setValue(msg->robomaster_mode);
}

void MainWindow::updateSlaveGripperControlIndicator2(const buaa_rescue_robot_msgs::msg::ControlMessageSlaveGripper::SharedPtr msg) {       
    ui->gripper_gm6020_position_2_control->setValue(msg->gripper_gm6020_position);
    ui->gripper_c610_position_2_control->setValue(msg->gripper_c610_position);
    ui->gripper_sts3032_position_2_control->setValue(msg->gripper_sts3032_position);
    ui->gripper2_gm6020->setValue(msg->gripper_gm6020_position);
    ui->gripper2_c610->setValue(msg->gripper_c610_position);
    ui->gripper2_sts3032->setValue(msg->gripper_sts3032_position);
}

void MainWindow::updateMasterControlIndicator(const buaa_rescue_robot_msgs::msg::ControlMessageMaster::SharedPtr msg) {
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

void MainWindow::updateJointSpaceIndicator(const std_msgs::msg::Float64MultiArray::SharedPtr theta_msg) {
    for (size_t i = 0; i < 12; i++)
    {
        thetaControls[i]->setValue(theta_msg->data[i] * 180 / M_PI);
    }
    
}
/************************************************************************************************************************/


// 重载keyPressEvent方法, control the robot by keyboard
void MainWindow::keyPressEvent(QKeyEvent *event)
{
    auto slave_msg_1 = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessageSlave>();
    auto slave_msg_2 = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessageSlave>();
    // snake motors control for robomaster 1 & 2
    for (size_t i = 0; i < 12; i++)
    {
        slave_msg_1-> snake_speed_control_array[i]  = speedControls_robomaster1[i]->value();
        slave_msg_1-> snake_position_control_array[i]  = positionControls_robomaster1[i]->value();
        slave_msg_2-> snake_speed_control_array[i]  = speedControls_robomaster2[i]->value();
        slave_msg_2-> snake_position_control_array[i]  = positionControls_robomaster2[i]->value();
    }
    // master devices control
    auto master_msg = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessageMaster>();
    master_msg->elevator_control = ui->elevator_speed_control->value();
    master_msg->lower_linear_module_control = ui->lower_LM_speed_control->value();
    master_msg->upper_linear_module_control = ui->upper_LM_speed_control->value();
    master_msg->pull_push_sensors_reset = ui->PP_sensors_reset->value();
    master_msg->elevator_counter_reset = ui->elevator_counter_reset->value();
    master_msg->lower_linear_module_encorder_reset = ui->lower_LM_encorder_reset->value();
    master_msg->upper_linear_module_encorder_reset = ui->upper_LM_encorder_reset->value();
    // 创建theta消息
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
                master_msg->elevator_control = 1;  // 举例，设定电梯控制为1
                master_control_topic_publisher->publish(*master_msg);
                break;
            case '2':
                master_msg->elevator_control = -1;  // 举例，设定电梯控制为-1
                master_control_topic_publisher->publish(*master_msg);
                break;
            case '4':
                master_msg->lower_linear_module_control = 1; // forward direction
                master_control_topic_publisher->publish(*master_msg);
                break;
            case '6':
                master_msg->lower_linear_module_control = -1; // backward direction
                master_control_topic_publisher->publish(*master_msg);
                break;
            case '7':
                master_msg->upper_linear_module_control = 1; // forward direction
                master_control_topic_publisher->publish(*master_msg);
                break;
            case '9':
                master_msg->upper_linear_module_control = -1; // backward direction
                master_control_topic_publisher->publish(*master_msg);
                break;
            case '5': // stop the master devices motors
                reset_flag = false;
                master_msg->elevator_control = 0;
                master_msg->lower_linear_module_control = 0;
                master_msg->upper_linear_module_control = 0;
                master_control_topic_publisher->publish(*master_msg);
                break;
            case '0': // stop the master devices motors
                reset_flag = true;
                master_msg->elevator_control = 0;
                master_msg->lower_linear_module_control = 0;
                master_msg->upper_linear_module_control = 0;
                master_msg->elevator_counter_reset = 1;
                master_msg->lower_linear_module_encorder_reset = 1;
                master_msg->upper_linear_module_encorder_reset = 1;
                master_control_topic_publisher->publish(*master_msg);
                break;
             case Qt::Key_C: // Calibration, Mode 2
                slave_msg_1->robomaster_mode = 2; // Calibration
                slave_msg_2->robomaster_mode = 2; // Calibration
                slave_control_topic_publisher_1->publish(*slave_msg_1);  // 发布消息
                slave_control_topic_publisher_2->publish(*slave_msg_2);  // 发布消息
                break;
            case Qt::Key_E: // Calibration, Mode 12
                slave_msg_1->robomaster_mode = 12; // Calibration at zero
                slave_msg_2->robomaster_mode = 12; // Calibration at zero
                slave_control_topic_publisher_1->publish(*slave_msg_1);  // 发布消息
                slave_control_topic_publisher_2->publish(*slave_msg_2);  // 发布消息
                break;
            case Qt::Key_O: // Calibration, Mode 5
                for (size_t i = 0; i < thetaControls.size(); ++i) {
                    theta_msg.data[i] = thetaControls[i]->value() * M_PI / 180;
                }
                joint_space_topic_publisher->publish(theta_msg);
                std::this_thread::sleep_for(std::chrono::milliseconds(10)); // sleep for 10ms
                slave_msg_1->robomaster_mode = 5; 
                slave_msg_2->robomaster_mode = 5;
                slave_control_topic_publisher_1->publish(*slave_msg_1);  // 发布消息
                slave_control_topic_publisher_2->publish(*slave_msg_2);  // 发布消息
                break;
            case Qt::Key_R: // Release, Mode 3
                for (size_t i = 0; i < 12; i++)
                {
                    slave_msg_1->snake_position_control_array[i] = -200000;
                    slave_msg_2->snake_position_control_array[i] = -200000;
                }                
                slave_msg_1->robomaster_mode = 3; // Release
                slave_msg_2->robomaster_mode = 3; // Release
                slave_control_topic_publisher_1->publish(*slave_msg_1);  // 发布消息
                slave_control_topic_publisher_2->publish(*slave_msg_2);  // 发布消息
                break;
            case Qt::Key_A: // enable, Mode 9
                slave_msg_1->robomaster_mode = 9; // enable
                slave_msg_2->robomaster_mode = 9; // enable
                slave_msg_1->snake_speed_control_array = {10,10,10,10,10,10,10,10,10,10,10,10};
                slave_msg_2->snake_speed_control_array = {10,10,10,10,10,10,10,10,10,10,10,10};
                slave_control_topic_publisher_1->publish(*slave_msg_1);  // 发布消息
                slave_control_topic_publisher_2->publish(*slave_msg_2);  // 发布消息
                break;
            case Qt::Key_Q: // quit, Mode 6
                slave_msg_1->robomaster_mode = 6; // quit
                slave_msg_2->robomaster_mode = 6; // quit
                slave_control_topic_publisher_1->publish(*slave_msg_1);  // 发布消息
                slave_control_topic_publisher_2->publish(*slave_msg_2);  // 发布消息
                break;
            case Qt::Key_Z: // reset the snake sensors, Mode 1
                reset_flag = true;
                slave_msg_1->snake_position_control_array = {0,0,0,0,0,0,0,0,0,0,0,0};
                slave_msg_2->snake_position_control_array = {0,0,0,0,0,0,0,0,0,0,0,0};            
                slave_msg_1->robomaster_mode = 1; // robomaster 1 motor encorders to be 1
                slave_msg_2->robomaster_mode = 1; // robomaster 2 motor encorders to be 1
                slave_control_topic_publisher_1->publish(*slave_msg_1);  // 发布消息
                slave_control_topic_publisher_2->publish(*slave_msg_2);  // 发布消息
                break;
            case Qt::Key_P: // reset the Pull-Push force sensors
                reset_flag = true;           
                master_msg->pull_push_sensors_reset = 1;
                master_control_topic_publisher->publish(*master_msg);  // 发布消息
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
    auto slave_msg_1 = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessageSlave>();
    auto slave_msg_2 = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessageSlave>();
    // snake motors control for robomaster 1 & 2
    for (size_t i = 0; i < 12; i++)
    {
        slave_msg_1-> snake_speed_control_array[i]  = speedControls_robomaster1[i]->value();
        slave_msg_1-> snake_position_control_array[i]  = positionControls_robomaster1[i]->value();
        slave_msg_2-> snake_speed_control_array[i]  = speedControls_robomaster2[i]->value();
        slave_msg_2-> snake_position_control_array[i]  = positionControls_robomaster2[i]->value();
    }
    // master devices control
    auto master_msg = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessageMaster>();
    master_msg->elevator_control = ui->elevator_speed_control->value();
    master_msg->lower_linear_module_control = ui->lower_LM_speed_control->value();
    master_msg->upper_linear_module_control = ui->upper_LM_speed_control->value();
    master_msg->pull_push_sensors_reset = ui->PP_sensors_reset->value();
    master_msg->elevator_counter_reset = ui->elevator_counter_reset->value();
    master_msg->lower_linear_module_encorder_reset = ui->lower_LM_encorder_reset->value();
    master_msg->upper_linear_module_encorder_reset = ui->upper_LM_encorder_reset->value();
    // 创建消息
    auto theta_msg = std_msgs::msg::Float64MultiArray();
    theta_msg.data = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    switch (event->key()) {
        case Qt::Key_0: // stop the master devices motors
                reset_flag = false;
                master_msg->elevator_control = 0;
                master_msg->lower_linear_module_control = 0;
                master_msg->upper_linear_module_control = 0;
                master_msg->elevator_counter_reset = 0;
                master_msg->lower_linear_module_encorder_reset = 0;
                master_msg->upper_linear_module_encorder_reset = 0;
                master_control_topic_publisher->publish(*master_msg);
                break;
        case Qt::Key_Z: // reset the snake sensors, Mode be reset back to 0
            reset_flag = false;
            slave_msg_1->snake_position_control_array = {0,0,0,0,0,0,0,0,0,0,0,0};
            slave_msg_2->snake_position_control_array = {0,0,0,0,0,0,0,0,0,0,0,0};

            slave_msg_1->robomaster_mode = 0; 
            slave_msg_2->robomaster_mode = 0; 
            slave_control_topic_publisher_1->publish(*slave_msg_1);  // 发布消息
            slave_control_topic_publisher_2->publish(*slave_msg_2);  // 发布消息
            break;
        case Qt::Key_A: // enable the snake sensors, Mode be reset back to 0
            reset_flag = false;
            slave_msg_1->robomaster_mode = 0; 
            slave_msg_2->robomaster_mode = 0; 
            slave_control_topic_publisher_1->publish(*slave_msg_1);  // 发布消息
            slave_control_topic_publisher_2->publish(*slave_msg_2);  // 发布消息
            break;
        case Qt::Key_O: // mode 5
            reset_flag = false;
            for (size_t i = 0; i < 12; ++i) {
                theta_msg.data[i] = thetaControls[i]->value() * M_PI / 180;
            }          
            // 发布消息
            joint_space_topic_publisher->publish(theta_msg); 
            // 然后发布新的控制消息
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // sleep for 10ms
            slave_msg_1->robomaster_mode = 5; 
            slave_msg_2->robomaster_mode = 5;
            slave_control_topic_publisher_1->publish(*slave_msg_1);  // 发布消息
            slave_control_topic_publisher_2->publish(*slave_msg_2);  // 发布消息
            break;
        case Qt::Key_P: // reset the Pull-Push force sensors, Mode 0
            reset_flag = false;        
            master_msg->pull_push_sensors_reset = 0;
            master_control_topic_publisher->publish(*master_msg);  // 发布消息
            break;
        // ... 其他按键
        default:
        QMainWindow::keyReleaseEvent(event);
    }
}

// 自定义槽函数：当 QSlider 的值改变时会被调用
void MainWindow::sliderValueChanged(int value)
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

    auto gripper_msg_1 = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessageSlaveGripper>();
    auto gripper_msg_2 = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessageSlaveGripper>();
    // 从 QSlider 读取值并设置到 ROS2 消息中
    // gripper control for robomaster 1
    gripper_msg_1->gripper_gm6020_position = ui->gripper1_gm6020->value();
    gripper_msg_1->gripper_c610_position = ui->gripper1_c610->value();
    gripper_msg_1->gripper_sts3032_position = ui->gripper1_sts3032->value();
    // gripper control for robomaster 2
    gripper_msg_2->gripper_gm6020_position = ui->gripper2_gm6020->value();
    gripper_msg_2->gripper_c610_position = ui->gripper2_c610->value();
    gripper_msg_2->gripper_sts3032_position = ui->gripper2_sts3032->value();

    auto slave_msg_1 = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessageSlave>();
    auto slave_msg_2 = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessageSlave>();
    // snake motors control for robomaster 1 & 2
    for (size_t i = 0; i < 12; i++)
    {
        slave_msg_1-> snake_speed_control_array[i]  = speedControls_robomaster1[i]->value();
        slave_msg_1-> snake_position_control_array[i]  = positionControls_robomaster1[i]->value();
        slave_msg_2-> snake_speed_control_array[i]  = speedControls_robomaster2[i]->value();
        slave_msg_2-> snake_position_control_array[i]  = positionControls_robomaster2[i]->value();
    }
    slave_msg_1->robomaster_mode = 0; // 
    slave_msg_2->robomaster_mode = 0; // 
    // 然后发布新的控制消息
    gripper_control_topic_publisher_1->publish(*gripper_msg_1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // sleep for 10ms
    slave_control_topic_publisher_1->publish(*slave_msg_1);

    gripper_control_topic_publisher_2->publish(*gripper_msg_2);
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // sleep for 10ms
    slave_control_topic_publisher_2->publish(*slave_msg_2);
}

void MainWindow::on_transButton_clicked(){
    // 创建消息
    auto theta_msg = std_msgs::msg::Float64MultiArray();
    theta_msg.data = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    for (size_t i = 0; i < 12; i++)
    {
        theta_msg.data[i] = thetaControls[i]->value() * M_PI / 180;
    }
    // 发布消息
    joint_space_topic_publisher->publish(theta_msg);

    auto slave_msg_1 = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessageSlave>();
    auto slave_msg_2 = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessageSlave>();
    // snake motors control for robomaster 1 & 2
    for (size_t i = 0; i < 12; i++)
    {
        slave_msg_1-> snake_speed_control_array[i]  = speedControls_robomaster1[i]->value();
        slave_msg_1-> snake_position_control_array[i]  = positionControls_robomaster1[i]->value();
        slave_msg_2-> snake_speed_control_array[i]  = speedControls_robomaster2[i]->value();
        slave_msg_2-> snake_position_control_array[i]  = positionControls_robomaster2[i]->value();
    }
    slave_msg_1->robomaster_mode = 5; // mode 5, control by omega7
    slave_msg_2->robomaster_mode = 5; // mode 5, control by omega7
    // 然后发布新的控制消息
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // sleep for 10ms
    slave_control_topic_publisher_1->publish(*slave_msg_1);
    slave_control_topic_publisher_2->publish(*slave_msg_2);
}

void MainWindow::on_publishButton_clicked()
{
    auto slave_msg_1 = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessageSlave>();
    auto slave_msg_2 = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessageSlave>();
    for (size_t i = 0; i < 12; i++)
    {
        slave_msg_1->snake_position_control_array[i] = 10;
        slave_msg_2->snake_position_control_array[i] = 10;
    }    

    // snake motors control for robomaster 1 & 2
    for (size_t i = 0; i < 12; i++)
    {
        slave_msg_1-> snake_speed_control_array[i]  = speedControls_robomaster1[i]->value();
        slave_msg_1-> snake_position_control_array[i]  = positionControls_robomaster1[i]->value();
        slave_msg_2-> snake_speed_control_array[i]  = speedControls_robomaster2[i]->value();
        slave_msg_2-> snake_position_control_array[i]  = positionControls_robomaster2[i]->value();
    }
    slave_msg_1->robomaster_mode = ui->robomaster1_mode->value();    
    slave_msg_2->robomaster_mode = ui->robomaster2_mode->value();
    slave_control_topic_publisher_1->publish(*slave_msg_1);
    slave_control_topic_publisher_2->publish(*slave_msg_2);
    // master devices control
    auto master_msg = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessageMaster>();
    master_msg->elevator_control = ui->elevator_speed_control->value();
    master_msg->lower_linear_module_control = ui->lower_LM_speed_control->value();
    master_msg->upper_linear_module_control = ui->upper_LM_speed_control->value();
    master_msg->pull_push_sensors_reset = ui->PP_sensors_reset->value();
    master_msg->elevator_counter_reset = ui->elevator_counter_reset->value();
    master_msg->lower_linear_module_encorder_reset = ui->lower_LM_encorder_reset->value();
    master_msg->upper_linear_module_encorder_reset = ui->upper_LM_encorder_reset->value();
    master_control_topic_publisher->publish(*master_msg);
}

MainWindow::~MainWindow()
{
    stopCameraThreads(); // 确保在销毁窗口前停止线程
    delete ui;
}

