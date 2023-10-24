#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <opencv2/opencv.hpp>
#include <QKeyEvent>  // 引入QKeyEvent头文件

// 重载keyPressEvent方法
void MainWindow::keyPressEvent(QKeyEvent *event)
{
    // Initialize msg before switch statement
    std::shared_ptr<buaa_rescue_robot_msgs::msg::ControlMessage> msg;
    // 判断按下的键
    switch(event->key()) {
        case Qt::Key_Up:
            // 上箭头键被按下
            // msg->elevator_control = 1;  // 举例，设定电梯控制为1
            // control_topic_publisher->publish(*msg);
            qDebug() << "Up Arrow Pressed and control message published!";
            break;
        case Qt::Key_Down:
            // 下箭头键被按下
            // TODO: 你的代码
            qDebug() << "Down Arrow Pressed!";
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
}


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    cap.open(0);  // 打开默认摄像头
    scene = new QGraphicsScene(this);
    ui->camera->setScene(scene);

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateCameraFrame()));
    timer->start(100); // 10 fps

    // 初始化 ControlMessageDisplay
    ControlMessageDisplay = new QStringListModel(this);  
    ui->ControlMessageDisplay->setModel(ControlMessageDisplay); 

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

    node = std::make_shared<rclcpp::Node>("mainwindow_node");

    // 初始化ROS 2发布器
    control_topic_publisher = node->create_publisher<buaa_rescue_robot_msgs::msg::ControlMessage>("control_topic", 10);
    // 连接Qt信号和槽
    connect(ui->publishButton, SIGNAL(clicked()), this, SLOT(on_publishButton_clicked()));
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
        ui->camera->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
    }
}

void MainWindow::on_publishButton_clicked()
{
    auto msg = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessage>();
    // snake motors control for robomaster 1
    msg-> snake_control_1_array[0]  = ui->robomaster1_snake_motor_position_control_1->value();
    msg-> snake_control_1_array[1]  = ui->robomaster1_snake_motor_position_control_2->value();
    msg-> snake_control_1_array[2]  = ui->robomaster1_snake_motor_position_control_3->value();
    msg-> snake_control_1_array[3]  = ui->robomaster1_snake_motor_position_control_4->value();
    msg-> snake_control_1_array[4]  = ui->robomaster1_snake_motor_position_control_5->value();
    msg-> snake_control_1_array[5]  = ui->robomaster1_snake_motor_position_control_6->value();
    msg-> snake_control_1_array[6]  = ui->robomaster1_snake_motor_position_control_7->value();
    msg-> snake_control_1_array[7]  = ui->robomaster1_snake_motor_position_control_8->value();
    msg-> snake_control_1_array[8]  = ui->robomaster1_snake_motor_position_control_9->value();
    msg-> snake_control_1_array[9]  = ui->robomaster1_snake_motor_position_control_10->value();
    msg-> snake_control_1_array[10] = ui->robomaster1_snake_motor_position_control_11->value();
    msg-> snake_control_1_array[11] = ui->robomaster1_snake_motor_position_control_12->value();
    // gripper control for robomaster 1
    msg->gripper_gm6020_position_1 = ui->gripper_gm6020_position_1_control->value();
    msg->gripper_c610_position_1 = ui->gripper_c610_position_1_control->value();
    msg->gripper_sts3032_position_1 = ui->gripper_sts3032_position_1_control->value();
    msg->robomaster_1_reset = ui->robomaster1_sensors_reset->value();
    // snake motors control for robomaster 2
    msg-> snake_control_2_array[0]  = ui->robomaster2_snake_motor_position_control_1->value();
    msg-> snake_control_2_array[1]  = ui->robomaster2_snake_motor_position_control_2->value();
    msg-> snake_control_2_array[2]  = ui->robomaster2_snake_motor_position_control_3->value();
    msg-> snake_control_2_array[3]  = ui->robomaster2_snake_motor_position_control_4->value();
    msg-> snake_control_2_array[4]  = ui->robomaster2_snake_motor_position_control_5->value();
    msg-> snake_control_2_array[5]  = ui->robomaster2_snake_motor_position_control_6->value();
    msg-> snake_control_2_array[6]  = ui->robomaster2_snake_motor_position_control_7->value();
    msg-> snake_control_2_array[7]  = ui->robomaster2_snake_motor_position_control_8->value();
    msg-> snake_control_2_array[8]  = ui->robomaster2_snake_motor_position_control_9->value();
    msg-> snake_control_2_array[9]  = ui->robomaster2_snake_motor_position_control_10->value();
    msg-> snake_control_2_array[10] = ui->robomaster2_snake_motor_position_control_11->value();
    msg-> snake_control_2_array[11] = ui->robomaster2_snake_motor_position_control_12->value();
    // gripper control for robomaster 1
    msg->gripper_gm6020_position_2 = ui->gripper_gm6020_position_2_control->value();
    msg->gripper_c610_position_2 = ui->gripper_c610_position_2_control->value();
    msg->gripper_sts3032_position_2 = ui->gripper_sts3032_position_2_control->value();
    msg->robomaster_2_reset = ui->robomaster2_sensors_reset->value();
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

