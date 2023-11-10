#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <opencv2/opencv.hpp>
#include <QKeyEvent>  // 引入QKeyEvent头文件
// #include <thread>
// #include <chrono>
#include <QTime>

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

    node = std::make_shared<rclcpp::Node>("mainwindow_node");

    // 初始化ROS 2发布器
    control_topic_publisher = node->create_publisher<buaa_rescue_robot_msgs::msg::ControlMessage>("control_topic", 10);
    // 连接Qt信号和槽
    connect(ui->publishButton, SIGNAL(clicked()), this, SLOT(on_publishButton_clicked()));
}



// 重载keyPressEvent方法, control the robot by keyboard
void MainWindow::keyPressEvent(QKeyEvent *event)
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
            case '5': // turn the reset_flag = 0
                reset_flag = false;
                msg->elevator_control = 0;
                msg->lower_linear_module_control = 0;
                msg->upper_linear_module_control = 0;
                msg->pull_push_sensors_reset = 0;
                msg->elevator_counter_reset = 0; // reset to be 1
                msg->lower_linear_module_encorder_reset = 0; // reset to be 1
                msg->upper_linear_module_encorder_reset = 0; // reset to be 1
                msg->robomaster_1_reset = 0; // robomaster 1 motor encorders to be 1
                msg->robomaster_2_reset = 0; // robomaster 2 motor encorders to be 1
                control_topic_publisher->publish(*msg);
                break;
             case Qt::Key_C: // Calibration, Mode 2
                msg->snake_control_1_array = {0,0,0,0,0,0,0,0,0,0,0,0};
                msg->gripper_gm6020_position_1 = 0;
                msg->gripper_c610_position_1 = 0;
                msg->gripper_sts3032_position_1 = 0;

                msg->snake_control_2_array = {0,0,0,0,0,0,0,0,0,0,0,0};
                msg->gripper_gm6020_position_2 = 0;
                msg->gripper_c610_position_2 = 0;
                msg->gripper_sts3032_position_2 = 0;

                msg->elevator_control = 0;
                msg->lower_linear_module_control = 0;
                msg->upper_linear_module_control = 0;
            

                msg->pull_push_sensors_reset = 0;
                msg->elevator_counter_reset = 0; // reset to be 1
                msg->lower_linear_module_encorder_reset = 0; // reset to be 1
                msg->upper_linear_module_encorder_reset = 0; // reset to be 1
                msg->robomaster_1_reset = 2; // Calibration
                msg->robomaster_2_reset = 2; // Calibration
                control_topic_publisher->publish(*msg);  // 发布消息
                break;
            case Qt::Key_E: // Calibration, Mode 12
                msg->snake_control_1_array = {0,0,0,0,0,0,0,0,0,0,0,0};
                msg->gripper_gm6020_position_1 = 0;
                msg->gripper_c610_position_1 = 0;
                msg->gripper_sts3032_position_1 = 0;

                msg->snake_control_2_array = {0,0,0,0,0,0,0,0,0,0,0,0};
                msg->gripper_gm6020_position_2 = 0;
                msg->gripper_c610_position_2 = 0;
                msg->gripper_sts3032_position_2 = 0;

                msg->elevator_control = 0;
                msg->lower_linear_module_control = 0;
                msg->upper_linear_module_control = 0;
            

                msg->pull_push_sensors_reset = 0;
                msg->elevator_counter_reset = 0; // reset to be 1
                msg->lower_linear_module_encorder_reset = 0; // reset to be 1
                msg->upper_linear_module_encorder_reset = 0; // reset to be 1
                msg->robomaster_1_reset = 12; // Calibration at zero
                msg->robomaster_2_reset = 12; // Calibration at zero
                control_topic_publisher->publish(*msg);  // 发布消息
                break;
            case Qt::Key_R: // Calibration, Mode 3
                for (size_t i = 0; i < 12; i++)
                {
                     msg->snake_control_1_array[i] = -200000;
                     msg->snake_control_2_array[i] = -200000;
                }                
                msg->gripper_gm6020_position_1 = 0;
                msg->gripper_c610_position_1 = 0;
                msg->gripper_sts3032_position_1 = 0;

                msg->gripper_gm6020_position_2 = 0;
                msg->gripper_c610_position_2 = 0;
                msg->gripper_sts3032_position_2 = 0;

                msg->elevator_control = 0;
                msg->lower_linear_module_control = 0;
                msg->upper_linear_module_control = 0;
            

                msg->pull_push_sensors_reset = 0;
                msg->elevator_counter_reset = 0; 
                msg->lower_linear_module_encorder_reset = 0; 
                msg->upper_linear_module_encorder_reset = 0; 
                msg->robomaster_1_reset = 3; // Release
                msg->robomaster_2_reset = 3; // Release
                control_topic_publisher->publish(*msg);  // 发布消息
                break;
            case Qt::Key_A: // enable, Mode 9
                msg->snake_control_1_array = {0,0,0,0,0,0,0,0,0,0,0,0};
                msg->gripper_gm6020_position_1 = 0;
                msg->gripper_c610_position_1 = 0;
                msg->gripper_sts3032_position_1 = 0;

                msg->snake_control_2_array = {0,0,0,0,0,0,0,0,0,0,0,0};
                msg->gripper_gm6020_position_2 = 0;
                msg->gripper_c610_position_2 = 0;
                msg->gripper_sts3032_position_2 = 0;

                msg->elevator_control = 0;
                msg->lower_linear_module_control = 0;
                msg->upper_linear_module_control = 0;
            

                msg->pull_push_sensors_reset = 0;
                msg->elevator_counter_reset = 0; // reset to be 1
                msg->lower_linear_module_encorder_reset = 0; // reset to be 1
                msg->upper_linear_module_encorder_reset = 0; // reset to be 1
                msg->robomaster_1_reset = 9; // enable
                msg->robomaster_2_reset = 9; // enable
                control_topic_publisher->publish(*msg);  // 发布消息
                break;
            case Qt::Key_Q: // quit, Mode 6
                msg->snake_control_1_array = {0,0,0,0,0,0,0,0,0,0,0,0};
                msg->gripper_gm6020_position_1 = 0;
                msg->gripper_c610_position_1 = 0;
                msg->gripper_sts3032_position_1 = 0;

                msg->snake_control_2_array = {0,0,0,0,0,0,0,0,0,0,0,0};
                msg->gripper_gm6020_position_2 = 0;
                msg->gripper_c610_position_2 = 0;
                msg->gripper_sts3032_position_2 = 0;

                msg->elevator_control = 0;
                msg->lower_linear_module_control = 0;
                msg->upper_linear_module_control = 0;
            

                msg->pull_push_sensors_reset = 0;
                msg->elevator_counter_reset = 0; // reset to be 1
                msg->lower_linear_module_encorder_reset = 0; // reset to be 1
                msg->upper_linear_module_encorder_reset = 0; // reset to be 1
                msg->robomaster_1_reset = 6; // quit
                msg->robomaster_2_reset = 6; // quit
                control_topic_publisher->publish(*msg);  // 发布消息
                break;
            case Qt::Key_Z: // reset the sensors, Mode 1
                reset_flag = true;
                msg->snake_control_1_array = {0,0,0,0,0,0,0,0,0,0,0,0};
                msg->gripper_gm6020_position_1 = 0;
                msg->gripper_c610_position_1 = 0;
                msg->gripper_sts3032_position_1 = 0;

                msg->snake_control_2_array = {0,0,0,0,0,0,0,0,0,0,0,0};
                msg->gripper_gm6020_position_2 = 0;
                msg->gripper_c610_position_2 = 0;
                msg->gripper_sts3032_position_2 = 0;

                msg->elevator_control = 0;
                msg->lower_linear_module_control = 0;
                msg->upper_linear_module_control = 0;
            

                msg->pull_push_sensors_reset = 1;
                msg->elevator_counter_reset = 1; // reset to be 1
                msg->lower_linear_module_encorder_reset = 1; // reset to be 1
                msg->upper_linear_module_encorder_reset = 1; // reset to be 1
                msg->robomaster_1_reset = 1; // robomaster 1 motor encorders to be 1
                msg->robomaster_2_reset = 1; // robomaster 2 motor encorders to be 1
                control_topic_publisher->publish(*msg);  // 发布消息
                RCLCPP_INFO(node->get_logger(), "Published control message to reset the sensors: 1");  // 打印日志
                break;
            case Qt::Key_P: // reset the Pull-Push force sensors, Mode 4
                reset_flag = true;    
                msg->elevator_control = 0;
                msg->lower_linear_module_control = 0;
                msg->upper_linear_module_control = 0;           
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
    switch (event->key()) {
        case Qt::Key_Z:
            reset_flag = false;
            msg->snake_control_1_array = {0,0,0,0,0,0,0,0,0,0,0,0};
            msg->gripper_gm6020_position_1 = 0;
            msg->gripper_c610_position_1 = 0;
            msg->gripper_sts3032_position_1 = 0;

            msg->snake_control_2_array = {0,0,0,0,0,0,0,0,0,0,0,0};
            msg->gripper_gm6020_position_2 = 0;
            msg->gripper_c610_position_2 = 0;
            msg->gripper_sts3032_position_2 = 0;

            msg->elevator_control = 0;
            msg->lower_linear_module_control = 0;
            msg->upper_linear_module_control = 0;
        

            msg->pull_push_sensors_reset = 0;
            msg->elevator_counter_reset = 0; 
            msg->lower_linear_module_encorder_reset = 0; 
            msg->upper_linear_module_encorder_reset = 0; 
            msg->robomaster_1_reset = 0; 
            msg->robomaster_2_reset = 0; 
            control_topic_publisher->publish(*msg);  // 发布消息
            RCLCPP_INFO(node->get_logger(), "Published control message to reset the sensors: 0");  // 打印日志
            break;
        case Qt::Key_A:
            reset_flag = false;
            msg->snake_control_1_array = {0,0,0,0,0,0,0,0,0,0,0,0};
            msg->gripper_gm6020_position_1 = 0;
            msg->gripper_c610_position_1 = 0;
            msg->gripper_sts3032_position_1 = 0;

            msg->snake_control_2_array = {0,0,0,0,0,0,0,0,0,0,0,0};
            msg->gripper_gm6020_position_2 = 0;
            msg->gripper_c610_position_2 = 0;
            msg->gripper_sts3032_position_2 = 0;

            msg->elevator_control = 0;
            msg->lower_linear_module_control = 0;
            msg->upper_linear_module_control = 0;
        

            msg->pull_push_sensors_reset = 0;
            msg->elevator_counter_reset = 0; 
            msg->lower_linear_module_encorder_reset = 0; 
            msg->upper_linear_module_encorder_reset = 0; 
            msg->robomaster_1_reset = 0; 
            msg->robomaster_2_reset = 0; 
            control_topic_publisher->publish(*msg);  // 发布消息
            RCLCPP_INFO(node->get_logger(), "Published control message to reset the sensors: 0");  // 打印日志
            break;
        case Qt::Key_P: // reset the Pull-Push force sensors, Mode 4
            reset_flag = false;    
            msg->elevator_control = 0;
            msg->lower_linear_module_control = 0;
            msg->upper_linear_module_control = 0;           
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
    // msg->gripper_gm6020_position_1 = ui->gripper_gm6020_position_1_control->value();
    // msg->gripper_c610_position_1 = ui->gripper_c610_position_1_control->value();
    // msg->gripper_sts3032_position_1 = ui->gripper_sts3032_position_1_control->value();
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
    // msg->gripper_gm6020_position_2 = ui->gripper_gm6020_position_2_control->value();
    // msg->gripper_c610_position_2 = ui->gripper_c610_position_2_control->value();
    // msg->gripper_sts3032_position_2 = ui->gripper_sts3032_position_2_control->value();
    msg->robomaster_2_reset = ui->robomaster2_sensors_reset->value();
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

