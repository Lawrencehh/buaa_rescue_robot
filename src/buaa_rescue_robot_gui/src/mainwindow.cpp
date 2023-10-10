#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    auto node = std::make_shared<rclcpp::Node>("buaa_rescue_robot_gui_node");
    snakePositionControlModel1 = new QStringListModel(this);  // 新增这一行
    ui->snakePositionControlListView1->setModel(snakePositionControlModel1);  // 新增这一行
}

MainWindow::~MainWindow()
{
    delete ui;
}

