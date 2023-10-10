#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    auto node = std::make_shared<rclcpp::Node>("buaa_rescue_robot_gui_node");
    ControlListView = new QStringListModel(this);  
    ui->NewControlListView->setModel(ControlListView);  
}

MainWindow::~MainWindow()
{
    delete ui;
}

