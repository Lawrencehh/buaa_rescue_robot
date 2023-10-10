#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <rclcpp/rclcpp.hpp>  
#include <QStringListModel>  
#include "buaa_rescue_robot_msgs/msg/control_message.hpp"  // 引入自定义消息类型

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void updateControlListView(const QStringList &list) {
        ControlListView->setStringList(list);
    }

private:
    Ui::MainWindow *ui;
    rclcpp::Subscription<buaa_rescue_robot_msgs::msg::ControlMessage>::SharedPtr ControlMessageSubscription_;
    QStringListModel *ControlListView;  // 新增这一行
};
#endif // MAINWINDOW_H
