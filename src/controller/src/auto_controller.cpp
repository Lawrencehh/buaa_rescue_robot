#include "rclcpp/rclcpp.hpp"
#include "buaa_rescue_robot_msgs/msg/control_message.hpp"
#include "buaa_rescue_robot_msgs/msg/sensors_message_robomaster.hpp"
#include "buaa_rescue_robot_msgs/msg/sensors_message_master_device_pull_push_sensors.hpp"
#include <array> // 包含array头文件
#include <chrono>
#include <thread>

class AutoController : public rclcpp::Node {
public:
  AutoController() : Node("auto_controller") {
    // 初始化订阅者，订阅各个话题
    subscriber_control_topic_1 = this->create_subscription<buaa_rescue_robot_msgs::msg::ControlMessage>(
      "control_topic", 10,
      std::bind(&AutoController::control_topic_callback, this, std::placeholders::_1));

    subscriber_sensors_robomaster_1 = this->create_subscription<buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster>(
      "Sensors_Robomaster_1", 10,
      std::bind(&AutoController::sensors_robomaster_1_callback, this, std::placeholders::_1));

    subscriber_sensors_pull_push_1 = this->create_subscription<buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors>(
      "Sensors_Pull_Push_Sensors_1", 10,
      std::bind(&AutoController::sensors_pull_push_1_callback, this, std::placeholders::_1));

    // 初始化发布者
    publisher_control_topic_ = this->create_publisher<buaa_rescue_robot_msgs::msg::ControlMessage>("control_topic", 10);
  }

private:
  void control_topic_callback(const buaa_rescue_robot_msgs::msg::ControlMessage::SharedPtr msg) {
    if(msg->robomaster_1_reset == 2){ // reset the encorders
      // 操作控制消息和其他传感器数据
      // 首先确保我们已经接收到了其他传感器的数据
      if (received_sensors_robomaster_1 && received_sensors_pull_push_1) {
        // 执行处理逻辑，可能会涉及到控制消息和其他传感器数据
        int16_t running_flag = 0;
        for (size_t i = 0; i < 12; i++)
        {
          int16_t delta;
          // stop calibration
          if ((last_sensors_pull_push_data_1.pull_push_sensors_1[i] <= encorder_zero_up_limit[i]) && 
          (last_sensors_pull_push_data_1.pull_push_sensors_1[i] >= encorder_zero_down_limit[i]))
          {
            running_flag = running_flag + 1;
          }
          
          // forward
          if(last_sensors_pull_push_data_1.pull_push_sensors_1[i] < encorder_zero_down_limit[i]){
            if(abs(last_sensors_pull_push_data_1.pull_push_sensors_1[i] - encorder_zero_down_limit[i]) > 10){
              delta = coarse_delta;
            } else{
              delta = fine_delta;
            }
            if((msg->snake_control_1_array[i] - last_sensors_robomaster_data_1.snake_motor_encorder_position[i]) < delta){
              msg->snake_control_1_array[i] = last_sensors_robomaster_data_1.snake_motor_encorder_position[i] + delta;
            }
          }

          // backward
          if(last_sensors_pull_push_data_1.pull_push_sensors_1[i] > encorder_zero_up_limit[i]){
            if(abs(last_sensors_pull_push_data_1.pull_push_sensors_1[i] - encorder_zero_up_limit[i]) > 10){
              delta = coarse_delta;
            } else{
              delta = fine_delta;
            }
            if((msg->snake_control_1_array[i] - last_sensors_robomaster_data_1.snake_motor_encorder_position[i]) > -delta){
              msg->snake_control_1_array[i] = last_sensors_robomaster_data_1.snake_motor_encorder_position[i] - delta;
            }
          }
        }

        if (running_flag == 12) {
          msg->robomaster_1_reset = 0;
          msg->robomaster_2_reset = 0;
          received_sensors_robomaster_1 = false;
          received_sensors_pull_push_1 = false;
        }

        // 然后发布新的控制消息
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // sleep for 50ms
        publisher_control_topic_->publish(*msg);

      } else {
        RCLCPP_INFO(this->get_logger(), "Waiting for all sensor data before processing control message");
      }
    }


  }

  void sensors_robomaster_1_callback(const buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster::SharedPtr msg) {
    // 保存Robomaster传感器数据
    last_sensors_robomaster_data_1 = *msg;
    received_sensors_robomaster_1 = true;
  }

  void sensors_pull_push_1_callback(const buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors::SharedPtr msg) {
    // 保存拉推传感器数据
    last_sensors_pull_push_data_1 = *msg;
    received_sensors_pull_push_1 = true;
  }

  rclcpp::Subscription<buaa_rescue_robot_msgs::msg::ControlMessage>::SharedPtr subscriber_control_topic_1;
  rclcpp::Subscription<buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster>::SharedPtr subscriber_sensors_robomaster_1;
  rclcpp::Subscription<buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors>::SharedPtr subscriber_sensors_pull_push_1;
  rclcpp::Publisher<buaa_rescue_robot_msgs::msg::ControlMessage>::SharedPtr publisher_control_topic_;

  // 成员变量用于保存最新接收到的传感器数据
  buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster last_sensors_robomaster_data_1;
  buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors last_sensors_pull_push_data_1;

  // 标志变量，用于确定是否接收到了传感器数据
  bool received_sensors_robomaster_1 = false;
  bool received_sensors_pull_push_1 = false;

  // 定义一个std::array类型的变量，大小为12
  std::array<int32_t, 12> encorder_zero_up_limit = {5,5,5,5,5,5,5,5,5,5,5,5}; // 所有元素都将初始化为0
  std::array<int32_t, 12> encorder_zero_down_limit = {0,0,0,0,0,0,0,0,0,0,0,0}; // 所有元素都将初始化为0
  int32_t fine_delta = 100;
  int32_t coarse_delta = 2000;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
