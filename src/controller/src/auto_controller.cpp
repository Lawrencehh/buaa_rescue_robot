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
    if(msg->robomaster_1_reset == 2 || msg->robomaster_1_reset == 12){ // reset the encorders
      if(msg->robomaster_1_reset == 2){
            encorder_zero_up_limit = {600,600,600,600,600,600,600,600,600,600,600,600}; // 所有元素都将初始化为0
            encorder_zero_down_limit = {500,500,500,500,500,500,500,500,500,500,500,500}; // 所有元素都将初始化为0
      }
      if(msg->robomaster_1_reset == 12){
        for (size_t i = 0; i < 12; i++)
        {
          encorder_zero_down_limit[i] = 30;
          encorder_zero_up_limit[i] = 60;
        } 
      }

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
            if(abs(last_sensors_pull_push_data_1.pull_push_sensors_1[i] - encorder_zero_down_limit[i]) > error_threshold){
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
            if(abs(last_sensors_pull_push_data_1.pull_push_sensors_1[i] - encorder_zero_up_limit[i]) > error_threshold){
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
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // sleep for 100ms
        publisher_control_topic_->publish(*msg);

      } else {
        RCLCPP_INFO(this->get_logger(), "Waiting for all sensor data before processing control message");
      }
    } else {

    }

  }

  void sensors_robomaster_1_callback(const buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster::SharedPtr msg) {
    // 保存Robomaster传感器数据
    last_sensors_robomaster_data_1 = *msg;
    received_sensors_robomaster_1 = true;
  }

  void sensors_pull_push_1_callback(const buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors::SharedPtr pull_push_sensors_msg) {
    // 保存拉推传感器数据
    last_sensors_pull_push_data_1 = *pull_push_sensors_msg;
    received_sensors_pull_push_1 = true;
    auto msg = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessage>();
    for (size_t i = 0; i < 12; i++)
    {
      if (abs(pull_push_sensors_msg->pull_push_sensors_1[i]) > 1800)
      {
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
        // 然后发布新的控制消息
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // sleep for 100ms
        publisher_control_topic_->publish(*msg);
      }
      
      
    }
    
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
  std::array<int32_t, 12> encorder_zero_up_limit = {1100,1100,800,800,800,800,800,800,800,800,1100,1100}; // 所有元素都将初始化为0
  std::array<int32_t, 12> encorder_zero_down_limit = {1000,1000,700,700,700,700,700,700,700,700,1000,1000}; // 所有元素都将初始化为0
  int32_t fine_delta = 500;
  int32_t coarse_delta = 5000;
  int32_t error_threshold = 2;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
