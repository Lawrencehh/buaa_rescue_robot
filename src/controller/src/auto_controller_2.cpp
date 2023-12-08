#include "rclcpp/rclcpp.hpp"
#include "buaa_rescue_robot_msgs/msg/control_message_slave.hpp"
#include "buaa_rescue_robot_msgs/msg/sensors_message_robomaster.hpp"
#include "buaa_rescue_robot_msgs/msg/sensors_message_master_device_pull_push_sensors.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <array> // 包含array头文件
#include <chrono>
#include <thread>

class AutoController : public rclcpp::Node {
public:
  AutoController() : Node("auto_controller") {
    // 初始化订阅者，订阅各个话题
    subscriber_slave_control_topic = this->create_subscription<buaa_rescue_robot_msgs::msg::ControlMessageSlave>(
      "slave_control_topic_2", 10,
      std::bind(&AutoController::slave_control_topic_callback, this, std::placeholders::_1));

    subscriber_sensors_robomaster_2 = this->create_subscription<buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster>(
      "Sensors_Robomaster_2", 10,
      std::bind(&AutoController::sensors_robomaster_2_callback, this, std::placeholders::_1));

    subscriber_sensors_pull_push_2 = this->create_subscription<buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors>(
      "Sensors_Pull_Push_Sensors_2", 10,
      std::bind(&AutoController::sensors_pull_push_2_callback, this, std::placeholders::_1));

    subscriber_joint_space = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "joint_space_topic", 10,
      std::bind(&AutoController::joint_space_callback, this, std::placeholders::_1));

    // 初始化发布者
    publisher_slave_control_topic_ = this->create_publisher<buaa_rescue_robot_msgs::msg::ControlMessageSlave>("slave_control_topic_2", 10);

    for (size_t i = 0; i < 12; i++)
    {
      if (i == 0 || i == 1 || i == 10 || i == 11)
      {
        encorder_zero_final_up_limit[i] = tension_segment_1;
      } else if(i == 2 || i == 3 || i == 8 || i == 9) {
        encorder_zero_final_up_limit[i] = tension_segment_2;
      } else if(i == 4 || i == 5 || i == 6 || i == 7) {
        encorder_zero_final_up_limit[i] = tension_segment_3;
      }
      encorder_zero_final_down_limit[i] = encorder_zero_final_up_limit[i] - 100;
    } 
  }

  // theta to rope
  std::array<double, 12> theta2rope(const std::array<double, 6>& theta) {
      // 定义各间隙变量，并赋予固定值
      std::array<double, 12> Gap_1 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      std::array<double, 12> Gap_2 = {29.63, 29.63, 29.09, 29.09, 29.63, 29.63, 29.63, 29.63, 29.09, 29.09, 29.63, 29.63};
      std::array<double, 12> Gap_3 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      std::array<double, 12> Gap_4 = {29.63, 29.63, 29.09, 29.09, 29.63, 29.63, 29.63, 29.63, 29.09, 29.09, 29.63, 29.63};
      std::array<double, 12> Gap_base = {24.57, 24.57, 26.11, 26.11, 28.12, 28.12, 28.12, 28.12, 26.11, 26.11, 24.57, 24.57};
      
      std::array<double, 12> Rope_length = {0}; // 初始化为0
      double beta = M_PI * 50 / 180; // 初始角度
      std::array<double, 12> R_Gap_1 = {23.03, 23.03, 18.82, 18.82, 13.33, 13.33, 13.33, 13.33, 18.82, 18.82, 23.03, 23.03};
      std::array<double, 12> R_Gap_3 = {13.33, 13.33, 18.82, 18.82, 23.03, 23.03, 23.03, 23.03, 18.82, 18.82, 13.33, 13.33};

      for (int N = 0; N < 5; ++N) {
          for (int i = 0; i < 12; ++i) {
              // 计算第一节的Gap1和Gap3
              if (i > 5) {
                  Gap_1[i] = sin((beta - theta[0] + M_PI/10) / 2) * 2 * R_Gap_1[i]; 
              } else {
                  Gap_1[i] = sin((beta + theta[0] - M_PI/10) / 2) * 2 * R_Gap_1[i];
              }

              if (i % 2 == 0) {
                  Gap_3[i] = sin((beta + theta[1]) / 2) * 2 * R_Gap_3[i];
              } else {
                  Gap_3[i] = sin((beta - theta[1]) / 2) * 2 * R_Gap_3[i];
              }

              Rope_length[i] += Gap_1[i] + Gap_2[i] + Gap_3[i] + Gap_4[i] + Gap_base[i];

              // 计算第二节的Gap1和Gap3，除了1,2,11,12
              if (i != 0 && i != 1 && i != 10 && i != 11) {
                  if (i > 5) {
                      Gap_1[i] = sin((beta - theta[2]) / 2) * 2 * R_Gap_1[i];
                  } else {
                      Gap_1[i] = sin((beta + theta[2]) / 2) * 2 * R_Gap_1[i];
                  }

                  if (i % 2 == 0) {
                      Gap_3[i] = sin((beta + theta[3]) / 2) * 2 * R_Gap_3[i];
                  } else {
                      Gap_3[i] = sin((beta - theta[3]) / 2) * 2 * R_Gap_3[i];
                  }

                  Rope_length[i] += Gap_1[i] + Gap_2[i] + Gap_3[i] + Gap_4[i];
              }

              // 计算第三节的Gap1和Gap3，除了1,2,11,12,3,4,9,10
              if (i != 0 && i != 1 && i != 10 && i != 11 && i != 2 && i != 3 && i != 8 && i != 9) {
                  if (i > 5) {
                      Gap_1[i] = sin((beta - theta[4]) / 2) * 2 * R_Gap_1[i];
                  } else {
                      Gap_1[i] = sin((beta + theta[4]) / 2) * 2 * R_Gap_1[i];
                  }

                  if (i % 2 == 0) {
                      Gap_3[i] = sin((beta + theta[5]) / 2) * 2 * R_Gap_3[i];
                  } else {
                      Gap_3[i] = sin((beta - theta[5]) / 2) * 2 * R_Gap_3[i];
                  }

                  Rope_length[i] += Gap_1[i] + Gap_2[i] + Gap_3[i] + Gap_4[i];
              }
          }
      }

      // 所有关节都计算完之后，每个关节和下一个关节连接处的圆盘内绳长需减掉一截Gap_4
      for (int i = 0; i < 12; ++i) {
          Rope_length[i] -= Gap_4[i];
      }

      return Rope_length;
  }

  // 计算adjust_speed， 该值用于绳驱电机的调速使用
  int32_t calculateAdjustedSpeed(int32_t sensor_value, int32_t mean_tension, int32_t error_tension, int32_t average_speed, int32_t motor_speed) {
      int32_t adjust_speed = (mean_tension - sensor_value) / error_tension;
      adjust_speed = std::clamp(adjust_speed, -average_speed / 2, average_speed / 2);
      if (motor_speed < 0) {
          adjust_speed = -adjust_speed;
      }
      return adjust_speed;
  }

private:
  void slave_control_topic_callback(const buaa_rescue_robot_msgs::msg::ControlMessageSlave::SharedPtr msg) {
    // 所能忍受的绳子张力与均值之前的误差（均值由同一段的4根绳子张力求得）
    int32_t error_tension = 5;
    // 第一段4根绳子的张力均值
    int32_t mean_tension_segment_2_1 = (last_sensors_pull_push_data_2.pull_push_sensors[0] + last_sensors_pull_push_data_2.pull_push_sensors[1]
    + last_sensors_pull_push_data_2.pull_push_sensors[10] + last_sensors_pull_push_data_2.pull_push_sensors[11]) / 4;
    // 第二段4根绳子的张力均值
    int32_t mean_tension_segment_2_2 = (last_sensors_pull_push_data_2.pull_push_sensors[2] + last_sensors_pull_push_data_2.pull_push_sensors[3]
    + last_sensors_pull_push_data_2.pull_push_sensors[8] + last_sensors_pull_push_data_2.pull_push_sensors[9]) / 4;
    // 第三段4根绳子的张力均值
    int32_t mean_tension_segment_2_3 = (last_sensors_pull_push_data_2.pull_push_sensors[4] + last_sensors_pull_push_data_2.pull_push_sensors[5]
    + last_sensors_pull_push_data_2.pull_push_sensors[6] + last_sensors_pull_push_data_2.pull_push_sensors[7]) / 4;

    // mode 6的时候将禁止其他模式，并失能所有绳驱电机
    if ((msg->robomaster_mode == 6))// mode 6, to quit
    {
      auto_lock = 1;
    }

    // mode 9的时候将使能所有绳驱电机
    if ((msg->robomaster_mode == 9))// mode 9, to enable
    {
      auto_lock = 0;
    }   

    // mode 2 及 mode 12
    if(msg->robomaster_mode == 2 || msg->robomaster_mode == 12){ // Mode 2 and mode 12
      running_flag_2 = 0;
      for (size_t i = 0; i < 12; i++)
      {
        // 当绳子张力处于设定区间的时候，让running_flag_2加一
        if ((last_sensors_pull_push_data_2.pull_push_sensors[i] <= encorder_zero_final_up_limit[i]) && 
        (last_sensors_pull_push_data_2.pull_push_sensors[i] >= encorder_zero_final_down_limit[i]))
        {
          running_flag_2 = running_flag_2 + 1;
        }
      }
      
      // mode 2，让绳子依照1、2、3段顺序进行绷紧
      if(msg->robomaster_mode == 2){
        if (running_flag_2 == 0)
        {
          for (size_t i = 0; i < 12; i++)
          {
            if (i == 0 || i == 1 || i == 10 || i == 11)
            {
              encorder_zero_up_limit_2[i] = tension_segment_1;
            } else {
              encorder_zero_up_limit_2[i] = 100;
            }
            encorder_zero_down_limit_2[i] = encorder_zero_up_limit_2[i] - 100;
          } 
        }
        if (running_flag_2 == 4)
        {
          for (size_t i = 0; i < 12; i++)
          {
            if (i == 0 || i == 1 || i == 10 || i == 11)
            {
              encorder_zero_up_limit_2[i] = tension_segment_1;
            } else if(i == 2 || i == 3 || i == 8 || i == 9) {
              encorder_zero_up_limit_2[i] = tension_segment_2;
            } else {
              encorder_zero_up_limit_2[i] = 100;
            }
            encorder_zero_down_limit_2[i] = encorder_zero_up_limit_2[i] - 100;
          } 
        }
        if (running_flag_2 == 8)
        {
          for (size_t i = 0; i < 12; i++)
          {
            if (i == 0 || i == 1 || i == 10 || i == 11)
            {
              encorder_zero_up_limit_2[i] = tension_segment_1;
            } else if(i == 2 || i == 3 || i == 8 || i == 9) {
              encorder_zero_up_limit_2[i] = tension_segment_2;
            } else if(i == 4 || i == 5 || i == 6 || i == 7) {
              encorder_zero_up_limit_2[i] = tension_segment_3;
            }
            encorder_zero_down_limit_2[i] = encorder_zero_up_limit_2[i] - 100;
          } 
        }
      }

      // mode 12，每根绳子都预紧到设定范围内
      if(msg->robomaster_mode == 12){
        for (size_t i = 0; i < 12; i++)
        {
          encorder_zero_down_limit_2[i] = 10;
          encorder_zero_up_limit_2[i] = 20;
        } 
      }

      // 如果正常收到反馈信号
      if (received_sensors_robomaster_2 && received_sensors_pull_push_2) {
        for (size_t i = 0; i < 12; i++)
        {
          int16_t delta;       
          // forward
          if(last_sensors_pull_push_data_2.pull_push_sensors[i] < encorder_zero_down_limit_2[i]){
            if(abs(last_sensors_pull_push_data_2.pull_push_sensors[i] - encorder_zero_down_limit_2[i]) > error_threshold){
              delta = coarse_delta;
            } else{
              delta = fine_delta;
            }
            if((msg->snake_position_control_array[i] - last_sensors_robomaster_data_2.snake_motor_encorder_position[i]) < delta){
              msg->snake_position_control_array[i] = last_sensors_robomaster_data_2.snake_motor_encorder_position[i] + delta;
            }
          }
          // backward
          if(last_sensors_pull_push_data_2.pull_push_sensors[i] > encorder_zero_up_limit_2[i]){
            if(abs(last_sensors_pull_push_data_2.pull_push_sensors[i] - encorder_zero_up_limit_2[i]) > error_threshold){
              delta = coarse_delta;
            } else{
              delta = fine_delta;
            }
            if((msg->snake_position_control_array[i] - last_sensors_robomaster_data_2.snake_motor_encorder_position[i]) > -delta){
              msg->snake_position_control_array[i] = last_sensors_robomaster_data_2.snake_motor_encorder_position[i] - delta;
            }
          }
        }

        // running flag为12的时候，且mode 2，让连续体机器人停止运动
        if (running_flag_2 == 12 && msg->robomaster_mode == 2) {         
          msg->robomaster_mode = 0;
          received_sensors_robomaster_2 = false;
          received_sensors_pull_push_2 = false;
        }

        // 未触发保护机制，通过力反馈矫正速度并发送指令
        if (auto_lock == 0) 
        { 
          for (size_t i = 0; i < 12; i++) {                
            if (i == 0 || i == 1 || i == 10 || i == 11) {            
                if (abs(last_sensors_pull_push_data_2.pull_push_sensors[i] - mean_tension_segment_2_1) > error_tension) {
                    msg->snake_speed_control_array[i] = average_speed + calculateAdjustedSpeed(last_sensors_pull_push_data_2.pull_push_sensors[i], mean_tension_segment_2_1, error_tension, average_speed, last_sensors_robomaster_data_2.snake_motor_encorder_speed[i]);
                }
            } else if(i == 2 || i == 3 || i == 8 || i == 9) {              
                if (abs(last_sensors_pull_push_data_2.pull_push_sensors[i] - mean_tension_segment_2_2) > error_tension) {
                    msg->snake_speed_control_array[i] = average_speed + calculateAdjustedSpeed(last_sensors_pull_push_data_2.pull_push_sensors[i], mean_tension_segment_2_2, error_tension, average_speed, last_sensors_robomaster_data_2.snake_motor_encorder_speed[i]);
                }
            } else if(i == 4 || i == 5 || i == 6 || i == 7) {
                if (abs(last_sensors_pull_push_data_2.pull_push_sensors[i] - mean_tension_segment_2_3) > error_tension) {
                    msg->snake_speed_control_array[i] = average_speed + calculateAdjustedSpeed(last_sensors_pull_push_data_2.pull_push_sensors[i], mean_tension_segment_2_3, error_tension, average_speed, last_sensors_robomaster_data_2.snake_motor_encorder_speed[i]);
                }
            }
          }
          // 然后发布新的控制消息
          std::this_thread::sleep_for(std::chrono::milliseconds(100)); // sleep for 100ms, being too fast will cause the process errors
          publisher_slave_control_topic_->publish(*msg);
        }
      }
    } 

    // mode 3
    if(msg->robomaster_mode == 3){ // Mode 3, Release, robomaster 2
      for (size_t i = 0; i < 12; i++)
      {
        if(last_sensors_pull_push_data_2.pull_push_sensors[i] < 10){
          msg-> snake_position_control_array[i]  = last_sensors_robomaster_data_2.snake_motor_encorder_position[i];
          msg-> snake_speed_control_array[i] = 0;
        } 
        if(last_sensors_pull_push_data_2.pull_push_sensors[i] >= 10) {
          msg-> snake_position_control_array[i]  = last_sensors_robomaster_data_2.snake_motor_encorder_position[i] - 200000;
          msg-> snake_speed_control_array[i] = 20;
        }
      }
      if (auto_lock == 0)
      {
        // 发布消息
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // sleep for 100ms, being too fast will cause the process errors
        publisher_slave_control_topic_->publish(*msg);
      }
    }

    // mode 5
    if(msg->robomaster_mode == 5){ // Mode 5, Omega7 joystick
        std::array<double, 6> theta_2; // right hand omega7 
        std::array<double, 6> theta_initial = {0,0,0,0,0,0};
        for (size_t i = 6; i < 12; i++)
        {
            theta_2[i-6] = last_joint_space_data.data[i];
        }
        std::array<double, 12> rope_2 = theta2rope(theta_2);
        std::array<double, 12> rope_initial = theta2rope(theta_initial);
        for (size_t i = 0; i < 12; i++)
        {
          rope_2[i] = rope_2[i] / (1 - last_sensors_pull_push_data_2.pull_push_sensors[i] * elastic_deformation); // 补偿上张力造成的误差
          rope_initial[i] = rope_initial[i] / (1 - encorder_zero_down_limit_2[i] * elastic_deformation); // 补偿上张力造成的误差
          msg-> snake_position_control_array[i]  = (rope_initial[i] - rope_2[i]) * 65536/4;
        }

        if (auto_lock == 0) {
          for (size_t i = 0; i < 12; i++) {                
            if (i == 0 || i == 1 || i == 10 || i == 11) {            
                if (abs(last_sensors_pull_push_data_2.pull_push_sensors[i] - mean_tension_segment_2_1) > error_tension) {
                    msg->snake_speed_control_array[i] = average_speed + calculateAdjustedSpeed(last_sensors_pull_push_data_2.pull_push_sensors[i], mean_tension_segment_2_1, error_tension, average_speed, last_sensors_robomaster_data_2.snake_motor_encorder_speed[i]);
                }
            } else if(i == 2 || i == 3 || i == 8 || i == 9) {              
                if (abs(last_sensors_pull_push_data_2.pull_push_sensors[i] - mean_tension_segment_2_2) > error_tension) {
                    msg->snake_speed_control_array[i] = average_speed + calculateAdjustedSpeed(last_sensors_pull_push_data_2.pull_push_sensors[i], mean_tension_segment_2_2, error_tension, average_speed, last_sensors_robomaster_data_2.snake_motor_encorder_speed[i]);
                }
            } else if(i == 4 || i == 5 || i == 6 || i == 7) {
                if (abs(last_sensors_pull_push_data_2.pull_push_sensors[i] - mean_tension_segment_2_3) > error_tension) {
                    msg->snake_speed_control_array[i] = average_speed + calculateAdjustedSpeed(last_sensors_pull_push_data_2.pull_push_sensors[i], mean_tension_segment_2_3, error_tension, average_speed, last_sensors_robomaster_data_2.snake_motor_encorder_speed[i]);
                }
            }
          }
          // 发布消息
          std::this_thread::sleep_for(std::chrono::milliseconds(100)); // sleep for 100ms, being too fast will cause the process errors
          publisher_slave_control_topic_->publish(*msg);
        }      
    }

  }

  // 取得Robomaster传感器数据
  void sensors_robomaster_2_callback(const buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster::SharedPtr msg) {
    last_sensors_robomaster_data_2 = *msg;
    received_sensors_robomaster_2 = true;
  }

  // 取得joint_space数据
  void joint_space_callback(const std_msgs::msg::Float64MultiArray::SharedPtr theta_msg) {
    last_joint_space_data = *theta_msg;
    received_joint_space_data = true;
  }

  // 取得推拉力传感器数据，并对绳驱电机拉力做保护措施
  void sensors_pull_push_2_callback(const buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors::SharedPtr pull_push_sensors_msg) {
    last_sensors_pull_push_data_2 = *pull_push_sensors_msg;
    received_sensors_pull_push_2 = true;
    auto msg = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessageSlave>();
    for (size_t i = 0; i < 12; i++)
    {
      if (abs(pull_push_sensors_msg->pull_push_sensors[i]) > tension_limit)
      {
        msg->snake_speed_control_array = {0,0,0,0,0,0,0,0,0,0,0,0};
        msg->snake_position_control_array = {0,0,0,0,0,0,0,0,0,0,0,0};
        msg->gripper_gm6020_position = 0;
        msg->gripper_c610_position = 0;
        msg->gripper_sts3032_position = 0;
        msg->robomaster_mode = 6; // quit
        // 然后发布新的控制消息
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // sleep for 100ms
        publisher_slave_control_topic_->publish(*msg);
      }
    }
  }

  rclcpp::Subscription<buaa_rescue_robot_msgs::msg::ControlMessageSlave>::SharedPtr subscriber_slave_control_topic;
  rclcpp::Subscription<buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster>::SharedPtr subscriber_sensors_robomaster_2;
  rclcpp::Subscription<buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors>::SharedPtr subscriber_sensors_pull_push_2;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_joint_space;
  rclcpp::Publisher<buaa_rescue_robot_msgs::msg::ControlMessageSlave>::SharedPtr publisher_slave_control_topic_;


  // 成员变量用于保存最新接收到的传感器数据
  buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster last_sensors_robomaster_data_2;
  buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors last_sensors_pull_push_data_2;
  std_msgs::msg::Float64MultiArray last_joint_space_data;

  // 标志变量，用于确定是否接收到了传感器数据
  bool received_sensors_robomaster_2 = false;
  bool received_sensors_pull_push_2 = false;
  bool received_joint_space_data = false;

  // 定义一个std::array类型的变量，大小为12
  std::array<int32_t, 12> encorder_zero_up_limit_2 = {0,0,0,0,0,0,0,0,0,0,0,0}; // 所有元素都将初始化为0
  std::array<int32_t, 12> encorder_zero_down_limit_2 = {0,0,0,0,0,0,0,0,0,0,0,0};  // 所有元素都将初始化为0

  int average_speed = 10;
  int32_t fine_delta = 500;
  int32_t coarse_delta = 5000;
  int32_t error_threshold = 2;
  int32_t auto_lock = 0;
  int32_t tension_limit = 2000; 
  int16_t running_flag_2 = 0;
  double elastic_deformation = 6.5e-6;
  // 设定在mode12下的各段的力的设定值
  int16_t tension_segment_1 = 1100; 
  int16_t tension_segment_2 = 800;
  int16_t tension_segment_3 = 150;
  std::array<int32_t, 12> encorder_zero_final_up_limit = {0,0,0,0,0,0,0,0,0,0,0,0};  // 所有元素都将初始化为0
  std::array<int32_t, 12> encorder_zero_final_down_limit = {0,0,0,0,0,0,0,0,0,0,0,0};  // 所有元素都将初始化为0

};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
