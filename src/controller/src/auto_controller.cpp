#include "rclcpp/rclcpp.hpp"
#include "buaa_rescue_robot_msgs/msg/control_message.hpp"
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
    subscriber_control_topic_1 = this->create_subscription<buaa_rescue_robot_msgs::msg::ControlMessage>(
      "control_topic", 10,
      std::bind(&AutoController::control_topic_callback, this, std::placeholders::_1));

    subscriber_sensors_robomaster_1 = this->create_subscription<buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster>(
      "Sensors_Robomaster_1", 10,
      std::bind(&AutoController::sensors_robomaster_1_callback, this, std::placeholders::_1));

    subscriber_sensors_pull_push_1 = this->create_subscription<buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors>(
      "Sensors_Pull_Push_Sensors_1", 10,
      std::bind(&AutoController::sensors_pull_push_1_callback, this, std::placeholders::_1));

    subscriber_joint_space = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "joint_space_topic", 10,
      std::bind(&AutoController::joint_space_callback, this, std::placeholders::_1));

    // 初始化发布者
    publisher_control_topic_ = this->create_publisher<buaa_rescue_robot_msgs::msg::ControlMessage>("control_topic", 10);
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
                  Gap_1[i] = sin((beta - theta[0]) / 2) * 2 * R_Gap_1[i];
              } else {
                  Gap_1[i] = sin((beta + theta[0]) / 2) * 2 * R_Gap_1[i];
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

private:
  void control_topic_callback(const buaa_rescue_robot_msgs::msg::ControlMessage::SharedPtr msg) {
    if ((msg->robomaster_1_mode == 6) || (msg->robomaster_2_mode == 6))// mode 6, to quit
    {
      auto_lock = 1;
    }

    if ((msg->robomaster_1_mode == 9) || (msg->robomaster_2_mode == 9))// mode 9, to enable
    {
      auto_lock = 0;
    }   
    // RCLCPP_INFO(this->get_logger(), "auto_lock: %d", auto_lock);

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

    if(msg->robomaster_1_mode == 2 || msg->robomaster_1_mode == 12){ // Mode 2 and mode 12
      running_flag = 0;
      for (size_t i = 0; i < 12; i++)
      {
        // stop calibration
        if ((last_sensors_pull_push_data_1.pull_push_sensors_1[i] <= encorder_zero_final_up_limit[i]) && 
        (last_sensors_pull_push_data_1.pull_push_sensors_1[i] >= encorder_zero_final_down_limit[i]))
        {
          running_flag = running_flag + 1;
        }
      }
      
      // RCLCPP_INFO(this->get_logger(), "Running_flag: %d", running_flag);

      if(msg->robomaster_1_mode == 2){
        

        if (running_flag == 0)
        {
          for (size_t i = 0; i < 12; i++)
          {
            if (i == 0 || i == 1 || i == 10 || i == 11)
            {
              encorder_zero_up_limit[i] = tension_segment_1;
            } else {
              encorder_zero_up_limit[i] = 100;
            }

            encorder_zero_down_limit[i] = encorder_zero_up_limit[i] - 100;
          } 
        }

        if (running_flag == 4)
        {
          for (size_t i = 0; i < 12; i++)
          {
            if (i == 0 || i == 1 || i == 10 || i == 11)
            {
              encorder_zero_up_limit[i] = tension_segment_1;
            } else if(i == 2 || i == 3 || i == 8 || i == 9) {
              encorder_zero_up_limit[i] = tension_segment_2;
            } else {
              encorder_zero_up_limit[i] = 100;
            }

            encorder_zero_down_limit[i] = encorder_zero_up_limit[i] - 100;
          } 
        }

        if (running_flag == 8)
        {
          for (size_t i = 0; i < 12; i++)
          {
            if (i == 0 || i == 1 || i == 10 || i == 11)
            {
              encorder_zero_up_limit[i] = tension_segment_1;
            } else if(i == 2 || i == 3 || i == 8 || i == 9) {
              encorder_zero_up_limit[i] = tension_segment_2;
            } else if(i == 4 || i == 5 || i == 6 || i == 7) {
              encorder_zero_up_limit[i] = tension_segment_3;
            }

            encorder_zero_down_limit[i] = encorder_zero_up_limit[i] - 100;
          } 
        }
      }

      if(msg->robomaster_1_mode == 12){
        for (size_t i = 0; i < 12; i++)
        {
          encorder_zero_down_limit[i] = 20;
          encorder_zero_up_limit[i] = 50;
        } 
      }
      // 操作控制消息和其他传感器数据
      // 首先确保我们已经接收到了其他传感器的数据
      if (received_sensors_robomaster_1 && received_sensors_pull_push_1) {
        // 执行处理逻辑，可能会涉及到控制消息和其他传感器数据
        
        for (size_t i = 0; i < 12; i++)
        {
          int16_t delta;       
          // forward
          if(last_sensors_pull_push_data_1.pull_push_sensors_1[i] < encorder_zero_down_limit[i]){
            if(abs(last_sensors_pull_push_data_1.pull_push_sensors_1[i] - encorder_zero_down_limit[i]) > error_threshold){
              delta = coarse_delta;
            } else{
              delta = fine_delta;
            }
            if((msg->snake_position_control_1_array[i] - last_sensors_robomaster_data_1.snake_motor_encorder_position[i]) < delta){
              msg->snake_position_control_1_array[i] = last_sensors_robomaster_data_1.snake_motor_encorder_position[i] + delta;
            }
          }
          // backward
          if(last_sensors_pull_push_data_1.pull_push_sensors_1[i] > encorder_zero_up_limit[i]){
            if(abs(last_sensors_pull_push_data_1.pull_push_sensors_1[i] - encorder_zero_up_limit[i]) > error_threshold){
              delta = coarse_delta;
            } else{
              delta = fine_delta;
            }
            if((msg->snake_position_control_1_array[i] - last_sensors_robomaster_data_1.snake_motor_encorder_position[i]) > -delta){
              msg->snake_position_control_1_array[i] = last_sensors_robomaster_data_1.snake_motor_encorder_position[i] - delta;
            }
          }
        }
        // running flag
        if (running_flag == 12 && msg->robomaster_1_mode == 2) {         
          msg->robomaster_1_mode = 0;
          msg->robomaster_2_mode = 0;
          received_sensors_robomaster_1 = false;
          received_sensors_pull_push_1 = false;
        }
        // mode 6 verified
        if (auto_lock == 0) 
        {          
          for (size_t i = 0; i < 12; i++) {
            if (last_sensors_pull_push_data_1.pull_push_sensors_1[i] > 150) {
              if (last_sensors_robomaster_data_1.snake_motor_encorder_speed[i] > 0)
              {
                msg->snake_speed_control_1_array[i] = 5;
              }
              if (last_sensors_robomaster_data_1.snake_motor_encorder_speed[i] < 0)
              {
                msg->snake_speed_control_1_array[i] = 10;
              }
            }
            if (last_sensors_pull_push_data_1.pull_push_sensors_1[i] <= 100) {
              if (last_sensors_robomaster_data_1.snake_motor_encorder_speed[i] > 0)
              {
                msg->snake_speed_control_1_array[i] = 10;
              }
              if (last_sensors_robomaster_data_1.snake_motor_encorder_speed[i] < 0)
              {
                msg->snake_speed_control_1_array[i] = 5;
              }
            }
          }
          // 然后发布新的控制消息
          std::this_thread::sleep_for(std::chrono::milliseconds(100)); // sleep for 100ms, being too fast will cause the process errors
          publisher_control_topic_->publish(*msg);
        }

      } else {
        RCLCPP_INFO(this->get_logger(), "Waiting for all sensor data before processing control message");
      }
    } 


      
    

    if(msg->robomaster_1_mode == 5){ // Mode 5, Omega7 joystick
        std::array<double, 6> theta_1; // left hand omega7 
        std::array<double, 6> theta_2; // right hand omega7 
        std::array<double, 6> theta_initial = {0,0,0,0,0,0};
        for (size_t i = 0; i < 6; i++)
        {
            theta_1[i] = last_joint_space_data.data[i];
        }
        for (size_t i = 6; i < 12; i++)
        {
            theta_2[i-6] = last_joint_space_data.data[i];
        }
        std::array<double, 12> rope_1 = theta2rope(theta_1);
        std::array<double, 12> rope_2 = theta2rope(theta_2);
        std::array<double, 12> rope_initial = theta2rope(theta_initial);

        // snake motors control for robomaster 1
        for (size_t i = 0; i < 12; i++)
        {
            msg-> snake_position_control_1_array[i]  = (rope_initial[i] - rope_1[i]) * 65536/4; // mm transformed to pulse

        }

        // snake motors control for robomaster 2
        for (size_t i = 0; i < 12; i++)
        {
            msg-> snake_position_control_2_array[i]  = (rope_initial[i] - rope_2[i]) * 65536/4;
        }

        if (auto_lock == 0) {
          for (size_t i = 0; i < 12; i++) {
            if (last_sensors_pull_push_data_1.pull_push_sensors_1[i] > 150) {
              if (last_sensors_robomaster_data_1.snake_motor_encorder_speed[i] > 0)
              {
                msg->snake_speed_control_1_array[i] = 5;
              }
              if (last_sensors_robomaster_data_1.snake_motor_encorder_speed[i] < 0)
              {
                msg->snake_speed_control_1_array[i] = 10;
              }
            }
            if (last_sensors_pull_push_data_1.pull_push_sensors_1[i] <= 100) {
              if (last_sensors_robomaster_data_1.snake_motor_encorder_speed[i] > 0)
              {
                msg->snake_speed_control_1_array[i] = 10;
              }
              if (last_sensors_robomaster_data_1.snake_motor_encorder_speed[i] < 0)
              {
                msg->snake_speed_control_1_array[i] = 5;
              }
            }
          }
          // 发布消息
          std::this_thread::sleep_for(std::chrono::milliseconds(100)); // sleep for 100ms, being too fast will cause the process errors
          publisher_control_topic_->publish(*msg);
        }      
    }

  }

  void sensors_robomaster_1_callback(const buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster::SharedPtr msg) {
    // 保存Robomaster传感器数据
    last_sensors_robomaster_data_1 = *msg;
    received_sensors_robomaster_1 = true;
  }

  void joint_space_callback(const std_msgs::msg::Float64MultiArray::SharedPtr theta_msg) {
    // 保存joint_space数据
    last_joint_space_data = *theta_msg;
    received_joint_space_data = true;
  }

  void sensors_pull_push_1_callback(const buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors::SharedPtr pull_push_sensors_msg) {
    // 保存拉推传感器数据
    last_sensors_pull_push_data_1 = *pull_push_sensors_msg;
    received_sensors_pull_push_1 = true;
    auto msg = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessage>();
    for (size_t i = 0; i < 12; i++)
    {
      if (abs(pull_push_sensors_msg->pull_push_sensors_1[i]) > tension_limit)
      {
        msg->snake_speed_control_1_array = {0,0,0,0,0,0,0,0,0,0,0,0};
        msg->snake_position_control_1_array = {0,0,0,0,0,0,0,0,0,0,0,0};
        msg->gripper_gm6020_position_1 = 0;
        msg->gripper_c610_position_1 = 0;
        msg->gripper_sts3032_position_1 = 0;

        msg->snake_speed_control_2_array = {0,0,0,0,0,0,0,0,0,0,0,0};
        msg->snake_position_control_2_array = {0,0,0,0,0,0,0,0,0,0,0,0};
        msg->gripper_gm6020_position_2 = 0;
        msg->gripper_c610_position_2 = 0;
        msg->gripper_sts3032_position_2 = 0;

        msg->elevator_control = 0;
        msg->lower_linear_module_control = 0;
        msg->upper_linear_module_control = 0;
    

        msg->pull_push_sensors_reset = 0;
        msg->elevator_counter_reset = 0; // reset to be 0
        msg->lower_linear_module_encorder_reset = 0; // reset to be 0
        msg->upper_linear_module_encorder_reset = 0; // reset to be 0
        msg->robomaster_1_mode = 6; // quit
        msg->robomaster_2_mode = 6; // quit
        // 然后发布新的控制消息
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // sleep for 100ms
        publisher_control_topic_->publish(*msg);
      }
      
      
    }
    
  }

  rclcpp::Subscription<buaa_rescue_robot_msgs::msg::ControlMessage>::SharedPtr subscriber_control_topic_1;
  rclcpp::Subscription<buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster>::SharedPtr subscriber_sensors_robomaster_1;
  rclcpp::Subscription<buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors>::SharedPtr subscriber_sensors_pull_push_1;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_joint_space;
  rclcpp::Publisher<buaa_rescue_robot_msgs::msg::ControlMessage>::SharedPtr publisher_control_topic_;


  // 成员变量用于保存最新接收到的传感器数据
  buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster last_sensors_robomaster_data_1;
  buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors last_sensors_pull_push_data_1;
  std_msgs::msg::Float64MultiArray last_joint_space_data;

  // 标志变量，用于确定是否接收到了传感器数据
  bool received_sensors_robomaster_1 = false;
  bool received_sensors_pull_push_1 = false;
  bool received_joint_space_data = false;

  // 定义一个std::array类型的变量，大小为12
  std::array<int32_t, 12> encorder_zero_up_limit = {0,0,0,0,0,0,0,0,0,0,0,0}; // 所有元素都将初始化为0
  std::array<int32_t, 12> encorder_zero_down_limit = {0,0,0,0,0,0,0,0,0,0,0,0};  // 所有元素都将初始化为0

  int32_t fine_delta = 500;
  int32_t coarse_delta = 5000;
  int32_t error_threshold = 2;
  int32_t auto_lock = 0;
  int32_t tension_limit = 2000; 
  int16_t running_flag = 0;
  int16_t tension_segment_1 = 800;
  int16_t tension_segment_2 = 800;
  int16_t tension_segment_3 = 800;
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
