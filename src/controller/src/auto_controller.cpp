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
      "slave_control_topic", 10,
      std::bind(&AutoController::slave_control_topic_callback, this, std::placeholders::_1));

    subscriber_sensors_robomaster_1 = this->create_subscription<buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster>(
      "Sensors_Robomaster_1", 10,
      std::bind(&AutoController::sensors_robomaster_1_callback, this, std::placeholders::_1));

    subscriber_sensors_robomaster_2 = this->create_subscription<buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster>(
      "Sensors_Robomaster_2", 10,
      std::bind(&AutoController::sensors_robomaster_2_callback, this, std::placeholders::_1));

    subscriber_sensors_pull_push_1 = this->create_subscription<buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors>(
      "Sensors_Pull_Push_Sensors_1", 10,
      std::bind(&AutoController::sensors_pull_push_1_callback, this, std::placeholders::_1));

    subscriber_sensors_pull_push_2 = this->create_subscription<buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors>(
      "Sensors_Pull_Push_Sensors_2", 10,
      std::bind(&AutoController::sensors_pull_push_2_callback, this, std::placeholders::_1));

    subscriber_joint_space = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "joint_space_topic", 10,
      std::bind(&AutoController::joint_space_callback, this, std::placeholders::_1));

    // 初始化发布者
    publisher_slave_control_topic_ = this->create_publisher<buaa_rescue_robot_msgs::msg::ControlMessageSlave>("slave_control_topic", 10);

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
  void slave_control_topic_callback(const buaa_rescue_robot_msgs::msg::ControlMessageSlave::SharedPtr msg) {
    if ((msg->robomaster_1_mode == 6) || (msg->robomaster_2_mode == 6))// mode 6, to quit
    {
      auto_lock = 1;
    }

    if ((msg->robomaster_1_mode == 9) || (msg->robomaster_2_mode == 9))// mode 9, to enable
    {
      auto_lock = 0;
    }   
    // RCLCPP_INFO(this->get_logger(), "auto_lock: %d", auto_lock);

    int32_t mean_tension_segment_1_1 = (last_sensors_pull_push_data_1.pull_push_sensors[0] + last_sensors_pull_push_data_1.pull_push_sensors[1]
    + last_sensors_pull_push_data_1.pull_push_sensors[10] + last_sensors_pull_push_data_1.pull_push_sensors[11]) / 4;
    int32_t mean_tension_segment_1_2 = (last_sensors_pull_push_data_1.pull_push_sensors[2] + last_sensors_pull_push_data_1.pull_push_sensors[3]
    + last_sensors_pull_push_data_1.pull_push_sensors[8] + last_sensors_pull_push_data_1.pull_push_sensors[9]) / 4;
    int32_t mean_tension_segment_1_3 = (last_sensors_pull_push_data_1.pull_push_sensors[4] + last_sensors_pull_push_data_1.pull_push_sensors[5]
    + last_sensors_pull_push_data_1.pull_push_sensors[6] + last_sensors_pull_push_data_1.pull_push_sensors[7]) / 4;
    int32_t error_tension = 5;

    int32_t mean_tension_segment_2_1 = (last_sensors_pull_push_data_2.pull_push_sensors[0] + last_sensors_pull_push_data_2.pull_push_sensors[1]
    + last_sensors_pull_push_data_2.pull_push_sensors[10] + last_sensors_pull_push_data_2.pull_push_sensors[11]) / 4;
    int32_t mean_tension_segment_2_2 = (last_sensors_pull_push_data_2.pull_push_sensors[2] + last_sensors_pull_push_data_2.pull_push_sensors[3]
    + last_sensors_pull_push_data_2.pull_push_sensors[8] + last_sensors_pull_push_data_2.pull_push_sensors[9]) / 4;
    int32_t mean_tension_segment_2_3 = (last_sensors_pull_push_data_2.pull_push_sensors[4] + last_sensors_pull_push_data_2.pull_push_sensors[5]
    + last_sensors_pull_push_data_2.pull_push_sensors[6] + last_sensors_pull_push_data_2.pull_push_sensors[7]) / 4;
    
    // robomaster 1
    if(msg->robomaster_1_mode == 2 || msg->robomaster_1_mode == 12){ // Mode 2 and mode 12
      running_flag_1 = 0;
      // robomaster 1
      for (size_t i = 0; i < 12; i++)
      {
        // stop calibration
        if ((last_sensors_pull_push_data_1.pull_push_sensors[i] <= encorder_zero_final_up_limit[i]) && 
        (last_sensors_pull_push_data_1.pull_push_sensors[i] >= encorder_zero_final_down_limit[i]))
        {
          running_flag_1 = running_flag_1 + 1;
        }
      }

      // robomaster 1
      if(msg->robomaster_1_mode == 2){
        if (running_flag_1 == 0)
        {
          for (size_t i = 0; i < 12; i++)
          {
            if (i == 0 || i == 1 || i == 10 || i == 11)
            {
              encorder_zero_up_limit_1[i] = tension_segment_1;
            } else {
              encorder_zero_up_limit_1[i] = 100;
            }
            encorder_zero_down_limit_1[i] = encorder_zero_up_limit_1[i] - 100;
          } 
        }
        if (running_flag_1 == 4)
        {
          for (size_t i = 0; i < 12; i++)
          {
            if (i == 0 || i == 1 || i == 10 || i == 11)
            {
              encorder_zero_up_limit_1[i] = tension_segment_1;
            } else if(i == 2 || i == 3 || i == 8 || i == 9) {
              encorder_zero_up_limit_1[i] = tension_segment_2;
            } else {
              encorder_zero_up_limit_1[i] = 100;
            }
            encorder_zero_down_limit_1[i] = encorder_zero_up_limit_1[i] - 100;
          } 
        }
        if (running_flag_1 == 8)
        {
          for (size_t i = 0; i < 12; i++)
          {
            if (i == 0 || i == 1 || i == 10 || i == 11)
            {
              encorder_zero_up_limit_1[i] = tension_segment_1;
            } else if(i == 2 || i == 3 || i == 8 || i == 9) {
              encorder_zero_up_limit_1[i] = tension_segment_2;
            } else if(i == 4 || i == 5 || i == 6 || i == 7) {
              encorder_zero_up_limit_1[i] = tension_segment_3;
            }
            encorder_zero_down_limit_1[i] = encorder_zero_up_limit_1[i] - 100;
          } 
        }
      }

      // robomaster 1
      if(msg->robomaster_1_mode == 12){
        for (size_t i = 0; i < 12; i++)
        {
          encorder_zero_down_limit_1[i] = 10;
          encorder_zero_up_limit_1[i] = 20;
        } 
      }

      // robomaster 1
      if (received_sensors_robomaster_1 && received_sensors_pull_push_1) {
        for (size_t i = 0; i < 12; i++)
        {
          int16_t delta;       
          // forward
          if(last_sensors_pull_push_data_1.pull_push_sensors[i] < encorder_zero_down_limit_1[i]){
            if(abs(last_sensors_pull_push_data_1.pull_push_sensors[i] - encorder_zero_down_limit_1[i]) > error_threshold){
              delta = coarse_delta;
            } else{
              delta = fine_delta;
            }
            if((msg->snake_position_control_1_array[i] - last_sensors_robomaster_data_1.snake_motor_encorder_position[i]) < delta){
              msg->snake_position_control_1_array[i] = last_sensors_robomaster_data_1.snake_motor_encorder_position[i] + delta;
            }
          }
          // backward
          if(last_sensors_pull_push_data_1.pull_push_sensors[i] > encorder_zero_up_limit_1[i]){
            if(abs(last_sensors_pull_push_data_1.pull_push_sensors[i] - encorder_zero_up_limit_1[i]) > error_threshold){
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
        if (running_flag_1 == 12 && msg->robomaster_1_mode == 2) {         
          msg->robomaster_1_mode = 0;
          received_sensors_robomaster_1 = false;
          received_sensors_pull_push_1 = false;
        }
        // mode 6 verified
        if (auto_lock == 0) 
        { 

          for (size_t i = 0; i < 12; i++)
          {                
            if (i == 0 || i == 1 || i == 10 || i == 11)
            {            
              if (abs(last_sensors_pull_push_data_1.pull_push_sensors[i] - mean_tension_segment_1_1) > error_tension)
              {
                int32_t adjust_speed = (mean_tension_segment_1_1 - last_sensors_pull_push_data_1.pull_push_sensors[i]) / error_tension;
                if (adjust_speed > 5)
                {
                  adjust_speed = 5;
                }
                if (adjust_speed < -5)
                {
                  adjust_speed = -5;
                }
                if (last_sensors_robomaster_data_1.snake_motor_encorder_speed[i] < 0)
                {
                  adjust_speed = -adjust_speed;
                }
                
                msg->snake_speed_control_1_array[i] = 10 + adjust_speed;
              }
            } 
            if(i == 2 || i == 3 || i == 8 || i == 9) 
            {
              if (abs(last_sensors_pull_push_data_1.pull_push_sensors[i] - mean_tension_segment_1_2) > error_tension)
              {
                int32_t adjust_speed = (mean_tension_segment_1_2 - last_sensors_pull_push_data_1.pull_push_sensors[i]) / error_tension;
                if (adjust_speed > 5)
                {
                  adjust_speed = 5;
                }
                if (adjust_speed < -5)
                {
                  adjust_speed = -5;
                }
                if (last_sensors_robomaster_data_1.snake_motor_encorder_speed[i] < 0)
                {
                  adjust_speed = -adjust_speed;
                }
                msg->snake_speed_control_1_array[i] = 10 + adjust_speed;
              }
            } 
            if(i == 4 || i == 5 || i == 6 || i == 7) 
            {
              if (abs(last_sensors_pull_push_data_1.pull_push_sensors[i] - mean_tension_segment_1_3) > error_tension)
              {
                int32_t adjust_speed = (mean_tension_segment_1_3 - last_sensors_pull_push_data_1.pull_push_sensors[i]) / error_tension;
                if (adjust_speed > 5)
                {
                  adjust_speed = 5;
                }
                if (adjust_speed < -5)
                {
                  adjust_speed = -5;
                }
                if (last_sensors_robomaster_data_1.snake_motor_encorder_speed[i] < 0)
                {
                  adjust_speed = -adjust_speed;
                }
                msg->snake_speed_control_1_array[i] = 10 + adjust_speed;
              }
            }
          }

          // 然后发布新的控制消息
          std::this_thread::sleep_for(std::chrono::milliseconds(100)); // sleep for 100ms, being too fast will cause the process errors
          publisher_slave_control_topic_->publish(*msg);
        }
      } 
    }

    // robomaster 2
    if(msg->robomaster_2_mode == 2 || msg->robomaster_2_mode == 12){ // Mode 2 and mode 12
      running_flag_2 = 0;
      // robomaster 2
      for (size_t i = 0; i < 12; i++)
      {
        // stop calibration
        if ((last_sensors_pull_push_data_2.pull_push_sensors[i] <= encorder_zero_final_up_limit[i]) && 
        (last_sensors_pull_push_data_2.pull_push_sensors[i] >= encorder_zero_final_down_limit[i]))
        {
          running_flag_2 = running_flag_2 + 1;
        }
      }
      
      // robomaster 2
      if(msg->robomaster_2_mode == 2){
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

      // robomaster 2
      if(msg->robomaster_2_mode == 12){
        for (size_t i = 0; i < 12; i++)
        {
          encorder_zero_down_limit_2[i] = 10;
          encorder_zero_up_limit_2[i] = 20;
        } 
      }

      // robomaster 2
      if (received_sensors_robomaster_2 && received_sensors_pull_push_2) {
        // RCLCPP_INFO(this->get_logger(), "running_flag_2: %d", running_flag_2);
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
            if((msg->snake_position_control_2_array[i] - last_sensors_robomaster_data_2.snake_motor_encorder_position[i]) < delta){
              msg->snake_position_control_2_array[i] = last_sensors_robomaster_data_2.snake_motor_encorder_position[i] + delta;
            }
          }
          // backward
          if(last_sensors_pull_push_data_2.pull_push_sensors[i] > encorder_zero_up_limit_2[i]){
            if(abs(last_sensors_pull_push_data_2.pull_push_sensors[i] - encorder_zero_up_limit_2[i]) > error_threshold){
              delta = coarse_delta;
            } else{
              delta = fine_delta;
            }
            if((msg->snake_position_control_2_array[i] - last_sensors_robomaster_data_2.snake_motor_encorder_position[i]) > -delta){
              msg->snake_position_control_2_array[i] = last_sensors_robomaster_data_2.snake_motor_encorder_position[i] - delta;
            }
          }
        }
        // running flag
        if (running_flag_2 == 12 && msg->robomaster_2_mode == 2) {         
          msg->robomaster_2_mode = 0;
          received_sensors_robomaster_2 = false;
          received_sensors_pull_push_2 = false;
        }
        // mode 6 verified
        if (auto_lock == 0) 
        { 

          for (size_t i = 0; i < 12; i++)
          {                
            if (i == 0 || i == 1 || i == 10 || i == 11)
            {            
              if (abs(last_sensors_pull_push_data_2.pull_push_sensors[i] - mean_tension_segment_2_1) > error_tension)
              {
                int32_t adjust_speed = (mean_tension_segment_2_1 - last_sensors_pull_push_data_2.pull_push_sensors[i]) / error_tension;
                if (adjust_speed > 5)
                {
                  adjust_speed = 5;
                }
                if (adjust_speed < -5)
                {
                  adjust_speed = -5;
                }
                if (last_sensors_robomaster_data_2.snake_motor_encorder_speed[i] < 0)
                {
                  adjust_speed = -adjust_speed;
                }
                
                msg->snake_speed_control_2_array[i] = 10 + adjust_speed;
              }
            } 
            if(i == 2 || i == 3 || i == 8 || i == 9) 
            {
              if (abs(last_sensors_pull_push_data_2.pull_push_sensors[i] - mean_tension_segment_2_2) > error_tension)
              {
                int32_t adjust_speed = (mean_tension_segment_2_2 - last_sensors_pull_push_data_2.pull_push_sensors[i]) / error_tension;
                if (adjust_speed > 5)
                {
                  adjust_speed = 5;
                }
                if (adjust_speed < -5)
                {
                  adjust_speed = -5;
                }
                if (last_sensors_robomaster_data_2.snake_motor_encorder_speed[i] < 0)
                {
                  adjust_speed = -adjust_speed;
                }
                msg->snake_speed_control_2_array[i] = 10 + adjust_speed;
              }
            } 
            if(i == 4 || i == 5 || i == 6 || i == 7) 
            {
              if (abs(last_sensors_pull_push_data_2.pull_push_sensors[i] - mean_tension_segment_2_3) > error_tension)
              {
                int32_t adjust_speed = (mean_tension_segment_2_3 - last_sensors_pull_push_data_2.pull_push_sensors[i]) / error_tension;
                if (adjust_speed > 5)
                {
                  adjust_speed = 5;
                }
                if (adjust_speed < -5)
                {
                  adjust_speed = -5;
                }
                if (last_sensors_robomaster_data_2.snake_motor_encorder_speed[i] < 0)
                {
                  adjust_speed = -adjust_speed;
                }
                msg->snake_speed_control_2_array[i] = 10 + adjust_speed;
              }
            }
          }

          // 然后发布新的控制消息
          std::this_thread::sleep_for(std::chrono::milliseconds(100)); // sleep for 100ms, being too fast will cause the process errors
          publisher_slave_control_topic_->publish(*msg);
        }
      }
    } 

    // robomaster 1
    if(msg->robomaster_1_mode == 3 || msg->robomaster_2_mode == 3){ // Mode 3, Release, robomaster 1
      for (size_t i = 0; i < 12; i++)
      {
        if(last_sensors_pull_push_data_1.pull_push_sensors[i] < 5){
          msg-> snake_position_control_1_array[i]  = last_sensors_robomaster_data_1.snake_motor_encorder_position[i];
          msg-> snake_speed_control_1_array[i] = 0;
        } 
        if(last_sensors_pull_push_data_1.pull_push_sensors[i] > 5) {
          msg-> snake_position_control_1_array[i]  = last_sensors_robomaster_data_1.snake_motor_encorder_position[i] - 200000;
          msg-> snake_speed_control_1_array[i] = 10;
        }
        if(last_sensors_pull_push_data_2.pull_push_sensors[i] < 5){
          msg-> snake_position_control_2_array[i]  = last_sensors_robomaster_data_2.snake_motor_encorder_position[i];
          msg-> snake_speed_control_2_array[i] = 0;
        } 
        if(last_sensors_pull_push_data_2.pull_push_sensors[i] > 5) {
          msg-> snake_position_control_2_array[i]  = last_sensors_robomaster_data_2.snake_motor_encorder_position[i] - 200000;
          msg-> snake_speed_control_2_array[i] = 10;
        }
      }
      if (auto_lock == 0)
      {
        // 发布消息
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // sleep for 100ms, being too fast will cause the process errors
        publisher_slave_control_topic_->publish(*msg);
      }
    }

    // robomaster 1 and robomaster 2
    if(msg->robomaster_1_mode == 5 || msg->robomaster_2_mode == 5){ // Mode 5, Omega7 joystick
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

        // snake motors control for robomaster 
        for (size_t i = 0; i < 12; i++)
        {
            msg-> snake_position_control_1_array[i]  = (rope_initial[i] - rope_1[i]) * 65536/4; // mm transformed to pulse
            msg-> snake_position_control_2_array[i]  = (rope_initial[i] - rope_2[i]) * 65536/4;

        }

        if (auto_lock == 0) {
          for (size_t i = 0; i < 12; i++)
          {                
            if (i == 0 || i == 1 || i == 10 || i == 11)
            {            
              if (abs(last_sensors_pull_push_data_1.pull_push_sensors[i] - mean_tension_segment_1_1) > error_tension)
              {
                int32_t adjust_speed = (mean_tension_segment_1_1 - last_sensors_pull_push_data_1.pull_push_sensors[i]) / error_tension;
                if (adjust_speed > 5)
                {
                  adjust_speed = 5;
                }
                if (adjust_speed < -5)
                {
                  adjust_speed = -5;
                }
                if (last_sensors_robomaster_data_1.snake_motor_encorder_speed[i] < 0)
                {
                  adjust_speed = -adjust_speed;
                }
                msg->snake_speed_control_1_array[i] = 10 + adjust_speed;
              }
              if (abs(last_sensors_pull_push_data_2.pull_push_sensors[i] - mean_tension_segment_2_1) > error_tension)
              {
                int32_t adjust_speed = (mean_tension_segment_2_1 - last_sensors_pull_push_data_2.pull_push_sensors[i]) / error_tension;
                if (adjust_speed > 5)
                {
                  adjust_speed = 5;
                }
                if (adjust_speed < -5)
                {
                  adjust_speed = -5;
                }
                if (last_sensors_robomaster_data_2.snake_motor_encorder_speed[i] < 0)
                {
                  adjust_speed = -adjust_speed;
                }
                msg->snake_speed_control_2_array[i] = 10 + adjust_speed;
              }
            } 
            if(i == 2 || i == 3 || i == 8 || i == 9) 
            {
              if (abs(last_sensors_pull_push_data_1.pull_push_sensors[i] - mean_tension_segment_1_2) > error_tension)
              {
                int32_t adjust_speed = (mean_tension_segment_1_2 - last_sensors_pull_push_data_1.pull_push_sensors[i]) / error_tension;
                if (adjust_speed > 5)
                {
                  adjust_speed = 5;
                }
                if (adjust_speed < -5)
                {
                  adjust_speed = -5;
                }
                if (last_sensors_robomaster_data_1.snake_motor_encorder_speed[i] < 0)
                {
                  adjust_speed = -adjust_speed;
                }
                msg->snake_speed_control_1_array[i] = 10 + adjust_speed;
              }
              if (abs(last_sensors_pull_push_data_2.pull_push_sensors[i] - mean_tension_segment_2_2) > error_tension)
              {
                int32_t adjust_speed = (mean_tension_segment_2_2 - last_sensors_pull_push_data_2.pull_push_sensors[i]) / error_tension;
                if (adjust_speed > 5)
                {
                  adjust_speed = 5;
                }
                if (adjust_speed < -5)
                {
                  adjust_speed = -5;
                }
                if (last_sensors_robomaster_data_2.snake_motor_encorder_speed[i] < 0)
                {
                  adjust_speed = -adjust_speed;
                }
                msg->snake_speed_control_2_array[i] = 10 + adjust_speed;
              }
            } 
            if(i == 4 || i == 5 || i == 6 || i == 7) 
            {
              if (abs(last_sensors_pull_push_data_1.pull_push_sensors[i] - mean_tension_segment_1_3) > error_tension)
              {
                int32_t adjust_speed = (mean_tension_segment_1_3 - last_sensors_pull_push_data_1.pull_push_sensors[i]) / error_tension;
                if (adjust_speed > 5)
                {
                  adjust_speed = 5;
                }
                if (adjust_speed < -5)
                {
                  adjust_speed = -5;
                }
                if (last_sensors_robomaster_data_1.snake_motor_encorder_speed[i] < 0)
                {
                  adjust_speed = -adjust_speed;
                }
                msg->snake_speed_control_1_array[i] = 10 + adjust_speed;
              }
              if (abs(last_sensors_pull_push_data_2.pull_push_sensors[i] - mean_tension_segment_2_3) > error_tension)
              {
                int32_t adjust_speed = (mean_tension_segment_2_3 - last_sensors_pull_push_data_2.pull_push_sensors[i]) / error_tension;
                if (adjust_speed > 5)
                {
                  adjust_speed = 5;
                }
                if (adjust_speed < -5)
                {
                  adjust_speed = -5;
                }
                if (last_sensors_robomaster_data_2.snake_motor_encorder_speed[i] < 0)
                {
                  adjust_speed = -adjust_speed;
                }
                msg->snake_speed_control_2_array[i] = 10 + adjust_speed;
              }
            }
          }
          // 发布消息
          std::this_thread::sleep_for(std::chrono::milliseconds(100)); // sleep for 100ms, being too fast will cause the process errors
          publisher_slave_control_topic_->publish(*msg);
        }      
    }

  }

  void sensors_robomaster_1_callback(const buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster::SharedPtr msg) {
    // 保存Robomaster传感器数据
    last_sensors_robomaster_data_1 = *msg;
    received_sensors_robomaster_1 = true;
  }

  void sensors_robomaster_2_callback(const buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster::SharedPtr msg) {
    // 保存Robomaster传感器数据
    last_sensors_robomaster_data_2 = *msg;
    received_sensors_robomaster_2 = true;
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
    auto msg = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessageSlave>();
    for (size_t i = 0; i < 12; i++)
    {
      if (abs(pull_push_sensors_msg->pull_push_sensors[i]) > tension_limit)
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

        msg->robomaster_1_mode = 6; // quit
        msg->robomaster_2_mode = 6; // quit
        // 然后发布新的控制消息
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // sleep for 100ms
        publisher_slave_control_topic_->publish(*msg);
      }
    }
  }

  void sensors_pull_push_2_callback(const buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors::SharedPtr pull_push_sensors_msg) {
    // 保存拉推传感器数据
    last_sensors_pull_push_data_2 = *pull_push_sensors_msg;
    received_sensors_pull_push_2 = true;
    auto msg = std::make_shared<buaa_rescue_robot_msgs::msg::ControlMessageSlave>();
    for (size_t i = 0; i < 12; i++)
    {
      if (abs(pull_push_sensors_msg->pull_push_sensors[i]) > tension_limit)
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

        msg->robomaster_1_mode = 6; // quit
        msg->robomaster_2_mode = 6; // quit
        // 然后发布新的控制消息
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // sleep for 100ms
        publisher_slave_control_topic_->publish(*msg);
      }
    }
  }

  rclcpp::Subscription<buaa_rescue_robot_msgs::msg::ControlMessageSlave>::SharedPtr subscriber_slave_control_topic;
  rclcpp::Subscription<buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster>::SharedPtr subscriber_sensors_robomaster_1;
  rclcpp::Subscription<buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors>::SharedPtr subscriber_sensors_pull_push_1;
  rclcpp::Subscription<buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster>::SharedPtr subscriber_sensors_robomaster_2;
  rclcpp::Subscription<buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors>::SharedPtr subscriber_sensors_pull_push_2;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_joint_space;
  rclcpp::Publisher<buaa_rescue_robot_msgs::msg::ControlMessageSlave>::SharedPtr publisher_slave_control_topic_;


  // 成员变量用于保存最新接收到的传感器数据
  buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster last_sensors_robomaster_data_1;
  buaa_rescue_robot_msgs::msg::SensorsMessageRobomaster last_sensors_robomaster_data_2;
  buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors last_sensors_pull_push_data_1;
  buaa_rescue_robot_msgs::msg::SensorsMessageMasterDevicePullPushSensors last_sensors_pull_push_data_2;
  std_msgs::msg::Float64MultiArray last_joint_space_data;

  // 标志变量，用于确定是否接收到了传感器数据
  bool received_sensors_robomaster_1 = false;
  bool received_sensors_pull_push_1 = false;
  bool received_sensors_robomaster_2 = false;
  bool received_sensors_pull_push_2 = false;
  bool received_joint_space_data = false;

  // 定义一个std::array类型的变量，大小为12
  std::array<int32_t, 12> encorder_zero_up_limit_1 = {0,0,0,0,0,0,0,0,0,0,0,0}; // 所有元素都将初始化为0
  std::array<int32_t, 12> encorder_zero_down_limit_1 = {0,0,0,0,0,0,0,0,0,0,0,0};  // 所有元素都将初始化为0
  std::array<int32_t, 12> encorder_zero_up_limit_2 = {0,0,0,0,0,0,0,0,0,0,0,0}; // 所有元素都将初始化为0
  std::array<int32_t, 12> encorder_zero_down_limit_2 = {0,0,0,0,0,0,0,0,0,0,0,0};  // 所有元素都将初始化为0

  int32_t fine_delta = 500;
  int32_t coarse_delta = 5000;
  int32_t error_threshold = 2;
  int32_t auto_lock = 0;
  int32_t tension_limit = 2000; 
  int16_t running_flag_1 = 0;
  int16_t running_flag_2 = 0;
  // int16_t tension_segment_1 = 1100;
  // int16_t tension_segment_2 = 1100;
  // int16_t tension_segment_3 = 1100;
  int16_t tension_segment_1 = 1100;
  int16_t tension_segment_2 = 600;
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
