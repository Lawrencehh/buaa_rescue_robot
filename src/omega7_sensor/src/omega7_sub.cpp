#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <cmath> // 为了使用 fabs 和 M_PI

// 定义增量和边界
const double delta = (M_PI * 50 / 180) / 10000; // 增量
const double upper_limit = M_PI * 50 / 180; // 上限
const double lower_limit = -M_PI * 50 / 180; // 下限

class Omaga7_to_joint_space : public rclcpp::Node
{
public:
    Omaga7_to_joint_space(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "大家好，我是%s.", name.c_str());
        // 创建一个订阅者订阅话题
        omega7_subscribe_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("omega7_sensor", 10, std::bind(&Omaga7_to_joint_space::joint_space_callback, this, std::placeholders::_1));
        // 创建一个发布者发布到joint_space_topic话题
        joint_space_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_space_topic", 10);

        // 初始化 theta 数组为0
        for(int i = 0; i < 12; ++i) {
            theta[i] = 0.0;
        }
        omega7_data_upper_boundary[0] = 0.032;  omega7_data_lower_boundary[0] = -0.025;
        omega7_data_upper_boundary[1] = 0.055;  omega7_data_lower_boundary[1] = -0.053;
        omega7_data_upper_boundary[2] = 0.059;      omega7_data_lower_boundary[2] = -0.037;
        omega7_data_upper_boundary[3] = 0.923;      omega7_data_lower_boundary[3] = -1.121;
        omega7_data_upper_boundary[4] = 0.658;      omega7_data_lower_boundary[4] = -0.508;
        omega7_data_upper_boundary[5] = -0.374;      omega7_data_lower_boundary[5] = -1.192;
        omega7_data_upper_boundary[6] = -0.125;      omega7_data_lower_boundary[6] = -0.375;
        omega7_data_upper_boundary[7] = 0.034;      omega7_data_lower_boundary[7] = -0.027;
        omega7_data_upper_boundary[8] = 0.053;      omega7_data_lower_boundary[8] = -0.055;
        omega7_data_upper_boundary[9] = 0.059;      omega7_data_lower_boundary[9] = -0.035;
        omega7_data_upper_boundary[10] = 0.913;     omega7_data_lower_boundary[10] = -1.128;
        omega7_data_upper_boundary[11] = 0.438;     omega7_data_lower_boundary[11] = -0.733;
        omega7_data_upper_boundary[12] = 1.732;     omega7_data_lower_boundary[12] = 0.117;
        omega7_data_upper_boundary[13] = 0.375;     omega7_data_lower_boundary[13] = 0.125;
    }

private:
     // 声明一个订阅者
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr omega7_subscribe_;
    // 声明一个发布者
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_space_pub_;

     // 收到话题数据的回调函数
    void joint_space_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "收到:%f %f %f %f %f %f %f %f %f %f %f %f %f %f", 
        msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5],msg->data[6],
        msg->data[7],msg->data[8],msg->data[9],msg->data[10],msg->data[11],msg->data[12],msg->data[13]);

        // 判断gripper作为使能变量
        if (fabs(msg->data[6]) < 0.1)  // omega7_left
        {
            // TODO: 进行转换逻辑，转换omega7_data到theta
            // 更新 theta 值
            for(int i = 0; i <= 5; i++) 
            {
                if(msg->data[i] > omega7_data_upper_boundary[i])  // upper limit
                {
                    // 根据 omega7 的值增加或减小 theta
                    theta[i] +=  delta;                    
                }
                if(msg->data[i] < omega7_data_lower_boundary[i])  // lower limit
                {
                    // 根据 omega7 的值增加或减小 theta
                    theta[i] +=  -delta;                    
                }

                // 限制 theta 在上下限之间
                if(theta[i] > upper_limit) theta[i] = upper_limit;
                if(theta[i] < lower_limit) theta[i] = lower_limit;
            }          
        }
        
        if (fabs(msg->data[13]) < 0.1)  // omega7_left
        {
            // TODO: 进行转换逻辑，转换omega7_data到theta
            // 更新 theta 值
            for(int i = 7; i <= 12; i++) 
            {
                if(msg->data[i] > omega7_data_upper_boundary[i])  // upper limit
                {
                    // 根据 omega7 的值增加或减小 theta
                    theta[i-1] +=  delta;                    
                }
                if(msg->data[i] < omega7_data_lower_boundary[i])  // lower limit
                {
                    // 根据 omega7 的值增加或减小 theta
                    theta[i-1] +=  -delta;                    
                }

                // 限制 theta 在上下限之间
                if(theta[i-1] > upper_limit) theta[i-1] = upper_limit;
                if(theta[i-1] < lower_limit) theta[i-1] = lower_limit;
            }          
        }

        if (fabs(msg->data[6]) < 0.1 || fabs(msg->data[13]) < 0.1){  // 这里0.1是一个阈值，可以根据需要更改
            // 创建一个Float64MultiArray消息并发布
            std_msgs::msg::Float64MultiArray joint_space_msg;
            joint_space_msg.data.assign(theta, theta + 12);  // 把12维的theta数据填充到joint_space_msg
            // 发布消息
            joint_space_pub_->publish(joint_space_msg);
        }

        if (fabs(msg->data[6]) > 0.4 && fabs(msg->data[13]) > 0.4){  // reset 0
            for (size_t i = 0; i < 12; i++)
            {
                theta[i] = 0;
            }  
            // 创建一个Float64MultiArray消息并发布
            std_msgs::msg::Float64MultiArray joint_space_msg;
            joint_space_msg.data.assign(theta, theta + 12);  // 把12维的theta数据填充到joint_space_msg
            // 发布消息
            joint_space_pub_->publish(joint_space_msg);          
        }

    }

    double omega7_data_upper_boundary[14]; //上边界
    double omega7_data_lower_boundary[14]; //下边界
    double theta[12];  // 用于存储两个6维数组的转换结果
    /*
        double px_left, py_left, pz_left;  //末端执行器位置
        double angle_x_left,angle_y_left,angle_z_left;  //末端执行器姿态
        double gripper_angle_left;  //夹爪角度
        px_left_forward_limit = -0.050; // forward_limit
        px_left_backward_limit = 0.065; // backward_limit
        py_left_leftward_limit = -0.107; // leftward_limit
        py_left_rightward_limit = 0.110; // rightward_limit
        pz_left_upward_limit = 0.118; // upward_limit
        pz_left_downward_limit = -0.074; // downward_limit
        angle_x_left_counterclockwise_limit = 1.946; // counterclockwise_limit
        angle_x_left_clockwise_limit = -2.144; // clockwise_limit
        angle_y_left_counterclockwise_limit = 1.242; // counterclockwise_limit
        angle_y_left_clockwise_limit = -1.092; // clockwise_limit
        angle_z_left_counterclockwise_limit = 0.435; // counterclockwise_limit
        angle_z_left_clockwise_limit = -2.801; // clockwise_limit
        gripper_angle_left_closed_limit = -0.008; // closed_limit
        gripper_angle_left_closed_limit = -0.504; // closed_limit
    */
    /*
        double px_right, py_right, pz_right;  //末端执行器位置
        double angle_x_right,angle_y_right,angle_z_right;  //末端执行器姿态
        double gripper_angle_right;  //夹爪角度
        px_right_forward_limit = -0.054; // forward_limit
        px_right_backward_limit = 0.069; // backward_limit
        py_right_leftward_limit = -0.110; // leftward_limit
        py_right_rightward_limit = 0.106; // rightward_limit
        pz_right_upward_limit = 0.119; // upward_limit
        pz_right_downward_limit = -0.070; // downward_limit
        angle_x_right_counterclockwise_limit = 1.934; // counterclockwise_limit
        angle_x_right_clockwise_limit = -2.149; // clockwise_limit
        angle_y_right_counterclockwise_limit = 1.024; // counterclockwise_limit
        angle_y_right_clockwise_limit = -1.319; // clockwise_limit
        angle_z_right_counterclockwise_limit = 2.540; // counterclockwise_limit
        angle_z_right_clockwise_limit = -0.691; // clockwise_limit
        gripper_angle_right_closed_limit = 0.010; // closed_limit
        gripper_angle_right_closed_limit = 0.514; // closed_limit
    */
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<Omaga7_to_joint_space>("joint_space_command_pub");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}