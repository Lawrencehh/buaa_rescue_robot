# serial

serial包是基于ROS 2 Humble版本的机器人控制系统，专为`buaa_rescue_robot`项目设计。它包含两个用于串口通信的ROS 2节点：`serial_sender` 和 `serial_receiver`。

## 依赖项

- ROS 2 Humble
- ASIO 库 (`libasio-dev`)

## 安装依赖项

在Ubuntu下，您可以使用以下命令安装必要的依赖项：

```bash
sudo apt-get install libasio-dev
```

# 构建项目
在您的buaa_rescue_robot工作空间中，运行以下命令以构建项目：
```bash
cd buaa_rescue_robot
colcon build
```
构建成功后，激活新构建的工作空间：
```bash
source install/setup.bash
```

# 节点介绍
## serial_sender
此节点负责向指定的串口发送字符串消息.  

使用方法
```bash
ros2 run serial serial_sender
```
## serial_receiver
此节点负责从指定的串口接收字符串消息，并在ROS 2日志中打印接收到的消息。  

使用方法
```bash
ros2 run serial serial_receiver
```

# 串口设置
默认情况下，两个节点都配置为使用 /dev/ttyUSB0 串口，并设置了以下串口参数：   


波特率: 115200   

数据位: 8   

停止位: 1   

校验: 无   

流控: 无   

请确保这些设置与您的硬件匹配，否则您需要在代码中进行适当的修改。