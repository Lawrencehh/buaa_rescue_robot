# cpp_pubsub

## 简介

`cpp_pubsub`是一个基于ROS 2 (Humble版本) 的C++包。本包包含两个简单的节点：`talker`和`listener`。

- `talker`：这是一个发布者节点，每隔1秒在名为`chatter`的话题上发布一个递增的数字。
- `listener`：这是一个订阅者节点，它订阅`chatter`话题，并将收到的消息打印到控制台。

## 依赖项

- ROS 2 Humble
- rclcpp
- std_msgs

## 构建和运行

在你的ROS 2 workspace中进行以下操作来构建和运行这个包：

1. 克隆或下载此包到你的workspace的`src`目录下。
2. 在workspace的根目录下运行`colcon build`。
3. 打开一个新的终端，并执行`source /path/to/your/workspace/install/setup.bash`。
4. 运行`talker`节点：

    ```
    ros2 run cpp_pubsub talker
    ```

5. 打开另一个新的终端，并执行`source /path/to/your/workspace/install/setup.bash`。
6. 运行`listener`节点：

    ```
    ros2 run cpp_pubsub listener
    ```

你现在应该在`listener`的终端窗口中看到它接收并打印`talker`发布的消息。

## 贡献和问题

如果你有任何问题、建议或想要贡献代码，请通过GitHub issues或pull requests与我们联系。

## 许可

此包根据Apache 2.0许可发布。详情请见[LICENSE](LICENSE)文件。
