# ROS-LYENBOT-

公司内部的 ROS2 培训项目，包含练习与原型代码。本仓库作为 ROS2 节点与通信模式的内部实践仓库。

**ROS-LYENBOT — ROS2 工作空间配置指南**

本仓库包含基于 ROS 2 (Humble) 的训练代码和示例。
 请按照以下步骤创建一个干净的工作空间、安装依赖、编译并运行。

## 1.创建工作空间

在 ROS2 开发中，所有的功能包、脚本、参数文件等都需要集中放在一个统一的文件夹中进行管理，这个文件夹就叫做 **工作空间 (workspace)**。
 它相当于我们开发项目的大本营，所有代码和编译结果都会在这里组织。

一个典型的 ROS2 工作空间结构如下：

```
dev_ws/               # 工作空间根目录
├── src/              # 源码空间，存放功能包、脚本等
├── build/            # 编译空间，存放中间文件（自动生成）
├── install/          # 安装空间，存放可执行文件与脚本（自动生成）
└── log/              # 日志空间，保存编译和运行时的日志（自动生成）
```

在开发过程中，开发者主要操作 **src/** 目录，编写或克隆功能包。编译后，可执行文件会出现在 **install/** 目录中，而 **build/** 与 **log/** 则主要由系统自动管理。

- 创建src文件夹并克隆项目

```shell
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# 将公司项目代码克隆到 src/ 中
git clone https://github.com/your-company/ROS-LYENBOT-.git
cd ..
```

## 2.安装依赖

工作空间中的功能包可能依赖系统库或其他 ROS2 包，可以使用 **rosdep** 自动安装：

```shell
# 安装 rosdep
sudo apt install -y python3-pip
sudo pip3 install -U rosdepc

#初始化 rosdep（首次执行）

sudo rosdepc init
rosdepc update

#自动安装 src 下所有依赖

rosdepc install -i --from-path src --rosdistro humble -y
```
## 3.编译工作空间

```shell
cd ~/ros2_ws
colcon build
```

## 4.设置环境变量

```shell
# 当前终端生效
source install/local_setup.sh

# 所有终端永久生效（追加到 .bashrc）
echo "source ~/ros2_ws/install/local_setup.sh" >> ~/.bashrc
#注意修改你自己的路径
```