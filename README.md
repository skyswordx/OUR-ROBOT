# 项目介绍

这是一个名为 "our_robot" 的工作区，用于开发一个爬楼梯机器人应用程序。

## 目录结构

```shell
our_robot/
├── src/
│   └── main.cpp # 主程序入口
└── lib/ 
    └── Our_Connect # 初始化 esp32 的 WiFi 模块，并自建 microROS 节点，通过局域网连接上位机 microROS Agent
    └── Our_Serial_Debugger # 自制串口调试协议，配置串口中断服务，用于调试在线调参 
    └── Our_Motor_Controller # 电机控制器，通过串口接收上位机的指令，用 pid 控制电机的转速
    └── Our_Servo # 舵机初始化和爬升过程的主要实现，通过串口接收上位机的指令，控制舵机的角度来爬升
    └── Our_Robot_Config # 机器人的配置文件，包括电机和舵机的参数（打算后期整理到一个配置文件中，目前还没整理）
```
## 使用方法

```shell
# 首先配置自己的 git 用户名和邮箱
# 然后在 gitee 配置好自己生成的 ssh 公钥

git clone git@gitee.com:yuanyueisf4/our_-robot.git # 克隆项目到本地
cd our_robot # 进入项目目录
code . # 使用 vscode 打开项目

# 在 vscode 中进行开发，开发完成后
git add . # 添加所有文件的修改到暂存区
git commit -m "描述本次提交的相关信息" # 正式提交修改
git push -u git@gitee.com:yuanyueisf4/our_-robot.git master # 推送到远程仓库
# 使用了 -u 参数后，下次推送时只需要输入 git push 即可
```

各个模块使用的说明