#ifndef OUR_CONNECT_H
#define OUR_CONNECT_H

#include <Arduino.h>
#include <micro_ros_platformio.h> 

#include <WiFi.h> // wifi 通信要包含的头文件

// 基本组件
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// 添加话题通信的消息类型文件
#include <std_msgs/msg/int32.h>

// 添加服务通信的服务类型文件
#include <std_srvs/srv/empty.h>

////////////////////////////////////////////////////////////////
//  后续的网络服务名称应该是御坂网络服务 Misaka Network Service  //
////////////////////////////////////////////////////////////////

void esp32_node_init(void);

void append_subscriber2Node(void);
void append_publisher2Node(void);
void append_service2Node(void);

void wifi_config_init(void);

void my_test(void);

#endif