#include "Arduino.h"
#include "our_connect.h"

rcl_node_t node; // 声明节点对象
// 用来存储一些节点的信息，比如节点名字，节点的命名空间等

rclc_executor_t executor; // 声明执行器对象
// 用于执行一些任务，比如订阅消息，发布消息，执行服务等

rclc_support_t support; // 声明初始化选项对象
// 用于初始化rcl并创建一些需要用到的数据结构体

rcl_allocator_t allocator; // 声明内存分配器对象
// 因为在微控制器上资源受限，内存的管理要很细致，所以需要一个内存分配器



// 话题通信相关的
rcl_subscription_t subscriber; // 声明订阅者对象
std_msgs__msg__Int32 msg; // 声明消息文件
// 这点和上位机不同，因为内存受限，所以需要提前声明消息文件的对象
// 这个命名的规范注意一下就行
void callback_subscription(const void* msgin); // 声明订阅回调函数

rcl_publisher_t publisher; // 声明发布者对象
rcl_timer_t timer; // 声明定时器对象

void timer_callback(rcl_timer_t * timer, int64_t last_call_time); 
// 声明定时器回调函数


// 服务通信相关的
rcl_service_t service; // 声明服务对象
std_srvs__srv__Empty_Request req; // 声明请求对象
std_srvs__srv__Empty_Response res; // 声明响应对象
// 这点和上位机不同，因为内存受限，所以需要提前声明请求和响应对象
// 这个命名的规范注意一下就行


// 初始化 microROS 节点
void esp32_node_init(void){

  // 设置通信协议，因为可以通过多种方式连接，所以需要进行提前设置  
  /**
   * 设置通过串口进行 micro-ROS 通信
   * set_microros_serial_transports(Serial);
   */

  wifi_config_init();
  
  delay(2000); //延时2s，等待通信协议初始化完成

  // 初始化内存分配器对象
  allocator = rcl_get_default_allocator();

  // 初始化 初始化选项对象
  rclc_support_init(&support, 0, NULL, &allocator);
  
  // 初始化 esp32_node 节点对象
  rclc_node_init_default(&node, "esp32_node", "", &support);
  
  // 初始化执行器对象
  rclc_executor_init(&executor, &support.context, 1, &allocator);

  Serial.println("node init!");
}

// void append_subscriber2Node(void){

//   // 初始化之前的订阅者对象，让订阅者对象和节点对象关联
//   rclc_subscription_init_default(&subscriber, 
//                                  &node, 
//                                  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//                                  "topic_communication");
//   // 将订阅者对象添加到执行器对象中
//   rclc_executor_add_subscription(&executor, &subscriber, &msg, &callback_subscription, ON_NEW_DATA);
// }



// void callback_subscription(const void* msgin){
//   // 定义话题接收回调函数
//   // 在这里处理接收到的消息
// }


// void append_publisher2Node(void){

//   // 初始化发布者对象，让发布者对象和节点对象关联
//   rclc_publisher_init_default(&publisher, 
//                               &node, 
//                               ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//                               "topic_communication");

  
// }

// void append_timer2Node(void){

//   const unsigned int timer_timeout = 200; // 200ms 发送一次消息

//   // 初始化定时器对象，让定时器对象和节点对象关联
//   rclc_timer_init_default(&timer, 
//                           &support, 
//                           RCL_MS_TO_NS(timer_timeout), 
//                           &timer_callback);
//   // 将定时器对象添加到执行器对象中
//   rclc_executor_add_timer(&executor, &timer);
// }


// void timer_callback(rcl_timer_t * timer, int64_t last_call_time){
//   // 定时器回调函数
//   // 在这里发布消息
//   RCLC_UNUSED(last_call_time);

//   if (timer != NULL) {
//     // 利用定时器对象设置发布频率，再利用发布者对象发布消息
//     rcl_publish(&publisher, &msg, NULL);
//   }
// }


// void service_callback(const void * req, void * res){
//   // 服务回调函数
//   // 在这里处理服务请求
// }


// void append_service2Node(void){

//   // 初始化服务对象，让服务对象和节点对象关联
//   rclc_service_init_default(&service, 
//                             &node, 
//                             ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Empty),
//                             "service_communication");

//   // 将服务对象添加到执行器对象中
//   rclc_executor_add_service(&executor, &service, &req, &res, &service_callback);
// }


void wifi_config_init(void){
  
  IPAddress agent_ip;

  // 通过 ip -4 a | grep inet 获得运行 ros agent 的主机 ip 地址
  agent_ip.fromString("192.168.43.95");
  // 置 wifi 名称，密码，主机 IP,端口号

  Serial.println("wifi config init start!");
  set_microros_wifi_transports("misaka", "xxx", agent_ip, 11454);
  Serial.println("wifi config init end!");
}


void my_test(void){

  // 执行器执行任务的测试代码
  delay(100);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  // Serial.println("in my test end!");
}


/**
 * 话题通信发布者
 * - esp32_node_init()
 * - append_publisher2Node()
 * - append_timer2Node()
 * 
 * 话题通信订阅者
 * - esp32_node_init()
 * - append_subscriber2Node()
 * 
 * 服务通信
 * - esp32_node_init()
 * - append_service2Node()
 */


/**
 * 和 ROS1 的区别：
 * 
 * 
 * 话题通信
 * - 在 ROS2 中 
 *  - 先创建一个节点，再通过一个方法关联节点和订阅者或者发布者对象
 *  - 然后为执行器添加订阅者或者发布者，执行器就可以执行这些任务了
 * 
 * - 在 ROS1 中
 *  - 先创建一个节点，然后再创建一个节点句柄
 *  - 通过节点句柄的方法，让订阅者或者发布者对象就节点关联
 *  - 再通过订阅者或者发布者对象的方法执行订阅或者发布的任务
 */