#include "our_servo.h"
#include "our_connect.h"
#include "our_robot_config.h"
#include "our_motor_controller.h"
#include "our_serial_debugger.h"
#include "Arduino.h"

// 如果使用舵机驱动底盘就使用这组
extern Servo base_servo1;
extern Servo base_servo2;
extern Servo base_servo3;
extern Servo base_servo4;

// 声明外部定义的全局舵机对象，用来执行爬升程序
extern Servo hook_servo;
extern Servo shell_servo1;
extern Servo shell_servo2;
extern Servo frame_servo;

// 声明外部定义的全局电机对象
extern Motor motor1;
extern Motor motor2;
extern Motor motor3;
extern Motor motor4;

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!注意跳转查看结构体定义以理解爬升函数接口!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
enum Climb_State climb_state = None;  // 初始时爬升状态为未进入爬升程序

extern bool flag_of_timer0_itr; // 定时器中断采集电机编码器数据的标志位
extern bool flag_of_serial_uart0_interrupt; // 串口 0 调试电机
extern char received_data_buff[RECEIVED_DATA_BUFF]; // 串口 0 接收数据缓冲区


void setup() {
  
  // 初始化串口 0 用于调试
  serial_uart0_init_with_mannual_ISR(115200);
  
  // 初始化所有的的舵机，包括舵机底盘和爬升用的舵机
  servo_init();
  Serial.println("ALL servo init!");
  
  // 初始化 microROS 节点
  // esp32_node_init(); // 需要 wifi 连接成功才能往下面运行，所以先注释掉
  
  // 这里先只初始化 motor1 以便于测试
  motors_and_board_init();

  Serial.println("ALL test init!");

  // 初始化定时器来定时采集编码器数据
  timer0_init_with_ISR();

  motor1.kp_in_velocity = 6.3066957;
  motor1.ki_in_velocity = 0.50338248;
  motor1.kd_in_velocity = 0.0;

  motor1.ki_in_position = 0.000570; // 用 matlab 脚本根据开环的阶跃响应曲线计算的
  motor1.kp_in_position = 0.0922583;
  motor1.kd_in_position = 0.0;

  

}

void loop() {

  // 串口软件中断
  /* 本来要使用软件中断的，但是发现把 service 函数强行塞进去是可以的，但是一旦加上定时器中断就会卡住，还是使用软件中断吧 */
   if(flag_of_serial_uart0_interrupt == true){
    // 进入中断置好标志位和准备好数据之后，就可以在主循环中处理数据了
    flag_of_serial_uart0_interrupt = false; // 进入中断后清除标志位

    // 显示接收到的数据
    Serial.print("received_data_buff:");
    for (int j = 0; j < RECEIVED_DATA_BUFF; j++){
      Serial.print(received_data_buff[j]);
    }

    serial_debug_service(); // 解析接收到的数据
    
    // 清楚接收缓冲区，准备下一次接收
    memset(received_data_buff, 0, sizeof(received_data_buff));
  }

  // 定时器软件中断
  if (flag_of_timer0_itr == true){
    // 定时器软件中断
    flag_of_timer0_itr = false; // 进入中断后清除标志位

    // 开环阶跃测试
    // motor1.open_loop_step_test(FOR_VELOCITY_PID);
    
    // 电机的位置环控制
    // motor1.only_position_control_service();

    // 电机的速度环控制
    motor1.only_velocity_control_service();   
    motor2.only_velocity_control_service();
 
  }

  // 利用状态机缝合起来
  switch(climb_state){
    case None:

    break;

    case Shell_rising:
    
    break;

    case Hook_rising:

    break;

    case Front_wheel_rising:

    break;

    case Back_wheel_driving:

    break;

    case Back_wheel_rising:

    break;

    case Front_wheel_driving:

    break;

  }

}


