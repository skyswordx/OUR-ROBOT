#ifndef _OUR_SERVO_H
#define _OUR_SERVO_H

#include <ESP32Servo.h>

// 定义底盘舵机的引脚
#define BASE_SERVO_1_PIN 15 //定义舵机的引脚 
#define BASE_SERVO_2_PIN 2 //定义舵机的引脚 
#define BASE_SERVO_3_PIN 4 //定义舵机的引脚 
#define BASE_SERVO_4_PIN 5 //定义舵机的引脚 

//定义4个爬升用的30kg舵机
#define HOOK_SERVO_PIN 18
#define SHELL_SERVO_1_PIN 19
#define SHELL_SERVO_2_PIN 21
#define FRAME_SERVO_PIN  22

//定义舵机的最大和最小脉冲宽度，单位为微秒
#define MAX_WIDTH   2500 // 2.5ms
#define MIN_WIDTH   500  // 0.5ms

#define HOOK_LOW   500  //平衡钩垂下时的脉冲宽度
#define HOOK_HIGH 1500  //平衡钩抬升时的脉冲宽度

#define SHELL_LOW 2300   //壳子抬升到最低点时的脉冲宽度
#define SHELL_HIGH 1500  //壳子抬升到最高点时的脉冲宽度


//两个支架之间的运动借助的舵机是360度舵机，脉冲宽度>1500时远离，<1500时接近
#define FRAME_APART 1700
#define FRAME_STOP_MOVING 1500
#define FRAME_CLOSE 1300



enum Climb_State{
      // 为爬升做准备的状态
      None,                      // 没进入爬升程序，在 0.5m 之外、或者没有和目标方块对齐
      Shell_rising,              // 抬升机壳的过程，如果没有抬升到位，就不会进入下一个状态
      Hook_rising,               // 抬升平衡钩的过程，如果没有抬升到位，就不会进入下一个状态
      Not_in_position,           // 方块还未到达真正开始爬升的位置，如果一直没有到达，就不会进入下一个状态
      In_position,               // 方块已经紧靠要爬上去的方块

      // 爬升过程中的状态
      Front_wheel_rising,        // 抬升前轮的过程，如果没有抬升到位，就不会进入下一个状态
      Back_wheel_driving,        // 后轮驱动前进靠紧目标方块，如果没有到达目标位置，就不会进入下一个状态
      Back_wheel_rising,         // 抬升后轮的过程，如果没有抬升到位，就不会进入下一个状态
      Front_wheel_driving      // 前轮驱动前进到被爬的方块大概正上方，如果没有到达目标位置，就不会进入下一个状态
      
      //并且一旦方块到达被爬方块的正上方，就会进入 None ，爬升结束，可以移动后继续爬升的状态循环
};

/*************************************** 函数声明 *******************************************/
void servo360_move(Servo* servo_obj, float wheel_omega_speed);
//对全向舵机进行正转或者反转的控制
// omega 的范围是 -100 -> 0 -> 100
//对于中轴的全向舵机，+100 是让白色底座全速上升，-100 是让白色底座全速下降

//初始化所有的舵机
void servo_init(void);

//实现爬升功能的主要函数
void climbing(void);
//状态机微调可能会用到，留个接口
Climb_State adjust_for_rising_shell(void);
void prepare_for_climbing(void);


//简单的【舵机】开环车身运动的接口
//非常原始，只能指定舵机的转速，还没计算对应的车身速度和角速度
void servo_openloop_ahead(float wheel_omega_speed);
void servo_openloop_backward(float wheel_omega_speed);
void servo_openloop_left(float wheel_omega_speed);
void servo_openloop_right(float wheel_omega_speed);
void servo_openloop_yaw_ccw(float wheel_omega_speed);
void servo_openloop_stop(void);


#endif