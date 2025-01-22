#ifndef _OUR_ROBOT_CONFIG_H
#define _OUR_ROBOT_CONFIG_H

// 在这个头文件声明一些全局的宏定义，或者是一些全局的变量
// 后续结合实际的重量和大小调参

// 用于麦轮运动逆解算的参数
#define FRONT_BACK_WHEEL_DISTANCE 0.1 //前后轮之间的距离
#define LEFT_RIGHT_WHEEL_DISTANCE 0.1 //左右轮之间的距离

// 预设与上位机通信得到小车本体应该要的速度 Vx, Vy, Omega
// double Vx_global = 0.0;
// double Vy_global = 0.0;
// double Omega_global = 0.0;

// 由麦轮运动逆解算得到四个轮子的线转速 wheel_omega_speedX
#define WHEEL_RADIUS 0.05 //轮子的半径
// 还要再除轮子的半径得到电机输出轴的角速度

// 注意减速电机的比值，输出轴还要用比例换算成编码器实际测得电机角速度的

// 然后还要换成编码器单位

#endif