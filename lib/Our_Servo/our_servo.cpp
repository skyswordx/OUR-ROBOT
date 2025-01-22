#include <Arduino.h>
#include <our_servo.h>

//在全局定义车身底盘的舵机对象
// 车身坐标系，车身前后是 x 轴，车身左右是 y 轴
//车身的右前轮是 1 号轮，按照逆时针方向依次是 2 号轮，3 号轮，4 号轮
Servo base_servo1;
Servo base_servo2;
Servo base_servo3;
Servo base_servo4;

Servo hook_servo;
Servo shell_servo1;
Servo shell_servo2;
Servo frame_servo;

extern Climb_State climb_state;

/**
 * 使用说明：
 * 1.这边已经完成了全向舵机的控制，可以通过调用 servo360_move 函数来控制舵机的转动，具体参数看函数的实现部分
 * 
 * 2.这边已经完成了底盘的简单开环运动控制，可以通过调用 servo_openloop_ahead, servo_openloop_backward, servo_openloop_left, servo_openloop_right, servo_openloop_yaw_ccw 函数来控制车身的运动
 * servo_openloop_ahead: 前进
 * servo_openloop_backward: 后退
 * servo_openloop_left: 左移
 * servo_openloop_right: 右移
 * servo_openloop_yaw_ccw: 逆时针旋转
 * 参数是轮子的转速，范围是 0 -> 100
 * 
 * 3.这边还完成了麦克纳姆轮的正逆运动学方程的解算，可以通过调用 backforward_kinematics 和 forward_kinematics 函数来完成
 * 车身的速度和角速度 和 四个轮子的转速之间的 互推
 * 但是这个还没实际使用，待会写 PID 控制的时候才会用到
 * 
 */

/*************************************** 函数定义 *******************************************/

void servo360_move(Servo* servo_obj, float wheel_omega_speed) {

  // wheel_omega_speed 的范围是 -100 -> 0 -> 100
  // width 的范围是 500 -> 2500
  if(wheel_omega_speed > 100){wheel_omega_speed = 100;}
  else if (wheel_omega_speed < -100){wheel_omega_speed = -100;}
  
  float width = map(wheel_omega_speed, -100, 100, MIN_WIDTH, MAX_WIDTH);

  servo_obj->writeMicroseconds(width);// 写入高电平脉冲宽度
  // delay(1); // 延时1ms
  
  // Serial.print("wheel_omega_speed: ");
  // Serial.print(wheel_omega_speed);
  // Serial.print(", width: ");
  // Serial.println(width);
  // 没有格式化字符匹配

}

void servo_init(void){

    ESP32PWM::allocateTimer(0);// 分配硬件定时器
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    base_servo1.setPeriodHertz(50);  // 设置频率为50Hz，周期为20ms
    base_servo1.attach(BASE_SERVO_1_PIN, MIN_WIDTH, MAX_WIDTH); // 附加舵机到引脚，设置最小和最大脉冲宽度

    base_servo2.setPeriodHertz(50);  // 设置频率为50Hz，周期为20ms
    base_servo2.attach(BASE_SERVO_2_PIN, MIN_WIDTH, MAX_WIDTH); // 附加舵机到引脚，设置最小和最大脉冲宽度
    
    base_servo3.setPeriodHertz(50);  // 设置频率为50Hz，周期为20ms
    base_servo3.attach(BASE_SERVO_3_PIN, MIN_WIDTH, MAX_WIDTH); // 附加舵机到引脚，设置最小和最大脉冲宽度

    base_servo4.setPeriodHertz(50);  // 设置频率为50Hz，周期为20ms
    base_servo4.attach(BASE_SERVO_4_PIN, MIN_WIDTH, MAX_WIDTH); // 附加舵机到引脚，设置最小和最大脉冲宽度

    hook_servo.setPeriodHertz(50);  // 设置频率为50Hz，周期为20ms
    hook_servo.attach(HOOK_SERVO_PIN, MIN_WIDTH, MAX_WIDTH); // 附加舵机到引脚，设置最小和最大脉冲宽度

    shell_servo1.setPeriodHertz(50);  // 设置频率为50Hz，周期为20ms
    shell_servo1.attach(SHELL_SERVO_1_PIN, MIN_WIDTH, MAX_WIDTH); // 附加舵机到引脚，设置最小和最大脉冲宽度

    shell_servo2.setPeriodHertz(50);  // 设置频率为50Hz，周期为20ms
    shell_servo2.attach(SHELL_SERVO_2_PIN, MIN_WIDTH, MAX_WIDTH); // 附加舵机到引脚，设置最小和最大脉冲宽度

    frame_servo.setPeriodHertz(50);  // 设置频率为50Hz，周期为20ms
    frame_servo.attach(FRAME_SERVO_PIN, MIN_WIDTH, MAX_WIDTH); // 附加舵机到引脚，设置最小和最大脉冲宽度
}

/**
 * todo
 *  1. 配置好各个传感器中断函数，然后设置标志位提供给后面的判断
 *    - while 阻塞的传感器是程序正常运行一定要有的
 *    - if 那边的传感器是可选的，要素来不及就这样吧
 */

Climb_State adjust_for_rising_shell(void){

  // 在这里实现准备抬升机壳前的距离检测和调整偏航角
  // 结合状态机结构
    
  if (true == true){ // 1.要检测到距离目标 0.5 m
  
    // 2.要在这里校位至目标踩踏方块与本方块偏航角一致
    // 调整本方块偏航角
    // 检测目标踩踏方块偏航角

    // mpu6050 读取角度
    return Shell_rising;
  }else{
    return None;
  }
}

void prepare_for_climbing(void){

  // 负责完成从距离目标 0.5 m 的位置前进对齐到爬升开始位置的准备工作
  // 并且在到达位置前完成对机壳的抬升，平衡钩的抬升

  climb_state = adjust_for_rising_shell();
  /**
   * 卡在 None，表面没有对齐目标方块，还没调整好偏航角
   * 正常应该返回是 Shell_rising 
   */

  if(climb_state == Shell_rising){
    // 此时距离目标 0.5 m，且偏航角一致
    // 然后先完成对机壳的抬升，平衡钩的抬升
    // 再前进至爬升开始位置

    Serial.println("SUCESS to adjust");
    // 按照实测的角度旋转舵机，抬升机壳至最大高度
    shell_servo1.writeMicroseconds(SHELL_HIGH);
    shell_servo2.writeMicroseconds(SHELL_HIGH);

    if (true == true){// 加一个传感器检测验证机壳是否到位

      Serial.println("SUCESS: shell to destination");
      // 按照实测的角度旋转舵机，放出平衡钩到完全出来
      climb_state = Hook_rising;
      hook_servo.writeMicroseconds(HOOK_HIGH);

      
      if ( true == true ){// 再加一个传感器检测验证钩子是否到位

        // 前进至爬升开始位置，这里其实就是再前进 0.5 m
        // 这里的运动接口应该更加精细一点
        
        //第一个方法是直接写死
        servo_openloop_ahead(100); // 前进
        delay(500); // 以一定速度前进 0.5s
        servo_openloop_stop(); // 然后停止

        //第二个方法是如果在机壳上有压力传感器之类的，可以使用阻塞函数，直至传感器触发中断
        /*
          servo_openloop_ahead(100); // 前进
          while(true == true){
            // 阻塞函数，直至传感器出发中断后跳出
            if (climb_state != Not_in_position)
            {climb_state = Not_in_position;}
          }
          servo_openloop_stop(); // 然后停止
          Serial.println("SUCESS: in position");
          climb_state == In_position;
        */

        // 再加再加一个传感器检测验证方块是否到爬升开始位置
        if (true == true){
          Serial.println("SUCESS: in position");
          climb_state == In_position;
        }else{
          Serial.println("Fail to reach climbing position");
          climb_state == Not_in_position;
        }
      
      }else{

        // 这里是平衡钩没有抬升到位，所以还要抬升平衡钩，返回 Hook_rising
        Serial.println("Fail to rise hook to destination");
        climb_state == Hook_rising;
      }
    }else{

      // 这里是机壳没有抬升到位，所以还要抬升机壳，返回 Shell_rising
      Serial.println("Fail to rise shell to destination");
      climb_state = Shell_rising;
    }
  }else{

    // 这里是没有完成在 0.5 m 的时候调整偏航角对齐那个方块
    // 所以没进入爬升准备状态，返回 None  
    Serial.println("Fail to adjust for rising shell");
    climb_state = None;
  }


}

void climbing(void){

  prepare_for_climbing();
  /**
   * 未进入爬升准备状态 None
   * 卡在 Shell_rising
   * 卡在 Hook_rising
   * 卡在 Not_in_position
   * 已经在爬升开始位置 In_position
   */
  //利用状态机进行补救和处理
  if (climb_state == In_position) {
    // 1.前轮抬升，其实就是舵机让前后底盘分开
    frame_servo.writeMicroseconds(FRAME_APART);
    while(true == true){
      /*阻塞函数，直至光电传感器出发中断后跳出*/
      if (climb_state != Front_wheel_rising)
      {climb_state = Front_wheel_rising;}  
    }
    frame_servo.writeMicroseconds(FRAME_STOP_MOVING);

    
    // 2.后轮驱动前进
      // 1.保证钩爪抓住目标踩踏方块
      // 2.柱子紧靠目标踩踏方块
        // 一旦开关除法进入外部中断，在中断里面让后轮舵机停止
    
    servo360_move(&base_servo3,100);
    servo360_move(&base_servo4,100);
    while(true == true){
      /*阻塞函数，直至接近开关触发外部中断后跳出*/
      if (climb_state != Back_wheel_driving)
      {climb_state = Back_wheel_driving;}  
    }
    servo_openloop_stop();

    // 3.后轮抬升，其实就是舵机让前后底盘靠近
    frame_servo.writeMicroseconds(FRAME_CLOSE);
    while(true == true){
      /*阻塞函数，直至光电传感器出发中断后跳出*/
      if (climb_state != Back_wheel_rising)
      {climb_state = Back_wheel_rising;}  
    }
    frame_servo.writeMicroseconds(FRAME_STOP_MOVING);
  
    // 4.前轮前进，使得方块正好四四方方叠在下面方块的正上方
    servo360_move(&base_servo1,100);
    servo360_move(&base_servo2,100);
    while(true == true){
      //用什么传感器？
      /*阻塞函数，直至外部中断后跳出*/
      if (climb_state != Front_wheel_driving)
      {climb_state = Front_wheel_driving;}  
    }
    servo_openloop_stop();
    climb_state = None;
  }
}



void servo_openloop_ahead(float wheel_omega_speed){
  //车身坐标系，车身前后是 y 轴，车身左右是 x 轴
  //车身的右前轮是 1 号轮，按照逆时针方向依次是 2 号轮，3 号轮，4 号轮
  //车身的速度分量 vx, vy, omega(角速度，量纲还是角度/s)

  // 参数检查，确保 wheel_omega_speed 在 0 -> 100 之间
  if (wheel_omega_speed > 100) {
    wheel_omega_speed = 100;
  } else if (wheel_omega_speed < 0) {
    Serial.println("wheel_omega_speed is less than 0");
    wheel_omega_speed = 0;
    return;
  }

  // 1,2,3,4 号轮的转速都被设定成 wheel_omega_speed
  servo360_move(&base_servo1, wheel_omega_speed);
  servo360_move(&base_servo2, wheel_omega_speed);
  servo360_move(&base_servo3, wheel_omega_speed);
  servo360_move(&base_servo4, wheel_omega_speed);

}
void servo_openloop_backward(float wheel_omega_speed){
  //车身坐标系，车身前后是 y 轴，车身左右是 x 轴
  //车身的右前轮是 1 号轮，按照逆时针方向依次是 2 号轮，3 号轮，4 号轮
  //车身的速度分量 vx, vy, omega(角速度，量纲还是角度/s)

  //参数检查，确保 wheel_omega_speed 在 0 -> 100 之间
  if (wheel_omega_speed > 100) {
    wheel_omega_speed = 100;
  } else if (wheel_omega_speed < 0) {
    Serial.println("wheel_omega_speed is less than 0");
    wheel_omega_speed = 0;
    return;
  }

  // 1,2,3,4 号轮的转速都被设定成 wheel_omega_speed
  servo360_move(&base_servo1, -wheel_omega_speed);
  servo360_move(&base_servo2, -wheel_omega_speed);
  servo360_move(&base_servo3, -wheel_omega_speed);
  servo360_move(&base_servo4, -wheel_omega_speed);

}
void servo_openloop_left(float wheel_omega_speed){
  //车身坐标系，车身前后是 y 轴，车身左右是 x 轴
  //车身的右前轮是 1 号轮，按照逆时针方向依次是 2 号轮，3 号轮，4 号轮
  //车身的速度分量 vx, vy, omega(角速度，量纲还是角度/s)


  //参数检查，确保 wheel_omega_speed 在 0 -> 100 之间
  if (wheel_omega_speed > 100) {
    wheel_omega_speed = 100;
  } else if (wheel_omega_speed < 0) {
    Serial.println("wheel_omega_speed is less than 0");
    wheel_omega_speed = 0;
    return;
  }

  servo360_move(&base_servo1, wheel_omega_speed);
  servo360_move(&base_servo2, -wheel_omega_speed);
  servo360_move(&base_servo3, wheel_omega_speed);
  servo360_move(&base_servo4, -wheel_omega_speed);

}
void servo_openloop_right(float wheel_omega_speed){
  //车身坐标系，车身前后是 y 轴，车身左右是 x 轴
  //车身的右前轮是 1 号轮，按照逆时针方向依次是 2 号轮，3 号轮，4 号轮
  //车身的速度分量 vx, vy, omega(角速度，量纲还是角度/s)

  //参数检查，确保 wheel_omega_speed 在 0 -> 100 之间
  if (wheel_omega_speed > 100) {
    wheel_omega_speed = 100;
  } else if (wheel_omega_speed < 0) {
    Serial.println("wheel_omega_speed is less than 0");
    wheel_omega_speed = 0;
    return;
  }

  servo360_move(&base_servo1, -wheel_omega_speed);
  servo360_move(&base_servo2, wheel_omega_speed);
  servo360_move(&base_servo3, -wheel_omega_speed);
  servo360_move(&base_servo4, wheel_omega_speed);

}
void servo_openloop_yaw_ccw(float wheel_omega_speed){
  //车身坐标系，车身前后是 y 轴，车身左右是 x 轴
  //车身的右前轮是 1 号轮，按照逆时针方向依次是 2 号轮，3 号轮，4 号轮
  //车身的速度分量 vx, vy, omega(角速度，量纲还是角度/s)

  //参数检查，确保 wheel_omega_speed 在 0 -> 100 之间
  if (wheel_omega_speed > 100) {
    wheel_omega_speed = 100;
  } else if (wheel_omega_speed < 0) {
    Serial.println("wheel_omega_speed is less than 0");
    wheel_omega_speed = 0;
    return;
  }

  servo360_move(&base_servo1, wheel_omega_speed);
  servo360_move(&base_servo2, -wheel_omega_speed);
  servo360_move(&base_servo3, -wheel_omega_speed);
  servo360_move(&base_servo4, wheel_omega_speed);


}
void servo_openloop_stop(void){
  // 1,2,3,4 号轮的转速都被设定成 0
  servo360_move(&base_servo1, 0);
  servo360_move(&base_servo2, 0);
  servo360_move(&base_servo3, 0);
  servo360_move(&base_servo4, 0);
}

