#include "JAKAZuRobot.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <cstring>
#include <pthread.h>
#include <csignal>
#include <atomic>
#include "timespec.h"

#define rad2deg(x) ((x)*180.0/M_PI)
#define deg2rad(x) ((x)*M_PI/180.0)

// 全局标志位，用于安全退出
std::atomic<bool> running(true);

// 信号处理函数
void signal_handler(int signal)
{
    if (signal == SIGINT) {
        std::cout << "\nReceived SIGINT. Exiting gracefully..." << std::endl;
        running = false;
    }
}

int main()
{
    // 设置信号处理
    std::signal(SIGINT, signal_handler);
    
    // 初始化机器人对象
    JAKAZuRobot robot;
    
    // 登录机器人
    errno_t ret = robot.login_in("192.168.2.200");
    if (ret != ERR_SUCC) {
        std::cerr << "Login failed with error code: " << ret << std::endl;
        return -1;
    }
    
    // 机器人初始化序列
    robot.clear_error();
    robot.servo_move_enable(0, -1); // 关闭所有伺服模式
    robot.servo_move_use_none_filter(); // 不使用滤波器
    robot.power_on();
    robot.enable_robot();
    robot.motion_abort();
    
    // 等待机器人准备就绪
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // 设置实时线程优先级
    sched_param sch;
    sch.sched_priority = 90;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch) != 0) {
        std::cerr << "Failed to set thread priority. Running with default priority." << std::endl;
    }

    // 启用双臂的伺服模式
    robot.servo_move_enable(1, 0);  // 左臂
    robot.servo_move_enable(1, 1);  // 右臂

    // 设置初始位置
    JointValue start_pos[2] = { 
        {deg2rad(-90), deg2rad(21), deg2rad(60), deg2rad(-110), deg2rad(-74), deg2rad(-57), deg2rad(0)},
        {deg2rad(85), deg2rad(21), deg2rad(-65), deg2rad(-105), deg2rad(-105), deg2rad(-71), deg2rad(0)}
    };
    
    // 发送初始位置指令
    robot.edg_servo_j(0, &start_pos[0], MoveMode::ABS);
    robot.edg_servo_j(1, &start_pos[1], MoveMode::ABS);
    
    uint32_t cmd_index = 0;
    robot.edg_send(&cmd_index);
    cmd_index++;
    
    // 获取当前位置作为基准
    JointValue now_jpos[2];     
    CartesianPose now_cpos[2];
    robot.edg_get_stat(0, &now_jpos[0], &now_cpos[0]);  
    robot.edg_get_stat(1, &now_jpos[1], &now_cpos[1]);
    
    // 初始化命令位置
    CartesianPose servo_cpos_cmd[2];
    memcpy(&servo_cpos_cmd[0], &now_cpos[0], sizeof(CartesianPose));
    memcpy(&servo_cpos_cmd[1], &now_cpos[1], sizeof(CartesianPose));

    // 设置控制循环参数
    const int cycle_time_ms = 8; // 8ms周期
    auto next_time = std::chrono::steady_clock::now();
    
    // 轨迹参数
    const double amplitude_z = 100.0; // Z轴振幅(mm)
    const double amplitude_x = 10.0;  // X轴振幅(mm)
    const double frequency = 0.001;   // 频率 (rad/ms)
    
    std::cout << "Starting servo control loop. Press Ctrl+C to exit." << std::endl;
    
    // 主控制循环
    int i = 0;
    while (running) {
        // 获取当前机器人状态
        robot.edg_get_stat(0, &now_jpos[0], &now_cpos[0]);  
        robot.edg_get_stat(1, &now_jpos[1], &now_cpos[1]);

        // 计算轨迹偏移量
        double coefficient = sin(i * frequency);
        double z_step = amplitude_z * coefficient;
        double x_step = amplitude_x * coefficient; 
        
        // 更新命令位置
        servo_cpos_cmd[0].tran.z = now_cpos[0].tran.z + z_step;
        servo_cpos_cmd[0].tran.x = now_cpos[0].tran.x + x_step;
        servo_cpos_cmd[1].tran.z = now_cpos[1].tran.z + z_step;
        servo_cpos_cmd[1].tran.x = now_cpos[1].tran.x + x_step;

        // 发送指令
        robot.edg_servo_p(0, &servo_cpos_cmd[0], MoveMode::ABS);
        robot.edg_servo_p(1, &servo_cpos_cmd[1], MoveMode::ABS);
        robot.edg_send(&cmd_index);
        cmd_index++;
        
        // 打印状态信息（可选）
        if (i % 100 == 0) { // 每100次循环打印一次
            std::cout << "Cycle: " << i 
                      << " | Left Z: " << now_cpos[0].tran.z
                      << " | Right Z: " << now_cpos[1].tran.z
                      << " | Cmd index: " << cmd_index << std::endl;
        }
        
        // 等待下一个周期
        next_time += std::chrono::milliseconds(cycle_time_ms);
        std::this_thread::sleep_until(next_time);
        i++;
    }

    // 安全关闭序列
    std::cout << "Initiating safe shutdown..." << std::endl;
    
    // 禁用伺服模式
    robot.servo_move_enable(0, 0); // 禁用左臂伺服
    robot.servo_move_enable(0, 1); // 禁用右臂伺服
    
    // 等待命令执行完成
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 登出
    robot.login_out();
    
    std::cout << "Servo control completed safely." << std::endl;
    return 0;
}