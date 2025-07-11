#include <JAKAZuRobot.h>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>
#include <common.h>
#include <math.h>
#include <string.h>
#include <timespec.h>

#define JK_PI (3.141592653589793)
#define deg_to_rad (JK_PI / 180.0)

int main()
{
    // 初始化
    JAKAZuRobot robot;
    RobotStatus robotStatus;
    errno_t ret;

    // 登录
    ret = robot.login_in("192.168.2.200"); // 替换为实际IP
    if (ret != ERR_SUCC) {
        std::cerr << "Login failed with error code: " << ret << std::endl;
        return -1;
    }
    robot.clear_error();
    robot.servo_move_enable(0, -1); // 关闭所有机器人的伺服模式
    robot.servo_move_use_none_filter(); // SERVO模式下不使用滤波器
    robot.power_on();
    robot.enable_robot();
    robot.motion_abort();
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // 设置线程优先级（确保实时性）
    sched_param sch;
    sch.sched_priority = 90;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);

    // 启用Servo模式
    robot.servo_move_enable(1, 0);  // 0表示左臂
    robot.servo_move_enable(1, 1);  // 1表示右臂

    // 设置初始位置
    JointValue start_pos[2] = { { -90 * deg_to_rad, 21 * deg_to_rad, 60 * deg_to_rad, -110 * deg_to_rad, -74 * deg_to_rad, -57 * deg_to_rad, 0 * deg_to_rad},
                                 { 85 * deg_to_rad, 21 * deg_to_rad, -65 * deg_to_rad, -105 * deg_to_rad, -105 * deg_to_rad, -71 * deg_to_rad, 0 * deg_to_rad} };  
    robot.edg_servo_j(0, &start_pos[0], MoveMode::ABS);
    robot.edg_servo_j(1, &start_pos[1], MoveMode::ABS);
    robot.edg_send();

    // 定义圆锥参数
    CartesianPose left_center = {190, 490, 425, 0, 0, 0};
    CartesianPose right_center = {195, -520, 390, 0, 0, 0};
    const double radius = 30.0; // 圆锥底面半径(mm)
    const double cone_angle = 30.0 * deg_to_rad; // 圆锥半角(30度)
    const double height = radius / tan(cone_angle); // 圆锥高度计算

    // 计算圆锥顶点位置 (倒圆锥，顶点在下方)
    CartesianPose left_vertex = left_center;
    left_vertex.tran.z -= height;
    CartesianPose right_vertex = right_center;
    right_vertex.tran.z -= height;

    // 机器人控制命令
    CartesianPose servo_cpos_cmd[2];
    servo_cpos_cmd[0] = left_center; // 左臂初始位置
    servo_cpos_cmd[1] = right_center; // 右臂初始位置

    // 获取初始时间
    timespec next;
    clock_gettime(CLOCK_REALTIME, &next);

    // 控制循环
    for(int i = 0;;i++)
    {
        // 获取当前机器人状态
        JointValue now_jpos[2];     
        CartesianPose now_cpos[2];
        robot.edg_get_stat(0, &now_jpos[0], &now_cpos[0]);  
        robot.edg_get_stat(1, &now_jpos[1], &now_cpos[1]);

        // 轨迹计算 - 倒圆锥运动
        const double theta = 2.0 * JK_PI * (i % 200) / 200.0; // 角度0-2π循环
        
        // 计算左臂末端位置 (在水平面上的圆)
        servo_cpos_cmd[0].tran.x = left_center.tran.x + radius * cos(theta);
        servo_cpos_cmd[0].tran.y = left_center.tran.y + radius * sin(theta);
        servo_cpos_cmd[0].tran.z = left_center.tran.z; // 保持高度不变
        
        // 计算右臂末端位置
        servo_cpos_cmd[1].tran.x = right_center.tran.x + radius * cos(theta);
        servo_cpos_cmd[1].tran.y = right_center.tran.y + radius * sin(theta);
        servo_cpos_cmd[1].tran.z = right_center.tran.z;
        
        // 计算末端姿态 (z轴指向圆锥顶点)
        // 左臂方向向量
        double dx_left = left_vertex.tran.x - servo_cpos_cmd[0].tran.x;
        double dy_left = left_vertex.tran.y - servo_cpos_cmd[0].tran.y;
        double dz_left = left_vertex.tran.z - servo_cpos_cmd[0].tran.z;
        
        // 右臂方向向量
        double dx_right = right_vertex.tran.x - servo_cpos_cmd[1].tran.x;
        double dy_right = right_vertex.tran.y - servo_cpos_cmd[1].tran.y;
        double dz_right = right_vertex.tran.z - servo_cpos_cmd[1].tran.z;
        
        // 计算俯仰角(绕Y轴旋转)
        double pitch_left = atan2(dz_left, sqrt(dx_left*dx_left + dy_left*dy_left));
        double pitch_right = atan2(dz_right, sqrt(dx_right*dx_right + dy_right*dy_right));
        
        // 计算偏航角(绕Z轴旋转)
        double yaw_left = atan2(dy_left, dx_left);
        double yaw_right = atan2(dy_right, dx_right);
        
        // 设置姿态 (使用RPY表示)
        servo_cpos_cmd[0].rpy.rx = 0; // 滚转角保持0
        servo_cpos_cmd[0].rpy.ry = pitch_left; // 俯仰角
        servo_cpos_cmd[0].rpy.rz = yaw_left; // 偏航角
        
        servo_cpos_cmd[1].rpy.rx = 0;
        servo_cpos_cmd[1].rpy.ry = pitch_right;
        servo_cpos_cmd[1].rpy.rz = yaw_right;

        // 发送指令
        robot.edg_servo_p(0, &servo_cpos_cmd[0], MoveMode::ABS);
        robot.edg_servo_p(1, &servo_cpos_cmd[1], MoveMode::ABS);
        robot.edg_send();

        // 等待下一个周期 (1ms)
        timespec dt;
        dt.tv_nsec = 1000000; // 1ms = 1,000,000 ns
        dt.tv_sec = 0;
        next = timespec_add(next, dt);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL);
    }

    std::cout << "Circular path execution completed" << std::endl;
    robot.login_out();
    return 0;
}