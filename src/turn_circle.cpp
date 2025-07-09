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

    // 机器人控制命令
    JointValue jpos_cmd;
    memset(&jpos_cmd,0,sizeof(jpos_cmd));
    JointValue jpos_cmd2;
    memset(&jpos_cmd2,0,sizeof(jpos_cmd2));

    // 获取初始时间
    timespec next;
    clock_gettime(CLOCK_REALTIME, &next);

    for(int i = 0;;i++)
    {
        // 获取当前机器人状态
        JointValue now_jpos[2];     
        CartesianPose now_cpos[2];
        robot.edg_get_stat(0, &now_jpos[0], &now_cpos[0]);  
        robot.edg_get_stat(1, &now_jpos[1], &now_cpos[1]);

        //
        unsigned long int details[3];
        robot.edg_stat_details(details);    

        // 计算轨迹
        double t = (i - 0)/10000.0;
        double kk = 15;
        jpos_cmd.jVal[0] = sin(kk*t)*30/180.0*3.14;
        jpos_cmd.jVal[1] = -cos(kk*t)*20 /180.0*3.14 + 20/180.0*3.14;
        jpos_cmd.jVal[3] = -cos(kk*t)*10/180.0*3.14 + 10/180.0*3.14;


        // 发送指令
        robot.edg_servo_j(0, &jpos_cmd, MoveMode::ABS);
        robot.edg_servo_j(1, &jpos_cmd, MoveMode::ABS);
        robot.edg_send();

    }

    std::cout << "Circular path execution completed" << std::endl;
    robot.login_out();
    return 0;
}
