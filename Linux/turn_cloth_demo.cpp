#include <JAKAZuRobot.h>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

#define JK_PI (3.141592653589793)
#define deg_tp_rad 1.0 / 180.0 * JK_PI

#define ASSERT_TRUE_OR_EXIT(expr, msg) \
    do { \
        if (!(expr)) \
        { \
            std::cout << (msg) << " fail:" << __LINE__ << std::endl; \
            std::flush(std::cout); \
            exit(EXIT_FAILURE); \
        } \
    } while (0)

#define ASSERT_TRUE_OR_LOG(expr, msg) \
    do { \
        if (!(expr)) \
        { \
            std::cout << (msg) << " fail:" << __LINE__ << std::endl; \
            std::flush(std::cout); \
        } \
    } while (0)



// 循环!!!
// const int POINTS_PER_CYCLE = 8; // 每圈8个点

// std::vector<CartesianPose> generate_cyclic_path(
//     const CartesianPose& center, 
//     double radius, 
//     int cycles) 
// {
//     std::vector<CartesianPose> path;
//     for (int cycle = 0; cycle < cycles; ++cycle) {
//         for (int i = 0; i < POINTS_PER_CYCLE; ++i) {
//             double theta = 2 * JK_PI * i / POINTS_PER_CYCLE;
//             double x_offset = radius * cos(theta);
//             double y_offset = radius * sin(theta);

//             CartesianPose point;
//             point.tran.x = center.tran.x + x_offset;
//             point.tran.y = center.tran.y + y_offset;
//             point.tran.z = center.tran.z;
//             point.rpy.rx = 0;
//             point.rpy.ry = 0;
//             point.rpy.rz = 0; // 叠加旋转角度
//             // point.rpy.rz = center.rpy.rz + theta; // 叠加旋转角度

//             path.push_back(point);
//         }
//     }
//     // 闭合路径：添加起始点作为结束点
//     if (!path.empty()) path.push_back(path[0]);
//     return path;
// }


int main()
{
    JointValue start_pos[2] = { { -90 * deg_tp_rad, 21 * deg_tp_rad, 60 * deg_tp_rad, -110 * deg_tp_rad, -74 * deg_tp_rad, -57 * deg_tp_rad, 0 * deg_tp_rad},
                                 { 85 * deg_tp_rad, 21 * deg_tp_rad, -65 * deg_tp_rad, -105 * deg_tp_rad, -105 * deg_tp_rad, -71 * deg_tp_rad, 0 * deg_tp_rad} };  
    
    MoveMode moveop[2] = {ABS, ABS};
    double vel[2] = {2, 2};
    double acc[2] = {2, 2};

    JAKAZuRobot robot;
    RobotStatus robotStatus;
    errno_t ret;

    // 1. 登录机器人
    ret = robot.login_in("192.168.2.200"); // 替换为实际IP
    if (ret != ERR_SUCC) {
        std::cerr << "Login failed with error code: " << ret << std::endl;
        return -1;
    }

    ret = robot.power_on();
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "power on");

    // ret = robot.disable_robot();

    robot.clear_error();


    ret = robot.enable_robot();
    if (ret != ERR_SUCC)
    {
        printf("enable failed! ret = %d\n", ret);
    }
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "enable robot");
    double mm = 1e-3;


    // 2. 定义动作路径点（笛卡尔空间坐标，单位：mm和度）
    double r = 30;
    double sqrt2_r = r * 0.7071;  // 45°投影长度
    double sin11_25 = 0.1951;     // sin(11.25°)
    double cos11_25 = 0.9808;     // cos(11.25°)
    double sin33_75 = 0.5556;     // sin(33.75°)
    double cos33_75 = 0.8315;     // cos(33.75°)
    double sin56_25 = 0.8315;     // sin(56.25°)
    double cos56_25 = 0.5556;     // cos(56.25°)
    double sin78_75 = 0.9808;     // sin(78.75°)
    double cos78_75 = 0.1951;     // cos(78.75°)

    // 左臂路径点 (中心点190,490,425)
    CartesianPose left_path[] = {
        // 新增45°插补点
        {190 - r,             490,                425, 0, 0, 0},       // 0°
        {190 - r*cos78_75,    490 - r*sin78_75,   425, 0, 0, 0},      // 11.25°
        {190 - r*cos56_25,    490 - r*sin56_25,   425, 0, 0, 0},      // 22.5°
        {190 - r*cos33_75,    490 - r*sin33_75,   425, 0, 0, 0},      // 33.75°
        {190 - sqrt2_r,       490 - sqrt2_r,      425, 0, 0, 0},      // 45°
        {190 - r*sin33_75,    490 - r*cos33_75,   425, 0, 0, 0},      // 56.25°
        {190 - r*sin56_25,    490 - r*cos56_25,   425, 0, 0, 0},      // 67.5°
        {190 - r*sin78_75,    490 - r*cos78_75,   425, 0, 0, 0},      // 78.75°
        {190,                 490 - r,            425, 0, 0, 0},      // 90°
        {190 + r*sin78_75,    490 - r*cos78_75,   425, 0, 0, 0},      // 101.25°
        {190 + r*sin56_25,    490 - r*cos56_25,   425, 0, 0, 0},      // 112.5°
        {190 + r*sin33_75,    490 - r*cos33_75,   425, 0, 0, 0},      // 123.75°
        {190 + sqrt2_r,       490 - sqrt2_r,      425, 0, 0, 0},      // 135°
        {190 + r*cos33_75,    490 - r*sin33_75,   425, 0, 0, 0},      // 146.25°
        {190 + r*cos56_25,    490 - r*sin56_25,   425, 0, 0, 0},      // 157.5°
        {190 + r*cos78_75,    490 - r*sin78_75,   425, 0, 0, 0},      // 168.75°
        {190 + r,             490,                425, 0, 0, 0},      // 180°
        {190 + r*cos78_75,    490 + r*sin78_75,   425, 0, 0, 0},      // 191.25°
        {190 + r*cos56_25,    490 + r*sin56_25,   425, 0, 0, 0},      // 202.5°
        {190 + r*cos33_75,    490 + r*sin33_75,   425, 0, 0, 0},      // 213.75°
        {190 + sqrt2_r,       490 + sqrt2_r,      425, 0, 0, 0},      // 225°
        {190 + r*sin33_75,    490 + r*cos33_75,   425, 0, 0, 0},      // 236.25°
        {190 + r*sin56_25,    490 + r*cos56_25,   425, 0, 0, 0},      // 247.5°
        {190 + r*sin78_75,    490 + r*cos78_75,   425, 0, 0, 0},      // 258.75°
        {190,                 490 + r,            425, 0, 0, 0},      // 270°
        {190 - r*sin78_75,    490 + r*cos78_75,   425, 0, 0, 0},      // 281.25°
        {190 - r*sin56_25,    490 + r*cos56_25,   425, 0, 0, 0},      // 292.5°
        {190 - r*sin33_75,    490 + r*cos33_75,   425, 0, 0, 0},      // 303.75°
        {190 - sqrt2_r,       490 + sqrt2_r,      425, 0, 0, 0},      // 315°
        {190 - r*cos33_75,    490 + r*sin33_75,   425, 0, 0, 0},      // 326.25°
        {190 - r*cos56_25,    490 + r*sin56_25,   425, 0, 0, 0},      // 337.5°
        {190 - r*cos78_75,    490 + r*sin78_75,   425, 0, 0, 0}       // 348.75°
        // {190 - r,       490,          425, 0, 0, 0},          // 0° (起始点)
        // {190 - sqrt2_r, 490 - sqrt2_r, 425, 0, 0, 0},         // 45°
        // {190,          490 - r,       425, 0, 0, 0},         // 90° (原第二点)
        // {190 + sqrt2_r, 490 - sqrt2_r, 425, 0, 0, 0},         // 135°
        // {190 + r,       490,          425, 0, 0, 0},         // 180° (原第三点)
        // {190 + sqrt2_r, 490 + sqrt2_r, 425, 0, 0, 0},         // 225°
        // {190,          490 + r,       425, 0, 0, 0},          // 270° (原第四点)
        // {190 - sqrt2_r, 490 + sqrt2_r, 425, 0, 0, 0}          // 315°
    };
    // std::cout << "left_path " << left_path << std::endl;
    // 右臂路径点 (中心点195,-520,390)
    CartesianPose right_path[] = {
        // 第一象限（0°~90°）
        {195 - r,             -520,                390, 0, 0, 0},       // 0°
        {195 - r*cos78_75,    -520 - r*sin78_75,   390, 0, 0, 0},      // 11.25°
        {195 - r*cos56_25,    -520 - r*sin56_25,   390, 0, 0, 0},      // 22.5°
        {195 - r*cos33_75,    -520 - r*sin33_75,   390, 0, 0, 0},      // 33.75°
        {195 - sqrt2_r,       -520 - sqrt2_r,      390, 0, 0, 0},      // 45°
        {195 - r*sin33_75,    -520 - r*cos33_75,   390, 0, 0, 0},      // 56.25°
        {195 - r*sin56_25,    -520 - r*cos56_25,   390, 0, 0, 0},      // 67.5°
        {195 - r*sin78_75,    -520 - r*cos78_75,   390, 0, 0, 0},      // 78.75°
        {195,                 -520 - r,            390, 0, 0, 0},      // 90°
        
        // 第二象限（90°~180°）
        {195 + r*sin78_75,    -520 - r*cos78_75,   390, 0, 0, 0},      // 101.25°
        {195 + r*sin56_25,    -520 - r*cos56_25,   390, 0, 0, 0},      // 112.5°
        {195 + r*sin33_75,    -520 - r*cos33_75,   390, 0, 0, 0},      // 123.75°
        {195 + sqrt2_r,       -520 - sqrt2_r,      390, 0, 0, 0},      // 135°
        {195 + r*cos33_75,    -520 - r*sin33_75,   390, 0, 0, 0},      // 146.25°
        {195 + r*cos56_25,    -520 - r*sin56_25,   390, 0, 0, 0},      // 157.5°
        {195 + r*cos78_75,    -520 - r*sin78_75,   390, 0, 0, 0},      // 168.75°
        {195 + r,             -520,                390, 0, 0, 0},      // 180°
        
        // 第三象限（180°~270°）
        {195 + r*cos78_75,    -520 + r*sin78_75,   390, 0, 0, 0},      // 191.25°
        {195 + r*cos56_25,    -520 + r*sin56_25,   390, 0, 0, 0},      // 202.5°
        {195 + r*cos33_75,    -520 + r*sin33_75,   390, 0, 0, 0},      // 213.75°
        {195 + sqrt2_r,       -520 + sqrt2_r,      390, 0, 0, 0},      // 225°
        {195 + r*sin33_75,    -520 + r*cos33_75,   390, 0, 0, 0},      // 236.25°
        {195 + r*sin56_25,    -520 + r*cos56_25,   390, 0, 0, 0},      // 247.5°
        {195 + r*sin78_75,    -520 + r*cos78_75,   390, 0, 0, 0},      // 258.75°
        {195,                 -520 + r,            390, 0, 0, 0},      // 270°
        
        // 第四象限（270°~360°）
        {195 - r*sin78_75,    -520 + r*cos78_75,   390, 0, 0, 0},      // 281.25°
        {195 - r*sin56_25,    -520 + r*cos56_25,   390, 0, 0, 0},      // 292.5°
        {195 - r*sin33_75,    -520 + r*cos33_75,   390, 0, 0, 0},      // 303.75°
        {195 - sqrt2_r,       -520 + sqrt2_r,      390, 0, 0, 0},      // 315°
        {195 - r*cos33_75,    -520 + r*sin33_75,   390, 0, 0, 0},      // 326.25°
        {195 - r*cos56_25,    -520 + r*sin56_25,   390, 0, 0, 0},      // 337.5°
        {195 - r*cos78_75,    -520 + r*sin78_75,   390, 0, 0, 0}       // 348.75°
        // {195 - r,       -520,          390, 0, 0, 0},        // 0° (起始点)
        // {195 - sqrt2_r, -520 - sqrt2_r, 390, 0, 0, 0},       // 45°
        // {195,          -520 - r,       390, 0, 0, 0},        // 90° (原第二点)
        // {195 + sqrt2_r, -520 - sqrt2_r, 390, 0, 0, 0},       // 135°
        // {195 + r,       -520,          390, 0, 0, 0},       // 180° (原第三点)
        // {195 + sqrt2_r, -520 + sqrt2_r, 390, 0, 0, 0},       // 225°
        // {195,          -520 + r,       390, 0, 0, 0},         // 270° (原第四点)
        // {195 - sqrt2_r, -520 + sqrt2_r, 390, 0, 0, 0}        // 315°
    };

    // CartesianPose left_center = {190, 490, 425, 0, 0, 0};
    // CartesianPose right_center = {195, -520, 390, 0, 0, 0};
    // double radius = 2.0;
    // int n_cycles = 10; // 循环10次

    // auto left_path = generate_cyclic_path(left_center, radius, n_cycles);
    // auto right_path = generate_cyclic_path(right_center, radius, n_cycles);
    // std::cout << "left_path " << left_path << std::endl;

    std::cout << "PATH INIT " << std::endl;
    // 3. 运动参数设置
    MoveMode move_mode[2] = {ABS, ABS}; // 绝对坐标模式
    double velocity[2] = {3000, 3000};     // 运动速度 (mm/s)
    double acceleration[2] = {3000, 3000}; // 加速度 (mm/s²)
    int point_count = sizeof(left_path) / sizeof(CartesianPose);
    
    //ret = robot.robot_run_multi_movj(LEFT, moveop, TRUE, start_pos, vel, acc);


    // 4. 执行动作
    for (int n = 0; n < 4; ++n) {
        for (int i = 0; i < point_count; ++i) {
            CartesianPose target_poses[2] = {left_path[i], right_path[i]};
            
            // if(i==2 || i==3)
            // {
            //     velocity[0] = 5000;     // 运动速度 (mm/s)
            //     velocity[1] = 5000; 
            //     acceleration[0] = 5000; // 加速度 (mm/s²)
            //     acceleration[1] = 5000;
            // }
            // else
            // {
            //     velocity[0] = 1000;     // 运动速度 (mm/s)
            //     velocity[1] = 1000; 
            //     acceleration[0] = 1000; // 加速度 (mm/s²)
            //     acceleration[1] = 1000;
            // }

            velocity[0] = 100000;     // 运动速度 (mm/s)
            velocity[1] = 100000; 
            acceleration[0] = 100000; // 加速度 (mm/s²)
            acceleration[1] = 100000;

            // 调用多臂笛卡尔空间运动接口
            ret = robot.robot_run_multi_movl(
                -1,             // -1表示双轴同步运动
                move_mode,      // 运动模式
                TRUE,           // 阻塞执行
                target_poses,   // 目标位姿
                velocity,       // 速度
                acceleration    // 加速度
            );

            if (ret != ERR_SUCC) {
                std::cerr << "Movement failed at point " << i 
                        << " with error code: " << ret << std::endl;
                break;
            }

            std::cout << "Executed point " << i 
                    << ": Left[" << target_poses[0].tran.x << ", " << target_poses[0].tran.y << ", " << target_poses[0].tran.z << "]"
                    << " Right[" << target_poses[1].tran.x << ", " << target_poses[1].tran.y << ", " << target_poses[1].tran.z << "]" << std::endl;
        }
    }
    std::cout << "hello world" << std::endl;
    return 0;
}
