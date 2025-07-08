#include <JAKAZuRobot.h>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

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
    
int main()
{
    // JAKAZuRobot robot;
    // RobotStatus robotStatus;
    // errno_t ret;

    // ret = robot.login_in("192.168.2.200");
    // ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");
    // JointValue jstep_pos[2] { { 0.2, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 }, { 0.2, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 } };
    // JointValue jstep_neg[2] { { -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1 }, { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 } };
    // JointValue start_pos[2] = { { -90 * deg_tp_rad, -75 * deg_tp_rad, 90 * deg_tp_rad, -120 * deg_tp_rad, -70 * deg_tp_rad, -90 * deg_tp_rad, 40 * deg_tp_rad},
    //                              { 90 * deg_tp_rad, -45 * deg_tp_rad, 0, -100 * deg_tp_rad, 0, -35 * deg_tp_rad, 90 * deg_tp_rad} };  
                                 
                                 
    
    
    // MoveMode moveop[2] = {ABS, ABS};
    // // double vel[2] = {2, 2};
    // // double acc[2] = {2, 2};
    // // double tol[2] = {0, 0};
    // // double id[2] = {0, 0};
    // // ret = robot.robot_run_multi_movj(LEFT, moveop, TRUE, start_pos, vel, acc);
    
    // CartesianPose c_pos[2] { { 10, 20, 30, 0, 0, 0 }, { 20, 40, 60, 0, 0, 0 } };
    // CartesianPose c_neg[2] { { -10, -20, -30, 0, 0, 0 }, { -20, -40, -60, 0, 0, 0 } };

    // double vel[2] = {2, 2};
    // double acc[2] = {2, 2};
    // ret = robot.robot_run_multi_movl(-1, moveop, TRUE, c_pos, vel, acc);

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
    // 2. 定义Fling动作路径点（笛卡尔空间坐标，单位：mm和度）
    // 左臂路径点 (转换为mm和SDK要求的坐标系)
    CartesianPose left_path[] = {
        {mm*200,  mm*220, mm*100, 160, 0, 0},  // 起始点
        {mm*200,  mm*200, mm*200, 160, 0, 0},
        {mm*320,  mm*200, mm*220, 160, 0, 0},
        {mm*420,  mm*200, mm*320, 160, 0, 0},
        {mm*350,  mm*200, mm*200, 160, 0, 0},
        {mm*300,  mm*200, mm*170, 160, 0, 0},
        {mm*200,  mm*200, mm*100, 160, 0, 0}   // 结束点
    };

    // 右臂路径点 (Y坐标取反)
    CartesianPose right_path[] = {
        {mm*200,  mm*-220, mm*100, 200, 0, 0}, // 起始点
        {mm*200,  mm*-200, mm*200, 200, 0, 0},
        {mm*320,  mm*-200, mm*220, 200, 0, 0},
        {mm*420,  mm*-200, mm*320, 200, 0, 0},
        {mm*350,  mm*-200, mm*200, 200, 0, 0},
        {mm*300,  mm*-200, mm*170, 200, 0, 0},
        {mm*200,  mm*-200, mm*100, 200, 0, 0}  // 结束点
    };

    // 3. 运动参数设置
    MoveMode move_mode[2] = {ABS, ABS}; // 绝对坐标模式
    double velocity[2] = {2, 2};     // 运动速度 (mm/s)
    double acceleration[2] = {2, 2}; // 加速度 (mm/s²)
    int point_count = sizeof(left_path) / sizeof(CartesianPose);
    
    // 4. 执行Fling动作
    for (int i = 0; i < point_count; ++i) {
        CartesianPose target_poses[2] = {left_path[i], right_path[i]};

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

    // PayLoad payload;
    // robot.robot_zero_ftsensor(LEFT, 1);
    // std::cout<<"payload.ok"<<std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000)); //校零后必须等待 1 秒
    
    // // ret = robot.robot_set_cst_ftframe(LEFT, 0); //设
    // ret= robot.robot_get_ftsensor_payload(LEFT, 1, &payload);
    // int status;
    // int errcode;
    // double ft_original;
    // double ft_actual;
    // ret= robot.robot_get_ftsensor_stat(LEFT, 1, &status, &
    //     errcode, & ft_original, &ft_actual);
    // std::cout<<"payload.mass::"<< payload.mass<<",status:"<<status<<",err:"<<errcode<<",ft_original"<<ft_original<<",ft_ac"<<ft_actual<<std::endl; 
    
    // DHParam dh[2] = {};
    // ret = robot.robot_get_multi_robot_dh(dh);
    // for (int i = 0; i < 2; i++)
    // {
    //     printf("Robot[%d] : Alpha, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", i, dh[i].alpha[0], dh[i].alpha[1], dh[i].alpha[2], dh[i].alpha[3], dh[i].alpha[4], dh[i].alpha[5], dh[i].alpha[6]);
    //     printf("Robot[%d] : d, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", i, dh[i].d[0], dh[i].d[1], dh[i].d[2], dh[i].d[3], dh[i].d[4], dh[i].d[5], dh[i].d[6]);
    //     printf("Robot[%d] : a, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", i, dh[i].a[0], dh[i].a[1], dh[i].a[2], dh[i].a[3], dh[i].a[4], dh[i].a[5], dh[i].a[6]);
    //     printf("Robot[%d] : joint_homeoff, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", i, dh[i].joint_homeoff[0], dh[i].joint_homeoff[1], dh[i].joint_homeoff[2], dh[i].joint_homeoff[3], dh[i].joint_homeoff[4], dh[i].joint_homeoff[5], dh[i].joint_homeoff[6]);
    // }

    // CartesianPose base_offset[2] = {};
    // ret = robot.robot_get_default_base(LEFT, base_offset);
    // if (ret == ERR_SUCC)
    // {
    //     printf("Left base offset: %lf, %lf, %lf\n", base_offset[0].tran.x, base_offset[0].tran.y, base_offset[0].tran.z);
    // }
    // ret = robot.robot_get_default_base(RIGHT, base_offset + 1);
    // if (ret == ERR_SUCC)    {
    //     printf("Right base offset: %lf, %lf, %lf\n", base_offset[1].tran.x, base_offset[1].tran.y, base_offset[1].tran.z);
    // }
    // CartesianPose pos[2];
    // robot.kine_forward(LEFT, &start_pos[0], &pos[0]);
    // // robot.kine_forward(RIGHT, &start_pos[1], &pos[1]);
    // printf("left pos = %lf, %lf, %lf, %lf, %lf, %lf\n", pos[0].tran.x, pos[0].tran.y, pos[0].tran.z, pos[0].rpy.rx, pos[0].rpy.ry, pos[0].rpy.rz);

    

    //printf("right pos = %lf, %lf, %lf, %lf, %lf, %lf\n", pos[1].tran.x, pos[1].tran.y, pos[1].tran.z, pos[1].rpy.rx, pos[1].rpy.ry, pos[1].rpy.rz);
    // for (int i = 0; i < 5; i++)
    // {
    //     MoveMode moveop[2] = {INCR, INCR};
    //     double vel[2] = {1, 1};
    //     double acc[2] = {1, 1};
    //     double tol[2] = {5, 5};
    //     double id[2] = {0, 0};
    //     ret = robot.robot_run_multi_movj_extend(LEFT, moveop, FALSE, jstep_pos, vel, acc, tol);
    //     if (ret != ERR_SUCC)
    //     {
    //         std::this_thread::sleep_for(std::chrono::milliseconds(500));
    //         std::cout << "joint_move pos failed. ret = " << ret << std::endl;
    //     }
    //     else
    //     {
    //         std::this_thread::sleep_for(std::chrono::milliseconds(500));
    //         std::cout << "joint_move pos ok.\n";
    //     }

    //     // ret = robot.robot_run_multi_movj_extend(LEFT, moveop, FALSE, jstep_neg, vel, acc, tol);
    //     // if (ret == ERR_SUCC)
    //     // {
    //     //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
    //     //     std::cout << "joint_move neg ok.\n";
    //     // }
    //     // else
    //     // {
    //     //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
    //     //     std::cout << "joint_move neg failed. ret = " << ret << std::endl;
    //     // }
    // }
    // JointValue end_pos[2];
    // pos[0].tran.x += 20;
    // pos[0].tran.y += 20;
    // pos[0].tran.z += 20;
    // pos[1].tran.x += 20;
    // pos[1].tran.y += 20;
    // pos[1].tran.z += 20;
    // robot.kine_inverse(LEFT, &start_pos[0], &pos[0], &end_pos[0]);
    // robot.kine_inverse(RIGHT, &start_pos[1], &pos[1], &end_pos[1]);

    // printf("left end pos = %lf, %lf, %lf, %lf, %lf, %lf\n", end_pos[0].jVal[0], end_pos[0].jVal[1], end_pos[0].jVal[2], end_pos[0].jVal[3], end_pos[0].jVal[4], end_pos[0].jVal[5], end_pos[0].jVal[6]);
    // printf("right end pos = %lf, %lf, %lf, %lf, %lf, %lf\n", end_pos[1].jVal[0], end_pos[1].jVal[1], end_pos[1].jVal[2], end_pos[1].jVal[3], end_pos[1].jVal[4], end_pos[1].jVal[5], end_pos[0].jVal[6]);

    // robot.kine_forward(LEFT, &end_pos[0], &pos[0]);
    // robot.kine_forward(RIGHT, &end_pos[1], &pos[1]);
    // printf("left pos = %lf, %lf, %lf, %lf, %lf, %lf\n", pos[0].tran.x, pos[0].tran.y, pos[0].tran.z, pos[0].rpy.rx, pos[0].rpy.ry, pos[0].rpy.rz);
    // printf("right pos = %lf, %lf, %lf, %lf, %lf, %lf\n", pos[1].tran.x, pos[1].tran.y, pos[1].tran.z, pos[1].rpy.rx, pos[1].rpy.ry, pos[1].rpy.rz);

    // ret = robot.login_in("127.0.0.1");
    // ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");

    // DHParam dh[2] = {};
    // ret = robot.robot_get_multi_robot_dh(dh);
    // for (int i = 0; i < 2; i++)
    // {
    //     printf("Robot[%d] : Alpha, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", i, dh[i].alpha[0], dh[i].alpha[1], dh[i].alpha[2], dh[i].alpha[3], dh[i].alpha[4], dh[i].alpha[5], dh[i].alpha[6]);
    //     printf("Robot[%d] : d, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", i, dh[i].d[0], dh[i].d[1], dh[i].d[2], dh[i].d[3], dh[i].d[4], dh[i].d[5], dh[i].d[6]);
    //     printf("Robot[%d] : a, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", i, dh[i].a[0], dh[i].a[1], dh[i].a[2], dh[i].a[3], dh[i].a[4], dh[i].a[5], dh[i].a[6]);
    //     printf("Robot[%d] : joint_homeoff, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", i, dh[i].joint_homeoff[0], dh[i].joint_homeoff[1], dh[i].joint_homeoff[2], dh[i].joint_homeoff[3], dh[i].joint_homeoff[4], dh[i].joint_homeoff[5], dh[i].joint_homeoff[6]);
    // }
    
    std::cout << "hello world" << std::endl;
    return 0;
}
