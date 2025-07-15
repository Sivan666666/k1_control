#include "JAKAZuRobot.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <pthread.h>
#include "timespec.h"
#define rad2deg(x) ((x)*180.0/M_PI)
#define deg2rad(x) ((x)*M_PI/180.0)
#define JK_PI (3.141592653589793)
#define deg_tp_rad 1.0 / 180.0 * JK_PI
int64_t edg_timespec2ns(timespec t)
{
    return t.tv_sec * 1000000000 + t.tv_nsec;
}
void edg_sync(timespec reftime_, int64_t *sys_time_offset)
{
    auto reftime = edg_timespec2ns(reftime_);
    static int64_t integral = 0;
    int64_t cycletime = 1000000;
    int64_t delta = (reftime - 0) % cycletime;
    if (delta > (cycletime / 2))
    {
        delta = delta - cycletime;
    }
    if (delta > 0)
    {
        integral++;
    }
    if (delta < 0)
    {
        integral--;
    }
    if (sys_time_offset)
    {
        *sys_time_offset = -(delta / 100) - (integral / 20);  //类似PI调节
    }
}

int servoj_test(JAKAZuRobot &robot)
{
    JointValue jpos[2];
    memset(&jpos,0,sizeof(jpos));
    
    double v[] = {deg2rad(30),deg2rad(30)};
    double a[] = {deg2rad(150),deg2rad(150)};
    MoveMode mode[]={MoveMode::ABS,MoveMode::ABS};
    robot.robot_run_multi_movj(DUAL,mode,true,jpos,v,a);
    robot.servo_move_use_none_filter();
    // robot.servo_move_use_joint_LPF(125.0);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    JointValue jpos_cmd;
    memset(&jpos_cmd,0,sizeof(jpos_cmd));
    JointValue jpos_cmd2;
    memset(&jpos_cmd2,0,sizeof(jpos_cmd2));

    bool rob1_change_to_servo = false;
    // std::thread t([&](){
    //     JointValue jpos[2];
    //     memset(&jpos,0,sizeof(jpos));
    //     double v[] = {deg2rad(30),deg2rad(30)};
    //     double a[] = {deg2rad(150),deg2rad(150)};
    //     MoveMode mode[]={MoveMode::ABS,MoveMode::ABS};
    //     for(int i = 0;i<1;i++)
    //     {
    //         jpos[1].jVal[0] = deg2rad(30);
    //         robot.robot_run_multi_movj(RIGHT,mode,true,jpos,v,a);
    //         std::this_thread::sleep_for(std::chrono::seconds(1));
    //         jpos[1].jVal[0] = deg2rad(-30);
    //         robot.robot_run_multi_movj(RIGHT,mode,true,jpos,v,a);
    //         std::this_thread::sleep_for(std::chrono::seconds(1));
    //     }
    //     jpos[1].jVal[0] = deg2rad(0);
    //     robot.robot_run_multi_movj(RIGHT,mode,true,jpos,v,a);
    //     robot.servo_move_enable(1, 1);
    //     rob1_change_to_servo = true;

    // });
    sched_param sch;
    sch.sched_priority = 90;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);
    robot.servo_move_enable(1, 0);
    robot.servo_move_enable(1, 1);
    timespec next;
    clock_gettime(CLOCK_REALTIME, &next);
    
    int rob2_start_k = 0;
    for(int i = 0;;i++)
    {
        timespec max_wkup_time;
        robot.edg_recv(&next);
        timespec cur_time;
        timespec cc;
        clock_gettime(CLOCK_REALTIME, &cc);

        JointValue jpos[2];
        CartesianPose cpos[2];
        int64_t recv_time = 0;
        robot.edg_get_stat(0, &jpos[0], &cpos[0]);
        robot.edg_get_stat(1, &jpos[1], &cpos[1]);

        EdgRobotStat rbstat[2];
        robot.edg_get_stat(0,&rbstat[0]);
        robot.edg_get_stat(1,&rbstat[1]);

        unsigned long int details[3];
        robot.edg_stat_details(details);
    
        double t = (i - 0)/10000.0;
        double kk = 15;
        jpos_cmd.jVal[0] = sin(kk*t)*30/180.0*3.14;
        jpos_cmd.jVal[1] = -cos(kk*t)*20 /180.0*3.14 + 20/180.0*3.14;
        jpos_cmd.jVal[3] = -cos(kk*t)*10/180.0*3.14 + 10/180.0*3.14;

        jpos[1].jVal[0]=0;
        jpos[1].jVal[1]=0;
        jpos[1].jVal[2]=0;
        jpos[1].jVal[3]=0;
        jpos[1].jVal[4]=0;
        jpos[1].jVal[5]=0;
        jpos[1].jVal[6]=0;


        robot.edg_servo_j(0, &jpos_cmd, MoveMode::ABS);
        robot.edg_servo_j(1, &jpos_cmd, MoveMode::ABS);
        robot.edg_send();
        
#if 1
        printf("%ld %ld %ld %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %ld %ld %ld\n", 
        //std::chrono::duration_cast<std::chrono::nanoseconds>(cur.time_since_epoch()).count(),
        edg_timespec2ns(cc),
        edg_timespec2ns(cur_time),
        edg_timespec2ns(max_wkup_time),
        jpos[0].jVal[0], jpos[0].jVal[1], jpos[0].jVal[3],
        jpos[1].jVal[0], jpos[1].jVal[1], jpos[1].jVal[3],
        jpos_cmd.jVal[0], jpos_cmd.jVal[1], jpos_cmd.jVal[3],
        details[0], details[1], details[2]
        );


        
#endif
        //等待下一个周期
        printf("111");
        timespec dt;
        dt.tv_nsec = 1000000;
        dt.tv_sec = 0;
        next = timespec_add(next, dt);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL);
    }
}

// void servop_test(JAKAZuRobot &robot)
// {
//     JointValue jpos[2];
//     memset(&jpos,0,sizeof(jpos));
//     jpos[0].jVal[0] = deg2rad(28);
//     jpos[0].jVal[1] = deg2rad(33);
//     jpos[0].jVal[2] = deg2rad(-14);
//     jpos[0].jVal[3] = deg2rad(-55);
//     jpos[0].jVal[4] = deg2rad(-24);
//     jpos[0].jVal[5] = deg2rad(-53);
//     jpos[0].jVal[6] = deg2rad(40);

//     jpos[1].jVal[0] = deg2rad(-27);
//     jpos[1].jVal[1] = deg2rad(50);
//     jpos[1].jVal[2] = deg2rad(11);
//     jpos[1].jVal[3] = deg2rad(-75);
//     jpos[1].jVal[4] = deg2rad(18);
//     jpos[1].jVal[5] = deg2rad(30);
//     jpos[1].jVal[6] = deg2rad(25);
 
//     double jv[2] = {deg2rad(10),deg2rad(10)};
//     double ja[2] = {deg2rad(100),deg2rad(100)};
//     MoveMode mode[]={MoveMode::ABS,MoveMode::ABS};
//     robot.robot_run_multi_movj(DUAL,mode,true,jpos,jv,ja);

//     double v[] = {10,10};
//     double a[] = {100,100};
//     CartesianPose cpos[2];
//     robot.kine_forward(0,&jpos[0],&cpos[0]);
//     robot.kine_forward(1,&jpos[1],&cpos[1]);
//     cpos[0].tran.z += 10;
//     cpos[1].tran.z += 10;
//     robot.robot_run_multi_movl(DUAL,mode,true,cpos,v,a);
//     std::this_thread::sleep_for(std::chrono::seconds(2));
//     // 1. 定义Fling轨迹路径点（左右臂）[用户提供的轨迹数据]
//     CartesianPose left_path[] = {
//         {250,  220, 200, deg2rad(160), 0, 0},
//         {320,  200, 220, deg2rad(160), 0, 0},
//         {520,  200, 420, deg2rad(130), deg2rad(-20), deg2rad(35)},
//         {350,  200, 200, deg2rad(160), 0, 0},
//         {300,  200, 170, deg2rad(160), 0, 0},
//         {200,  220, 150, deg2rad(160), 0, 0}
//     };

//     CartesianPose right_path[] = {
//         {200,  -220, 200, deg2rad(-160), deg2rad(-15), 0},
//         {320,  -200, 220, deg2rad(-160), deg2rad(-15), 0},
//         {520,  -200, 420, deg2rad(-150), deg2rad(-30), deg2rad(-40)},
//         {350,  -200, 200, deg2rad(-160), deg2rad(-15), 0},
//         {300,  -200, 170, deg2rad(-160), deg2rad(-15), 0},
//         {200,  -220, 150, deg2rad(-160), deg2rad(-15), 0}
//     };

//     // 2. 轨迹参数设置
//     const int num_points = sizeof(left_path) / sizeof(left_path[0]);  // 路径点数量
//     const int step_per_segment = 1000;  // 每段轨迹执行周期数（1秒）
//     const int total_segments = num_points - 1;  // 总段数
//     const int total_steps = total_segments * step_per_segment;  // 总周期数

//     // 3. 设置实时优先级
//     sched_param sch;
//     sch.sched_priority = 90;
//     pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);
//     robot.servo_move_enable(1, 0);
//     robot.servo_move_enable(1, 1);
    
//     // 4. 初始化时间控制
//     timespec next;
//     clock_gettime(CLOCK_REALTIME, &next);
    
//     // 5. 主控制循环
//     for(int i = 0; ; i++)
//     {
//         robot.edg_recv(&next);
        
//         // 6. 轨迹插值计算
//         int current_step = i % total_steps;  // 循环执行轨迹
//         int segment_index = current_step / step_per_segment;
//         int step_in_segment = current_step % step_per_segment;
//         double ratio = static_cast<double>(step_in_segment) / step_per_segment;
        
//         // 7. 左右臂位置插值
//         CartesianPose* left_start = &left_path[segment_index];
//         CartesianPose* left_end = &left_path[segment_index + 1];
//         CartesianPose* right_start = &right_path[segment_index];
//         CartesianPose* right_end = &right_path[segment_index + 1];
        
//         // 8. 计算当前目标位置（线性插值）
//         CartesianPose target_left, target_right;
        
//         // 左臂插值
//         target_left.tran.x = left_start->tran.x + (left_end->tran.x - left_start->tran.x) * ratio;
//         target_left.tran.y = left_start->tran.y + (left_end->tran.y - left_start->tran.y) * ratio;
//         target_left.tran.z = left_start->tran.z + (left_end->tran.z - left_start->tran.z) * ratio;
//         target_left.rpy.rx = left_start->rpy.rx + (left_end->rpy.rx - left_start->rpy.rx) * ratio;
//         target_left.rpy.ry = left_start->rpy.ry + (left_end->rpy.ry - left_start->rpy.ry) * ratio;
//         target_left.rpy.rz = left_start->rpy.rz + (left_end->rpy.rz - left_start->rpy.rz) * ratio;
        
//         // 右臂插值
//         target_right.tran.x = right_start->tran.x + (right_end->tran.x - right_start->tran.x) * ratio;
//         target_right.tran.y = right_start->tran.y + (right_end->tran.y - right_start->tran.y) * ratio;
//         target_right.tran.z = right_start->tran.z + (right_end->tran.z - right_start->tran.z) * ratio;
//         target_right.rpy.rx = right_start->rpy.rx + (right_end->rpy.rx - right_start->rpy.rx) * ratio;
//         target_right.rpy.ry = right_start->rpy.ry + (right_end->rpy.ry - right_start->rpy.ry) * ratio;
//         target_right.rpy.rz = right_start->rpy.rz + (right_end->rpy.rz - right_start->rpy.rz) * ratio;
        
//         // 9. 发送控制指令
//         robot.edg_servo_p(0, &target_left, MoveMode::ABS);
//         robot.edg_servo_p(1, &target_right, MoveMode::ABS);
//         robot.edg_send();
        
//         // 10. 调试输出
//         CartesianPose actcpos;
//         robot.edg_get_stat(0, nullptr, &actcpos);
//         printf("Target: (%.1f, %.1f, %.1f) Actual: (%.1f, %.1f, %.1f)\n",
//                target_left.tran.x, target_left.tran.y, target_left.tran.z,
//                actcpos.tran.x, actcpos.tran.y, actcpos.tran.z);
        
//         // 11. 时间控制（1ms周期）
//         timespec dt = {0, 1000000};  // 1ms
//         next = timespec_add(next, dt);
//         clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, nullptr);
//     }
    
//     // 12. 退出时关闭伺服（虽然实际不会执行到这里）
//     robot.servo_move_enable(0, 0);
//     robot.servo_move_enable(0, 1);
// }

void servop_test(JAKAZuRobot &robot)
{
    JointValue jpos[2];
    memset(&jpos,0,sizeof(jpos));
    jpos[0].jVal[0] = deg2rad(28);
    jpos[0].jVal[1] = deg2rad(33);
    jpos[0].jVal[2] = deg2rad(-14);
    jpos[0].jVal[3] = deg2rad(-55);
    jpos[0].jVal[4] = deg2rad(-24);
    jpos[0].jVal[5] = deg2rad(-53);
    jpos[0].jVal[6] = deg2rad(40);

    jpos[1].jVal[0] = deg2rad(-27);
    jpos[1].jVal[1] = deg2rad(50);
    jpos[1].jVal[2] = deg2rad(11);
    jpos[1].jVal[3] = deg2rad(-75);
    jpos[1].jVal[4] = deg2rad(18);
    jpos[1].jVal[5] = deg2rad(30);
    jpos[1].jVal[6] = deg2rad(25);
 
    double jv[2] = {deg2rad(10),deg2rad(10)};
    double ja[2] = {deg2rad(100),deg2rad(100)};
    MoveMode mode[]={MoveMode::ABS,MoveMode::ABS};
    robot.robot_run_multi_movj(DUAL,mode,true,jpos,jv,ja);

    double v[] = {10,10};
    double a[] = {100,100};
    CartesianPose cpos[2];
    robot.kine_forward(0,&jpos[0],&cpos[0]);
    robot.kine_forward(1,&jpos[1],&cpos[1]);
    cpos[0].tran.z += 10;
    cpos[1].tran.z += 10;
    robot.robot_run_multi_movl(DUAL,mode,true,cpos,v,a);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    sched_param sch;
    sch.sched_priority = 90;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);
    robot.servo_move_enable(1, 0);
    robot.servo_move_enable(1, 1);
    timespec next;
    clock_gettime(CLOCK_REALTIME, &next);
    CartesianPose servo_cpos[2];
    memcpy(servo_cpos,cpos,sizeof(servo_cpos));
    for(int i=0;;i++)
    {
        robot.edg_recv(&next);

        JointValue actjpos;
        CartesianPose actcpos;
        robot.edg_get_stat(0, &actjpos, &actcpos);
        robot.edg_get_stat(1, &actjpos, &actcpos);

        double coefficient = sin(i/1000.0);
        double z_step = 100 * coefficient;
        double x_step = 10 * coefficient;  
        double ori_step = deg2rad(10) * coefficient;
  
        servo_cpos[0].tran.z = cpos[0].tran.z;
        servo_cpos[0].tran.x = cpos[0].tran.x;
        servo_cpos[1].tran.z = cpos[1].tran.z;
        servo_cpos[1].tran.x = cpos[1].tran.x;

        servo_cpos[0].tran.z = cpos[0].tran.z + z_step;
        servo_cpos[0].tran.x = cpos[0].tran.x + x_step;
        servo_cpos[1].tran.z = cpos[1].tran.z + z_step;
        servo_cpos[1].tran.x = cpos[1].tran.x + x_step;

        CartesianPose left_path[] = {  // 起始点
            {200,  220, 200, deg_tp_rad * 160, 0, 0},
            {320,  200, 220, deg_tp_rad * 160, 0, 0},
            {520,  200, 420, deg_tp_rad * 130, deg_tp_rad * -20, deg_tp_rad * 35},
            {350,  200, 200, deg_tp_rad * 160, 0, 0},
            {300,  200, 170, deg_tp_rad * 160, 0, 0},
            {200,  220, 150, deg_tp_rad * 160, 0, 0}   // 结束点
        };
    
        // 右臂路径点 (Y坐标取反)
        CartesianPose right_path[] = {// 起始点
            {49,  -756, 314, deg_tp_rad * 93 , deg_tp_rad * -22, },
            {320,  -200, 220, deg_tp_rad * -160 , deg_tp_rad * -15, 0},
            {520,  -200, 420, deg_tp_rad * -150 , deg_tp_rad * -30, deg_tp_rad * -40},
            {350,  -200, 200, deg_tp_rad * -160 , deg_tp_rad * -15, 0},
            {300,  -200, 170, deg_tp_rad * -160 , deg_tp_rad * -15, 0},
            {200,  -220, 150, deg_tp_rad * -160 , deg_tp_rad * -15, 0}  // 结束点
        };
        robot.edg_servo_p(0,&servo_cpos[0],MoveMode::ABS);
        robot.edg_servo_p(1,&servo_cpos[1],MoveMode::ABS);
        robot.edg_send();
        printf("%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f\n", actcpos.tran.x, actcpos.tran.y, actcpos.tran.z, actcpos.rpy.rx, actcpos.rpy.ry, actcpos.rpy.rz);


        //等待下一个周期
        timespec dt;
        dt.tv_nsec = 1000000;
        dt.tv_sec = 0;
        next = timespec_add(next, dt);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL);
    }
    robot.servo_move_enable(0, 0);
    robot.servo_move_enable(0, 1);
}

int main()
{
    //
    JAKAZuRobot robot;

    robot.login_in("192.168.2.200");
    // robot.login_in("192.168.2.9");
    robot.clear_error();
    robot.servo_move_enable(0, -1); //关闭所有机器人的伺服模式
    robot.servo_move_use_none_filter();
    robot.power_on();
    robot.enable_robot();
    robot.motion_abort();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // servoj_test(robot);
    servop_test(robot);

    // JointValue jpos[2];
    // memset(&jpos,0,sizeof(jpos));
    // jpos[0].jVal[0] = deg2rad(28);
    // jpos[0].jVal[1] = deg2rad(33);
    // jpos[0].jVal[2] = deg2rad(-14);
    // jpos[0].jVal[3] = deg2rad(-55);
    // jpos[0].jVal[4] = deg2rad(-24);
    // jpos[0].jVal[5] = deg2rad(-53);
    // jpos[0].jVal[6] = deg2rad(40);

    // jpos[1].jVal[0] = deg2rad(-27);
    // jpos[1].jVal[1] = deg2rad(50);
    // jpos[1].jVal[2] = deg2rad(11);
    // jpos[1].jVal[3] = deg2rad(-75);
    // jpos[1].jVal[4] = deg2rad(18);
    // jpos[1].jVal[5] = deg2rad(30);
    // jpos[1].jVal[6] = deg2rad(25);
 
    // double jv[2] = {deg2rad(10),deg2rad(10)};
    // double ja[2] = {deg2rad(100),deg2rad(100)};
    // MoveMode mode[]={MoveMode::ABS,MoveMode::ABS};
    // robot.robot_run_multi_movj(DUAL,mode,true,jpos,jv,ja);

    // double v[] = {10,10};
    // double a[] = {100,100};
    // CartesianPose cpos[2];
    // robot.kine_forward(0,&jpos[0],&cpos[0]);
    // robot.kine_forward(1,&jpos[1],&cpos[1]);
    // cpos[0].tran.z += 10;
    // cpos[1].tran.z += 10;
    // robot.robot_run_multi_movl(DUAL,mode,true,cpos,v,a);
    // std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // sched_param sch;
    // sch.sched_priority = 90;
    // pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);
    // robot.servo_move_enable(1, 0);
    // robot.servo_move_enable(1, 1);
    // timespec next;
    // clock_gettime(CLOCK_REALTIME, &next);
    // CartesianPose servo_cpos[2];
    // memcpy(servo_cpos,cpos,sizeof(servo_cpos));
    // for(int i=0;;i++)
    // {
    //     robot.edg_recv(&next);

    //     JointValue actjpos;
    //     CartesianPose actcpos;
    //     robot.edg_get_stat(0, &actjpos, &actcpos);
    //     robot.edg_get_stat(1, &actjpos, &actcpos);

    //     printf("%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f\n", actcpos.tran.x, actcpos.tran.y, actcpos.tran.z, actcpos.rpy.rx, actcpos.rpy.ry, actcpos.rpy.rz);
    //     printf("%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f\n", servo_cpos[0].tran.x, servo_cpos[0].tran.y, servo_cpos[0].tran.z, servo_cpos[0].rpy.rx, servo_cpos[0].rpy.ry, servo_cpos[0].rpy.rz);

    //     //printf("%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f\n", actcpos.tran.x, actcpos.tran.y, actcpos.tran.z, actcpos.rpy.rx, actcpos.rpy.ry, actcpos.rpy.rz);
    //     //printf("%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f\n", servo_cpos[1].tran.x, servo_cpos[1].tran.y, servo_cpos[1].tran.z, servo_cpos[1].rpy.rx, servo_cpos[1].rpy.ry, servo_cpos[1].rpy.rz);

    //     double coefficient = sin(i/1000.0);
    //     double z_step = 100 * coefficient;
    //     double x_step = 10 * coefficient;  
    //     double ori_step = deg2rad(10) * coefficient;
  
    //     servo_cpos[0].tran.z = cpos[0].tran.z;
    //     servo_cpos[0].tran.x = cpos[0].tran.x;
    //     servo_cpos[1].tran.z = cpos[1].tran.z;
    //     servo_cpos[1].tran.x = cpos[1].tran.x;

    //     servo_cpos[0].tran.z = 300;
    //     servo_cpos[0].tran.x = cpos[0].tran.x + x_step;
    //     servo_cpos[1].tran.z = cpos[1].tran.z + z_step;
    //     servo_cpos[1].tran.x = cpos[1].tran.x + x_step;

    //     actcpos.rpy.rx = deg2rad(actcpos.rpy.rx);
    //     actcpos.rpy.ry = deg2rad(actcpos.rpy.ry);
    //     actcpos.rpy.rz = deg2rad(actcpos.rpy.rz);

    //     robot.edg_servo_p(0,&servo_cpos[0],MoveMode::ABS);
    //     robot.edg_servo_p(1,&actcpos,MoveMode::ABS);
    //     robot.edg_send();
        

    //     //等待下一个周期
    //     timespec dt;
    //     dt.tv_nsec = 1000000;
    //     dt.tv_sec = 0;
    //     next = timespec_add(next, dt);
    //     clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL);
    // }
    // robot.servo_move_enable(0, 0);
    // robot.servo_move_enable(0, 1);
    
    robot.login_out();
    return 0;
}