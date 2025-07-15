#include <ros/ros.h>
#include <JAKAZuRobot.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mutex>
#include <pthread.h>
#include <thread>
#include <signal.h>
#include <atomic>
#include "timespec.h"

// 角度/弧度转换宏
#define deg2rad(x) ((x)*M_PI/180.0)
#define rad2deg(x) ((x)*180.0/M_PI)

// 安全退出标志
std::atomic<bool> keep_running(true);
CartesianPose servo_cpos[2];
CartesianPose cpos[2];
// 自定义信号处理函数
void SignalHandler(int signal) {
    keep_running = false;
    ros::shutdown();
    // 添加1秒缓冲确保线程收到退出信号
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

// 关节限位保护函数
template <typename T>
inline T clamp(T value, T min_val, T max_val) {
    return (value < min_val) ? min_val : (value > max_val) ? max_val : value;
}

// 机械臂控制全局变量
JAKAZuRobot robot;
std::mutex control_mutex;
JointValue target_joint;
timespec next_cycle;
double tt=0;
// 关节指令回调函数
void jointCommandCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    ROS_INFO_THROTTLE(0.5, "[Callback] Received joint command"); 
    std::lock_guard<std::mutex> lock(control_mutex);
    // if (msg->position.size() >= 7) {
    //     for (int i = 0; i < 7; i++) {
    //         // 关节限位保护 (-170°~170°)
    //         target_joint.jVal[i] = clamp(msg->position[i], 
    //                                      deg2rad(-170.0), 
    //                                      deg2rad(170.0));
    //     }
    //     ROS_INFO_THROTTLE(1.0, "receive joint cmd, joint 6: %.3f", target_joint.jVal[5]);
    // }
    ROS_INFO_THROTTLE(1.0, "receive joint cmd");
    double coefficient = sin(tt/1000.0);
    double z_step = 100 * coefficient;
    double x_step = 10 * coefficient;  
    double ori_step = deg2rad(10) * coefficient;

    servo_cpos[0].tran.z = cpos[0].tran.z + z_step;
    servo_cpos[0].tran.x = cpos[0].tran.x + x_step;
    servo_cpos[1].tran.z = cpos[1].tran.z + z_step;
    servo_cpos[1].tran.x = cpos[1].tran.x + x_step;

    tt+=50;
}

// 实时控制线程函数
void controlThread(JAKAZuRobot &robot) {
    // 设置实时优先级 (需要sudo权限)

    sched_param sch;
    sch.sched_priority = 50;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);

    // 初始化机械臂伺服模式
    robot.servo_move_enable(1, 0);
    robot.servo_move_enable(1, 1);

    clock_gettime(CLOCK_REALTIME, &next_cycle);
    
    // 状态变量
    JointValue current_joint;
    const timespec control_period = {0, 1000000}; // 1ms周期
    
    ROS_INFO("start real-time thread control (1kHz)");
    
    while (keep_running && ros::ok()) {
        ROS_DEBUG_THROTTLE(0.5, "[ControlThread] Sending command at %ld ns", next_cycle.tv_nsec);
        
        // 每次循环前检查退出标志
        if (!keep_running) break;  // 新增退出点

        // 同步控制周期
        robot.edg_recv(&next_cycle);
        
        // 获取当前机械臂状态
        JointValue now_jpos[2];
        CartesianPose now_cpos[2];
        // int64_t recv_time = 0;
        robot.edg_get_stat(0, &now_jpos[0], &now_cpos[0]);
        robot.edg_get_stat(1, &now_jpos[1], &now_cpos[1]);

        unsigned long int details[3];
        robot.edg_stat_details(details);
        
        {
            std::lock_guard<std::mutex> lock(control_mutex);
            // 发送关节指令
            robot.edg_servo_j(1, &target_joint, MoveMode::ABS);
            robot.edg_servo_j(0, &target_joint, MoveMode::ABS);
            robot.edg_send();
            ROS_INFO("edg_send, joint: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", rad2deg(target_joint.jVal[0]), rad2deg(target_joint.jVal[1]), 
            rad2deg(target_joint.jVal[2]), rad2deg(target_joint.jVal[3]), rad2deg(target_joint.jVal[4]), 
            rad2deg(target_joint.jVal[5]), rad2deg(target_joint.jVal[6]));
        }
        
        // 状态监控 (限流输出)
        ROS_INFO_THROTTLE(0.5, "joint_1: %.1f deg ...to... cmd: %.1f deg", 
                          rad2deg(current_joint.jVal[0]),
                          rad2deg(target_joint.jVal[0]));
        
        // 精确周期控制
        next_cycle = timespec_add(next_cycle, control_period);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next_cycle, NULL);
    }
    
    // 关闭伺服模式
    robot.servo_move_enable(0, 0);
    robot.servo_move_enable(0, 1);
    ROS_WARN("Control thread exited safely");  // 添加退出日志
    // return nullptr;
}

int main(int argc, char** argv) {
    // 自定义信号处理
    ros::init(argc, argv, "arm_control_node", ros::init_options::NoSigintHandler);
    signal(SIGINT, SignalHandler);
    ros::NodeHandle nh;
    ros::Subscriber joint_sub = nh.subscribe("joint_commands", 1, jointCommandCallback);
    // ros::Subscriber cart_sub = nh.subscribe("cartesian_commands", 1, cartesianCommandCallback);
    // // 机械臂初始化
    // JAKAZuRobot robot;
    // // errno_t ret;
    // robot.login_in("192.168.2.200");
    // robot.clear_error();
    // // robot.servo_move_enable(0, -1); //关闭所有机器人的伺服模式
    // // robot.servo_move_use_none_filter();
    // robot.power_on();
    // robot.enable_robot();
    // robot.motion_abort();
    // std::this_thread::sleep_for(std::chrono::seconds(5));
    // ROS_INFO("Finished robot initial!!! ");
    
    // // 创建订阅者
    // ros::Subscriber joint_sub = nh.subscribe("joint_commands", 10, jointCommandCallback);
    
    // // controlThread(robot);
    // // 启动控制线程
    // std::thread rt_thread(controlThread, std::ref(robot));

    // // 主线程ROS循环
    // ros::Rate rate(10); // 10Hz检查频率
    // while (keep_running && ros::ok()) {
    //     ros::spinOnce();  // 处理回调
    //     rate.sleep();
    // }

    // // 等待控制线程退出
    // keep_running = false;
    // if (rt_thread.joinable()) {
    //     rt_thread.join();  // 关键！等待线程结束
    // }

    // // 安全释放机械臂资源
    // robot.disable_robot();
    // robot.power_off();
    // robot.login_out();
    // ROS_INFO("Node closed gracefully");
    // return 0;

    JAKAZuRobot robot;

    robot.login_in("192.168.2.200");
    // robot.login_in("192.168.2.9");
    robot.clear_error();
    // robot.servo_move_enable(0, -1); //关闭所有机器人的伺服模式
    // robot.servo_move_use_none_filter();
    robot.power_on();
    robot.enable_robot();
    robot.motion_abort();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // servoj_test(robot);
    // servop_test(robot);

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
    
    robot.kine_forward(0,&jpos[0],&cpos[0]);
    robot.kine_forward(1,&jpos[1],&cpos[1]);
    cpos[0].tran.z += 10;
    cpos[1].tran.z += 10;
    // robot.robot_run_multi_movl(DUAL,mode,true,cpos,v,a);
    // std::this_thread::sleep_for(std::chrono::seconds(2));
    
    sched_param sch;
    sch.sched_priority = 90;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);
    robot.servo_move_enable(1, 0);
    robot.servo_move_enable(1, 1);
    timespec next;
    clock_gettime(CLOCK_REALTIME, &next);
    
    memcpy(servo_cpos,cpos,sizeof(servo_cpos));
    for(int i=0;;i++)
    {
        ros::spinOnce();  // 每次循环处理回调
        robot.edg_recv(&next);

        JointValue actjpos;
        CartesianPose actcpos;
        robot.edg_get_stat(0, &actjpos, &actcpos);
        robot.edg_get_stat(1, &actjpos, &actcpos);

        // double coefficient = sin(i/1000.0);
        // double z_step = 100 * coefficient;
        // double x_step = 10 * coefficient;  
        // double ori_step = deg2rad(10) * coefficient;
  
        // servo_cpos[0].tran.z = cpos[0].tran.z;
        // servo_cpos[0].tran.x = cpos[0].tran.x;
        // servo_cpos[1].tran.z = cpos[1].tran.z;
        // servo_cpos[1].tran.x = cpos[1].tran.x;

        // servo_cpos[0].tran.z = cpos[0].tran.z + z_step;
        // servo_cpos[0].tran.x = cpos[0].tran.x + x_step;
        // servo_cpos[1].tran.z = cpos[1].tran.z + z_step;
        // servo_cpos[1].tran.x = cpos[1].tran.x + x_step;

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
    
    robot.login_out();
    return 0;
}