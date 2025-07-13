#include <ros/ros.h>
#include <JAKAZuRobot.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mutex>
#include <pthread.h>
#include "timespec.h"
#include <thread>
#include <signal.h>
#include <atomic>

// 机械臂控制模式
enum ControlMode { NONE, JOINT_MODE, CARTESIAN_MODE };
// 角度/弧度转换宏
#define deg2rad(x) ((x)*M_PI/180.0)
#define rad2deg(x) ((x)*180.0/M_PI)
// 全局变量
JAKAZuRobot robot;
std::mutex control_mutex;
ControlMode current_mode = NONE;
JointValue target_joint;
JointValue zero_joint;
CartesianPose target_cartesian;
timespec next_cycle;

// 关节指令回调
void jointCommandCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(control_mutex);
    if (msg->position.size() >= 7) {
        for (int i = 0; i < 7; i++) {
            target_joint.jVal[i] = msg->position[i];
            // printf("target_joint: %.3f", target_joint.jVal[0]);
        }
        current_mode = JOINT_MODE;
        // printf("11");
        ROS_INFO("Received new joint command");
    }
}

// 笛卡尔指令回调
void cartesianCommandCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(control_mutex);
    
    // 位置
    target_cartesian.tran.x = msg->pose.position.x;
    target_cartesian.tran.y = msg->pose.position.y;
    target_cartesian.tran.z = msg->pose.position.z;
    
    // 姿态（四元数转欧拉角）
    tf2::Quaternion quat(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w
    );
    tf2::Matrix3x3 m(quat);
    m.getRPY(
        target_cartesian.rpy.rx,
        target_cartesian.rpy.ry,
        target_cartesian.rpy.rz
    );
    
    current_mode = CARTESIAN_MODE;
    
    ROS_INFO("Received new cartesian command");
}

// 实时控制线程
void* controlThread(void* arg) {
    // ros::NodeHandle* nh = (ros::NodeHandle*)arg;
    
    // // 新增：频率控制参数 [2,4](@ref)
    // double control_freq = 10; // 默认500Hz (2ms周期)
    // nh->getParam("control_frequency", control_freq); // 从ROS参数服务器读取
    // const long period_ns = static_cast<long>(1e9 / control_freq); // 计算周期(ns)
    ROS_INFO("Servo control thread init");
    // 设置实时线程优先级
    sched_param sch;
    sch.sched_priority = 90;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);
    
    // 初始化伺服模式
    robot.servo_move_enable(1, 0);
    robot.servo_move_enable(1, 1);
    clock_gettime(CLOCK_REALTIME, &next_cycle);
    
    ROS_INFO("Servo control thread started");
    
    // 用于存储当前机械臂状态【关键修改】
    JointValue current_joint;
    CartesianPose current_cartesian;
    
    while (1) {
        // 同步周期
        robot.edg_recv(&next_cycle);
        
        {
            std::lock_guard<std::mutex> lock(control_mutex);
            
            // 【关键修改】获取当前机械臂状态（索引0）
            robot.edg_get_stat(1, &current_joint, &current_cartesian);
            ROS_WARN("CHenking Joint State: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                current_joint.jVal[0], current_joint.jVal[1], current_joint.jVal[2],
                current_joint.jVal[3], current_joint.jVal[4], current_joint.jVal[5], current_joint.jVal[6]);
            // 执行当前控制模式
            switch (current_mode) {
                case JOINT_MODE:
                    // printf("22");
                    ROS_WARN("Sending Joint Cmd: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                        target_joint.jVal[0], target_joint.jVal[1], target_joint.jVal[2],
                        target_joint.jVal[3], target_joint.jVal[4], target_joint.jVal[5], target_joint.jVal[6]);
                        current_joint.jVal[0]=0;
                        current_joint.jVal[1]=0;
                        current_joint.jVal[2]=0;
                        current_joint.jVal[3]=0;
                        current_joint.jVal[4]=0;
                        current_joint.jVal[5]=0;
                        current_joint.jVal[6]=0;
                    robot.edg_servo_j(0, &current_joint, MoveMode::ABS);
                    // robot.edg_servo_j(0, &target_joint, MoveMode::ABS);
                    robot.edg_send();
                    break;
                case CARTESIAN_MODE:
                    robot.edg_servo_p(1, &target_cartesian, MoveMode::ABS);
                    robot.edg_send();
                    break;
                // default:
                //     // 【关键修改】无指令时发送当前位置保持
                //     // printf("33");
                //     robot.edg_servo_j(1, &current_joint, MoveMode::ABS);
            }
            
            
            //printf("33");
        }
        
        //等待下一个周期
        timespec dt;
        dt.tv_nsec = 800000;
        dt.tv_sec = 0;
        next_cycle = timespec_add(next_cycle, dt);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next_cycle, NULL);
    }
    return NULL;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_control_node");
    ros::NodeHandle nh;
    
    // 机械臂初始化
    robot.login_in("192.168.2.200");
    robot.clear_error();
    // robot.servo_move_enable(0, -1); //关闭所有机器人的伺服模式
    // robot.servo_move_use_none_filter();
    robot.power_on();
    robot.enable_robot();
    robot.motion_abort();
    // std::this_thread::sleep_for(std::chrono::seconds(5));
    
    ROS_INFO("Arm initialized");
    
    // 创建订阅者
    ros::Subscriber joint_sub = nh.subscribe("joint_commands", 1, jointCommandCallback);
    // ros::Subscriber cart_sub = nh.subscribe("cartesian_commands", 1, cartesianCommandCallback);
    CartesianPose servo_cpos[2];
    CartesianPose cpos[2];
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
    robot.robot_run_multi_movl(DUAL,mode,true,cpos,v,a);
    std::this_thread::sleep_for(std::chrono::seconds(5));



    // 启动实时控制线程
    pthread_t control_thread;
    pthread_create(&control_thread, NULL, controlThread, &nh);
    
    ROS_INFO("Control node ready");
    ros::spin();
    
    // 清理
    robot.disable_robot();
    robot.power_off();
    robot.login_out();
    pthread_join(control_thread, NULL);
    
    return 0;
}