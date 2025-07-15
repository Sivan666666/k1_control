#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <pthread.h>
#include <string.h>
#include <stdio.h>
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


// 角度/弧度转换宏
#define JK_PI (3.141592653589793)
#define deg_tp_rad 1.0 / 180.0 * JK_PI
#define deg2rad(x) ((x)*M_PI/180.0)
#define rad2deg(x) ((x)*180.0/M_PI)
// 定义JointValue结构体
JAKAZuRobot robot;
// 全局变量（直接访问，无保护）
JointValue target_left_joint;
JointValue target_right_joint;

JointValue state_jpos[2];
CartesianPose state_cpos[2];
uint32_t cmd_index = 0;


// 左臂回调函数
void leftArmCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
    robot.edg_get_stat(0, &state_jpos[0], &state_cpos[0]);
    // printf("left_stat=[%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f]\n", 
    //     state_jpos[0].jVal[0], state_jpos[0].jVal[1], state_jpos[0].jVal[2],
    //     state_jpos[0].jVal[3], state_jpos[0].jVal[4], state_jpos[0].jVal[5],state_jpos[0].jVal[6]
    //     );
    if (msg->points.size() > 0) {
        const trajectory_msgs::JointTrajectoryPoint* point = &msg->points.back();
        for (int i = 0; i < 7 && i < point->positions.size(); ++i) {
            target_left_joint.jVal[i] = point->positions[i];
        }
        ROS_INFO("Left Arm Joints: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                target_left_joint.jVal[0], target_left_joint.jVal[1],
                target_left_joint.jVal[2], target_left_joint.jVal[3],
                target_left_joint.jVal[4], target_left_joint.jVal[5],
                target_left_joint.jVal[6]);
        robot.edg_servo_j(0, &target_left_joint, MoveMode::ABS);
        int result = robot.edg_send(&cmd_index);
        cmd_index++;
        if (result != 0)
        {
            ROS_INFO("Failed to send servo_p command. Error code: %d", result);
        }
        
    }
    
}

// 右臂回调函数
void rightArmCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
    robot.edg_get_stat(1, &state_jpos[1], &state_cpos[1]);
    if (msg->points.size() > 0) {
        const trajectory_msgs::JointTrajectoryPoint* point = &msg->points.back();
        for (int i = 0; i < 7 && i < point->positions.size(); ++i) {
            target_right_joint.jVal[i] = point->positions[i];
        }
        // ROS_INFO("Right Arm Joints: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
        //         target_right_joint.jVal[0], target_right_joint.jVal[1],
        //         target_right_joint.jVal[2], target_right_joint.jVal[3],
        //         target_right_joint.jVal[4], target_right_joint.jVal[5],
        //         target_right_joint.jVal[6]);
    }
        robot.edg_servo_j(1, &target_right_joint, MoveMode::ABS);
        int result = robot.edg_send(&cmd_index);
        cmd_index++;
        if (result != 0)
        {
            ROS_INFO("Failed to send servo_p command. Error code: %d", result);
        }
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "k1_joint_subscriber_c");
    ros::NodeHandle nh;

    

    robot.login_in("192.168.2.200");
    // robot.login_in("192.168.2.9");
    robot.clear_error();
    robot.servo_move_enable(0, -1); //关闭所有机器人的伺服模式
    robot.servo_move_use_joint_NLF(80,80,80);
    robot.power_on();
    robot.enable_robot();
    robot.motion_abort();

    std::this_thread::sleep_for(std::chrono::seconds(5));

    JointValue jpos[2];
    memset(&jpos,0,sizeof(jpos));
    jpos[0].jVal[0] = deg2rad(-75.72);
    jpos[0].jVal[1] = deg2rad(-48.64);
    jpos[0].jVal[2] = deg2rad(69.35);
    jpos[0].jVal[3] = deg2rad(-132.21);
    jpos[0].jVal[4] = deg2rad(33.56);
    jpos[0].jVal[5] = deg2rad(-47.04);
    jpos[0].jVal[6] = deg2rad(-126.27);

    jpos[1].jVal[0] = deg2rad(53.13);
    jpos[1].jVal[1] = deg2rad(-31.42);
    jpos[1].jVal[2] = deg2rad(-29.38);
    jpos[1].jVal[3] = deg2rad(-132.20);
    jpos[1].jVal[4] = deg2rad(-50.51);
    jpos[1].jVal[5] = deg2rad(68.58);
    jpos[1].jVal[6] = deg2rad(153.78);
 
    double jv[2] = {deg2rad(50),deg2rad(50)};
    double ja[2] = {deg2rad(500),deg2rad(500)};
    MoveMode mode[]={MoveMode::ABS,MoveMode::ABS};
    robot.robot_run_multi_movj(DUAL,mode,true,jpos,jv,ja);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    double v[] = {50,50};
    double a[] = {200,200};
    CartesianPose cpos[2];
    robot.kine_forward(0,&jpos[0],&cpos[0]);
    robot.kine_forward(1,&jpos[1],&cpos[1]);
    cpos[0].tran.z += 10;
    cpos[1].tran.z += 10;
    // robot.robot_run_multi_movl(DUAL,mode,true,cpos,v,a);
    //std::this_thread::sleep_for(std::chrono::seconds(2));

    // MOVE INIT
    // cpos[0] = {200,  220, 200, deg_tp_rad * 160, 0, 0};
    // cpos[1] = {200,  -220, 200, deg_tp_rad * -160 , deg_tp_rad * -15, 0};
    // robot.robot_run_multi_movl(DUAL,mode,true,cpos,v,a);
    // std::this_thread::sleep_for(std::chrono::seconds(5));
    ROS_INFO("INIT");
    // MOVE INIT END

    int enable0 = robot.servo_move_enable(1, 0);
    int enable1 =robot.servo_move_enable(1, 1);

    if (enable0 != 0)
    {
            ROS_INFO("Failed to enable servo. Error code: %d", enable0);
    }
    if (enable1 != 0)
    {
            ROS_INFO("Failed to enable servo. Error code: %d", enable1);
    }

    // 初始化目标关节值
    memset(&target_left_joint, 0, sizeof(JointValue));
    memset(&target_right_joint, 0, sizeof(JointValue));
    
    // 创建订阅者
    ros::Subscriber left_sub = nh.subscribe("/left_arm_controller/command", 10, leftArmCallback);
    ros::Subscriber right_sub = nh.subscribe("/right_arm_controller/command", 10, rightArmCallback);
    
    // 直接进入ROS事件循环（不再需要手动循环）
    ros::spin();
    
    return 0;
}