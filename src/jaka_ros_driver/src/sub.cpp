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

ros::Publisher left_arm_jpos_pub;
ros::Publisher left_arm_cpos_pub;
ros::Publisher right_arm_jpos_pub;
ros::Publisher right_arm_cpos_pub;
ros::Publisher left_end_joint_pub;
ros::Publisher joint_states_pub;

const std::vector<std::string> ALL_JOINT_NAMES = {
    "r-j1", "r-j2", "r-j3", "r-j4", "r-j5", "r-j6", "r-j7",
    "right_finger1_joint", "right_finger2_joint",  
    "l-j1", "l-j2", "l-j3", "l-j4", "l-j5", "l-j6", "l-j7",
    "left_finger1_joint", "left_finger2_joint" 
};

// 左臂回调函数
void leftArmCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
    // robot.edg_get_stat(0, &state_jpos[0], &state_cpos[0]);
    // printf("left_stat=[%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f]\n", 
    //     state_jpos[0].jVal[0], state_jpos[0].jVal[1], state_jpos[0].jVal[2],
    //     state_jpos[0].jVal[3], state_jpos[0].jVal[4], state_jpos[0].jVal[5],state_jpos[0].jVal[6]
    //     );
    if (msg->points.size() > 0) {
        const trajectory_msgs::JointTrajectoryPoint* point = &msg->points.back();
        for (int i = 0; i < 7 && i < point->positions.size(); ++i) {
            target_left_joint.jVal[i] = point->positions[i];
        }
        // ROS_INFO("Left Arm Joints: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
        //         target_left_joint.jVal[0], target_left_joint.jVal[1],
        //         target_left_joint.jVal[2], target_left_joint.jVal[3],
        //         target_left_joint.jVal[4], target_left_joint.jVal[5],
        //         target_left_joint.jVal[6]);
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
    // robot.edg_get_stat(1, &state_jpos[1], &state_cpos[1]);
    if (msg->points.size() > 0) {
        const trajectory_msgs::JointTrajectoryPoint* point = &msg->points.back();
        for (int i = 0; i < 7 && i < point->positions.size(); ++i) {
            target_right_joint.jVal[i] = point->positions[i];
        }
        ROS_INFO("Right Arm Joints: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                target_right_joint.jVal[0], target_right_joint.jVal[1],
                target_right_joint.jVal[2], target_right_joint.jVal[3],
                target_right_joint.jVal[4], target_right_joint.jVal[5],
                target_right_joint.jVal[6]);
    }
        robot.edg_servo_j(1, &target_right_joint, MoveMode::ABS);
        int result = robot.edg_send(&cmd_index);
        cmd_index++;
        if (result != 0)
        {
            ROS_INFO("Failed to send servo_p command. Error code: %d", result);
        }
}

// 控制定时器回调函数
void controlTimerCallback(const ros::TimerEvent& event) {
    
    
    // 获取当前状态
    robot.edg_get_stat(0, &state_jpos[0], &state_cpos[0]);
    robot.edg_get_stat(1, &state_jpos[1], &state_cpos[1]);
    // ROS_INFO("trans: left[%.3f, %.3f, %.3f], right[%.3f, %.3f, %.3f]", 
    //     state_cpos[0].tran.x, state_cpos[0].tran.y, state_cpos[0].tran.z,
    //     state_cpos[1].tran.x, state_cpos[1].tran.y, state_cpos[1].tran.z);
    // ROS_INFO("rotation: left[%.3f, %.3f, %.3f], right[%.3f, %.3f, %.3f]", 
    //     state_cpos[0].rpy.rx, state_cpos[0].rpy.ry, state_cpos[0].rpy.rz,
    //     state_cpos[1].rpy.rx, state_cpos[1].rpy.ry, state_cpos[1].rpy.rz);
    // printf("rotation=[%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f]\n", 
    //     state_cpos[0].rpy.rx, state_cpos[0].rpy.ry, state_cpos[0].rpy.rz,
    //     state_cpos[1].rpy.rx, state_cpos[1].rpy.ry, state_cpos[1].rpy.rz
    //     );
    // printf("left_stat=[%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f]\n", 
    //     state_jpos[1].jVal[0], state_jpos[1].jVal[1], state_jpos[1].jVal[2],
    //     state_jpos[1].jVal[3], state_jpos[1].jVal[4], state_jpos[1].jVal[5],state_jpos[1].jVal[6]
    //     );

    ros::Time now = ros::Time::now();

    // 2.创建并填充统一的 /joint_states 消息
    sensor_msgs::JointState unified_joint_state_msg;
    unified_joint_state_msg.header.stamp = now;
    unified_joint_state_msg.name = ALL_JOINT_NAMES;
    unified_joint_state_msg.position.reserve(18);
    unified_joint_state_msg.position.insert(unified_joint_state_msg.position.end(), state_jpos[1].jVal, state_jpos[1].jVal + 7); // 右臂
    unified_joint_state_msg.position.push_back(0.0); // 右夹爪 joint 1
    unified_joint_state_msg.position.push_back(0.0); // 右夹爪 joint 2
    
    unified_joint_state_msg.position.insert(unified_joint_state_msg.position.end(), state_jpos[0].jVal, state_jpos[0].jVal + 7); // 左臂
    unified_joint_state_msg.position.push_back(0.0); // 左夹爪 joint 1
    unified_joint_state_msg.position.push_back(0.0); // 左夹爪 joint 2
    
    // 如果你的JAKA SDK `edg_get_stat` 不返回夹爪状态，可以先用0.0占位
    // 确保 `position` 数组的大小与 `name` 数组的大小 (18) 一致
    // 假设夹爪占位
    unified_joint_state_msg.position.insert(unified_joint_state_msg.position.begin() + 7, 2, 0.0); // 在右臂后插入2个0
    unified_joint_state_msg.position.insert(unified_joint_state_msg.position.end(), 2, 0.0);     // 在左臂后插入2个0

    // 发布这个统一的消息
    joint_states_pub.publish(unified_joint_state_msg);


    // 创建并发布左臂关节状态消息
    sensor_msgs::JointState left_jpos_msg;
    left_jpos_msg.header.stamp = ros::Time::now();
    left_jpos_msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    left_jpos_msg.position.assign(state_jpos[0].jVal, state_jpos[0].jVal + 7);
    left_arm_jpos_pub.publish(left_jpos_msg);

    // 创建并发布左臂末端位姿消息
    geometry_msgs::PoseStamped left_cpos_msg;
    left_cpos_msg.header.stamp = ros::Time::now();
    left_cpos_msg.header.frame_id = "base_link";
    left_cpos_msg.pose.position.x = state_cpos[0].tran.x;
    left_cpos_msg.pose.position.y = state_cpos[0].tran.y;
    left_cpos_msg.pose.position.z = state_cpos[0].tran.z;
    
    
    // 将RPY转换为四元数
    tf2::Quaternion quat;
    quat.setRPY(state_cpos[0].rpy.rx, state_cpos[0].rpy.ry, state_cpos[0].rpy.rz);
    left_cpos_msg.pose.orientation.x = quat.x();
    left_cpos_msg.pose.orientation.y = quat.y();
    left_cpos_msg.pose.orientation.z = quat.z();
    left_cpos_msg.pose.orientation.w = quat.w();
    left_arm_cpos_pub.publish(left_cpos_msg);
    

    // 创建并发布右臂关节状态消息
    sensor_msgs::JointState right_jpos_msg;
    right_jpos_msg.header.stamp = ros::Time::now();
    right_jpos_msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    right_jpos_msg.position.assign(state_jpos[1].jVal, state_jpos[1].jVal + 7);
    right_arm_jpos_pub.publish(right_jpos_msg);

    // 创建并发布右臂末端位姿消息
    geometry_msgs::PoseStamped right_cpos_msg;
    right_cpos_msg.header.stamp = ros::Time::now();
    right_cpos_msg.header.frame_id = "base_link";
    right_cpos_msg.pose.position.x = state_cpos[1].tran.x;
    right_cpos_msg.pose.position.y = state_cpos[1].tran.y;
    right_cpos_msg.pose.position.z = state_cpos[1].tran.z;
    
    // 将RPY转换为四元数
    quat.setRPY(state_cpos[1].rpy.rx, state_cpos[1].rpy.ry, state_cpos[1].rpy.rz);
    right_cpos_msg.pose.orientation.x = quat.x();
    right_cpos_msg.pose.orientation.y = quat.y();
    right_cpos_msg.pose.orientation.z = quat.z();
    right_cpos_msg.pose.orientation.w = quat.w();
    right_arm_cpos_pub.publish(right_cpos_msg);
    
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "k1_joint_subscriber_c");
    ros::NodeHandle nh;

    

    robot.login_in("192.168.2.200");
    // robot.login_in("192.168.2.9");
    robot.clear_error();
    robot.servo_move_enable(0, -1); //关闭所有机器人的伺服模式
    robot.servo_move_use_joint_NLF(90,180,680);
    robot.power_on();
    robot.enable_robot();
    robot.motion_abort();

    std::this_thread::sleep_for(std::chrono::seconds(5));

    JointValue jpos[2];
    memset(&jpos,0,sizeof(jpos));

    // fling
    jpos[0].jVal[0] = deg2rad(-75.72);
    jpos[0].jVal[1] = deg2rad(-48.64);
    jpos[0].jVal[2] = deg2rad(69.35);
    jpos[0].jVal[3] = deg2rad(-132.21);
    jpos[0].jVal[4] = deg2rad(33.56);
    jpos[0].jVal[5] = deg2rad(-47.04);
    jpos[0].jVal[6] = deg2rad(-126.27);

    jpos[1].jVal[0] = deg2rad(75.72);
    jpos[1].jVal[1] = deg2rad(-48.64);
    jpos[1].jVal[2] = deg2rad(-69.35);
    jpos[1].jVal[3] = deg2rad(-132.21);
    jpos[1].jVal[4] = deg2rad(-33.56);
    jpos[1].jVal[5] = deg2rad(47.04);
    jpos[1].jVal[6] = deg2rad(126.27);
    
    // grasp
    jpos[0].jVal[0] = deg2rad(1.4);
    jpos[0].jVal[1] = deg2rad(-78);
    jpos[0].jVal[2] = deg2rad(72.5);
    jpos[0].jVal[3] = deg2rad(-82.3);
    jpos[0].jVal[4] = deg2rad(50.5);
    jpos[0].jVal[5] = deg2rad(-5.6);
    jpos[0].jVal[6] = deg2rad(-126.3);

    jpos[1].jVal[0] = deg2rad(-1.4);
    jpos[1].jVal[1] = deg2rad(-78);
    jpos[1].jVal[2] = deg2rad(-72.5);
    jpos[1].jVal[3] = deg2rad(-82.3);
    jpos[1].jVal[4] = deg2rad(-50.5);
    jpos[1].jVal[5] = deg2rad(5.6);
    jpos[1].jVal[6] = deg2rad(126.3);



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
    // STATE PUB
    // 定义发布者
    left_arm_jpos_pub = nh.advertise<sensor_msgs::JointState>("/left_arm/jpos", 10);
    left_arm_cpos_pub = nh.advertise<geometry_msgs::PoseStamped>("/left_arm/cpos", 10);
    right_arm_jpos_pub = nh.advertise<sensor_msgs::JointState>("/right_arm/jpos", 10);
    right_arm_cpos_pub = nh.advertise<geometry_msgs::PoseStamped>("/right_arm/cpos", 10);
    joint_states_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    // 创建8ms定时器
    ros::Timer control_timer = nh.createTimer(ros::Duration(0.008), controlTimerCallback);
    // 直接进入ROS事件循环（不再需要手动循环）
    ros::spin();
    
    return 0;
}