#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <JAKAZuRobot.h>
#include "jktypes.h"
#include "timespec.h"

#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <string>
#include <signal.h>
#include <pthread.h>


// 角度/弧度转换宏
#define JK_PI (3.141592653589793)
#define deg_tp_rad 1.0 / 180.0 * JK_PI
#define deg2rad(x) ((x)*M_PI/180.0)
#define rad2deg(x) ((x)*180.0/M_PI)


JAKAZuRobot robot;
JointValue target_left_joint;
JointValue target_right_joint;
JointValue current_left_joint;
JointValue current_right_joint;
CartesianPose current_left_pose;
CartesianPose current_right_pose;

JointValue state_jpos[2];
CartesianPose state_cpos[2];
uint32_t cmd_index = 0;


std::mutex robot_data_mutex; // 用于保护共享数据的互斥锁
std::atomic<bool> app_running(true); // 控制程序运行的原子标


const std::vector<std::string> ALL_JOINT_NAMES = {
    "r-j1", "r-j2", "r-j3", "r-j4", "r-j5", "r-j6", "r-j7",
    "right_finger1_joint", "right_finger2_joint",  
    "l-j1", "l-j2", "l-j3", "l-j4", "l-j5", "l-j6", "l-j7",
    "left_finger1_joint", "left_finger2_joint" 
};

void convertJointValueToRad(JointValue& joint_value) {
    for (int i = 0; i < 7; ++i) {
        joint_value.jVal[i] = deg2rad(joint_value.jVal[i]);
    }
};

void signalHandler(int signum) {
    ROS_INFO("Interrupt signal (%d) received. Shutting down.", signum);
    app_running = false;
}


// 左臂回调函数
void leftArmCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
    if (msg->points.empty()) {
        return;
    }

    const auto& point = msg->points.back();
    std::lock_guard<std::mutex> lock(robot_data_mutex);
    for (int i = 0; i < 7 && i < point.positions.size(); ++i) {
        target_left_joint.jVal[i] = point.positions[i];
    }
    ROS_INFO_THROTTLE(1.0, "Received new target for left arm.");
}

// 右臂指令回调函数
void rightArmCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
    if (msg->points.empty()) {
        return;
    }

    const auto& point = msg->points.back();
    std::lock_guard<std::mutex> lock(robot_data_mutex);
    for (int i = 0; i < 7 && i < point.positions.size(); ++i) {
        target_right_joint.jVal[i] = point.positions[i];
    }
    // ROS_INFO_THROTTLE(1.0, "Received new target for right arm.");
}

void real_time_control_thread() {
    
    // 提升线程优先级为实时
    sched_param sch;
    sch.sched_priority = 90; // 优先级可以根据系统情况调整
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch) != 0) {
        ROS_WARN("Failed to set real-time scheduling policy. Timing precision may be affected.");
    }

    timespec next_cycle_time;
    clock_gettime(CLOCK_REALTIME, &next_cycle_time);

    uint32_t cmd_index = 0;

    ROS_INFO("Real-time control thread started.");

    while (app_running) {
        // 1. 触发内部接收事件并刷新缓存
        robot.edg_recv(&next_cycle_time);

        // 2. 获取机器人当前状态
        {
            std::lock_guard<std::mutex> lock(robot_data_mutex);
            robot.edg_get_stat(0, &current_left_joint, &current_left_pose);
            robot.edg_get_stat(1, &current_right_joint, &current_right_pose);
        }


        // 3. 设置双臂的伺服指令
        {
            std::lock_guard<std::mutex> lock(robot_data_mutex);
            // 即使没有新指令，也用当前目标值（即上一周期的目标值）设置，实现位置保持
            robot.edg_servo_j(0, &target_left_joint, MoveMode::ABS);
            robot.edg_servo_j(1, &target_right_joint, MoveMode::ABS);
        }


        // 4. 将指令同步发送给机器人
        int result = robot.edg_send();
        if (result != 0) {
            ROS_WARN("Failed to send servo command. Error code: %d", result);
        }

        // 5. 计算下一个周期的唤醒时间 (8ms周期)
        timespec cycle_duration = {0, 8000000}; // 8ms
        next_cycle_time = timespec_add(next_cycle_time, cycle_duration);

        // 6. 精确睡眠到下一个周期开始
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next_cycle_time, NULL);
    }
    ROS_INFO("Real-time control thread finished.");
}


void state_publisher_thread(ros::NodeHandle& nh) {
    // 定义发布者
    ros::Publisher left_arm_jpos_pub = nh.advertise<sensor_msgs::JointState>("/left_arm/jpos", 10);
    ros::Publisher left_arm_cpos_pub = nh.advertise<geometry_msgs::PoseStamped>("/left_arm/cpos", 10);
    ros::Publisher right_arm_jpos_pub = nh.advertise<sensor_msgs::JointState>("/right_arm/jpos", 10);
    ros::Publisher right_arm_cpos_pub = nh.advertise<geometry_msgs::PoseStamped>("/right_arm/cpos", 10);
    ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    ros::Rate rate(125); // 以125Hz（8ms）的频率发布

    ROS_INFO("State publisher thread started.");

    while (ros::ok() && app_running) {
        ros::Time now = ros::Time::now();
        JointValue local_left_joint, local_right_joint;
        CartesianPose local_left_pose, local_right_pose;

        // 从共享内存中拷贝状态数据
        {
            std::lock_guard<std::mutex> lock(robot_data_mutex);
            local_left_joint = current_left_joint;
            local_right_joint = current_right_joint;
            local_left_pose = current_left_pose;
            local_right_pose = current_right_pose;
        }

        // 发布统一的 /joint_states 消息
        sensor_msgs::JointState unified_joint_state_msg;
        unified_joint_state_msg.header.stamp = now;
        unified_joint_state_msg.name = ALL_JOINT_NAMES;
        unified_joint_state_msg.position.resize(18, 0.0);

        // 填充右臂数据 (ID: 1)
        for(int i=0; i<7; ++i) unified_joint_state_msg.position[i] = local_right_joint.jVal[i];
        // 填充左臂数据 (ID: 0)
        for(int i=0; i<7; ++i) unified_joint_state_msg.position[i+9] = local_left_joint.jVal[i];
        // 夹爪位置可以根据实际情况更新，此处为0
        joint_states_pub.publish(unified_joint_state_msg);


        // 发布左臂独立状态
        sensor_msgs::JointState left_jpos_msg;
        left_jpos_msg.header.stamp = now;
        left_jpos_msg.position.assign(local_left_joint.jVal, local_left_joint.jVal + 7);
        left_arm_jpos_pub.publish(left_jpos_msg);

        geometry_msgs::PoseStamped left_cpos_msg;
        left_cpos_msg.header.stamp = now;
        left_cpos_msg.header.frame_id = "base_link";
        left_cpos_msg.pose.position.x = local_left_pose.tran.x;
        left_cpos_msg.pose.position.y = local_left_pose.tran.y;
        left_cpos_msg.pose.position.z = local_left_pose.tran.z;
        tf2::Quaternion quat_l;
        quat_l.setRPY(local_left_pose.rpy.rx, local_left_pose.rpy.ry, local_left_pose.rpy.rz);
        left_cpos_msg.pose.orientation.x = quat_l.x();
        left_cpos_msg.pose.orientation.y = quat_l.y();
        left_cpos_msg.pose.orientation.z = quat_l.z();
        left_cpos_msg.pose.orientation.w = quat_l.w();
        left_arm_cpos_pub.publish(left_cpos_msg);


        // 发布右臂独立状态
        sensor_msgs::JointState right_jpos_msg;
        right_jpos_msg.header.stamp = now;
        right_jpos_msg.position.assign(local_right_joint.jVal, local_right_joint.jVal + 7);
        right_arm_jpos_pub.publish(right_jpos_msg);

        geometry_msgs::PoseStamped right_cpos_msg;
        right_cpos_msg.header.stamp = now;
        right_cpos_msg.header.frame_id = "base_link";
        right_cpos_msg.pose.position.x = local_right_pose.tran.x;
        right_cpos_msg.pose.position.y = local_right_pose.tran.y;
        right_cpos_msg.pose.position.z = local_right_pose.tran.z;
        tf2::Quaternion quat_r;
        quat_r.setRPY(local_right_pose.rpy.rx, local_right_pose.rpy.ry, local_right_pose.rpy.rz);
        right_cpos_msg.pose.orientation.x = quat_r.x();
        right_cpos_msg.pose.orientation.y = quat_r.y();
        right_cpos_msg.pose.orientation.z = quat_r.z();
        right_cpos_msg.pose.orientation.w = quat_r.w();
        right_arm_cpos_pub.publish(right_cpos_msg);

        rate.sleep();
    }
    ROS_INFO("State publisher thread finished.");
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "jaka_dual_arm_servo_control", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    signal(SIGINT, signalHandler);

    // ---- 机器人初始化 ----
    ROS_INFO("Logging in to robot...");
    robot.login_in("192.168.2.200");
    robot.power_on();
    robot.enable_robot();
    std::this_thread::sleep_for(std::chrono::seconds(1)); // 等待机器人稳定
    ROS_INFO("Robot enabled.");
    // robot.set_full_dh_flag(1);
    // robot.disable_robot();
    // robot.power_off();
    // std::this_thread::sleep_for(std::chrono::seconds(5)); // 等待机器人稳定
    // robot.power_on();
    // robot.enable_robot();
    // std::this_thread::sleep_for(std::chrono::seconds(1)); // 等待机器人稳定
    // ---- 移动到初始位置 ----
    ROS_INFO("Moving to initial joint positions...");
    JointValue initial_jpos[2];
    initial_jpos[0] = {1.707, -78.003, 72.538, -82.305, 50.506, -5.6, -126.290}; 
    initial_jpos[1] = {-1.554, -78.013, -72.530, -82.317, -50.502, 5.610, 126.298}; 

    // 将角度值转换为弧度
    convertJointValueToRad(initial_jpos[0]);
    convertJointValueToRad(initial_jpos[1]);

    // 测试的初始位置
    JointValue jpos[2];
    memset(&jpos,0,sizeof(jpos));
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
    // grasp init point
    jpos[0].jVal[0] = deg2rad(-21.3);
    jpos[0].jVal[1] = deg2rad(-51.7);
    jpos[0].jVal[2] = deg2rad(65.7);
    jpos[0].jVal[3] = deg2rad(-134.2);
    jpos[0].jVal[4] = deg2rad(121.25);
    jpos[0].jVal[5] = deg2rad(-40.5);
    jpos[0].jVal[6] = deg2rad(-136.6);

    jpos[1].jVal[0] = deg2rad(21.3);
    jpos[1].jVal[1] = deg2rad(-51.7);
    jpos[1].jVal[2] = deg2rad(-65.7);
    jpos[1].jVal[3] = deg2rad(-134.2);
    jpos[1].jVal[4] = deg2rad(-121.25);
    jpos[1].jVal[5] = deg2rad(40.5);
    jpos[1].jVal[6] = deg2rad(136.6);
    double jv[] = {deg2rad(10), deg2rad(10)};
    double ja[] = {deg2rad(50), deg2rad(50)};
    MoveMode mode[] = {MoveMode::ABS, MoveMode::ABS};
    robot.robot_run_multi_movj(DUAL, mode, true, jpos, jv, ja);
    ROS_INFO("Initial position reached.");
    CartesianPose cpos[2];
    robot.kine_forward(0,&initial_jpos[0],&cpos[0]);
    robot.kine_forward(1,&initial_jpos[1],&cpos[1]);
    ROS_INFO("trans: left[%.3f, %.3f, %.3f], right[%.3f, %.3f, %.3f]", 
        cpos[0].tran.x, cpos[0].tran.y, cpos[0].tran.z,
        cpos[1].tran.x, cpos[1].tran.y, cpos[1].tran.z);
    ROS_INFO("rotation: left[%.3f, %.3f, %.3f], right[%.3f, %.3f, %.3f]", 
        cpos[0].rpy.rx, cpos[0].rpy.ry, cpos[0].rpy.rz,
        cpos[1].rpy.rx, cpos[1].rpy.ry, cpos[1].rpy.rz);
    // ---- 准备伺服模式 ----
    // 使用初始位置初始化目标关节值
    {
        std::lock_guard<std::mutex> lock(robot_data_mutex);
        target_left_joint = jpos[0];
        target_right_joint = jpos[1];
    }
    
    // 启用伺服模式
    ROS_INFO("Enabling servo mode for both arms...");
    robot.servo_move_use_joint_NLF(80, 120, 120);
    int enable0 = robot.servo_move_enable(1, 0); // 开启左臂伺服
    int enable1 = robot.servo_move_enable(1, 1); // 开启右臂伺服

    if (enable0 != 0 || enable1 != 0) {
        ROS_ERROR("Failed to enable servo mode. Left arm code: %d, Right arm code: %d. Exiting.", enable0, enable1);
        robot.login_out();
        return -1;
    }
    ROS_INFO("Servo mode enabled.");


    // ---- 创建ROS订阅者和线程 ----
    ros::Subscriber left_sub = nh.subscribe("/left_arm_controller/command", 10, leftArmCallback);
    ros::Subscriber right_sub = nh.subscribe("/right_arm_controller/command", 10, rightArmCallback);

    // 使用多线程Spinner来处理回调
    ros::AsyncSpinner spinner(2); // 使用2个线程处理回调
    spinner.start();

    // 启动状态发布线程和实时控制线程
    std::thread publisher_th(state_publisher_thread, std::ref(nh));
    std::thread control_th(real_time_control_thread);


    // ---- 等待程序结束 ----
    control_th.join(); // 等待控制线程结束
    spinner.stop();
    publisher_th.join(); // 等待发布线程结束


    // ---- 清理和关闭 ----
    ROS_INFO("Disabling servo mode and logging out...");
    robot.servo_move_enable(0, -1); // 关闭所有伺服
    robot.disable_robot();
    robot.power_off();
    robot.login_out();
    ROS_INFO("Shutdown complete.");

    return 0;
}