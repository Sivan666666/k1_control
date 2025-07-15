#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <sensor_msgs/JointState.h>

#define rad2deg(x) ((x)*180.0/M_PI)
#define deg2rad(x) ((x)*M_PI/180.0)

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_command_publisher");
    ros::NodeHandle nh;
    
    // 创建关节指令发布者
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_commands", 1);
    
    // 等待订阅者连接
    while (joint_pub.getNumSubscribers() == 0) {
        ros::Duration(0.5).sleep();
        ROS_INFO("wait for sub...");
    }
    
    ros::Rate rate(10); // 100Hz频率（匹配1ms控制周期）
    double time = 0.0;
    
    // 关节名称定义（7轴机械臂）
    std::vector<std::string> joint_names = {
        "joint1", "joint2", "joint3", "joint4", 
        "joint5", "joint6", "joint7"
    };
    
    ROS_INFO("begin the trajectory");
    
    while (ros::ok()) {
        sensor_msgs::JointState joint_msg;
        joint_msg.header.stamp = ros::Time::now();
        joint_msg.name = joint_names;
        joint_msg.position.resize(7);  // 7个关节
        
        // 核心轨迹生成逻辑（参考您的伺服控制代码）
        double t = time;
        double k = 2.0;  // 基础频率系数
        
        // 关节1：15Hz正弦波 ±30°
        joint_msg.position[0] = sin(10.0 * t) * deg2rad(30);
        joint_msg.position[0] = deg2rad(0);
        // 关节2：15Hz余弦波 ±20° + 偏移量
        joint_msg.position[1] = -cos(10.0 * t) * deg2rad(20) + deg2rad(20);
        joint_msg.position[1] = deg2rad(0);
        // 关节3：固定位置
        joint_msg.position[2] = deg2rad(0);  
        
        // 关节4：15Hz余弦波 ±10° + 偏移量
        joint_msg.position[3] = -cos(10.0 * t) * deg2rad(10) + deg2rad(10);
        joint_msg.position[3] = deg2rad(0);
        // 关节5-7：固定位置（可根据需要扩展）
        joint_msg.position[4] = deg2rad(0);
        joint_msg.position[5] = deg2rad(0);
        joint_msg.position[6] = deg2rad(0);
        
        // start_pos[2] = { { -90 * deg_tp_rad, -75 * deg_tp_rad, 90 * deg_tp_rad, -120 * deg_tp_rad, -70 * deg_tp_rad, -90 * deg_tp_rad, 40 * deg_tp_rad},
        //                          { 90 * deg_tp_rad, -45 * deg_tp_rad, 0, -100 * deg_tp_rad, 0, -35 * deg_tp_rad, 90 * deg_tp_rad} };  
        // // 关节限位保护（示例）
        // for (auto& pos : joint_msg.position) {
        //     pos = std::clamp(pos, deg2rad(-170), deg2rad(170));
        // }
        
        joint_pub.publish(joint_msg);
        
        time += 0.1; // 100Hz对应的0.01秒步长
        rate.sleep();
    }
    return 0;
}