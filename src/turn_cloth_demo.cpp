#include <JAKAZuRobot.h>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>
#include <common.h>

#define JK_PI (3.141592653589793)
#define deg_to_rad (JK_PI / 180.0)


// 生成圆形路径函数
std::vector<std::array<CartesianPose, 2>> generate_circular_path(
    const CartesianPose& left_center, 
    const CartesianPose& right_center,
    double radius,
    double step_angle,  // 步长角度（度）
    int cycles)
{
    std::vector<std::array<CartesianPose, 2>> path;
    
    // 计算总点数（360度/步长角度 * 循环次数）
    int points_per_cycle = static_cast<int>(360.0 / step_angle);
    int total_points = points_per_cycle * cycles;
    
    for (int i = 0; i <= total_points; ++i) {
        double angle = i * step_angle * deg_to_rad;  // 当前角度（弧度）
        
        // 计算偏移量
        double x_offset = radius * cos(angle);
        double y_offset = radius * sin(angle);
        
        // 左臂路径点
        CartesianPose left_point;
        left_point.tran.x = left_center.tran.x + x_offset;
        left_point.tran.y = left_center.tran.y + y_offset;
        left_point.tran.z = left_center.tran.z;
        left_point.rpy = left_center.rpy;
        
        // 右臂路径点
        CartesianPose right_point;
        right_point.tran.x = right_center.tran.x + x_offset;
        right_point.tran.y = right_center.tran.y + y_offset;
        right_point.tran.z = right_center.tran.z;
        right_point.rpy = right_center.rpy;
        
        path.push_back({left_point, right_point});
    }
    
    return path;
}


int main()
{
    // 初始化
    JAKAZuRobot robot;
    RobotStatus robotStatus;
    errno_t ret;

    JointValue start_pos[2] = { { -90 * deg_to_rad, 21 * deg_to_rad, 60 * deg_to_rad, -110 * deg_to_rad, -74 * deg_to_rad, -57 * deg_to_rad, 0 * deg_to_rad},
                                 { 85 * deg_to_rad, 21 * deg_to_rad, -65 * deg_to_rad, -105 * deg_to_rad, -105 * deg_to_rad, -71 * deg_to_rad, 0 * deg_to_rad} };  
    MoveMode moveop[2] = {ABS, ABS};
    double vel[2] = {2, 2};
    double acc[2] = {2, 2};

    // 登录
    ret = robot.login_in("192.168.2.200"); // 替换为实际IP
    if (ret != ERR_SUCC) {
        std::cerr << "Login failed with error code: " << ret << std::endl;
        return -1;
    }

    // 上电和使能
    ret = robot.power_on();
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "power on");
    robot.clear_error();
    ret = robot.enable_robot();
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "enable robot");
    
    // 2. 设置圆形路径参数
    double radius = 30.0;       // 圆半径（mm）
    double step_angle = 10.0;   // 步长角度（度）
    int cycles = 4;             // 循环次数

    // 左右臂中心点
    CartesianPose left_center = {190, 490, 425, 0, 0, 0};
    CartesianPose right_center = {195, -520, 390, 0, 0, 0};

    // 生成圆形路径
    auto circular_path = generate_circular_path(left_center, right_center, radius, step_angle, cycles);

    // 3. 运动参数设置
    MoveMode move_mode[2] = {ABS, ABS};      // 绝对坐标模式
    double velocity[2] = {5000, 5000};       // 运动速度 (mm/s)
    double acceleration[2] = {5000, 5000};   // 加速度 (mm/s²)
    std::cout << "Starting circular path with " << circular_path.size() << " points" << std::endl;

    // 4. 执行动作
    for (const auto& target_poses : circular_path) {
        // 发送控制命令
        ret = robot.robot_run_multi_movl(
            -1,             // -1表示双轴同步运动
            move_mode,      // 运动模式
            FALSE,          // 阻塞执行
            target_poses.data(), // 目标位姿
            velocity,       // 速度
            acceleration    // 加速度
        );

        if (ret != ERR_SUCC) {
            std::cerr << "Movement failed with error code: " << ret << std::endl;
            break;
        }

        // 打印当前点信息（可选）
        std::cout << "Executing point: "
                  << "Left[" << target_poses[0].tran.x << ", " << target_poses[0].tran.y << ", " << target_poses[0].tran.z << "]"
                  << " Right[" << target_poses[1].tran.x << ", " << target_poses[1].tran.y << ", " << target_poses[1].tran.z << "]" << std::endl;

        // 读取当前状态
        JointValue output_jpos[2];
        CartesianPose output_cpos[2];
        robot.edg_get_stat(0, &output_jpos[0], &output_cpos[0]);
        robot.edg_get_stat(1, &output_jpos[1], &output_cpos[1]);
        std::cout << "Current joint positions: "
                  << "Left[" << output_jpos[0].jVal[0] << ", " << output_jpos[0].jVal[1] << ", " << output_jpos[0].jVal[2] << ", "
                  << output_jpos[0].jVal[3] << ", " << output_jpos[0].jVal[4] << ", " << output_jpos[0].jVal[5] << ", " << output_jpos[0].jVal[6] << "]. "
                  << "Right[" << output_jpos[1].jVal[0] << ", " << output_jpos[1].jVal[1] << ", " << output_jpos[1].jVal[2] << ", "
                  << output_jpos[1].jVal[3] << ", " << output_jpos[1].jVal[4] << ", " << output_jpos[1].jVal[5] << ", " << output_jpos[1].jVal[6] << "]. " 
                  << std::endl;
        std::cout << "Current Cartesian positions: "
                  << "Left[" << output_cpos[0].tran.x << ", " << output_cpos[0].tran.y << ", " << output_cpos[0].tran.z << ", "
                  << output_cpos[0].rpy.rx << ", " << output_cpos[0].rpy.ry << ", " << output_cpos[0].rpy.rz << "]. "
                  << "Right[" << output_cpos[1].tran.x << ", " << output_cpos[1].tran.y << ", " << output_cpos[1].tran.z << ", "
                  << output_cpos[1].rpy.rx << ", " << output_cpos[1].rpy.ry << ", " << output_cpos[1].rpy.rz << "]. "
                  << std::endl;

        // 添加短暂延时（可选）
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    std::cout << "Circular path execution completed" << std::endl;
    return 0;
}
