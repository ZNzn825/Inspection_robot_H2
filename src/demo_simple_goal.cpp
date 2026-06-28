#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/Twist.h"
#include <vector>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
// 简化类型定义
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// 任务类型枚举
enum TaskType {
    NAVIGATE,  // 导航任务
    MOVE       // 移动任务（前进/倒退/左转/右转）
};

// 任务结构体
struct Task {
    TaskType type;         // 任务类型
    double x;              // 导航点x坐标（仅用于NAVIGATE）
    double y;              // 导航点y坐标（仅用于NAVIGATE）
    double qz;             // 四元数z分量（仅用于NAVIGATE）
    double w;              // 四元数w分量（仅用于NAVIGATE）
    double linear_speed;   // 线速度（仅用于MOVE，正值前进，负值倒退）
    double angular_speed;  // 角速度（仅用于MOVE，正值右转，负值左转）
    double duration;       // 移动持续时间（仅用于MOVE）
    double wait_time;      // 任务完成后等待时间
};

// 导航到指定点
bool navigateToPoint(MoveBaseClient& ac, ros::Publisher& status_pub, int task_index, double x, double y, double qz, double w, const std::string& frame_id = "map") {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = frame_id;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.z = qz;
    goal.target_pose.pose.orientation.w = w;
    
    ROS_INFO("导航到点 (%.2f, %.2f)，四元数: z=%.2f, w=%.2f", x, y, qz, w);
    
    // 发布导航开始状态
    if (status_pub) {
        std_msgs::String status_msg;
        status_msg.data = "开始导航到点" + std::to_string(task_index) + " (" + std::to_string(x) + ", " + std::to_string(y) + ")";
        status_pub.publish(status_msg);
    }
    
    ac.sendGoal(goal);
    ac.waitForResult();
    
    // 发布导航结果状态
    if (status_pub) {
        std_msgs::String status_msg;
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            status_msg.data = "导航到点" + std::to_string(task_index) + "成功";
            ROS_INFO("导航到点%d成功!", task_index);
        } else {
            status_msg.data = "导航到点" + std::to_string(task_index) + "失败: " + ac.getState().toString();
            ROS_WARN("导航到点%d失败: %s", task_index, ac.getState().toString().c_str());
        }
        status_pub.publish(status_msg);
    }
    
    return ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

// 移动（前进/倒退/左转/右转），以100Hz频率发送速度指令
// 修改moveRobot函数，添加任务完成标志发布
// ... existing code ...
void moveRobot(ros::Publisher& cmd_vel_pub, ros::Publisher& status_pub, int task_index, double linear_speed, double angular_speed, double duration) {
    geometry_msgs::Twist twist;
    twist.linear.x = linear_speed;
    twist.angular.z = angular_speed;
    
    // 确定移动类型和方向
    std::string movement_type;
    if (linear_speed != 0 && angular_speed == 0) {
        movement_type = (linear_speed > 0) ? "前进" : "倒退";
        ROS_INFO("%s %.1f秒，速度: %.2f m/s", movement_type.c_str(), duration, fabs(linear_speed));
    } else if (linear_speed == 0 && angular_speed != 0) {
        movement_type = (angular_speed > 0) ? "右转" : "左转";
        ROS_INFO("%s %.1f秒，角速度: %.2f rad/s", movement_type.c_str(), duration, fabs(angular_speed));
    } else {
        movement_type = "复合移动";
        ROS_INFO("%s %.1f秒，线速度: %.2f m/s，角速度: %.2f rad/s", 
                movement_type.c_str(), duration, linear_speed, angular_speed);
    }
    
    // 发布移动开始状态
    if (status_pub) {
        std_msgs::String status_msg;
        status_msg.data = "开始移动任务" + std::to_string(task_index) + ": " + movement_type;
        status_pub.publish(status_msg);
    }
    
    // 设置100Hz的发布频率
    ros::Rate rate(100);
    
    // 计算需要发送的总次数
    int total_cycles = static_cast<int>(duration * 100);
    
    // 持续发送速度指令
    for(int i = 0; i < total_cycles; ++i) {
        cmd_vel_pub.publish(twist);
        rate.sleep();  // 保持100Hz频率
    }
    
    // 停止移动
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    cmd_vel_pub.publish(twist);
    
    // 发布移动完成状态
    if (status_pub) {
        std_msgs::String status_msg;
        status_msg.data = "移动任务" + std::to_string(task_index) + "完成";
        status_pub.publish(status_msg);
    }
    
    ROS_INFO("%s完成!", movement_type.c_str());
}


// 等待指定时间
void wait(double seconds) {
    if(seconds > 0) {
        ROS_INFO("停留 %.1f秒...", seconds);
        ros::Duration(seconds).sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_simple_goal");
    setlocale(LC_ALL, "");  // 支持中文
    ros::NodeHandle nh;
    
    // 创建客户端和发布者
    MoveBaseClient ac("move_base", true);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Publisher nav_status_pub = nh.advertise<std_msgs::String>("/navigation_status", 200);//发布导航状态信息

    // 等待move_base服务器
    while(!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("等待move_base服务启动...");
    }
    
    // 任务列表 - 在这里添加所有任务
    std::vector<Task> tasks = {
        // 导航任务格式: {NAVIGATE, x坐标, y坐标, qz, w, 0, 0, 等待时间}
        // 移动任务格式: {MOVE, 0, 0, 0, 0, 线速度(正值前进/负值倒退), 角速度(正值左转/负值右转), 持续时间, 等待时间}

        // 20251229黄埔实验室-全局-a(机械臂动作代号)
        //巡检点1
        /*{NAVIGATE, 0.4, 0.0, 0.0, 1.0, 0, 0, 0.0, 1.0},  //过渡点，导航到点1
        {MOVE, 0, 0, 0, 0, 0.0, -0.1, 20.0, 1.0},     //右转15秒，角速度0.1rad/s，停留1秒，移动任务1
        {MOVE, 0, 0, 0, 0, 0.1,  0, 18.0, 1.0},        //前进18秒，速度0.1m/s，停留1秒，移动任务2，机械臂动作
        {MOVE, 0, 0, 0, 0, -0.1, 0, 15.0, 1.0},       //后退15秒，速度0.1m/s，停留1秒，移动任务3
        {MOVE, 0, 0, 0, 0, 0.0, 0.1, 20.0, 1.0},      //左转15秒，角速度0.1rad/s，停留1秒，移动任务4
        //巡检点2
        {NAVIGATE, 3.0, -0.5, 0.0, 1.0, 0, 0, 0.0, 1.0},   //过渡点，导航到点2
        {NAVIGATE, 5.2, -1.8, 0.0, 1.0, 0, 0, 0.0, 10.0},  // 导航到点巡检点2，停留10秒，导航到点3，机械臂动作
        {MOVE, 0, 0, 0, 0, -0.1, 0, 7.0, 0.0},        // 倒退7秒，速度0.1m/s，   移动任务5
        {MOVE, 0, 0, 0, 0, 0, 0.1, 10.0, 0.0},        // 左转10秒，角速度0.1rad/s，  移动任务6
        {MOVE, 0, 0, 0, 0, 0.1, 0, 15.0, 0.0},        // 前进3秒，速度0.1m/s，停留1秒，移动任务7
        {NAVIGATE, 8.5, 0.6, 0.0, 1.0, 0, 0, 0.0, 60.0},    // 导航到窄道门口，停留60S过窄道!!!!!!!!!!!!!!!!!!
        //巡检点3
        //刚出门口大致坐标{NAVIGATE, 17.8，1.1，0.0，1.0，0.0，0.0，0.0，1.0}
        {NAVIGATE, 24.8, 1.2, 0.0, 1.0, 0.0, 0.0, 0.0, 10.0},   //到达巡检点3,停留10s完成监测任务
         //原地掉头开始返回
        {MOVE, 0, 0, 0, 0, 0.0, -0.1, 30.0, 1.0},        // 右转30秒，速度0.1m/s，停留
        {NAVIGATE, 17.8, 1.1, 1.0, 0.0, 0.0, 0.0, 0.0, 60.0},    //返回窄道门口，停留60S过窄道!!!!!!!!!!!!!!!!!
        //出窄道后，巡检点4
        {NAVIGATE, 8.0, 0.5, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0},   //过渡点
        {MOVE, 0, 0, 0, 0, 0.1, 0.0, 10.0, 1.0},        // 前进到巡检点4，前进10s，速度0.1m/s，停留1秒
        //巡检点5
        {MOVE, 0, 0, 0, 0, 0.1, 0.0, 10.0, 1.0},        // 前进到巡检点5，前进10s，速度0.1m/s，停留1秒
        //巡检点6
        {MOVE, 0, 0, 0, 0, 0.1, 0.0, 10.0, 1.0},        // 前进到巡检点6，前进10s，速度0.1m/s，停留1秒
        //返回起点
        {MOVE, 0, 0, 0, 0, 0.1, 0.1, 6.0, 1.0},         //过渡点，左前行驶6s
        {NAVIGATE, 0.4, -0.35, 1.0, 0.0, 0, 0, 0.0, 1.0},  //过渡点
        {MOVE, 0, 0, 0, 0, 0.0, -0.1, 42.0, 1.0},        // 右转40秒，速度0.1m/s，停留
        {MOVE, 0, 0, 0, 0, -0.1, 0,1.0, 5.0},        // 倒退7秒，速度0.1m/s，返回原点 */

        //20260122新路线，实验室A-连接门起点-b(机械臂动作代号)
        {NAVIGATE, 2.0, 0.35, 0.0, 1.0, 0, 0, 0.0, 1.0},  //过渡点,导航到点1
        {MOVE, 0, 0, 0, 0, 0.1, 0.1, 4.5, 0.0},  //移动任务2
        {MOVE, 0, 0, 0, 0, 0.1, -0.1, 4.3, 0.0},  //移动任务3
        {MOVE, 0, 0, 0, 0, 0.1, 0, 14.5, 20.0},  //移动任务4完成，停留60s等待氢气检测，机械臂动作,检测点位1
        {MOVE, 0, 0, 0, 0, 0.1, 0, 16.0, 0.0},  //移动任务5完成，停留60s等待氢气检测，机械臂动作，检测点位2
        {MOVE, 0, 0, 0, 0, 0.1, 0.0, 8.0,0.0},  //移动任务6完成，停留60s等待氢气检测，机械臂动作，检测点位3
        {MOVE, 0, 0, 0, 0, 0.1, 0.0, 12.0, 0.0},  //移动任务7完成，停留60s等待氢气检测，机械臂动作，检测点位4
        {MOVE, 0, 0, 0, 0, 0.1, 0.0, 14.0, 60.0},  //移动任务8完成，停留60s等待氢气检测，机械臂动作,检测点位5，随后机械臂复位

        {MOVE, 0, 0, 0, 0, 0.1, 0.1, 2.0, 0.0},  //移动任务9完成，拐弯出来，确保导航到位不撞上
        {MOVE, 0, 0, 0, 0, 0.1, 0, 5.0, 0.0},  //移动任务10完成

        {NAVIGATE, 13.0, 1.0, 0.5, 0.86, 0, 0, 0.0, 1.0},  //过渡点,导航到点11
        {MOVE, 0, 0, 0, 0, 0.1, 0, 17.0, 40.0},  //移动任务12完成,停留60s等待氢气检测，机械臂动作，到两个设备中间，有2个检测点
        {MOVE, 0, 0, 0, 0, -0.1, 0, 17.0, 0.0},  //移动任务13完成，退出检测点
        {MOVE, 0, 0, 0, 0, 0.0, 0.1, 20.0, 0.0},     //左转20秒，停留1秒，移动任务14
        {NAVIGATE, 11.0, 1.5, 0.95, 0.3, 0, 0, 0.0, 1.0},  //过渡点,导航到点15

        {MOVE, 0, 0, 0, 0, 0.1, 0, 25.0, 1.0},  //移动任务16完成，移动到桌子旁边，身子斜着
        {MOVE, 0, 0, 0, 0, 0.0, 0.1, 9.0, 30.0},  //移动任务17完成，机械臂动作，桌子旁边
        {MOVE, 0, 0, 0, 0, -0.1, 0, 7.0, 45.0},  //移动任务18完成，机械臂动作，墙壁,3个检测点
        {MOVE, 0, 0, 0, 0, 0.0, 0.1, 10.0, 15.0},  //移动任务19完成，小车左转，从墙壁出来,等待15秒让机械臂复位

        {MOVE, 0, 0, 0, 0, 0.1, 0.0, 10.0, 0.0},  //移动任务20完成
        {NAVIGATE, 6.5, 0.5, 1.0, 0.0, 0, 0, 0.0, 5.0},  //导航到点21，窄道前
        {MOVE, 0, 0, 0, 0, 0.1, 0.1, 6.0, 0.0},  //移动任务23完成
        {MOVE, 0, 0, 0, 0, 0.1, -0.1, 6.0, 0.0},  //移动任务24完成
        {MOVE, 0, 0, 0, 0, 0.1, 0.1, 2.0, 0.0},  //移动任务25完成
        {MOVE, 0, 0, 0, 0, 0.1, -0.1, 2.0, 0.0},  //移动任务26完成
        {MOVE, 0, 0, 0, 0, 0.1, 0.0, 30.0, 0.0},  //移动任务27完成
        {MOVE, 0, 0, 0, 0, 0.1, 0.1, 6.0, 0.0},  //移动任务28完成
        {MOVE, 0, 0, 0, 0, 0.1, -0.1, 6.0, 0.0},  //移动任务29完成
        {MOVE, 0, 0, 0, 0, 0.1, 0.0, 10.0, 0.0},  //移动任务30完成


    };
    
    // 依次执行所有任务
    for(size_t i = 0; i < tasks.size(); ++i) {
        const Task& task = tasks[i];
        int task_index = i + 1;  // 任务序号从1开始
        
        bool success = true;
        
        // 根据任务类型执行
        if(task.type == NAVIGATE) {
            success = navigateToPoint(ac, nav_status_pub, task_index, task.x, task.y, task.qz, task.w, "map");
        } else if(task.type == MOVE) {
            moveRobot(cmd_vel_pub, nav_status_pub, task_index, task.linear_speed, task.angular_speed, task.duration);
        }       

        // 如果任务成功完成，执行等待
        if(success) {
            wait(task.wait_time);
        } else {
            ROS_WARN("任务 %zu 执行失败，跳过后续任务", i+1);
            break;
        }
    }
    
    ROS_INFO("所有任务完成!");
    return 0;
}