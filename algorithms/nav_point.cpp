#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <vector>
#include <queue>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MultiGoalNavigator {
private:
    ros::NodeHandle nh_;
    ros::Subscriber clicked_point_sub_;
    ros::Publisher goal_visualization_pub_;
    
    MoveBaseClient move_base_client_;
    tf::TransformListener tf_listener_;
    
    std::queue<geometry_msgs::PoseStamped> goals_queue_;
    bool is_navigating_;
    geometry_msgs::PoseStamped current_goal_;
    
public:
    MultiGoalNavigator() : 
        move_base_client_("move_base", true),
        is_navigating_(false)
    {
        // 等待move_base action server启动
        ROS_INFO("Waiting for move_base action server...");
        move_base_client_.waitForServer();
        ROS_INFO("Connected to move_base action server");
        
        // 订阅RViz中发布的点
        clicked_point_sub_ = nh_.subscribe("/clicked_point", 10, &MultiGoalNavigator::clickedPointCallback, this);
        
        // 发布目标点用于可视化
        goal_visualization_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/multi_goals", 10);
        
        ROS_INFO("Multi-goal navigator initialized. Click points in RViz to set goals.");
    }
    
    void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        // 将点击的点转换为地图坐标系中的目标点
        geometry_msgs::PointStamped map_point;
        
        try {
            // 转换点到map坐标系
            tf_listener_.transformPoint("map", *msg, map_point);
            
            // 创建目标位姿
            geometry_msgs::PoseStamped goal;
            goal.header.frame_id = "map";
            goal.header.stamp = ros::Time::now();
            goal.pose.position = map_point.point
            goal.pose.orientation.w = 1.0;
            
            // 将目标点加入队列
            goals_queue_.push(goal);
            ROS_INFO("Added new goal: (%.2f, %.2f, %.2f)", 
                     goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
            
            // 发布目标点用于可视化
            goal_visualization_pub_.publish(goal);
            
            // 如果没有正在导航，开始导航
            if (!is_navigating_) {
                navigateToNextGoal();
            }
        }
        catch (tf::TransformException& ex) {
            ROS_ERROR("TF transform failed: %s", ex.what());
        }
    }
    
    void navigateToNextGoal() {
        if (goals_queue_.empty()) {
            is_navigating_ = false;
            ROS_INFO("All goals completed!");
            return;
        }
        
        is_navigating_ = true;
        current_goal_ = goals_queue_.front();
        goals_queue_.pop();
        
        // 创建move_base目标
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = current_goal_;
        
        // 发送目标
        move_base_client_.sendGoal(goal, 
            boost::bind(&MultiGoalNavigator::goalDoneCallback, this, _1, _2),
            MoveBaseClient::SimpleActiveCallback(),
            boost::bind(&MultiGoalNavigator::goalFeedbackCallback, this, _1));
        
        ROS_INFO("Navigating to goal: (%.2f, %.2f, %.2f)", 
                 current_goal_.pose.position.x, current_goal_.pose.position.y, current_goal_.pose.position.z);
    }
    
    void goalDoneCallback(const actionlib::SimpleClientGoalState& state, 
                         const move_base_msgs::MoveBaseResultConstPtr& result) {
        ROS_INFO("Finished navigating to goal: %s", state.toString().c_str());
        
        // 无论成功还是失败，都尝试下一个目标
        navigateToNextGoal();
    }
    
    void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
        // 可以在这里处理导航反馈信息
        ROS_INFO("Current position: (%.2f, %.2f)", 
                feedback->base_position.pose.position.x, 
                  feedback->base_position.pose.position.y);
    }
    
    void run() {
        ros::spin();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "nav_point");
    MultiGoalNavigator navigator;
    navigator.run();
    return 0;
}