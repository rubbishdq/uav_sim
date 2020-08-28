#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <gazebo_msgs/ModelStates.h>
#include <uav_sim/PointWithState.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unistd.h>
#include <vector>
#include <deque>
#include <string>
#include <cstdlib>
#include <cmath>
using namespace std;

class CoreControllerNode
{
public:
    CoreControllerNode();
    
    void initFromYaml();  // 从yaml文件中初始化部分参数
    double targetDist();
    double targetYawDiff(); // in degree ranged from -180 to 180
    bool isAlreadyDetected(Eigen::Vector3d pos);
    bool isReallyDetected();

    void popUAVTargetPose(); // 从无人机路径位姿队列头部取出一个点作为下一目标点，更新uav_target_pose_global_和uav_target_pose_local_
    void globalTrajPushFront(Eigen::Vector3d pos, double yaw);
    void localTrajPushFront(Eigen::Vector3d pos, double yaw);
    void globalTrajPushBack(Eigen::Vector3d pos, double yaw);
    void localTrajPushBack(Eigen::Vector3d pos, double yaw);
    void clearTraj();  // 清空无人机路径位姿队列
    
private:
    void mavstateCallback(const mavros_msgs::State::ConstPtr& msg);
    void gazeboposeCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void targetposeCallback(const uav_sim::PointWithState::ConstPtr& msg);
    void statusloopCallback(const ros::TimerEvent& event);  // 定时检测无人机状态，根据情况发布arming和起降命令
    void publishloopCallback(const ros::TimerEvent& event);
    void decisionloopCallback(const ros::TimerEvent& event);   // 决策策略可以在此函数中实现
    
    // ROS相关内容
    ros::NodeHandle n_;
    
    ros::Duration pub_interval_;
    ros::Duration decision_interval_;

    ros::Publisher uav_target_pose_global_pub_;
    ros::Publisher uav_target_pose_local_pub_;
    ros::Publisher gazeboposePub_;

    ros::Subscriber mavstateSub_;
    ros::Subscriber gazeboposeSub_;
    ros::Subscriber mavposeSub_;
    ros::Subscriber targetposeSub_;

    ros::ServiceClient arming_client_;
    ros::ServiceClient land_client_;
    ros::ServiceClient set_mode_client_;

    ros::Timer statusloop_timer_;
    ros::Timer publishloop_timer_;
    ros::Timer decisionloop_timer_;

    ros::Time last_request_;
    mavros_msgs::SetMode offb_set_mode_;
    mavros_msgs::CommandBool arm_cmd_;
    mavros_msgs::State current_state_;

    geometry_msgs::PoseStamped uav_target_pose_global_, uav_target_pose_local_;
    geometry_msgs::PoseStamped current_pose_; // 世界坐标

    // 无人机在世界坐标系下的位姿
    Eigen::Matrix3d R_wu_;
    Eigen::Vector3d t_wu_;
    // 相机相对于无人机的位姿
    Eigen::Matrix3d R_uc_;
    Eigen::Vector3d t_uc_;

    // 无人机的初始位姿，在第一次收到相关topic后初始化，用于进行Gazebo世界坐标与MAVROS本地坐标的转换
    Eigen::Vector3d t_init_;
    double yaw_init_;  // 角度制
    Eigen::Matrix3d R_init_;  // 只包含yaw信息，与yaw_init_等价
    bool is_traj_local_init_;

    // 以下4个变量为无人机路径位姿队列，需要同时进行维护
    deque<Eigen::Vector3d> traj_global_, traj_local_;
    deque<double> yaw_global_, yaw_local_;  // 角度制

    // 目标检测相关变量
    int queue_msize_;
    deque<Eigen::Vector3d> location_queue_;
    double max_dist_in_queue_;
    int target_state_; // 目标状态（1=侦测到，0=未侦测到）
    double min_dist_;  // 在无人机与当前目标点（uav_target_pose_）距离小于min_dist_且yaw差距小于min_yaw_diff_时，当前飞行目标点改为路径队列中的下一个点
    double min_yaw_diff_;
    vector<Eigen::Vector3d> already_detected_target_;  // 记录已被侦测到的目标坐标
    double target_dist_threshold_;  // 目标间距离不得小于该值（防止重复侦测到同一目标）
    
    enum FlightState {
        WAITING, FLYING, READY_TO_LAND, LANDING
    } flight_state_;
};


