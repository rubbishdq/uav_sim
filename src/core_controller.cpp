#include "core_controller.h"
#include <xmlrpcpp/XmlRpcValue.h>
using namespace std;

CoreControllerNode::CoreControllerNode()
{
    // 读取yaml文件中事先设定的参数
    initFromYaml();

    // 初始化各PoseStamped消息的header(用于rviz可视化)
    uav_target_pose_global_.header.seq = 1;
    uav_target_pose_global_.header.frame_id = "map";
    uav_target_pose_local_.header.seq = 1;
    uav_target_pose_local_.header.frame_id = "map";
    current_pose_.header.seq = 1;
    current_pose_.header.frame_id = "map";

    is_traj_local_init_ = false;
    target_state_ = 0;
    min_dist_ = 0.3;
    min_yaw_diff_ = 5;
    target_dist_threshold_ = 1;
    flight_state_ = WAITING;
    last_request_ = ros::Time::now();

    pub_interval_ = ros::Duration(0.02);
    decision_interval_ = ros::Duration(0.05);

    uav_target_pose_global_pub_ = n_.advertise<geometry_msgs::PoseStamped>("/core_controller/uav_traj_pose", 100);
    uav_target_pose_local_pub_ = n_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 100);
    gazeboposePub_ = n_.advertise<geometry_msgs::PoseStamped>("/core_controller/uav_pose_gazebo", 100);

    mavstateSub_ = n_.subscribe("/mavros/state", 1, &CoreControllerNode::mavstateCallback, this, ros::TransportHints().tcpNoDelay());  
    gazeboposeSub_ = n_.subscribe("/gazebo/model_states", 1, &CoreControllerNode::gazeboposeCallback, this, ros::TransportHints().tcpNoDelay());  
    targetposeSub_ = n_.subscribe("/target_location/camera", 1, &CoreControllerNode::targetposeCallback, this, ros::TransportHints().tcpNoDelay());

    arming_client_ = n_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    land_client_ = n_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/land");
    set_mode_client_ = n_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    statusloop_timer_ = n_.createTimer(ros::Duration(1), &CoreControllerNode::statusloopCallback, this);
    publishloop_timer_ = n_.createTimer(pub_interval_, &CoreControllerNode::publishloopCallback, this);
    decisionloop_timer_ = n_.createTimer(decision_interval_, &CoreControllerNode::decisionloopCallback, this);

    ros::Rate rate(20);
    for(int i = 0; ros::ok() && i < 10; i++)
    {
        uav_target_pose_local_pub_.publish(uav_target_pose_local_);
        ros::spinOnce();
        rate.sleep();
    }
}

void CoreControllerNode::initFromYaml()
{
    // 初始化世界坐标系下的路径traj_global_和yaw_global_
    XmlRpc::XmlRpcValue traj_global_temp;
    Eigen::Vector3d v;
    if(!n_.getParam("/core_controller/trajectory", traj_global_temp))
        ROS_ERROR("Failed to get param.");
    ROS_ASSERT(traj_global_temp.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(traj_global_temp[0].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(traj_global_temp[0][0].getType() == XmlRpc::XmlRpcValue::TypeDouble || 
        traj_global_temp[0][0].getType() == XmlRpc::XmlRpcValue::TypeInt);

    for(int i = 0; i < traj_global_temp.size(); i++)
    {
        v << double(traj_global_temp[i][0]), double(traj_global_temp[i][1]), double(traj_global_temp[i][2]);
        traj_global_.push_back(v);
        yaw_global_.push_back(double(traj_global_temp[i][3]));
    }
    // 初始化相机位姿
    XmlRpc::XmlRpcValue pose_temp;
    if(!n_.getParam("/core_controller/camera_pose", pose_temp))
        ROS_ERROR("Failed to get param.");
    ROS_ASSERT(pose_temp.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(pose_temp[0].getType() == XmlRpc::XmlRpcValue::TypeDouble || 
        pose_temp[0].getType() == XmlRpc::XmlRpcValue::TypeInt);
    t_uc_ << double(pose_temp[0]), double(pose_temp[1]), double(pose_temp[2]);
    R_uc_ = Eigen::AngleAxisd(double(pose_temp[3]), Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(double(pose_temp[4]), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(double(pose_temp[5]), Eigen::Vector3d::UnitX());  // 欧拉角转旋转矩阵
}

double CoreControllerNode::targetDist()
{
    return sqrt((current_pose_.pose.position.x - uav_target_pose_global_.pose.position.x)
        * (current_pose_.pose.position.x - uav_target_pose_global_.pose.position.x)
        + (current_pose_.pose.position.y - uav_target_pose_global_.pose.position.y)
        * (current_pose_.pose.position.y - uav_target_pose_global_.pose.position.y)
        + (current_pose_.pose.position.z - uav_target_pose_global_.pose.position.z)
        * (current_pose_.pose.position.z - uav_target_pose_global_.pose.position.z));
}

double CoreControllerNode::targetYawDiff()
{
    double uav_target_y, current_y;
    uav_target_y = tf::getYaw(uav_target_pose_global_.pose.orientation);
    current_y = tf::getYaw(current_pose_.pose.orientation);
    double diff = (uav_target_y - current_y) / M_PI * 180.0;
    if(diff > 180.0)
        return diff - 360.0;
    else if(diff < -180.0)
        return diff + 360.0;
    else
        return diff;
}

bool CoreControllerNode::isAlreadyDetected(Eigen::Vector3d pos)
{
    for(auto iter = already_detected_target_.begin(); iter != already_detected_target_.end(); iter++)
    {
        if((pos - *iter).norm() < target_dist_threshold_)
            return true;
    }
    return false;
}

void CoreControllerNode::popUAVTargetPose()
{
    if(traj_global_.empty() || traj_local_.empty())
    {
        return;
    }
    
    Eigen::Vector3d v;

    v = traj_global_.front();
    uav_target_pose_global_.header.stamp = ros::Time::now();
    uav_target_pose_global_.pose.position.x = v[0];
    uav_target_pose_global_.pose.position.y = v[1];
    uav_target_pose_global_.pose.position.z = v[2];
    uav_target_pose_global_.pose.orientation = tf::createQuaternionMsgFromYaw((yaw_global_.front()/180.0)*M_PI);
    traj_global_.pop_front();
    yaw_global_.pop_front();

    v = traj_local_.front();
    uav_target_pose_local_.header.stamp = ros::Time::now();
    uav_target_pose_local_.pose.position.x = v[0];
    uav_target_pose_local_.pose.position.y = v[1];
    uav_target_pose_local_.pose.position.z = v[2];
    uav_target_pose_local_.pose.orientation = tf::createQuaternionMsgFromYaw((yaw_local_.front()/180.0)*M_PI);
    traj_local_.pop_front();
    yaw_local_.pop_front();
}

void CoreControllerNode::globalTrajPushFront(Eigen::Vector3d pos, double yaw)
{
    // global
    traj_global_.push_front(pos);
    yaw_global_.push_front(yaw);
    // local
    if(is_traj_local_init_)
    {
        traj_local_.push_front(R_init_.inverse()*(pos-t_init_));
        yaw_local_.push_front(-yaw_init_+yaw < 0 ? -yaw_init_+yaw+360 : -yaw_init_+yaw);
    }
}

void CoreControllerNode::localTrajPushFront(Eigen::Vector3d pos, double yaw)
{
    // global
    traj_global_.push_front(R_init_*pos+t_init_);
    yaw_global_.push_front(yaw_init_+yaw > 360 ? yaw_init_+yaw-360 : yaw_init_+yaw);
    // local
    if(is_traj_local_init_)
    {
        traj_local_.push_front(pos);
        yaw_local_.push_front(yaw);
    }
}

void CoreControllerNode::globalTrajPushBack(Eigen::Vector3d pos, double yaw)
{
    // global
    traj_global_.push_back(pos);
    yaw_global_.push_back(yaw);
    // local
    if(is_traj_local_init_)
    {
        traj_local_.push_back(R_init_.inverse()*(pos-t_init_));
        yaw_local_.push_back(-yaw_init_+yaw < 0 ? -yaw_init_+yaw+360 : -yaw_init_+yaw);
    }
}

void CoreControllerNode::localTrajPushBack(Eigen::Vector3d pos, double yaw)
{
    // global
    traj_global_.push_back(R_init_*pos+t_init_);
    yaw_global_.push_back(yaw_init_+yaw > 360 ? yaw_init_+yaw-360 : yaw_init_+yaw);
    // local
    if(is_traj_local_init_)
    {
        traj_local_.push_back(pos);
        yaw_local_.push_back(yaw);
    }
}

void CoreControllerNode::clearTraj()
{
    traj_global_.clear();
    yaw_global_.clear();
    traj_local_.clear();
    yaw_local_.clear();
}

void CoreControllerNode::mavstateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_ = *msg;
}

void CoreControllerNode::gazeboposeCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    int i;
    for(i = 0; i < msg->name.size(); i++)
    {
        if(msg->name[i] == "iris")
            break;
    }
    if(i < msg->name.size())
    {
        geometry_msgs::Pose pose = msg->pose[i];
        current_pose_.header.stamp = ros::Time::now();
        current_pose_.pose = pose;
        t_wu_ << pose.position.x, pose.position.y, pose.position.z;
        Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        R_wu_ = q.toRotationMatrix();

        if(!is_traj_local_init_) 
        {
            // 利用第一次收到的世界坐标将traj_global_、yaw_global_中的轨迹信息转化为本地坐标（以无人机初始化位置为原点的坐标系）
            t_init_ = t_wu_;
            yaw_init_ = tf::getYaw(pose.orientation)/M_PI*180.0;
            R_init_ = Eigen::AngleAxisd(yaw_init_/180.0*M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            for(auto trajPtr = traj_global_.begin(); trajPtr != traj_global_.end(); trajPtr++)
            {
                traj_local_.push_back(R_init_.inverse()*(*trajPtr-t_init_));
            }
            for(auto yawPtr = yaw_global_.begin(); yawPtr != yaw_global_.end(); yawPtr++)
            {
                yaw_local_.push_back(-yaw_init_+*yawPtr < 0 ? -yaw_init_+*yawPtr+360 : -yaw_init_+*yawPtr);
            }
            // 初始化第一个飞行目标点
            popUAVTargetPose();
        }
        is_traj_local_init_ = true;
        
    }
}

void CoreControllerNode::targetposeCallback(const uav_sim::PointWithState::ConstPtr& msg)
{
    target_state_ = msg->state;
    if(target_state_)
    {
        Eigen::Vector3d p_ct(msg->position.x, msg->position.y, msg->position.z), p_wt;
        p_wt = R_wu_*(R_uc_*p_ct+t_uc_)+t_wu_;
        // p_wt = R_wu_*p_ct+t_wu_;
        if(!this->isAlreadyDetected(p_wt))
        {
            already_detected_target_.push_back(p_wt);
            ROS_INFO("Target detected at %lf, %lf, %lf", p_wt[0], p_wt[1], p_wt[2]);
        }
    }
}

void CoreControllerNode::statusloopCallback(const ros::TimerEvent& event)
{
    switch(flight_state_)
    {
    case WAITING:
        arm_cmd_.request.value = true;
        offb_set_mode_.request.custom_mode = "OFFBOARD";
        if( current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(4.0)) && is_traj_local_init_)
        {
    	    if( set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent)
            {
    	        ROS_INFO("Offboard enabled");
	    }
	    last_request_ = ros::Time::now();
        } 
        else 
        {
	    if( !current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(4.0)))
	    {
	        if( arming_client_.call(arm_cmd_) && arm_cmd_.response.success)
	        {
	            ROS_INFO("Vehicle armed");
                    flight_state_ = FLYING;
	        }
	        last_request_ = ros::Time::now();
	    }    
        }
        break;
    case FLYING:
        break;
    case READY_TO_LAND:
        ROS_INFO("Ready to land");
        offb_set_mode_.request.custom_mode = "AUTO.LAND";   
        if( current_state_.mode != "AUTO.LAND")
        {
            if(set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent)
	    {
	        ROS_INFO("Vehicle landing");
                flight_state_ = LANDING;
	    }
	    last_request_ = ros::Time::now();
        }
        break;
    case LANDING:
        break;
    default:
        break;
    }
}

void CoreControllerNode::publishloopCallback(const ros::TimerEvent& event)
{
    if(is_traj_local_init_)
    {
        uav_target_pose_global_pub_.publish(uav_target_pose_global_);
        uav_target_pose_local_pub_.publish(uav_target_pose_local_);
        gazeboposePub_.publish(current_pose_);
    }
}

void CoreControllerNode::decisionloopCallback(const ros::TimerEvent& event)
{
    // 如果当前无人机位姿与下一路径位姿差别足够小，做出决策
    if(flight_state_ == FLYING && targetDist() < min_dist_ && fabs(targetYawDiff()) < min_yaw_diff_) 
    {
        if(traj_local_.empty())  // 已经到达最终路径点，准备降落
        {
            flight_state_ = READY_TO_LAND;
        }
        else // 从队列中取出下一个路径位姿
        {
            popUAVTargetPose();
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "core_controller_node");

    CoreControllerNode f;

    if(!ros::ok())
    {
	return 0;
    }
    ROS_INFO("flight controller node set up already.");
    ros::spin();

    return 0;
}
