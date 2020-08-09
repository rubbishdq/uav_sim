#!/usr/bin/python
#-*- encoding: utf8 -*-

from scipy.spatial.transform import Rotation as R
import rospy
import numpy as np
import math
import collections
from enum import Enum
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from gazebo_msgs.msg import ModelStates
from uav_sim.msg import PointWithState
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode

class CoreControllerNode:

    class FlightState(Enum):
        WAITING = 1
        FLYING = 2
        READY_TO_LAND = 3
        LANDING = 4


    def __init__(self):
        rospy.init_node('core_controller_node', anonymous=True)

        #以下4个变量为无人机路径位姿队列，需要同时进行维护
        self.traj_global_ = collections.deque()
        self.yaw_global_ = collections.deque()  #角度制
        self.traj_local_ = collections.deque()
        self.yaw_local_ = collections.deque()  #角度制

        #读取yaml文件中事先设定的参数
        self.initFromYaml();

        #初始化各PoseStamped消息及其header(用于rviz可视化)
        self.uav_target_pose_global_ = PoseStamped()
        self.uav_target_pose_global_.header.seq = 1
        self.uav_target_pose_global_.header.frame_id = 'map'
        self.uav_target_pose_global_.pose.orientation.w = 1
        self.uav_target_pose_local_ = PoseStamped()
        self.uav_target_pose_local_.header.seq = 1
        self.uav_target_pose_local_.header.frame_id = 'map'
        self.uav_target_pose_local_.pose.orientation.w = 1
        self.current_pose_ = PoseStamped()  #世界坐标
        self.current_pose_.header.seq = 1
        self.current_pose_.header.frame_id = 'map'
        self.current_pose_.pose.orientation.w = 1

        #目标检测相关变量
        self.already_detected_target_ = []  #已侦测到的目标坐标
        self.target_state_ = 0  #目标状态（1=侦测到，0=未侦测到）
        self.min_dist_ = 0.3  #在无人机与当前目标点（uav_target_pose_）距离小于min_dist_且yaw差距小于min_yaw_diff_时，当前飞行目标点改为路径队列中的下一个点
        self.min_yaw_diff_ = 5
        self.target_dist_threshold_ = 1

        #无人机在世界坐标系下的位姿
        self.R_wu_ = R.from_quat([0, 0, 0, 1])
        self.t_wu_ = np.zeros([3], dtype = 'float')

        #无人机的初始位姿，在第一次收到相关topic后初始化，用于进行Gazebo世界坐标与MAVROS本地坐标的转换
        self.t_init_ = np.zeros([3], dtype = 'float')
        self.yaw_init_ = 0.0  #角度制
        self.R_init_ = R.from_quat([0, 0, 0, 1])
        self.is_traj_local_init_ = False

        #用于发送起降命令和追踪飞行状态
        self.last_request_ = rospy.Time.now()
        self.offb_set_mode_ = SetMode()
        self.arm_cmd_ = CommandBool()
        self.current_state_ = State()
        self.flight_state_ = self.FlightState.WAITING

        #其他ROS相关内容
        self.pub_interval_ = rospy.Duration(0.02)
        self.decision_interval_ = rospy.Duration(0.05)

        self.uav_target_pose_global_pub_ = rospy.Publisher('/core_controller/uav_traj_pose', PoseStamped, queue_size=100)
        self.uav_target_pose_local_pub_ = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=100)
        self.gazeboposePub_ = rospy.Publisher('/core_controller/uav_pose_gazebo', PoseStamped, queue_size=100)

        self.mavstateSub_ = rospy.Subscriber('/mavros/state', State, self.mavstateCallback)
        self.gazeboposeSub_ = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazeboposeCallback)
        self.targetposeSub_ = rospy.Subscriber('/target_location/location', PointWithState, self.targetposeCallback)

        self.arming_client_ = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.land_client_ = rospy.ServiceProxy('/mavros/cmd/land', CommandBool)
        self.set_mode_client_ = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.statusloop_timer_ = rospy.Timer(rospy.Duration(1), self.statusloopCallback)
        self.publishloop_timer_ = rospy.Timer(self.pub_interval_, self.publishloopCallback)
        self.decisionloop_timer_ = rospy.Timer(self.decision_interval_, self.decisionloopCallback)

        rate = rospy.Rate(20);
        for i in range(10):
            self.uav_target_pose_local_pub_.publish(self.uav_target_pose_local_);
            rate.sleep()

        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass


    #从yaml文件中初始化部分参数
    def initFromYaml(self):
        #初始化世界坐标系下的路径traj_global_和yaw_global_
        traj_global_temp = rospy.get_param('/core_controller/trajectory')
        for pt in traj_global_temp:
            self.traj_global_.append(np.array(pt[:3], dtype = 'float'))
            self.yaw_global_.append(float(pt[3]))
        
        #初始化相机位姿
        pose_temp = rospy.get_param('/core_controller/camera_pose')
        self.t_uc_ = np.array(pose_temp[:3], dtype = 'float')
        self.R_uc_ = R.from_euler('zyx', pose_temp[3:6], degrees = True)
        pass


    def targetDist(self):
        return math.sqrt((self.current_pose_.pose.position.x - self.uav_target_pose_global_.pose.position.x) ** 2 \
                       + (self.current_pose_.pose.position.y - self.uav_target_pose_global_.pose.position.y) ** 2 \
                       + (self.current_pose_.pose.position.z - self.uav_target_pose_global_.pose.position.z) ** 2)


    def targetYawDiff(self):
        #四元数转欧拉角
        uav_target_r = R.from_quat([self.uav_target_pose_global_.pose.orientation.x, \
                                   self.uav_target_pose_global_.pose.orientation.y, \
                                   self.uav_target_pose_global_.pose.orientation.z, \
                                   self.uav_target_pose_global_.pose.orientation.w])
        current_r = R.from_quat([self.current_pose_.pose.orientation.x, \
                                self.current_pose_.pose.orientation.y, \
                                self.current_pose_.pose.orientation.z, \
                                self.current_pose_.pose.orientation.w])
        [uav_target_y, pitch, roll] = uav_target_r.as_euler('zyx', degrees = True)
        [current_y, pitch, roll] = current_r.as_euler('zyx', degrees = True)
        diff = uav_target_y - current_y
        if diff > 180.0:
            return diff - 360.0
        elif diff < -180.0:
            return diff + 360.0
        else:
            return diff


    def isAlreadyDetected(self, pos):
        for target in self.already_detected_target_:
            if np.linalg.norm(pos - target) < self.target_dist_threshold_:
                return True
        return False


    #从无人机路径位姿队列头部取出一个点作为下一目标点，更新uav_target_pose_global_和uav_target_pose_local_
    def popUAVTargetPose(self):
        if len(self.traj_global_) == 0 or len(self.traj_local_) == 0:
            return
        
        v = self.traj_global_.popleft()
        yaw = self.yaw_global_.popleft()
        (x, y, z, w) = (R.from_euler('zyx', [yaw, 0, 0], degrees = True)).as_quat()
        self.uav_target_pose_global_.header.stamp = rospy.Time.now();
        self.uav_target_pose_global_.pose.position.x = v[0]
        self.uav_target_pose_global_.pose.position.y = v[1]
        self.uav_target_pose_global_.pose.position.z = v[2]
        self.uav_target_pose_global_.pose.orientation.x = x
        self.uav_target_pose_global_.pose.orientation.y = y
        self.uav_target_pose_global_.pose.orientation.z = z
        self.uav_target_pose_global_.pose.orientation.w = w
        
        v = self.traj_local_.popleft()
        yaw = self.yaw_local_.popleft()
        (x, y, z, w) = (R.from_euler('zyx', [yaw, 0, 0], degrees = True)).as_quat()
        self.uav_target_pose_local_.header.stamp = rospy.Time.now();
        self.uav_target_pose_local_.pose.position.x = v[0]
        self.uav_target_pose_local_.pose.position.y = v[1]
        self.uav_target_pose_local_.pose.position.z = v[2]
        self.uav_target_pose_local_.pose.orientation.x = x
        self.uav_target_pose_local_.pose.orientation.y = y
        self.uav_target_pose_local_.pose.orientation.z = z
        self.uav_target_pose_local_.pose.orientation.w = w
        

    def globalTrajPushFront(self, pos, yaw):
        #global
        self.traj_global_.appendleft(pos);
        self.yaw_global_.appendleft(yaw);
        #local
        if is_traj_local_init_:
            self.traj_local_.appendleft(np.matmul(self.R_init_.inv().as_dcm(), pos-self.t_init_))
            self.yaw_local_.appendleft(-self.yaw_init_+yaw+360 if (-self.yaw_init_+yaw < 0) else -self.yaw_init_+yaw)


    def localTrajPushFront(self, pos, yaw):
        #global
        self.traj_global_.appendleft(np.matmul(self.R_init_.as_dcm(), pos) + self.t_init_);
        self.yaw_global_.appendleft(self.yaw_init_+yaw-360 if (self.yaw_init_+yaw > 360) else self.yaw_init_+yaw);
        #local
        if is_traj_local_init_:
            self.traj_local_.appendleft(pos)
            self.yaw_local_.appendleft(yaw)


    def globalTrajPushBack(self, pos, yaw):
        #global
        self.traj_global_.append(pos);
        self.yaw_global_.append(yaw);
        #local
        if is_traj_local_init_:
            self.traj_local_.append(np.matmul(self.R_init_.inv().as_dcm(), pos-self.t_init_))
            self.yaw_local_.append(-self.yaw_init_+yaw+360 if (-self.yaw_init_+yaw < 0) else -self.yaw_init_+yaw)


    def localTrajPushBack(self, pos, yaw):
        #global
        self.traj_global_.append(np.matmul(self.R_init_.as_dcm(), pos) + self.t_init_);
        self.yaw_global_.append(self.yaw_init_+yaw-360 if (self.yaw_init_+yaw > 360) else self.yaw_init_+yaw);
        #local
        if is_traj_local_init_:
            self.traj_local_.append(pos)
            self.yaw_local_.append(yaw)


    #清空无人机路径位姿队列
    def clearTraj(self):
        self.traj_global_ = collections.deque()
        self.yaw_global_ = collections.deque()
        self.traj_local_ = collections.deque()
        self.yaw_local_ = collections.deque()


    def mavstateCallback(self, msg):
        self.current_state_ = msg


    def gazeboposeCallback(self, msg):
        if 'iris' in msg.name:
            i = msg.name.index('iris')
            pose = msg.pose[i]
            self.current_pose_.header.stamp = rospy.Time.now()
            self.current_pose_.pose = pose
            self.t_wu_ = np.array([pose.position.x, pose.position.y, pose.position.z])
            self.R_wu_ = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            if not self.is_traj_local_init_:
                #利用第一次收到的世界坐标将traj_global_、yaw_global_中的轨迹信息转化为本地坐标（以无人机初始化位置为原点的坐标系）
                self.t_init_ = self.t_wu_
                (self.yaw_init_, pitch, roll) = self.R_wu_.as_euler('zyx', degrees = True)
                self.R_init_ = self.R_wu_
                for trajPtr in self.traj_global_:
                    self.traj_local_.append(np.matmul(self.R_init_.inv().as_dcm(), trajPtr-self.t_init_))
                for yawPtr in self.yaw_global_:
                    self.yaw_local_.append(-self.yaw_init_+yawPtr+360 if (-self.yaw_init_+yawPtr < 0) else -self.yaw_init_+yawPtr)
                #初始化第一个飞行目标点
                self.popUAVTargetPose()
            self.is_traj_local_init_ = True


    def targetposeCallback(self, msg):
        self.target_state_ = msg.state
        if self.target_state_:
            p_ct = np.array([msg.position.x, msg.position.y, msg.position.z])
            p_ut = np.matmul(self.R_uc_.as_dcm(), p_ct) + self.t_uc_
            p_wt = np.matmul(self.R_wu_.as_dcm(), p_ut) + self.t_wu_
            if not self.isAlreadyDetected(p_wt):
                self.already_detected_target_.append(p_wt)
                log_str = 'Target detected at %f, %f, %f' % (p_wt[0], p_wt[1], p_wt[2])
                rospy.loginfo(log_str)


    #定时检测无人机状态，根据情况发布arming和起降命令
    def statusloopCallback(self, event):
        if self.flight_state_ == self.FlightState.WAITING:
            if self.current_state_.mode != 'OFFBOARD' and (rospy.Time.now() - self.last_request_ > rospy.Duration(4.0)) and self.is_traj_local_init_:
                response = self.set_mode_client_(0, 'OFFBOARD')  #请求解锁
                if response.mode_sent:
                    rospy.loginfo('Offboard enabled')
                self.last_request_ = rospy.Time.now()
            else:
                if (not self.current_state_.armed) and rospy.Time.now() - self.last_request_ > rospy.Duration(4.0):
                    response = self.arming_client_(True)  #请求起飞
                    if response.success:
                        rospy.loginfo('Vehicle armed')
                        self.flight_state_ = self.FlightState.FLYING
                    self.last_request_ = rospy.Time.now()
        elif self.flight_state_ == self.FlightState.FLYING:
            pass
        elif self.flight_state_ == self.FlightState.READY_TO_LAND:
            rospy.loginfo('Ready to land')
            if self.current_state_.mode != 'AUTO.LAND':
                response = self.set_mode_client_(0, 'AUTO.LAND')  #请求降落
                if response.mode_sent:
                    rospy.loginfo('Vehicle landing')
                    self.flight_state_ = self.FlightState.LANDING
                self.last_request_ = rospy.Time.now()
        else:
            pass
            

    def publishloopCallback(self, event):
        if self.is_traj_local_init_:
            self.uav_target_pose_global_pub_.publish(self.uav_target_pose_global_);
            self.uav_target_pose_local_pub_.publish(self.uav_target_pose_local_);
            self.gazeboposePub_.publish(self.current_pose_);


    #决策策略可以在此函数中实现
    def decisionloopCallback(self, event):
        #如果当前无人机位姿与下一路径位姿差别足够小，做出决策
        if self.flight_state_ == self.FlightState.FLYING and self.targetDist() < self.min_dist_ \
                   and abs(self.targetYawDiff()) < self.min_yaw_diff_: 
            if len(self.traj_local_) == 0:  #已经到达最终路径点，准备降落
                self.flight_state_ = self.FlightState.READY_TO_LAND;
            else: #从队列中取出下一个路径位姿
                self.popUAVTargetPose();


if __name__ == '__main__':
    cc = CoreControllerNode()

