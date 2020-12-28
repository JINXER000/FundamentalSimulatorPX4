import rospy
import tf
import yaml
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandVtolTransition, SetMode
from geometry_msgs.msg import PoseStamped, Pose, Twist
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
import time
from pyquaternion import Quaternion
import math
from multiprocessing import Process
import sys
import platform
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelStates

class Communication:

    def __init__(self, vehicle_type, vehicle_id):
        
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        self.imu = None
        self.local_pose = None
        self.current_state = None
        self.current_heading = None 
        self.hover_flag = 0
        self.target_motion = PositionTarget()
        self.global_target = None
        self.arm_state = False
        self.offboard_state = False
        self.motion_type = 0
        self.flight_mode = None
        self.mission = None
        self.transition_state = None
        self.transition = None
        
        self.platform = platform.platform()
            
        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber(self.vehicle_type+'_'+self.vehicle_id+"/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.mavros_sub = rospy.Subscriber(self.vehicle_type+'_'+self.vehicle_id+"/mavros/state", State, self.mavros_state_callback)
        self.imu_sub = rospy.Subscriber(self.vehicle_type+'_'+self.vehicle_id+"/mavros/imu/data", Imu, self.imu_callback)
        self.cmd_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd",String,self.cmd_callback)
        self.cmd_pose_flu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_pose_flu", Pose, self.cmd_pose_flu_callback)
        self.offboard_cmd_pose_sub = rospy.Subscriber("/command/pose", PoseStamped, self.offboard_cmd_pose_callback)

        self.cmd_pose_enu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_pose_enu", Pose, self.cmd_pose_enu_callback)
        self.cmd_vel_flu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_vel_flu", Twist, self.cmd_vel_flu_callback)
        self.cmd_vel_enu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_vel_enu", Twist, self.cmd_vel_enu_callback)
        self.cmd_accel_flu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_accel_flu", Twist, self.cmd_accel_flu_callback)
        self.cmd_accel_enu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_accel_enu", Twist, self.cmd_accel_enu_callback)
            
        ''' 
        ros publishers
        '''
        self.target_motion_pub = rospy.Publisher(self.vehicle_type+'_'+self.vehicle_id+"/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        self.odom_groundtruth_pub = rospy.Publisher('/xtdrone/'+self.vehicle_type+'_'+self.vehicle_id+'/ground_truth/odom', Odometry, queue_size=10)
        self.odom_GTego_pub = rospy.Publisher('/ground_truth/state', Odometry, queue_size=10)
        # self.sub_odom = rospy.Subscriber('/gazebo/model_states',ModelStates,self.odom_callback)
        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy(self.vehicle_type+'_'+self.vehicle_id+"/mavros/cmd/arming", CommandBool)
        self.flightModeService = rospy.ServiceProxy(self.vehicle_type+'_'+self.vehicle_id+"/mavros/set_mode", SetMode)
        self.gazeboModelstate = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        self.in_air= rospy.Service('engage',Empty,self.takeoff_handler)

        print(self.vehicle_type+'_'+self.vehicle_id+": "+"communication initialized")

    def takeoff_handler(self,msg):
        print("Engage!!")
        for i in range(10):
            if self.current_heading is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)
        self.coordinate_frame = 1
        self.target_motion=self.construct_target(z=2,yaw=self.current_heading)

        for i in range(5):
            self.arm_state =self.arm()
            print(self.vehicle_type+'_'+self.vehicle_id+": Armed "+str(self.arm_state))

            self.offboard_state=True
            self.flight_mode = 'OFFBOARD'
            self.flight_mode_switch()
            # self.target_motion_pub.publish(self.target_motion)
            time.sleep(0.2)
        return []
    # def odom_callback(self,msg):
    #     # https://blog.csdn.net/weixin_43180028/article/details/104587506
    #     # https://github.com/LizhiyuanBest/PROBOT_Anno/blob/master/probot_grasping/scripts/grasping_demo.py

    #     for i,x in enumerate(msg.name):
    #         if x =='iris_0':

    #             trueodom=Odometry()
    #             trueodom.header.stamp = rospy.Time.now()
    #             pose=msg.pose[i]
    #             quat=pose.orientation
    #             pos=pose.position
    #             # print(quat)
    #             trueodom.pose.pose.orientation=quat
    #             trueodom.pose.pose.position=pos

    #             vel=msg.twist[i]
    #             vx=vel.linear.x
    #             vy=vel.linear.y
    #             wz=vel.angular.z
    #             v = math.sqrt(math.pow(vx,2)+math.pow(vy,2))
    #             trueodom.twist.twist.linear.x=v
    #             trueodom.twist.twist.angular.z=wz

    #             self.odom_groundtruth_pub.publish(trueodom)
    #             break

    def start(self):
        rospy.init_node(self.vehicle_type+'_'+self.vehicle_id+"_communication")
        rate = rospy.Rate(100)
        '''
        main ROS thread
        '''
        while not rospy.is_shutdown():
            self.target_motion_pub.publish(self.target_motion)
            
            if (self.flight_mode is "LAND") and (self.local_pose.pose.position.z < 0.15):
                if(self.disarm()):
                    self.flight_mode = "DISARMED"
                    
            try:
                response = self.gazeboModelstate (self.vehicle_type+'_'+self.vehicle_id,'ground_plane')
            except rospy.ServiceException, e:
                print "Gazebo model state service call failed: %s"%e
            odom = Odometry()
            odom.header.stamp = response.header.stamp
            odom.header.frame_id ="map"
            odom.pose.pose = response.pose
            odom.twist.twist = response.twist
            self.odom_groundtruth_pub.publish(odom)

            trueodom=Odometry()
            trueodom.header.stamp = self.local_pose.header.stamp
            trueodom.header.frame_id ="world"
            pose=self.local_pose.pose
            quat=pose.orientation
            pos=pose.position
            # print(quat)
            trueodom.pose.pose.orientation=quat
            trueodom.pose.pose.position=pos
            self.odom_GTego_pub.publish(trueodom)
            rate.sleep()

    def local_pose_callback(self, msg):
        self.local_pose = msg


    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode

    def imu_callback(self, msg):
        self.current_heading = self.q2yaw(msg.orientation)

    def construct_target(self, x=0, y=0, z=0, vx=0, vy=0, vz=0, afx=0, afy=0, afz=0, yaw=0, yaw_rate=0):
        target_raw_pose = PositionTarget()
        target_raw_pose.coordinate_frame = self.coordinate_frame
        
        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        target_raw_pose.velocity.x = vx
        target_raw_pose.velocity.y = vy
        target_raw_pose.velocity.z = vz
        
        target_raw_pose.acceleration_or_force.x = afx
        target_raw_pose.acceleration_or_force.y = afy
        target_raw_pose.acceleration_or_force.z = afz

        target_raw_pose.yaw = yaw
        target_raw_pose.yaw_rate = yaw_rate

        if(self.motion_type == 0):
            target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                            + PositionTarget.IGNORE_YAW_RATE
        elif(self.motion_type == 1):
            target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                            + PositionTarget.IGNORE_YAW
        elif(self.motion_type == 2):
            target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                            + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                            + PositionTarget.IGNORE_YAW
        else: # pva
            target_raw_pose.type_mask= 0b0000000000000000

        return target_raw_pose

    def cmd_pose_flu_callback(self, msg):
        self.coordinate_frame = 9
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z)

    def offboard_cmd_pose_callback(self, msg):
        self.coordinate_frame = 1
        self.target_motion = self.construct_target(x=msg.pose.position.x,y=msg.pose.position.y,z=msg.pose.position.z)
 
    def cmd_pose_enu_callback(self, msg):
        self.coordinate_frame = 1
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z)
        
    def cmd_vel_flu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 8
            self.motion_type = 1     
            self.target_motion = self.construct_target(vx=msg.linear.x,vy=msg.linear.y,vz=msg.linear.z,yaw_rate=msg.angular.z)       
 
    def cmd_vel_enu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 1
            self.motion_type = 1
            self.target_motion = self.construct_target(vx=msg.linear.x,vy=msg.linear.y,vz=msg.linear.z,yaw_rate=msg.angular.z)

    def cmd_accel_flu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 8
            self.motion_type = 2
            self.target_motion = self.construct_target(afx=msg.linear.x,afy=msg.linear.y,afz=msg.linear.z,yaw_rate=msg.angular.z)
            
    def cmd_accel_enu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 1 
            self.motion_type = 2
            self.target_motion = self.construct_target(afx=msg.linear.x,afy=msg.linear.y,afz=msg.linear.z,yaw_rate=msg.angular.z)
            
    def hover_state_transition(self,x,y,z,w):
        if abs(x) > 0.005 or abs(y)  > 0.005 or abs(z)  > 0.005 or abs(w)  > 0.005:
            self.hover_flag = 0

    def cmd_callback(self, msg):
        if msg.data == '':
            return

        elif msg.data == 'ARM':
            self.arm_state =self.arm()
            print(self.vehicle_type+'_'+self.vehicle_id+": Armed "+str(self.arm_state))

        elif msg.data == 'DISARM':
            self.arm_state = not self.disarm()
            print(self.vehicle_type+'_'+self.vehicle_id+": Armed "+str(self.arm_state))

        elif msg.data[:-1] == "mission" and not msg.data == self.mission:
            self.mission = msg.data
            print(self.vehicle_type+'_'+self.vehicle_id+": "+msg.data)

        elif not msg.data == self.flight_mode:
            self.flight_mode = msg.data
            self.flight_mode_switch()
            

    def q2yaw(self, q):
        if isinstance(q, Quaternion):
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad
    
    def arm(self):
        if self.armService(True):
            return True
        else:
            print(self.vehicle_type+'_'+self.vehicle_id+": arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print(self.vehicle_type+'_'+self.vehicle_id+": disarming failed!")
            return False

    def hover(self):
        self.coordinate_frame = 1
        self.motion_type = 0
        self.target_motion = self.construct_target(x=self.local_pose.pose.position.x,y=self.local_pose.pose.position.y,z=self.local_pose.pose.position.z)

    def flight_mode_switch(self):
        if self.flight_mode == 'HOVER':
            self.hover_flag = 1
            self.hover()
            print(self.vehicle_type+'_'+self.vehicle_id+":"+self.flight_mode)
        elif self.flightModeService(custom_mode=self.flight_mode):
            print(self.vehicle_type+'_'+self.vehicle_id+": "+self.flight_mode)
            return True
        else:
            print(self.vehicle_type+'_'+self.vehicle_id+": "+self.flight_mode+"failed")
            return False

    def takeoff_detection(self):
        if self.local_pose.pose.position.z > 0.3 and self.arm_state:
            return True
        else:
            return False

if __name__ == '__main__':
    communication = Communication(sys.argv[1],sys.argv[2])
    communication.start()
