#! /usr/bin/env python3

import ctypes
import numpy
import os
import rospkg
import rospy
import sys
import utils
import transformations as T
import yaml

from kinova_optimization.msg import EEPoseGoals, JointAngles
from geometry_msgs.msg import Pose
from robot import Robot
from std_msgs.msg import Float32MultiArray, Bool, String
from sensor_msgs.msg import JointState
from timeit import default_timer as timer
from urdf_load import urdf_load

class Opt(ctypes.Structure):
    _fields_ = [("data", ctypes.POINTER(ctypes.c_double)), ("length", ctypes.c_int)]

class RelaxedIK():
    def __init__(self):

        path_to_src = rospkg.RosPack().get_path('kinova_optimization')
        env_settings_file_path = path_to_src + '/kinova_optimization_core/config/settings.yaml'

        os.chdir(path_to_src + "/kinova_optimization_core")

        self.lib = ctypes.cdll.LoadLibrary(path_to_src + '/kinova_optimization_core/target/debug/librelaxed_ik_lib.so')
        self.lib.solve.restype = Opt
        
        # Load the infomation
        env_settings_file = open(env_settings_file_path, 'r')
        env_settings = yaml.load(env_settings_file, Loader=yaml.FullLoader)

        if 'loaded_robot' in env_settings:
            robot_info = env_settings['loaded_robot']
        else:
            raise NameError('Please define the relevant information of the robot!')

        info_file_name = robot_info['name']
        
        # Publishers
        self.angles_pub = rospy.Publisher('joint_angle_solutions', JointAngles, queue_size=1)

        ##===
        info_file_path = path_to_src + '/kinova_optimization_core/config/info_files/' + info_file_name
        info_file = open(info_file_path, 'r')
        y = yaml.load(info_file, Loader=yaml.FullLoader)
        starting_config = y['starting_config']

        rospy.Subscriber('ee_pose_goals', EEPoseGoals, self.eePoseGoals_cb)
        rospy.Subscriber('task_prepare', JointState, self.task_prepare_cb)
        
        self.init_pose = Pose()
        self.init_pose.position.x = 0
        self.init_pose.position.y = 0
        self.init_pose.position.z = 0
        self.init_pose.orientation.x = 0
        self.init_pose.orientation.y = 0
        self.init_pose.orientation.z = 0
        self.init_pose.orientation.w = 1

        self.pos_arr = None
        self.quat_arr = None
        self.msg = None
        self.rate = rospy.get_param("~rate")
        self.update_num = 0
        self.timer = rospy.Timer(rospy.Duration(1.0/self.rate), self.timer_cb)

    def timer_cb(self, timer):
        if self.update_num == 0:
            return
        self.update_num -= 1
        if (self.pos_arr and self.quat_arr ):
            xopt = self.lib.solve(self.pos_arr, len(self.pos_arr), self.quat_arr, len(self.quat_arr))

            ja = JointAngles()
            ja.header = self.eeMsg.header

            ja.notes.data = self.eeMsg.notes.data
            self.eeMsg.notes.data = ""

            for i in range(xopt.length):
                ja.angles.data.append(xopt.data[i])

            self.angles_pub.publish(ja)

    def eePoseGoals_cb(self, msg):

        pose_goals = msg.ee_poses
        self.eeMsg = msg

        self.pos_arr = (ctypes.c_double * (3 * len(pose_goals)))()
        self.quat_arr = (ctypes.c_double * (4 * len(pose_goals)))()

        for i in range(len(pose_goals)):
            p = pose_goals[i]
            self.pos_arr[3*i] = p.position.x
            self.pos_arr[3*i+1] = p.position.y
            self.pos_arr[3*i+2] = p.position.z

            self.quat_arr[4*i] = p.orientation.x
            self.quat_arr[4*i+1] = p.orientation.y
            self.quat_arr[4*i+2] = p.orientation.z
            self.quat_arr[4*i+3] = p.orientation.w
        
        self.update_num = 500

    def task_prepare_cb(self, msg):

        init_arr = (ctypes.c_double * 7)()

        if len(msg.position) == 7:
            init_arr[0] = msg.position[0]
            init_arr[1] = msg.position[1]
            init_arr[2] = msg.position[2]
            init_arr[3] = msg.position[3]
            init_arr[4] = msg.position[4]
            init_arr[5] = msg.position[5]
            init_arr[6] = msg.position[6]
        else:
            init_arr[0] = 0.0001
            init_arr[1] = 0.0001
            init_arr[2] = -1.5708
            init_arr[3] =  1.5708
            init_arr[4] =  0.0001
            init_arr[5] =  -1.5708
            init_arr[6] =  0.175

        if (self.pos_arr and self.quat_arr ):
            self.pos_arr[0] = 0.
            self.pos_arr[1] = 0.
            self.pos_arr[2] = 0.
            self.quat_arr[0] = 0.
            self.quat_arr[1] = 0.
            self.quat_arr[2] = 0.
            self.quat_arr[3] = 1.

        self.lib.recover_vars(init_arr, len(init_arr))

        print("recovered arm poses")
        self.update_num = 5
        # rospy.sleep(0.01)
        # eeMsg = EEPoseGoals()
        # eeMsg.ee_poses.append(self.init_pose)
        # eeMsg.notes.data = ""
        # self.eePoseGoals_cb(eeMsg)


if __name__ == '__main__':
    rospy.init_node('relaxed_ik')
    relaxedIK = RelaxedIK()
    rospy.spin()