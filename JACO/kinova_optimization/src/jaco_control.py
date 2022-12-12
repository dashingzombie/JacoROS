#! /usr/bin/env python3

__author__ = 'yepw'

import rospy
import actionlib

from kinova_optimization.msg import EEPoseGoals, JointAngles
import kinova_msgs.msg
from std_msgs.msg import Bool
import kinova_msgs.msg
import numpy as np

USE_GRIPPER = False
prefix = 'j2s7s300_'

class JacoControl():
    def __init__(self):

        if USE_GRIPPER:
            try:
                pass
            except (ValueError, OSError) as e:
                rospy.logerr("Could not detect an electric gripper attached to the robot.")
                self.clean_shutdown()
                return

            self.original_deadzone = self.gripper.get_dead_zone()
            self.pre_grasping_msg = False
            self.gripper_status = "open"

        rospy.Subscriber('/relaxed_ik/joint_angle_solutions', JointAngles, self.joint_solution_cb)
        
        self.curr_joint_degrees = [0] * 7
        rospy.Subscriber('/{}driver/out/joint_angles'.format(prefix), kinova_msgs.msg.JointAngles, self.curr_joint_angle_cb)
        
        if USE_GRIPPER:
            rospy.Subscriber('/robot_state/grasping', Bool, self.gripper_cb)

        self.joint_vel_pub = rospy.Publisher('/{}driver/in/joint_velocity'.format(prefix), kinova_msgs.msg.JointVelocity)
        rospy.sleep(1)

        """Send a joint angle goal to the action server."""
        action_address = '/' + prefix + 'driver/joints_action/joint_angles'
        self.client = actionlib.SimpleActionClient(action_address,
                                            kinova_msgs.msg.ArmJointAnglesAction)
        self.client.wait_for_server()

        # move to init pose
        self.move_to_init_pose()
        
    #    self.move_jaco_joint_angles()

        print("Control starts!")

    def curr_joint_angle_cb(self, msg):
        self.curr_joint_degrees[0] = msg.joint1
        self.curr_joint_degrees[1] = msg.joint2
        self.curr_joint_degrees[2] = msg.joint3
        self.curr_joint_degrees[3] = msg.joint4
        self.curr_joint_degrees[4] = msg.joint5
        self.curr_joint_degrees[5] = msg.joint6
        self.curr_joint_degrees[6] = msg.joint7

    def move_jaco_joint_angles(self, angle_set):
        '''
        angle_set are in degrees
        '''
        
        assert len(angle_set) == 7

        goal = kinova_msgs.msg.ArmJointAnglesGoal()

        goal.angles.joint1 = angle_set[0]
        goal.angles.joint2 = angle_set[1]
        goal.angles.joint3 = angle_set[2]
        goal.angles.joint4 = angle_set[3]
        goal.angles.joint5 = angle_set[4]
        goal.angles.joint6 = angle_set[5]
        goal.angles.joint7 = angle_set[6]

        self.client.send_goal(goal)
        if self.client.wait_for_result(rospy.Duration(20.0)):
            return self.client.get_result()
        else:
            print('        the joint angle action timed-out')
            self.client.cancel_all_goals()
            return None

    def clean_shutdown(self):
        if self.gripper and self.original_deadzone:
            self.gripper.set_dead_zone(self.original_deadzone)
            print("Exiting sawyer control.")

    def move_to_init_pose(self):
        angles = [0]*7
        angles[0] = np.rad2deg(0.0)
        
        angles[1] = np.rad2deg(3.596565083)
        angles[2] = np.rad2deg(0.0)
        angles[3] = np.rad2deg(5.19409985393)
        angles[4] = np.rad2deg(0.0)
        angles[5] = np.rad2deg(3.14159265359)
        angles[6] = np.rad2deg(1.57)
        self.move_jaco_joint_angles(angles)
        
    def move_to_new_pose(self):
        angles = [0]*7
        angles[0] = np.rad2deg(0.0)
        angles[1] = np.rad2deg(5.596565083)
        angles[2] = np.rad2deg(0.0)
        angles[3] = np.rad2deg(5.19409985393)
        angles[4] = np.rad2deg(0.0)
        angles[5] = np.rad2deg(1.14159265359)
        angles[6] = np.rad2deg(1.57)
        self.move_jaco_joint_angles(angles)

    def joint_solution_cb(self, msg):
        assert len(msg.angles.data) == 7

        # position control
        angles = [0]*7
        angles[0] = np.rad2deg(msg.angles.data[0])
        angles[1] = np.rad2deg(msg.angles.data[1])
        angles[2] = np.rad2deg(msg.angles.data[2])
        angles[3] = np.rad2deg(msg.angles.data[3])
        angles[4] = np.rad2deg(msg.angles.data[4])
        angles[5] = np.rad2deg(msg.angles.data[5])
        angles[6] = np.rad2deg(msg.angles.data[6])
        # self.move_jaco_joint_angles(angles)

        # velocity control
        self.move_jaco_joint_velocities(angles)

    def move_jaco_joint_velocities(self, goal_degrees):
        assert len(goal_degrees) == 7
        vels = []
        for i,x in enumerate(goal_degrees):
            vels.append( 2 * (x - self.curr_joint_degrees[i]))

        msg = kinova_msgs.msg.JointVelocity()

        msg.joint1 = vels[0]
        msg.joint2 = vels[1]
        msg.joint3 = vels[2]
        msg.joint4 = vels[3]
        msg.joint5 = vels[4]
        msg.joint6 = vels[5]
        msg.joint7 = vels[6]

        # print(goal_degrees, self.curr_joint_degrees, vels)
        self.joint_vel_pub.publish(msg)


    def gripper_cb(self, msg):
        if (msg.data == True and self.pre_grasping_msg != True):
            if (self.gripper_status == "open"):
                self.gripper.close()
                self.gripper_status = "closed"
            else:
                self.gripper.open()
                self.gripper_status = "open"
        self.pre_grasping_msg = msg.data

if __name__ == '__main__':
    rospy.init_node('jaco_control')
    jacoControl = JacoControl()
    rospy.spin()

