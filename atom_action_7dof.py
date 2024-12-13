#!/usr/bin/env python3
import sys
import random
import pandas as pd
import math
import tf
import geometry_msgs
import moveit_msgs.msg
import moveit_commander 
import rospy
import numpy as np
import random
from std_srvs.srv import Empty
from trajectory_parser import parse_trajectory
import csv
import pickle
import os
from gazebo_msgs.msg import ModelStates

def _create_pose(x_p, y_p, z_p, x_o, y_o, z_o, w_o):
    """Creates a pose using quaternions

    Creates a pose for use with MoveIt! using XYZ coordinates and XYZW
    quaternion values

    :param x_p: The X-coordinate for the pose
    :param y_p: The Y-coordinate for the pose
    :param z_p: The Z-coordinate for the pose
    :param x_o: The X-value for the orientation
    :param y_o: The Y-value for the orientation
    :param z_o: The Z-value for the orientation
    :param w_o: The W-value for the orientation
    :type x_p: float
    :type y_p: float
    :type z_p: float
    :type x_o: float
    :type y_o: float
    :type z_o: float
    :type w_o: float
    :returns: Pose
    :rtype: PoseStamped
    """
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = x_p
    pose_target.position.y = y_p
    pose_target.position.z = z_p
    pose_target.orientation.x = x_o
    pose_target.orientation.y = y_o
    pose_target.orientation.z = z_o
    pose_target.orientation.w = w_o
    return pose_target


def create_pose_euler(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad):

    """Creates a pose using euler angles

    Creates a pose for use with MoveIt! using XYZ coordinates and RPY
    orientation in radians

    :param x_p: The X-coordinate for the pose
    :param y_p: The Y-coordinate for the pose
    :param z_p: The Z-coordinate for the pose
    :param roll_rad: The roll angle for the pose
    :param pitch_rad: The pitch angle for the pose
    :param yaw_rad: The yaw angle for the pose
    :type x_p: float
    :type y_p: float
    :type z_p: float
    :type roll_rad: float
    :type pitch_rad: float
    :type yaw_rad: float
    :returns: Pose
    :rtype: PoseStamped
    """

    quaternion = tf.transformations.quaternion_from_euler(
            roll_rad, pitch_rad, yaw_rad)
    return _create_pose(
            x_p, y_p, z_p,
            quaternion[0], quaternion[1],
            quaternion[2], quaternion[3])


GRIPPER_OPEN_VALUES = (0.04, 0.04)


class MCInterface(object):

    def __init__(self, planning_id='RRT', robot_name='/my_gen3/'):
     
        super(MCInterface, self).__init__()
   
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_commander_interface')

        try:
            self.is_gripper_present = rospy.get_param(robot_name + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(robot_name + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                self.gripper_joint_name = ""
            self.degrees_of_freedom = rospy.get_param(robot_name + "degrees_of_freedom", 7)

            # Create the MoveItInterface necessary objects
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander(robot_name+ "robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns=robot_name)
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=robot_name)
            self.display_trajectory_publisher = rospy.Publisher(robot_name + 'move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)
            
            self.arm_group.set_planner_id(planning_id)
            self.arm_group.set_planning_time(1.0)
            self.arm_group.allow_replanning(False)
            self.arm_group.set_goal_position_tolerance(0.0005)
            self.arm_group.set_goal_orientation_tolerance(0.005)

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=robot_name)

            rospy.loginfo("Initializing node in namespace " + robot_name)
        except Exception as e:
            print (e)
            self.is_init_success = False
        else:
            self.is_init_success = True



    def reach_gripper_position(self, relative_position):
        gripper_group = self.gripper_group
        
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        print(self.gripper_joint_name,gripper_joint,gripper_max_absolute_pos,gripper_min_absolute_pos)
        val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos)
                                  + gripper_min_absolute_pos, True)

        try:
            val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
            return val
        except:
            return False 

    # Works but need to modify
    def plan_end_effector_to_position(self, x_p, y_p, z_p,  roll_rad, pitch_rad, yaw_rad):
        # Uncomment this line of code will cause the kinova point to the sky before doing the movement
        #///////////////////////////////////////////////////////////////////////////////////////////////
        # self.reset()
        #///////////////////////////////////////////////////////////////////////////////////////////////

        pose = create_pose_euler(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
        self.arm_group.set_pose_target(pose)
        plan = self.arm_group.plan()
        return plan[1]

    #Works
    def do_plan(self, plan):
        succeed = self.arm_group.execute(plan)
        self.arm_group.stop()
        return succeed

    def reset(self):
        self.arm_group.clear_pose_targets()
    
    # Works 
    def reach_joint_angles(self, target_angle = [0.0,np.pi/2,0.0,0.0,0.0,0.0,0.0],tolerance = 0.01):
        arm_group = self.arm_group
        success = True
        joint_positions = arm_group.get_current_joint_values()

        # Print out the pose information before movement
        # //////////////////////////////////////////////////////////////////////
        # rospy.loginfo("Printing current joint positions before movement :")
        # self._print_current_pose()
        # //////////////////////////////////////////////////////////////////////

        # Set the goal joint tolerance
        self.arm_group.set_goal_joint_tolerance(tolerance)
        # print(type(joint_positions),joint_positions)
        # joint_positions[0] = target_angle[0]
        # joint_positions[1] = target_angle[1]
        # joint_positions[2] = target_angle[2]
        # joint_positions[3] = target_angle[3]
        # joint_positions[4] = target_angle[4]
        # joint_positions[5] = target_angle[5]
        # joint_positions[6] = target_angle[6]
        joint_positions[0] = 0
        joint_positions[1] = np.pi/2
        joint_positions[2] = 0
        joint_positions[3] = 0
        joint_positions[4] = 0
        joint_positions[5] = 0
        joint_positions[6] = 0

        # Set the joint target configuration   
        arm_group.set_joint_value_target(joint_positions)
        (success_flag, trajectory_message, planning_time, error_code)=arm_group.plan()

        # Plan and execute in one command
        success &= arm_group.go(wait=True)

        # Show joint positions after movement
        #/////////////////////////////////////////////////////////////////////////
        # rospy.loginfo("Printing current joint positions after movement :")
        # self._print_current_pose()
        #/////////////////////////////////////////////////////////////////////////

        return success
    

    def _get_current_joint_values(self):
        return self.arm_group.get_current_joint_values()

    def _get_joints(self):
        return self.arm_group.get_joints()

    # Works
    def _get_current_pose(self):
        current_pose = self.arm_group.get_current_pose()
        # print("Printing current pose informations")
        
        return [current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,\
                    current_pose.pose.orientation.x,
                    current_pose.pose.orientation.y,
                    current_pose.pose.orientation.z,
                    current_pose.pose.orientation.w,]
    # Works
    def _print_current_pose(self):
        pose = self.arm_group.get_current_pose()
        print('Current Pose:')
        print(pose)
        print(self.arm_group.get_current_rpy())

    # Works
    def _print_current_joint_states(self):
        print('Current Joint Values:')
        print(self._get_current_joint_values())

class SimulationInterface(MCInterface):

    def __init__(self, planning_id='RRT', robot_name = '/my_gen3/'):

        super(SimulationInterface, self).__init__( planning_id=planning_id, robot_name = robot_name)

    def read_start_end_pose(self, path):
        file = pd.read_csv(path)
        file = (file.to_numpy()[:,1:])
        start = file[0]
        end = file[1]
        print(start)
        print(end)
        plan_start = self.plan_end_effector_to_position(float(start[0]), float(start[1]), float(start[2]), float(start[3]), float(start[4]), float(start[5]))
        print(plan_start)
        succeed = self.do_plan(plan_start)
        if succeed ==True:
            print("Readly to start")
            plan_end = self.plan_end_effector_to_position(float(end[0]), float(end[1]), float(end[2]), float(end[3]), float(end[4]), float(end[5]))
            succeed2 = self.do_plan(plan_end)
            if succeed2 ==True:
                print("done")
        
    def _to_degree(self,traj):
        # Change radious to degree
        trajectory_degree = []
        for i in range(len(traj)):
            lines = traj[i]
            degree_line = []
            for j in range(len(lines)):
                degrees = math.degrees(lines[j])
                if degrees > 180:
                    degrees -= 360
                elif degrees < -180:
                    degrees += 360
                degree_line.append(degrees)
            trajectory_degree.append(degree_line)
        return trajectory_degree
            
    # Not work, need to modify
    def reset(self, duration):

        self.arm_group.clear_pose_targets()
        reset = rospy.ServiceProxy('lumi_mujoco/reset', Empty)
        try:
            reset()
        except rospy.ServiceException as exc:
            print("Reset did not work:" + str(exc))

        rospy.sleep(duration)
    
    def go_to(self):
        # blue
        # position: 
        #   x: 0.4917820348026094
        #   y: -0.009267880613591076
        #   z: 0.03171447099165203
        # orientation: 
        #   x: 0.6580356555120044
        #   y: 0.6617188193311005
        #   z: 0.25484321922295944
        #   w: 0.2533223516273688
        stop = False
        while stop == False:
            roll_rad_start = 3.14#random.uniform(0, np.pi)
            pitch_rad_start = 0
            yaw_rad_start = -np.pi
            plan_start = self.plan_end_effector_to_position(float(0.0), float(0.500267880613591076), float(0.14171447099165203), roll_rad_start, pitch_rad_start, yaw_rad_start)
            if (plan_start.joint_trajectory.header.frame_id == 'world'):
                # print("Successfully Finding a plan for new start pose: #" + str(i)+" with orientation# "+str(counter1))
                print("Going to the new start pose")
                # counter1=counter1+1
                # start_end_pose = []
                start_pose = [float(0.4917820348026094), float(-0.009267880613591076), float(0.13171447099165203), roll_rad_start, pitch_rad_start, yaw_rad_start]
                print("Start pose: ")
                print(start_pose)
                # start_end_pose.append(start_pose)
                succeed = self.do_plan(plan_start)
                stop = True

    def go_to_object(self, object_list, obj_name):
        for obj in object_list:
            if obj['name']==obj_name:
                target_position = obj['position']
                print(target_position)
                stop = False
                while stop == False:
                    roll_rad_start = 3.14
                    pitch_rad_start = 0
                    yaw_rad_start = -np.pi
                    plan_start = self.plan_end_effector_to_position(float(target_position[0]), float(target_position[1]), float(target_position[2]+0.25), roll_rad_start, pitch_rad_start, yaw_rad_start)
                    if (plan_start.joint_trajectory.header.frame_id == 'world'):
                        # print("Successfully Finding a plan for new start pose: #" + str(i)+" with orientation# "+str(counter1))
                        print("Going to "+ obj_name)
                        # start_pose = [float(0.4917820348026094), float(-0.009267880613591076), float(0.13171447099165203), roll_rad_start, pitch_rad_start, yaw_rad_start]
                        # print("Start pose: ")
                        # print(start_pose)
                        # start_end_pose.append(start_pose)
                        succeed = self.do_plan(plan_start)
                        stop = True
                        if succeed == True:
                            return True
    

    def pick_up_object(self, object_list, obj_name):
        self.reach_gripper_position(0)
        for obj in object_list:
            if obj['name']==obj_name:
                target_position = obj['position']
                print(target_position)
                stop = False
                while stop == False:
                    roll_rad_start = 3.14
                    pitch_rad_start = 0
                    yaw_rad_start = -np.pi
                    plan_start = self.plan_end_effector_to_position(float(target_position[0]), float(target_position[1]), float(target_position[2]+0.03), roll_rad_start, pitch_rad_start, yaw_rad_start)
                    if (plan_start.joint_trajectory.header.frame_id == 'world'):
                        succeed = self.do_plan(plan_start)
                        print("Picking up "+ obj_name)
                        stop = True
                        if succeed == True:
                            self.reach_gripper_position(0.72)
                            time.sleep(2)
                            plan_start = self.plan_end_effector_to_position(float(target_position[0]), float(target_position[1]), float(target_position[2]+0.25), roll_rad_start, pitch_rad_start, yaw_rad_start)
                            succeed = self.do_plan(plan_start)
                            if succeed == True:
                                return True
    

    def drop_object(self, object_list, obj_name):
        for obj in object_list:
            if obj['name']==obj_name:
                target_position = obj['position']
                print(target_position)
                stop = False
                while stop == False:
                    roll_rad_start = 3.14
                    pitch_rad_start = 0
                    yaw_rad_start = -np.pi
                    plan_start = self.plan_end_effector_to_position(float(target_position[0]), float(target_position[1]), float(target_position[2]+0.08), roll_rad_start, pitch_rad_start, yaw_rad_start)
                    if (plan_start.joint_trajectory.header.frame_id == 'world'):
                        succeed = self.do_plan(plan_start)
                        print("Picking up "+ obj_name)
                        stop = True
                        if succeed == True:
                            time.sleep(1)
                            self.reach_gripper_position(0)
                            time.sleep(1)
                            plan_start = self.plan_end_effector_to_position(float(target_position[0]), float(target_position[1]), float(target_position[2]+0.25), roll_rad_start, pitch_rad_start, yaw_rad_start)
                            succeed = self.do_plan(plan_start)
                            if succeed == True:
                                return True

def test_generated_gazebo(self,filename):
    file_list = []
    print("running: ", 1)
    with open(filename, mode ='r')as file:
        csvFile = csv.reader(file)
        
        for lines in csvFile:
            lines.pop(0)
            file_list.append(lines)
    
    file_list.pop(0)
    self.reach_joint_angles(0.01)
    for i in range(len(file_list)):
        self.set_joint_angles(0.01, file_list[i])


def update_object_state():
    object_list = []
    msg = rospy.wait_for_message("/gazebo/model_states", ModelStates)
    for i in range(len(msg.name)):
        print(msg.name[i])
        print("")
        position = []
        position.append(msg.pose[i].position.x)
        position.append(msg.pose[i].position.y)
        position.append(msg.pose[i].position.z)
        position.append(msg.pose[i].orientation.x)
        position.append(msg.pose[i].orientation.y)
        position.append(msg.pose[i].orientation.z)
        position.append(msg.pose[i].orientation.w)
        object = {'name':msg.name[i], 'position': position}
        object_list.append(object)
    return object_list

if __name__ == '__main__':
    import time
    

    
   

    simulation = SimulationInterface(robot_name = f"/my_gen3/")
    
    object_list = update_object_state()
    # simulation.go_to()
    # simulation.reach_gripper_position(0.65)
    state = simulation.go_to_object(object_list, 'Red_box')
    if state==True:
        state = simulation.pick_up_object(object_list, 'Red_box')

    object_list = update_object_state()

    if state==True:
        state = simulation.go_to_object(object_list, 'Green_box')
    if state==True:
        state = simulation.drop_object(object_list, 'Green_box')

    object_list = update_object_state()

    if state==True:
        state = simulation.go_to_object(object_list, 'Blue_box')
    if state==True:
        state = simulation.pick_up_object(object_list, 'Blue_box')

    object_list = update_object_state()

    if state==True:
        state = simulation.go_to_object(object_list, 'Red_box')
    if state==True:
        state = simulation.drop_object(object_list, 'Red_box')

