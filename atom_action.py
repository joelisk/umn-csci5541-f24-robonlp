#!/usr/bin/env python3

# import sys
# import random
# import pandas as pd
# import math
# import tf
# import geometry_msgs
# import moveit_msgs.msg
# import moveit_commander 
# import rospy
# import numpy as np
# import random
# from std_srvs.srv import Empty
# from trajectory_parser import parse_trajectory
# import csv
# import pickle
# import os
# from gazebo_msgs.msg import ModelStates
# def _create_pose(x_p, y_p, z_p, x_o, y_o, z_o, w_o):
#     """Creates a pose using quaternions

#     Creates a pose for use with MoveIt! using XYZ coordinates and XYZW
#     quaternion values

#     :param x_p: The X-coordinate for the pose
#     :param y_p: The Y-coordinate for the pose
#     :param z_p: The Z-coordinate for the pose
#     :param x_o: The X-value for the orientation
#     :param y_o: The Y-value for the orientation
#     :param z_o: The Z-value for the orientation
#     :param w_o: The W-value for the orientation
#     :type x_p: float
#     :type y_p: float
#     :type z_p: float
#     :type x_o: float
#     :type y_o: float
#     :type z_o: float
#     :type w_o: float
#     :returns: Pose
#     :rtype: PoseStamped
#     """
#     pose_target = geometry_msgs.msg.Pose()
#     pose_target.position.x = x_p
#     pose_target.position.y = y_p
#     pose_target.position.z = z_p
#     pose_target.orientation.x = x_o
#     pose_target.orientation.y = y_o
#     pose_target.orientation.z = z_o
#     pose_target.orientation.w = w_o
#     return pose_target


# def create_pose_euler(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad):

#     """Creates a pose using euler angles

#     Creates a pose for use with MoveIt! using XYZ coordinates and RPY
#     orientation in radians

#     :param x_p: The X-coordinate for the pose
#     :param y_p: The Y-coordinate for the pose
#     :param z_p: The Z-coordinate for the pose
#     :param roll_rad: The roll angle for the pose
#     :param pitch_rad: The pitch angle for the pose
#     :param yaw_rad: The yaw angle for the pose
#     :type x_p: float
#     :type y_p: float
#     :type z_p: float
#     :type roll_rad: float
#     :type pitch_rad: float
#     :type yaw_rad: float
#     :returns: Pose
#     :rtype: PoseStamped
#     """

#     quaternion = tf.transformations.quaternion_from_euler(
#             roll_rad, pitch_rad, yaw_rad)
#     return _create_pose(
#             x_p, y_p, z_p,
#             quaternion[0], quaternion[1],
#             quaternion[2], quaternion[3])


# GRIPPER_OPEN_VALUES = (0.04, 0.04)


# class MCInterface(object):

#     def __init__(self, planning_id='RRT', robot_name='/my_gen3/'):
     
#         super(MCInterface, self).__init__()
   
#         moveit_commander.roscpp_initialize(sys.argv)
#         rospy.init_node('moveit_commander_interface')

#         try:
#             self.is_gripper_present = rospy.get_param(robot_name + "is_gripper_present", False)
#             if self.is_gripper_present:
#                 gripper_joint_names = rospy.get_param(robot_name + "gripper_joint_names", [])
#                 self.gripper_joint_name = gripper_joint_names[0]
#             else:
#                 self.gripper_joint_name = ""
#             self.degrees_of_freedom = rospy.get_param(robot_name + "degrees_of_freedom", 7)

#             # Create the MoveItInterface necessary objects
#             arm_group_name = "arm"
#             self.robot = moveit_commander.RobotCommander(robot_name+ "robot_description")
#             self.scene = moveit_commander.PlanningSceneInterface(ns=robot_name)
#             print("////////////////")
#             self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns="/my_gen3_lite/")
#             print("////////////////")
#             self.display_trajectory_publisher = rospy.Publisher(robot_name + 'move_group/display_planned_path',
#                                                             moveit_msgs.msg.DisplayTrajectory,
#                                                             queue_size=20)
            
#             self.arm_group.set_planner_id(planning_id)
#             self.arm_group.set_planning_time(1.0)
#             self.arm_group.allow_replanning(False)
#             self.arm_group.set_goal_position_tolerance(0.0005)
#             self.arm_group.set_goal_orientation_tolerance(0.005)

#             if self.is_gripper_present:
#                 gripper_group_name = "gripper"
#                 self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=robot_name)

#             rospy.loginfo("Initializing node in namespace " + robot_name)
#         except Exception as e:
#             print (e)
#             self.is_init_success = False
#         else:
#             self.is_init_success = True



#     def reach_gripper_position(self, relative_position):
#         gripper_group = self.gripper_group
        
#         gripper_joint = self.robot.get_joint(self.gripper_joint_name)
#         gripper_max_absolute_pos = gripper_joint.max_bound()
#         gripper_min_absolute_pos = gripper_joint.min_bound()
#         print(self.gripper_joint_name,gripper_joint,gripper_max_absolute_pos,gripper_min_absolute_pos)
#         val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos)
#                                   + gripper_min_absolute_pos, True)

#         try:
#             val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
#             return val
#         except:
#             return False 

#     # Works but need to modify
#     def plan_end_effector_to_position(self, x_p, y_p, z_p,  roll_rad, pitch_rad, yaw_rad):
#         # Uncomment this line of code will cause the kinova point to the sky before doing the movement
#         #///////////////////////////////////////////////////////////////////////////////////////////////
#         # self.reset()
#         #///////////////////////////////////////////////////////////////////////////////////////////////

#         pose = create_pose_euler(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
#         self.arm_group.set_pose_target(pose)
#         plan = self.arm_group.plan()
#         return plan[1]

#     #Works
#     def do_plan(self, plan):
#         succeed = self.arm_group.execute(plan)
#         self.arm_group.stop()
#         return succeed

#     def reset(self):
#         self.arm_group.clear_pose_targets()
    
#     # Works 
#     def reach_joint_angles(self, target_angle = [0.0,np.pi/2,0.0,0.0,0.0,0.0,0.0],tolerance = 0.01):
#         arm_group = self.arm_group
#         success = True
#         joint_positions = arm_group.get_current_joint_values()

#         # Print out the pose information before movement
#         # //////////////////////////////////////////////////////////////////////
#         # rospy.loginfo("Printing current joint positions before movement :")
#         # self._print_current_pose()
#         # //////////////////////////////////////////////////////////////////////

#         # Set the goal joint tolerance
#         self.arm_group.set_goal_joint_tolerance(tolerance)
#         # print(type(joint_positions),joint_positions)
#         # joint_positions[0] = target_angle[0]
#         # joint_positions[1] = target_angle[1]
#         # joint_positions[2] = target_angle[2]
#         # joint_positions[3] = target_angle[3]
#         # joint_positions[4] = target_angle[4]
#         # joint_positions[5] = target_angle[5]
#         # joint_positions[6] = target_angle[6]
#         joint_positions[0] = 0
#         joint_positions[1] = np.pi/2
#         joint_positions[2] = 0
#         joint_positions[3] = 0
#         joint_positions[4] = 0
#         joint_positions[5] = 0
#         joint_positions[6] = 0

#         # Set the joint target configuration   
#         arm_group.set_joint_value_target(joint_positions)
#         (success_flag, trajectory_message, planning_time, error_code)=arm_group.plan()

#         # Plan and execute in one command
#         success &= arm_group.go(wait=True)

#         # Show joint positions after movement
#         #/////////////////////////////////////////////////////////////////////////
#         # rospy.loginfo("Printing current joint positions after movement :")
#         # self._print_current_pose()
#         #/////////////////////////////////////////////////////////////////////////

#         return success
    

#     def _get_current_joint_values(self):
#         return self.arm_group.get_current_joint_values()

#     def _get_joints(self):
#         return self.arm_group.get_joints()

#     # Works
#     def _get_current_pose(self):
#         current_pose = self.arm_group.get_current_pose()
#         # print("Printing current pose informations")
        
#         return [current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,\
#                     current_pose.pose.orientation.x,
#                     current_pose.pose.orientation.y,
#                     current_pose.pose.orientation.z,
#                     current_pose.pose.orientation.w,]
#     # Works
#     def _print_current_pose(self):
#         pose = self.arm_group.get_current_pose()
#         print('Current Pose:')
#         print(pose)
#         print(self.arm_group.get_current_rpy())

#     # Works
#     def _print_current_joint_states(self):
#         print('Current Joint Values:')
#         print(self._get_current_joint_values())

# class SimulationInterface(MCInterface):

#     def __init__(self, planning_id='RRT', robot_name = '/my_gen3/'):

#         super(SimulationInterface, self).__init__( planning_id=planning_id, robot_name = robot_name)

#         # self.workspace = pd.read_csv(WORKSPACE).to_numpy()
#         # self.len = len(self.workspace)
#         # with open(START_PATH, 'rb') as f:
#         #     self.start = pickle.load(f)
#         # with open(TARGET_PATH, 'rb') as f:
#         #     self.target = pickle.load(f)
#     # This function will generate trajectories to a random position and always using current pose as the start pose
#     # def generate(self,output_path):
#     #     num = 1
#     #     num_movement = 0
#     #     shuffle_index = random.sample(range(self.len),num)
#     #     print("asdfasdfas", shuffle_index)
#     #     for i in shuffle_index:
#     #         position = self.workspace[i]
#     #         counter = 0
#     #         while counter<5:
#     #             roll_rad = random.uniform(0, np.pi)
#     #             pitch_rad = random.uniform(0, np.pi)
#     #             yaw_rad = random.uniform(0, np.pi)
#     #             plan = self.plan_end_effector_to_position(float(position[0]), float(position[1]), float(position[2]), roll_rad, pitch_rad, yaw_rad)

#     #             if plan.joint_trajectory.header.frame_id == 'world':
#     #                 print("Successfully Finding the plan for position: #" + str(i) + " with orientation# "+str(counter))
#     #                 num_movement = num_movement +1
#     #                 counter = counter+1
#     #                 time_steps_raw, positions_raw, velocities_raw, accelerations_raw = parse_trajectory(plan)
#     #                 _, trajectory, _, _ = smooth_trajectory(time_steps_raw, positions_raw,velocities_raw,  accelerations_raw, 60,7)
#     #                 trajectory_degree = self._to_degree(trajectory)
#     #                 trajectory_degree = np.array(trajectory_degree)
#     #                 pos = pd.DataFrame(trajectory_degree)
#     #                 pos.to_csv(output_path + str(num_movement)+ "velocity.csv")
#     #             else:
#     #                 print("Try again for position: " + str(i) + " with orientation #"+str(counter))
#     #                 counter = counter

#     # This function will generate trajectories to a random position and randomly using a pose as the start pose
#     # def rand_generate(self,output_path, output_se_path, num_start_pose, num_target_pose, num_orientation):
#     #     # num = num_start_pose
#     #     num_movement = 0
#     #     shuffle_index_start = random.sample(range(self.len),num_start_pose)
#     #     shuffle_index_target= random.sample(range(self.len),num_target_pose)
#     #     for i in shuffle_index_start:
#     #         position_start = self.workspace[i]
#     #         counter1 = 0
#     #         while counter1<num_orientation:
#     #             print("finding a new start pose.....")
#     #             roll_rad_start = random.uniform(0, np.pi)
#     #             pitch_rad_start = random.uniform(0, np.pi)
#     #             yaw_rad_start = random.uniform(0, np.pi)
#     #             plan_start = self.plan_end_effector_to_position(float(position_start[0]), float(position_start[1]), float(position_start[2]), roll_rad_start, pitch_rad_start, yaw_rad_start)
#     #             if (plan_start.joint_trajectory.header.frame_id == 'world'):
#     #                 print("Successfully Finding a plan for new start pose: #" + str(i)+" with orientation# "+str(counter1))
#     #                 print("Going to the new start pose")
#     #                 counter1=counter1+1
#     #                 start_end_pose = []
#     #                 start_pose = [float(position_start[0]), float(position_start[1]), float(position_start[2]), roll_rad_start, pitch_rad_start, yaw_rad_start]
#     #                 print("Start pose: ")
#     #                 print(start_pose)
#     #                 start_end_pose.append(start_pose)
#     #                 succeed = self.do_plan(plan_start)
#     #                 if succeed == True:
#     #                     print("Start pose is ready")
#     #                     for j in shuffle_index_target:
#     #                         position_target = self.workspace[j]
#     #                         counter2=0
#     #                         while counter2<num_orientation:
#     #                             roll_rad_end = random.uniform(0, np.pi)
#     #                             pitch_rad_end = random.uniform(0, np.pi)
#     #                             yaw_rad_end = random.uniform(0, np.pi)
#     #                             plan_end = self.plan_end_effector_to_position(float(position_target[0]), float(position_target[1]), float(position_target[2]), roll_rad_end, pitch_rad_end, yaw_rad_end)
#     #                             if (plan_end.joint_trajectory.header.frame_id == 'world'):
#     #                                 print("Successfully Finding a plan for target pose: #" + str(j)+" with orientation# "+str(counter2))
#     #                                 num_movement = num_movement +1
#     #                                 counter2 = counter2+1
#     #                                 end_pose = [float(position_target[0]), float(position_target[1]), float(position_target[2]), roll_rad_end, pitch_rad_end, yaw_rad_end]
#     #                                 print("end pose: ")
#     #                                 print(end_pose)
#     #                                 start_end_pose.append(end_pose)
#     #                                 pos = pd.DataFrame(start_end_pose)
#     #                                 pos.to_csv(output_se_path + str(num_movement)+ "_se_pose.csv")
#     #                                 time_steps_raw, positions_raw, velocities_raw, accelerations_raw = parse_trajectory(plan_end)
#     #                                 _, trajectory, _, _ = smooth_trajectory(time_steps_raw, positions_raw,velocities_raw,  accelerations_raw, 60,7)
#     #                                 trajectory_degree = self._to_degree(trajectory)
#     #                                 trajectory_degree = np.array(trajectory_degree)
#     #                                 pos = pd.DataFrame(trajectory_degree)
#     #                                 pos.to_csv(output_path + str(num_movement)+ ".csv")
#     #                                 start_end_pose.pop(1)
                                    
#     #                             else:
#     #                                 print("Try again for position: " + str(j) + " with orientation #"+str(counter2))
#     #                                 counter2 = counter2
#     #             else:
#     #                 print("Retrying finding a plan for new start pose: #" + str(i))
#     #                 counter1 = counter1

#     # def rand_velocity_generate(self,output_path,job, total_worker, root, num_start_pose, num_target_pose, num_orientation):
#     #     num_movement = 0
#     #     # shuffle_index_start = random.sample(range(self.len),num_start_pose)
#     #     # shuffle_index_target= random.sample(range(self.len),num_target_pose)
        
        
#     #     shuffle_index_start = self.start[int(job*(1000/total_worker)):int((job*(1000/total_worker))+num_start_pose)]

#     #     if ("su" in root) or ("felix" in root):
#     #         # print("felix")
#     #         shuffle_index_target= self.target[500:1000]
#     #     elif ("shi" in root) or ("haoyi" in root):
#     #         # print("haoyi")
#     #         shuffle_index_target= self.target[0:500]

#     #     # print("job:", job)
#     #     # print("total_worker:", total_worker)
#     #     # print("root:", root)
#     #     # print("num_start_pose:", num_start_pose)
#     #     # exit()

#     #     # shuffle_index_start = shuffle_index_start[0:2]
#     #     # shuffle_index_target = shuffle_index_target[0:2]
#     #     print(len(shuffle_index_start))
#     #     print(len(shuffle_index_target))
        
#     #     for i in shuffle_index_start:
#     #         position_start = self.workspace[i]
#     #         counter1 = 0
#     #         while counter1<num_orientation:
#     #             print("finding a new start pose.....")
#     #             roll_rad_start = random.uniform(0, np.pi)
#     #             pitch_rad_start = random.uniform(0, np.pi)
#     #             yaw_rad_start = random.uniform(0, np.pi)
#     #             plan_start = self.plan_end_effector_to_position(float(position_start[0]), float(position_start[1]), float(position_start[2]), roll_rad_start, pitch_rad_start, yaw_rad_start)
#     #             if (plan_start.joint_trajectory.header.frame_id == 'world'):
#     #                 print("Successfully Finding a plan for new start pose: #" + str(i)+" with orientation# "+str(counter1))
#     #                 print("Going to the new start pose")
#     #                 counter1=counter1+1
#     #                 # start_end_pose = []
#     #                 start_pose = [float(position_start[0]), float(position_start[1]), float(position_start[2]), roll_rad_start, pitch_rad_start, yaw_rad_start]
#     #                 print("Start pose: ")
#     #                 print(start_pose)
#     #                 # start_end_pose.append(start_pose)
#     #                 succeed = self.do_plan(plan_start)
#     #                 if succeed == True:
#     #                     print("Start pose is ready")
#     #                     for j in shuffle_index_target:
#     #                         position_target = self.workspace[j]
#     #                         counter2=0
#     #                         while counter2<num_orientation:
#     #                             roll_rad_end = random.uniform(0, np.pi)
#     #                             pitch_rad_end = random.uniform(0, np.pi)
#     #                             yaw_rad_end = random.uniform(0, np.pi)
#     #                             plan_end = self.plan_end_effector_to_position(float(position_target[0]), float(position_target[1]), float(position_target[2]), roll_rad_end, pitch_rad_end, yaw_rad_end)
#     #                             if (plan_end.joint_trajectory.header.frame_id == 'world'):
#     #                                 print("Successfully Finding a plan for target pose: #" + str(j)+" with orientation# "+str(counter2))
#     #                                 num_movement = num_movement +1
#     #                                 counter2 = counter2+1
#     #                                 end_pose = [float(position_target[0]), float(position_target[1]), float(position_target[2]), roll_rad_end, pitch_rad_end, yaw_rad_end]
#     #                                 print("end pose: ")
#     #                                 print(end_pose)
#     #                                 # start_end_pose.append(end_pose)
#     #                                 # pos = pd.DataFrame(start_end_pose)
#     #                                 # pos.to_csv(output_se_path + str(num_movement)+ "_se_pose.csv")
#     #                                 time_steps_raw, positions_raw, velocities_raw, accelerations_raw = parse_trajectory(plan_end)
                                    
#     #                                 positions_raw = np.array(self._to_degree(positions_raw))
#     #                                 # print(time_steps_raw.shape)
#     #                                 # print(positions_raw.shape)
#     #                                 # print(velocities_raw.shape)
#     #                                 # print(accelerations_raw.shape)
#     #                                 tmp = np.concatenate((time_steps_raw, positions_raw,velocities_raw, accelerations_raw), axis=1)
#     #                                 print(tmp.shape)
#     #                                 # _, trajectory, velocity, _ = smooth_trajectory(time_steps_raw, positions_raw,velocities_raw,  accelerations_raw, 40,7)
#     #                                 # trajectory_degree = self._to_degree(trajectory)
#     #                                 # trajectory_degree = np.array(trajectory_degree)
#     #                                 # combine = np.hstack((trajectory_degree,velocity))
#     #                                 pos = pd.DataFrame(tmp)
#     #                                 pos.to_csv(os.path.join(output_path ,f"traj_{str(num_movement)}.csv"))
                                    
#     #                             else:
#     #                                 print("Try again for position: " + str(j) + " with orientation #"+str(counter2))
#     #                                 counter2 = counter2
#     #             else:
#     #                 print("Retrying finding a plan for new start pose: #" + str(i))
#     #                 counter1 = counter1

#     # def read_start_end_pose(self, path):
#     #     file = pd.read_csv(path)
#     #     file = (file.to_numpy()[:,1:])
#     #     start = file[0]
#     #     end = file[1]
#     #     print(start)
#     #     print(end)
#     #     plan_start = self.plan_end_effector_to_position(float(start[0]), float(start[1]), float(start[2]), float(start[3]), float(start[4]), float(start[5]))
#     #     print(plan_start)
#     #     succeed = self.do_plan(plan_start)
#     #     if succeed ==True:
#     #         print("Readly to start")
#     #         plan_end = self.plan_end_effector_to_position(float(end[0]), float(end[1]), float(end[2]), float(end[3]), float(end[4]), float(end[5]))
#     #         succeed2 = self.do_plan(plan_end)
#     #         if succeed2 ==True:
#     #             print("done")
        
#     def _to_degree(self,traj):
#         # Change radious to degree
#         trajectory_degree = []
#         for i in range(len(traj)):
#             lines = traj[i]
#             degree_line = []
#             for j in range(len(lines)):
#                 degrees = math.degrees(lines[j])
#                 if degrees > 180:
#                     degrees -= 360
#                 elif degrees < -180:
#                     degrees += 360
#                 degree_line.append(degrees)
#             trajectory_degree.append(degree_line)
#         return trajectory_degree
            
#     # Not work, need to modify
#     def reset(self, duration):

#         self.arm_group.clear_pose_targets()
#         reset = rospy.ServiceProxy('lumi_mujoco/reset', Empty)
#         try:
#             reset()
#         except rospy.ServiceException as exc:
#             print("Reset did not work:" + str(exc))

#         rospy.sleep(duration)

# def test_generated_gazebo(self,filename):
#     file_list = []
#     print("running: ", 1)
#     with open(filename, mode ='r')as file:
#         csvFile = csv.reader(file)
        
#         for lines in csvFile:
#             lines.pop(0)
#             file_list.append(lines)
    
#     file_list.pop(0)
#     self.reach_joint_angles(0.01)
#     for i in range(len(file_list)):
#         self.set_joint_angles(0.01, file_list[i])

# if __name__ == '__main__':
#     import time
#     # rospy.init_node('atom_action')
#     # root = rospy.get_param('my_gen3_lite/root')# node_name/argsname
#     # print(root)
    
    
#     device = rospy.get_param('/my_gen3_lite/my_gen3_lite_driver/robot_name') # node_name/argsname
#     # job = int(rospy.get_param('my_gen3_lite/job_asign')) # node_name/argsname
#     # num_worker = int(rospy.get_param('my_gen3_lite/num_worker'))# node_name/argsname
#     # print(device)
#     time.sleep(3)
    
#     # OUTPUT_PATH = os.path.join(os.path.join(root,"data"),device)


#     simulation = SimulationInterface(robot_name = f"/{device}/")

    

#     msg = rospy.wait_for_message("/gazebo/model_states", ModelStates)
#     # msg = np.array(msg.pose[0])
#     # print(msg.pose[0].position.x)
#     # print("")
#     # print(len(msg.pose))
#     # print(msg.pose)
#     x = str(msg.pose[4].position.x)
#     y = str(msg.pose[4].position.y)
#     z = str(msg.pose[4].position.z)
#     # num_start_pose = int(1000/num_worker)
#     simulation.plan_end_effector_to_position(float(x), float(y), float(z), 0, 0, 0)
#     # simulation.rand_velocity_generate(OUTPUT_PATH, job, num_worker, root,num_start_pose, num_target_pose=1000, num_orientation=2)

#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

# Inspired from http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# Modified by Alexandre Vannobel to test the FollowJointTrajectory Action Server for the Kinova Gen3 robot

# To run this node in a given namespace with rosrun (for example 'my_gen3'), start a Kortex driver and then run :
# rosrun kortex_examples example_moveit_trajectories.py __ns:=my_gen3

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import tf
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelStates
import random
import numpy as np

# Joe Lisk code
#from chatGPTService import getTargetFromGPT
#from jointGPTPrompt import system_string, user_string
# import error: moduleNotFound

# from chatGPTService
import os
# import openai
import rospy

# openai.api_key = os.environ["OPENAI_API_KEY"]

# def getTargetFromGPT(system_string, user_string):
#     messages = [{"role": "system", "content": system_string}]
#     messages.append({"role": "user", "content": user_string})
#     completion = openai.ChatCompletion.create(
#     model="gpt-4",
#     messages=messages)
#     response = completion['choices'][0]['message']['content']
#     rospy.loginfo('Successfully got response from GPT-4!')
#     rospy.loginfo('Response from GPT-4:')
#     rospy.loginfo(response)
#     target = eval(response)
#     return target

# From jointGPTPrompt
system_string = """You are a Kinova gen3 arm that is tasked with determining which objects in the environment are trash, picking up the
            objects that are trash, and moving them to a specified point. You have 7 degrees of freedom and a gripper. It is possible
            that none of the objects are trash. You have a known starting cartesian pose called "home".

            Your output should be a string that represents the object that is trash (if any).

            Also, I'm going to execute your code in Python, so please format any steps you outline as a comment or the code will
            fail to run.
            """

# user_string = """There are two objects in the evnironment. beer_A: an empty can of beer at the position (x=0.7, y=0.0, z=0.0). And beer_B: an full, unopened can of beer at (x=0, y=-0.7, z=0).
#                Provide the variable name of the object that is trash. Also, I'm going to execute your code in Python, so please do not explain, just provide the variable name"""

#Joe Lisk code
#Helper function for joint angles
# def deg2rad(x):
#   return x * pi / 180

#Adapted from https://github.com/turtlebot/turtlebot/blob/kinetic/turtlebot_teleop/scripts/turtlebot_teleop_key
msg = """
Give your environment for your Kinova arm!
---------------------------
Describe your workspace and the objects present.

ChatGPT will then help control the robot.

Currently, you need to supply the poses

Of the objects in the form:

position.x = num, position.y = num, position.z = num, orientation.x = num, orientation.y = num, orientation.z = num, orientation.w = num

Grasp pose detection coming soon!

commands
---------------------------
enter : sends your input to ChatGPT

CTRL-C to quit
"""

def getUserString():
    str = input()
    return str

class ExampleMoveItTrajectories(object):
  """ExampleMoveItTrajectories"""
  def __init__(self):

    # Initialize the node
    super(ExampleMoveItTrajectories, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_it_service')

    try:
      self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
      if self.is_gripper_present:
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
      else:
        self.gripper_joint_name = ""
      self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("robot_description")
      self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
      self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

      if self.is_gripper_present:
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True


  def reach_named_position(self, target):
    arm_group = self.arm_group

    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
    # Execute the trajectory and block while it's not finished
    return arm_group.execute(trajectory_message, wait=True)

  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    # rospy.loginfo(pose.pose)

    return pose.pose

  def reach_cartesian_pose(self, target_pose, tolerance, constraints):
    arm_group = self.arm_group

    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Joe Lisk code
    # Just spotting ChatGPT the trash can pose
    # if target_obj == 'trash_can':
    # target_pose.position.x = -0.007253318946339758
    # target_pose.position.y = 0.4578600037286752
    # target_pose.position.z = 0.1964750840769468
    # target_pose.orientation.x = 0.08187678131496287
    # target_pose.orientation.y = 0.6676512236268767
    # target_pose.orientation.z = 0.7399520009621545
    # target_pose.orientation.w = 0.003012066257614919

    # Get the current Cartesian Position
    arm_group.set_pose_target(target_pose)

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    return arm_group.go(wait=True)

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
 
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()

    try:
      val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
      return val
    except:
      return False
    

def main():
  example = ExampleMoveItTrajectories()

  # For testing purposes
  success = example.is_init_success
  try:
      rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
  except:
      pass


  # comment out when using Gazebo sim. for real arm need to send home first
#   if success:
#     rospy.loginfo("Reaching Named Target Home...")
#     success &= example.reach_named_position("home")
#     print(success)
  
  # Joe Lisk code
  # get user input
#   print(msg)
#   gettingInput = True
#   while(gettingInput):
#     str = getUserString()
#     if str != '':
#       print("THIS IS YOUR INPUT:")
#       print(str)
#       user_string = str
#       gettingInput = False

  # Joe Lisk code
  # chatgpt service
#   gpt_list = getTargetFromGPT(system_string, user_string)

#   targetObj = gpt_list[0] # string
#   targetPreGraspPose = gpt_list[1] # dictionary
#   targetGraspPose = gpt_list[2] # dictionary

  # Pre-Grasp
  if success:
    rospy.loginfo("Reaching Cartesian Pose...")

    actual_pose = example.get_cartesian_pose()
    msg = rospy.wait_for_message("/gazebo/model_states", ModelStates)
    print(msg.pose[3].position)
    actual_pose.position.x = msg.pose[4].position.x
    actual_pose.position.y = msg.pose[4].position.y
    actual_pose.position.z = msg.pose[4].position.z
    # actual_pose.orientation.x = msg.pose[4].orientation.x
    # actual_pose.orientation.y = msg.pose[4].orientation.y
    # actual_pose.orientation.z = msg.pose[4].orientation.z
    # actual_pose.orientation.w = msg.pose[4].orientation.w
    roll_rad_start = 0
    pitch_rad_start = 1
    yaw_rad_start = 0
    quaternion = tf.transformations.quaternion_from_euler(
            roll_rad_start, pitch_rad_start, yaw_rad_start)
    actual_pose.orientation.x = quaternion[0]
    actual_pose.orientation.y = quaternion[1]
    actual_pose.orientation.z = quaternion[2]
    actual_pose.orientation.w = quaternion[3]
    
    # actual_pose.orientation.x = 0.0
    # actual_pose.orientation.y = 0.0
    # actual_pose.orientation.z = 0.0
    # actual_pose.orientation.w = 1.0

    success &= example.reach_cartesian_pose(target_pose=actual_pose, tolerance=10, constraints=None)
    print (success)

#   if success:
#     actual_pose = example.get_cartesian_pose()
#     rospy.loginfo("Cartesian Pose: PRE-GRASP")
#     rospy.loginfo(actual_pose)

  # Grasp
#   if success:
#     rospy.loginfo("Reaching Cartesian Pose...")

#     actual_pose = example.get_cartesian_pose()

#     actual_pose.position.x = targetGraspPose['position']['x']
#     actual_pose.position.y = targetGraspPose['position']['y']
#     actual_pose.position.z = targetGraspPose['position']['z']
#     actual_pose.orientation.x = targetGraspPose['orientation']['x']
#     actual_pose.orientation.y = targetGraspPose['orientation']['y']
#     actual_pose.orientation.z = targetGraspPose['orientation']['z']
#     actual_pose.orientation.w = targetGraspPose['orientation']['w']
  
#     success &= example.reach_cartesian_pose(targetObj, target_pose=actual_pose, tolerance=0.01, constraints=None)
#     print (success)

#   if success:
#     actual_pose = example.get_cartesian_pose()
#     rospy.loginfo("Cartesian Pose: GRASP")
#     rospy.loginfo(actual_pose)

#     rospy.loginfo("Closing the gripper 100%...")
#     success &= example.reach_gripper_position(0)
#     print (success)

#   if success:
#     rospy.loginfo("Reaching Cartesian Pose...")

#     success &= example.reach_cartesian_pose('trash_can', target_pose=actual_pose, tolerance=0.01, constraints=None)
#     print (success)

#   if success:
#     actual_pose = example.get_cartesian_pose()
#     rospy.loginfo("Cartesian Pose: TRASH CAN")
#     rospy.loginfo(actual_pose)

#   if example.is_gripper_present and success:
#     rospy.loginfo("Opening the gripper...")
#     success &= example.reach_gripper_position(1)
#     print (success)

#   if success:
#     rospy.loginfo("Reaching Named Target Home...")
#     success &= example.reach_named_position("home")
#     print (success)

  # comment out when using Gazebo sim and running real arm tests. this is just to guide the robot to a safe off position when done testing.
  # if success:
  #   rospy.loginfo("Reaching Target Off Position...")

  #   off_list = ["off_position"
  #     {"position": {"x": 0.002531766299089388, "y": -0.6028243179191973, "z": 0.02571023473658379},
  #       "orientation": {"x": 0.046498299219275584, "y": -0.7693984296865566, "z": 0.6370337922518455, "w": 0.0072050048444401256}}]

  #   targetGraspPose = off_list[1] # dictionary

  #   actual_pose = example.get_cartesian_pose()

  #   actual_pose.position.x = targetGraspPose['position']['x']
  #   actual_pose.position.y = targetGraspPose['position']['y']
  #   actual_pose.position.z = targetGraspPose['position']['z']
  #   actual_pose.orientation.x = targetGraspPose['orientation']['x']
  #   actual_pose.orientation.y = targetGraspPose['orientation']['y']
  #   actual_pose.orientation.z = targetGraspPose['orientation']['z']
  #   actual_pose.orientation.w = targetGraspPose['orientation']['w']
  
  #   success &= example.reach_cartesian_pose(targetObj, target_pose=actual_pose, tolerance=0.01, constraints=None)
  #   print(success)

  # For testing purposes
  rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

  if not success:
      rospy.logerr("The fk example encountered an error.")

if __name__ == '__main__':
  
  main()

# trash: beer_B
# user input: There are two objects in the evnironment. beer_A: A full, unopened can of beer with a pre-grasp pose of target_pose.position.x = 0.5338950844247246, target_pose.position.y = 0.013882795317536337, target_pose.position.z = 0.03560667199606036, target_pose.orientation.x = 0.5233042320042509, target_pose.orientation.y = 0.4906332994586916, target_pose.orientation.z = 0.47538816211295915, target_pose.orientation.w = 0.509350313194742 and a grasp pose of, target_pose.position.x = 0.7069753526986098, target_pose.position.y = 0.012251526063553923, target_pose.position.z = 0.030152609462021807, target_pose.orientation.x = 0.517385163838614, target_pose.orientation.y = 0.49491109344252465, target_pose.orientation.z = 0.4797642455445715, target_pose.orientation.w = 0.507150737477788. And beer_B: an empty can of beer with a pre-grasp pose of, target_pose.position.x = -0.008528227056494208, target_pose.position.y = -0.4545102051969075, target_pose.position.z = 0.031031940091151258, target_pose.orientation.x = 0.05375719323178181, target_pose.orientation.y = -0.7757844834263641, target_pose.orientation.z = 0.6283301669563721, target_pose.orientation.w = 0.021674887388595025 and a grasp pose of, target_pose.position.x = 0.002531766299089388, target_pose.position.y = -0.6028243179191973, target_pose.position.z = 0.02571023473658379, target_pose.orientation.x = 0.046498299219275584, target_pose.orientation.y = -0.7693984296865566, target_pose.orientation.z = 0.6370337922518455, target_pose.orientation.w = 0.0072050048444401256. Provide a Python list containing 3 elements: the variable name of the object that is trash, the pre-grasp pose of the object that is trash, the grasp pose of the object that is trash. Also, I'm going to execute your code in Python, so please do not explain or provide any additional text, just provide the list. You don't need to assign the list to a variable.

# trash: beer_A
# user input: There are two objects in the evnironment. beer_A: An empty can of beer with a pre-grasp pose of target_pose.position.x = 0.5338950844247246, target_pose.position.y = 0.013882795317536337, target_pose.position.z = 0.03560667199606036, target_pose.orientation.x = 0.5233042320042509, target_pose.orientation.y = 0.4906332994586916, target_pose.orientation.z = 0.47538816211295915, target_pose.orientation.w = 0.509350313194742 and a grasp pose of, target_pose.position.x = 0.7069753526986098, target_pose.position.y = 0.012251526063553923, target_pose.position.z = 0.030152609462021807, target_pose.orientation.x = 0.517385163838614, target_pose.orientation.y = 0.49491109344252465, target_pose.orientation.z = 0.4797642455445715, target_pose.orientation.w = 0.507150737477788. And beer_B: a full, unopened can of beer with a pre-grasp pose of, target_pose.position.x = -0.008528227056494208, target_pose.position.y = -0.4545102051969075, target_pose.position.z = 0.031031940091151258, target_pose.orientation.x = 0.05375719323178181, target_pose.orientation.y = -0.7757844834263641, target_pose.orientation.z = 0.6283301669563721, target_pose.orientation.w = 0.021674887388595025 and a grasp pose of, target_pose.position.x = 0.002531766299089388, target_pose.position.y = -0.6028243179191973, target_pose.position.z = 0.02571023473658379, target_pose.orientation.x = 0.046498299219275584, target_pose.orientation.y = -0.7693984296865566, target_pose.orientation.z = 0.6370337922518455, target_pose.orientation.w = 0.0072050048444401256. Provide a Python list containing 3 elements: the variable name of the object that is trash, the pre-grasp pose of the object that is trash, the grasp pose of the object that is trash. Also, I'm going to execute your code in Python, so please do not explain or provide any additional text, just provide the list. You don't need to assign the list to a variable.