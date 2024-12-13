#!/usr/bin/env python3

import requests
import json
from groq import Groq
import ast
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
import time

from faster_whisper import WhisperModel
import pyaudio
import wave
import keyboard
from pydub import AudioSegment
import pyaudio
# import keyboard
import time
import wave
from pynput import keyboard

import os
import sys
import wave
import subprocess
import requests
import json
import time
import pyaudio
# from groq import Groq
# Parameters for audio recording
FORMAT = pyaudio.paInt16  # 16-bit resolution
CHANNELS = 1  # Mono channel
RATE = 44100  # 44.1kHz sampling rate
CHUNK = 1024  # 1024 samples per frame
RECORD_SECONDS = 10  # Duration to record (change as needed)
WAVE_OUTPUT_FILENAME = "audio.wav"
MP3_OUTPUT_FILENAME = "audio.mp3"

model_size = "large-v3"

# def wait_for_space_to_start():
#     print("Press SPACE to start recording.")
#     def on_press(key):
#         print("what")
#         if key == keyboard.Key.space:
#             print("Recording started!")
#             return False  # Exit the listener
#     with keyboard.Listener(on_press=on_press) as listener:
#         print("w")
#         listener.join()
#     return False


def wait_for_space_to_start():
    print("Waiting to start recording. Press SPACE...")
    is_space_pressed = False

    # Function to check for key presses
    def on_press(key):
        nonlocal is_space_pressed
        if key == keyboard.Key.space:
            is_space_pressed = True
            print("Start recording detected!")

    # Create a listener
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    # Run the while loop to output messages
    while not is_space_pressed:
        print("...still waiting for SPACE to start recording...")
        # time.sleep(1)  # Delay to avoid flooding the terminal

    listener.stop()
    return True


def wait_for_space_to_stop():
    print("Press SPACE to stop recording.")
    def on_press(key):
        if key == keyboard.Key.space:
            print("Recording stopped!")
            return False  # Exit the listener
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()
    return False


def record_audio(file_name):
    chunk = 4096*100
    format = pyaudio.paInt16
    channels = 1
    rate = 48000
    Output_Filename = file_name+".wav"
    
    p = pyaudio.PyAudio()

    stream = p.open(format=format,
                    channels=channels,
                    rate=rate,
                    input=True,
                    frames_per_buffer=chunk)

    frames = []
    # print("Press SPACE to start recording")
    # keyboard.wait('space')
    # print("Recording... Press SPACE to stop.")
    # time.sleep(0.2)
    print(wait_for_space_to_start())
   

    print("Press SPACE to stop recording")
    is_space_pressed = False

    # Function to check for key presses
    def on_press(key):
        nonlocal is_space_pressed
        if key == keyboard.Key.space:
            is_space_pressed = True
            print("Recording stopped")

    # Create a listener
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    # Run the while loop to output messages
    while not is_space_pressed:
        try:
            print("listening")
            data = stream.read(chunk)  
            frames.append(data)
        except KeyboardInterrupt:
            break
        # data = stream.read(chunk)  
        # frames.append(data)
        # print("...still waiting for SPACE to start recording...")
        # time.sleep(1)  # Delay to avoid flooding the terminal

    listener.stop()
    



    # while True:
    #     try:
    #         print("listening")
    #         data = stream.read(chunk)  
    #         frames.append(data)
    #     except KeyboardInterrupt:
    #         break  

    stream.stop_stream()
    stream.close()
    p.terminate()

    wf = wave.open(Output_Filename, 'wb')  
    wf.setnchannels(channels)
    wf.setsampwidth(p.get_sample_size(format))
    wf.setframerate(rate)
    wf.writeframes(b''.join(frames))
    wf.close()
    return True

def wav_to_mp3(file_name):
    audio = AudioSegment.from_wav(file_name+".wav")

    # Export as MP3
    audio.export(file_name+".mp3", format="mp3")

def audio_to_text(file_name):
    # Run on GPU with FP16
    # model = WhisperModel(model_size, device="cuda", compute_type="float16")

    # or run on GPU with INT8
    # model = WhisperModel(model_size, device="cuda", compute_type="int8_float16")
    # or run on CPU with INT8
    model = WhisperModel(model_size, device="cpu", compute_type="int8")

    segments, info = model.transcribe(file_name+".mp3", beam_size=5)

    print("Detected language '%s' with probability %f" % (info.language, info.language_probability))

    for segment in segments:
        print("[%.2fs -> %.2fs] %s" % (segment.start, segment.end, segment.text))
    return segment.text




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
        # print(self.gripper_joint_name,gripper_joint,gripper_max_absolute_pos,gripper_min_absolute_pos)
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
                # print(target_position)
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
                # print(target_position)
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
                # print(target_position)
                stop = False
                while stop == False:
                    roll_rad_start = 3.14
                    pitch_rad_start = 0
                    yaw_rad_start = -np.pi
                    plan_start = self.plan_end_effector_to_position(float(target_position[0]), float(target_position[1]), float(target_position[2]+0.08), roll_rad_start, pitch_rad_start, yaw_rad_start)
                    if (plan_start.joint_trajectory.header.frame_id == 'world'):
                        succeed = self.do_plan(plan_start)
                        print("drop to "+ obj_name)
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
        # print(msg.name[i])
        # print("")
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


def generate_content_groq(selected_model, system_prompt, user_prompt):
    client = Groq(api_key="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")

    common_params = {
        "model": selected_model,
        "messages": [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt}
        ],
        "temperature": 1,
        "max_tokens": 1024,
        "top_p": 1,
        "stream": False,
        "stop": None,
    }

    if "llama3-groq" in selected_model:
        common_params["temperature"] = 0.5
        common_params["top_p"] = 0.65

    completion = client.chat.completions.create(**common_params)
    return completion

def generate_content_gemini(system_prompt, user_prompt):
    API_ENDPOINT = "https://generativelanguage.googleapis.com/v1beta/models/gemini-1.5-flash-latest:generateContent"
    API_KEY = "AIzaSyA0dIQzZAjEeXO8DzVriIusrvq7N2DB1O4"

    combined_prompt = f"{system_prompt}\nUser: {user_prompt}"

    payload = {
        "contents": [
            {
                "parts": [
                    {
                        "text": combined_prompt
                    }
                ]
            }
        ]
    }

    headers = {
        "Content-Type": "application/json"
    }

    # Send POST request to the API
    response = requests.post(
        f"{API_ENDPOINT}?key={API_KEY}",
        headers=headers,
        data=json.dumps(payload)
    )

    # Check if the request was successful
    if response.status_code == 200:
        return response.json()
    else:
        print("Request failed:", response.status_code, response.text)
        return None

def run_llms(model_number, speech):
    models = [
        "gemma-7b-it",
        "gemma2-9b-it",
        "llama3-groq-70b-8192-tool-use-preview",
        "llama3-groq-8b-8192-tool-use-preview",
        "mixtral-8x7b-32768",
        "llama-3.1-70b-versatile",
        "llama-3.1-8b-instant",
        "llama-3.2-11b-text-preview",
        "llama-3.2-1b-preview",
        "llama-3.2-3b-preview",
        "llama-3.3-70b-specdec",
        "llama-3.3-70b-versatile",
        "gemini-1.5-flash-latest"
    ]

    print("Available LLMs:")
    for idx, model in enumerate(models, 1):
        print(f"{idx}. {model}")

   
    # while True:
    #     try:
    #         selection = int(input("Please select the LLM to use (enter the number): "))
    #         if 1 <= selection <= len(models):
    #             selected_model = models[selection - 1]
    #             break
    #         else:
    #             print(f"Please enter a number between 1 and {len(models)}.")
    #     except ValueError:
    #         print("Invalid input. Please enter a number.")

    selected_model = models[model_number - 1]
    print(selected_model)

    # User prompt
    # user_prompt = input("Please enter your prompt: ")

    user_prompt = """You can perform the following actions:
                        go_to(target): Controls the end effector of the robot arm to move to the target object and position itself directly above the target, maintaining a vertical alignment.
                        pick_up(target): Opens the gripper, then controls the end effector of the robot arm to approach the target object vertically from above. Once the end effector reaches the target, it closes the gripper to securely grasp and lift the object.
                        drop_to(target): Assumes the gripper is already picked up an object using the pick_up(target) action. Controls the end effector of the robot arm to approach the target object or location vertically from above. Once positioned, the gripper opens to release the object, and the end effector retracts, leaving the object at the target location.
                        If I ask “Stack the blocks in the following order - blue_block at the bottom, red_block in the middle, and green_block at the top ”. Your response should be a list in the following format
                        [{'action': 'go_to', 'target': 'red_block'}, {'action': 'pick_up', 'target': 'red_block '}, {'action': 'go_to', 'target': 'blue_block'}, {'action': 'drop_to', target': 'blue_block '}, {'action': 'go_to', 'target': 'green_block'}, {'action': 'pick_up', 'target': 'green_block'}, {'action': 'go_to', 'target': 'red_block'}, {'action': 'drop_to', 'target': 'red_block'}]                 
                        
                        Remember, only give the action list as the example. I don't need any other suggestion or explanation.        
                        
                        """

    system_prompt = """ You are a robot arm action planner working with blocks in the environment.
    The environment includes three blocks as the objects: a red block labeled as number one, a green block labeled as number two, and a blue block labeled as number three. The environment also includes four locations: pyramid_base_left (one of the bottom layers of the pyramid), pyramid_base_right (one of the bottom layers of the pyramid), pyramid_top (the topmost layer of the pyramid, placed on top of other blocks), and target_center (a target location with no object initially on it).
    You can also consider the location of an object as the location of a target"""
    
    user_prompt = user_prompt + speech
    print(user_prompt)
    # exit()
    combined_prompt = f"{system_prompt}\nUser: {user_prompt}"

    if selected_model == "gemini-1.5-flash-latest":
        output = generate_content_gemini(system_prompt, user_prompt)
        if output:
            print("LLM Output:")
            #print(json.dumps(output, indent=2))
            try:
                action_text = output["candidates"][0]["content"]["parts"][0]["text"]
                action_json = json.loads(action_text.strip("```json\n").strip("```"))

                for action in action_json.get("actions", []):
                  print(action)
            except (KeyError, IndexError, json.JSONDecodeError) as e:
                print("Error parsing actions:", e)
        else:
            print("No output received from the Gemini API.")
    else:
        completion = generate_content_groq(selected_model, system_prompt, user_prompt)
        # print("\nLLM Output:")
        list_of_dicts = ast.literal_eval(completion.choices[0].message.content)
        # print(list_of_dicts)
        # print(type(list_of_dicts))
        return list_of_dicts
        # print(type(completion.choices[0].message.content))

# def generate_content_groq(selected_model, system_prompt, user_prompt):
#     client = Groq(api_key="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")

#     common_params = {
#         "model": selected_model,
#         "messages": [
#             {"role": "system", "content": system_prompt},
#             {"role": "user", "content": user_prompt}
#         ],
#         "temperature": 1,
#         "max_tokens": 1024,
#         "top_p": 1,
#         "stream": False,
#         "stop": None,
#     }

#     if "llama3-groq" in selected_model:
#         common_params["temperature"] = 0.5
#         common_params["top_p"] = 0.65

#     completion = client.chat.completions.create(**common_params)
#     return completion

# def generate_content_gemini(system_prompt, user_prompt):
#     API_ENDPOINT = "https://generativelanguage.googleapis.com/v1beta/models/gemini-1.5-flash-latest:generateContent"
#     API_KEY = "AIzaSyDmEeQf6U0xpnW-5-Taqv7lrBO3RNUQHiU"

#     combined_prompt = f"{system_prompt}\nUser: {user_prompt}"

#     payload = {
#         "contents": [
#             {
#                 "parts": [
#                     {
#                         "text": combined_prompt
#                     }
#                 ]
#             }
#         ]
#     }

#     headers = {
#         "Content-Type": "application/json"
#     }

#     # Send POST request to the API
#     response = requests.post(
#         f"{API_ENDPOINT}?key={API_KEY}",
#         headers=headers,
#         data=json.dumps(payload)
#     )

#     # Check if the request was successful
#     if response.status_code == 200:
#         # Parse JSON response
#         return response.json()
#     else:
#         print("Request failed:", response.status_code, response.text)
#         return None

if __name__ == "__main__":

    location_list = [{"name": "pyramid_top", "position": [-0.2,0.2,0.1]}, {"name": "pyramid_base_left", "position": [(-0.2)+0.0152,0.2,0.1]}, {"name": "pyramid_base_right", "position": [(-0.2)-0.0152,0.2,0.1]}, {"name": "target_center", "position": [-0.2,-0.2,0.1]}]
    

    # audio_file_name = "/home/felix/target_speech"
    # done = record_audio(audio_file_name)

    # while True:
    #     if done == True:
    #         wav_to_mp3(audio_file_name)
    #         speech = audio_to_text(audio_file_name)
    #         break

    # print(speech)
    speech = "stack all three cubes on top of each other"

    output= run_llms(2,speech)
    print("//////////////////////////////")
    print("LLM Output:")
    print(output)
    print("")
    print("//////////////////////////////")

    
    simulation = SimulationInterface(robot_name = f"/my_gen3/")

    for i in range(len(output)):
        object_list = update_object_state()
        if output[i]['target'] == "pyramid_top" or output[i]['target'] == "pyramid_base_left" or output[i]['target'] == "pyramid_base_right" or output[i]['target'] == "target_center":
            if output[i]['action'] == 'go_to':
                state = simulation.go_to_object(location_list, output[i]['target'])
            elif output[i]['action'] == 'pick_up':
                state = simulation.pick_up_object(location_list, output[i]['target'])
            elif output[i]['action'] == 'drop_to':
                state = simulation.drop_object(location_list, output[i]['target'])
        else:
            if output[i]['action'] == 'go_to':
                state = simulation.go_to_object(object_list, output[i]['target'])
            elif output[i]['action'] == 'pick_up':
                state = simulation.pick_up_object(object_list, output[i]['target'])
            elif output[i]['action'] == 'drop_to':
                state = simulation.drop_object(object_list, output[i]['target'])
        
        time.sleep(3)