#!/usr/bin/env python

import baxter_interface
import numpy as np
import rospy
from geometry_msgs.msg import(
    PoseStamped,
    Pose
)

from baxter_core_msgs.msg import (
    EndpointState
)
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)
import alloy
from success_baxter_tools.motion.ik_solver import (
    solve_IK,
    # solve_IK_PseudoInverse
)
from success_baxter_tools.motion.set_posture import(
    move_to_posture,
    move_arm_to_pose
)
import copy
import threading

class BaseInstructionController(object):
    """Interface for the functionalities an instruction
    Controller should have
    """

    def __init__(self, limb_name):

        #class members
        self._endpose = None
        self._prev_state_dict = dict()
        self._curr_rot = 0 #a placeholder for current rotation in 2D space
        self._dir_rot = 0 
        self._w2_offset = 0
        
        self._movement_space_max_point = np.array([0.7,0.5])
        self._movement_space_min_point = np.array([0.5,0.3])

        #map information
        self._movement_space_width = -1
        self._movement_space_height = -1

        self._limb = baxter_interface.Limb(limb_name)
        self._limb_name = limb_name
        self._last_joint_position = self._limb.joint_angles()

        self._base_joint_position = None 
        self._default_joint_angles = None
        self._default_pose = None

        #save a copy of the endpoint
        rospy.Subscriber('/robot/limb/' + limb_name + '/endpoint_state',EndpointState,self._endpoint_cb, queue_size=1)
        while self._endpose == None:
            rospy.sleep(0.05)
        #run each controllers init code 

        self.init()

    def save_default_pose(self):
        self._default_joint_angles = self._limb.joint_angles()
        #default pose, make sure the direction is correct
        self._default_pose = Pose()
        self._default_pose.orientation.y = 1
        self._default_pose.orientation.w = 0
        self._default_pose.position = self._endpose.pose.position



    def start(self):
        pass

    def stop(self):
        pass

    def setup_prepare(self, speaker):
        pass

    def _distance_scaler(self, dist):
        return np.log10(dist) * 0.05

    def _endpoint_cb(self, msg):
        self._endpose = PoseStamped()
        self._endpose.header = msg.header
        self._endpose.header.frame_id = 'base'
        self._endpose.pose = msg.pose        
        

    def save_curr_state(self, i):
        self._prev_state_dict[i] = {
            "pose": copy.deepcopy(self._endpose),
            "joint_angles": copy.deepcopy(self._last_joint_position),
            "curr_rot": copy.deepcopy(self._curr_rot),
            "dir_rot": copy.deepcopy(self._dir_rot)
        }

    def go_to_start(self, joint_angles=None, pose=None):
        """
        Go to this initial starting position. The outer function
        can change this to maximize movements.
        """
        if joint_angles is not None:
            joint_angles = dict(zip(self._limb.joint_names(), joint_angles))
            self._limb.move_to_joint_positions(joint_angles)
        elif pose is not None:
            print(self._limb_name)
            move_arm_to_pose(self._limb_name, pose)
            #wait for a second
            rospy.sleep(0.5)
            joint_angles = self._limb.joint_angles()
            self._base_joint_position = joint_angles
            self._w2_offset = joint_angles['{}_w2'.format(self._limb_name)]
            rospy.loginfo("W2 offset: {}".format(self._w2_offset))
        elif self._default_joint_angles is not None:
            self._limb.move_to_joint_positions(self._default_joint_angles)
        else:
            # move_to_posture('claw')
            # joint_angles = self._limb.joint_angles()
            # joint_angles['left_w2'] = 0
            self._limb.move_to_joint_positions(self._base_joint_position)

    def chart_space(self, semantic_list,):
        """
        This code goes through the whole semantic_list and figure out the 
        """
        #get the movement space info
        self._movement_space_x = self._movement_space_max_point[0] - self._movement_space_min_point[0]
        self._movement_space_y = self._movement_space_max_point[1] - self._movement_space_min_point[1]

        min_x = 1000
        max_x = -1000
        min_y = 1000
        max_y = -1000

        min_x_node_index = -1
        max_x_node_index = -1

        min_y_node_index = -1
        max_y_node_index = -1

        #now we go through the whole list and figure it out
        for i, chunk in enumerate(semantic_list['route']):
            if chunk['x'] > max_x:
                max_x = chunk['x']
                max_x_node_index = i
            if chunk['x'] < min_x:
                min_x = chunk['x']
                min_x_node_index = i
            if chunk['y'] > max_y:
                max_y = chunk['y']
                max_y_node_index = i
            if chunk['y'] < min_y:
                min_y = chunk['y']
                min_y_node_index = i

        #width 
        self._map_width = max_x - min_x
        self._map_height = max_y - min_y

        #the scaling point
        #flipped x and y because of the mirroring
        x_scale = self._movement_space_y/self._map_width
        y_scale = self._movement_space_x/self._map_height

        #find the smallest scale point
        self._map_linear_scale = x_scale if x_scale < y_scale else y_scale

        #the minimum point of the map
        self._map_min_pt = np.array([min_x, min_y])
        self._map_max_pt = np.array([max_x, max_y])

        self._map_min_pt_true = self._map_min_pt * self._map_linear_scale

        true_min = np.array([self._movement_space_max_point[0], self._movement_space_min_point[1]])
        self._map_zero_in_baxter_space = np.array([self._movement_space_max_point[0] + self._map_min_pt_true[1], self._movement_space_min_point[1] - self._map_min_pt_true[0]])
        #self._map_zero_in_baxter_space = true_min + np.flipud(self._map_min_pt_true)
        self._trans_mat = np.array([[0,-1,self._map_zero_in_baxter_space[0]],[1,0,self._map_zero_in_baxter_space[1]],[0,0,1]])



    def transform_map_to_movement_space(self, pos):
        # print(self._map_linear_scale)
        # print(self._map_min_pt)
        # print(self._map_max_pt)
        # print(pos)

        #true min point relative to the map
        pos = pos * self._map_linear_scale
        parr = np.array([pos[0],pos[1],1])
        #arr = (pos - self._map_min_pt)*self._map_linear_scale + true_min
        # rot_mat = np.array([[0,-1],[1,0]])
        arr = np.matmul(self._trans_mat,parr)
        #arr = arr * rot_mat
        return arr[0:2]

    def move_to_starting_position(self, chunk):
        pos = np.array([chunk['x'], chunk['y']])
        pos = self.transform_map_to_movement_space(pos)
        #print(pos)
        
        goto_pose = copy.deepcopy(self._default_pose)
        goto_pose.position.z = self._endpose.pose.position.z

        goto_pose.position.z += 0.1
        print(goto_pose)
        move_arm_to_pose(self._limb_name, goto_pose)

        goto_pose.position.x = pos[0]
        goto_pose.position.y = pos[1]
        goto_pose.position.z -= 0.1
        print(goto_pose)
        move_arm_to_pose(self._limb_name, goto_pose)
        #now we need to copy the w2_off_set
        joint_angles = self._limb.joint_angles()
        self._w2_offset = joint_angles['{}_w2'.format(self._limb_name)]
        rospy.loginfo("new W2 offset: {}".format(self._w2_offset))



    def init(self, **kwargs):
        pass

    def rotation_action(self, rad, **kwargs):
        """ Present rotation physical information.

        parameters:
        -----------
        rad: float
            Radian in how much the user should rotate their body

        """
        raise NotImplementedError()

    def cue_action(self):
        """ Some action that signify some kind of cue or important information
        """
        pass 


    def movement_action(self, dist):
        """ Present movement information. The distance is actual distance in feet
        parameters:
        dist: float
            In feet, how much should the user move
        """
        raise NotImplementedError()

    def roll_back_action(self, i):
        """Roll back the previous instructions
        """
        if i in self._prev_state_dict:
            pose_to_go = self._prev_state_dict[i]["pose"]
            self._curr_rot = self._prev_state_dict[i]['curr_rot']
            self._dir_rot = self._prev_state_dict[i]['dir_rot']
            self._last_joint_position = self._prev_state_dict[i]['joint_angles']
            
            slight_above_pose = copy.deepcopy(pose_to_go.pose)
            slight_above_pose.position.z += 0.05
            move_arm_to_pose(self._limb_name, slight_above_pose)
            self._limb.move_to_joint_positions(self._last_joint_position)
        else:
            rospy.logwarn("Unknown Error, trying to request state {}".format(i))
        #raise NotImplementedError()

    def solveIK(self, stamped_pose):
        #use the pseudo inverse first
        joint_angles = None#solve_IK_PseudoInverse('left', stamped_pose.pose)
        if joint_angles == None:
            joint_state = solve_IK(self._limb_name,stamped_pose)
            if joint_state is None:
                rospy.logerror("UNABLE TO SOLVE IK")
                #joint_state = solve_IK('left',self._endpose)
            #zip joint state to something baxter_interface likes
            joint_angles = dict(zip(joint_state.name,joint_state.position))
        return joint_angles


class DummyInstructionController(BaseInstructionController):

    def start(self, initial_rotation=0):
        self._curr_rot = 0

    def rotation_action(self, rad, **kwargs):
        rospy.sleep(1.5) #wait one second to make it fair
        pass

    def movement_action(self, dist):
        rospy.sleep(1)
        pass


class PalmMovementController(BaseInstructionController):

    def start(self, initial_rotation=0):
        self._curr_rot = 0
        self._dir_rot = 0
        self._limb.set_joint_position_speed(0.4)

    def rotation_action(self, rad, **kwargs):

        #we are assuming the rotation direction faces the robot (meaning at w2 = 0 position)
        #we want the rotation to be relative to the user as it is facing the robot, so it's the reverse
        self._curr_rot -= rad #it should be positive if it's from the robot's perspective

        #update the directional rotation
        self._dir_rot -= rad 
        self._dir_rot = alloy.math.clip_radian_rotation(self._dir_rot) 

        #clip the rotation
        if np.abs(self._curr_rot) > np.pi:
            self._curr_rot = alloy.math.clip_radian_rotation(self._curr_rot)
        self._rotate_to_curr_rot()

    def chart_palm_space(self,size=(0.06,0.04)):
        
        #get current pose
        curr_pose = self._endpose.pose
        palm_bottom_x = curr_pose.position.x
        palm_bottom_y_participant_right_corner = curr_pose.position.y
        #get the minimum corner
        max_x_corner = palm_bottom_x
        min_y_corner = palm_bottom_y_participant_right_corner - size[1]
        #debug information
        rospy.logdebug("chart_palm: gripper|x:{},y:{}".format(palm_bottom_x, palm_bottom_y_participant_right_corner))
        rospy.loginfo("corner:{} {}".format(max_x_corner, min_y_corner))

        #TODO simplify code
        #the scaling point
        #flipped x and y because of the mirroring
        x_scale = size[1]/self._map_width
        y_scale = size[0]/self._map_height

        #find the smallest scale point
        self._map_linear_scale = x_scale if x_scale < y_scale else y_scale

        #the minimum point of the map update the position
        self._map_min_pt_true = self._map_min_pt * self._map_linear_scale

        true_min = np.array([max_x_corner, min_y_corner])
        self._map_zero_in_baxter_space = np.array([max_x_corner + self._map_min_pt_true[1], min_y_corner - self._map_min_pt_true[0]])
        #self._map_zero_in_baxter_space = true_min + np.flipud(self._map_min_pt_true)
        self._trans_mat = np.array([[0,-1,self._map_zero_in_baxter_space[0]],[1,0,self._map_zero_in_baxter_space[1]],[0,0,1]])


    def init(self):
        self._movement_space_max_point = np.array([0.65,0.35])
        self._movement_space_min_point = np.array([0.6,0.3])

    def cue_action(self):
        loop_times = 2
        loop_rad = 0.2
        pause_time = 0.25

        init_rot = self._curr_rot
        for i in range(0, loop_times):
            self._curr_rot += loop_rad
            self._rotate_to_curr_rot(move_type='set')
            rospy.sleep(pause_time)
            self._curr_rot -= loop_rad
            self._rotate_to_curr_rot(move_type='set')
            rospy.sleep(pause_time)
        rospy.sleep(pause_time)
        self._curr_rot = init_rot
        self._rotate_to_curr_rot()
         
    def _rotate_to_curr_rot(self, move_type='block'):
        #now map the rotation to the w2 space and include the limits
        joint_angles = self._limb.joint_angles()
        val = joint_angles['{}_w2'.format(self._limb_name)]
        val = self._curr_rot + self._w2_offset
        val = np.min([3.059, val])
        val = np.max([-3.059, val])
        joint_angles['{}_w2'.format(self._limb_name)] = val
        self._last_joint_position = joint_angles
        #move to that joint position
        if move_type == 'block':
            self._limb.move_to_joint_positions(joint_angles)
        else:
            self._limb.set_joint_positions(joint_angles)

    def set_pose(self, desire_pose):

        joint_angles = self.solveIK(new_pose)
        self._limb.set_joint_positions(joint_angles, raw=True)


    def movement_action(self, next_chunk, end_sensor=None):

        goal_pos = self.transform_map_to_movement_space(np.array([next_chunk['x'], next_chunk['y']]))

        start_x = self._endpose.pose.position.x
        start_y = self._endpose.pose.position.y

        x_val = goal_pos[0] - start_x
        y_val = goal_pos[1] - start_y

        new_pose = copy.deepcopy(self._endpose)
        new_pose.pose.position.x = goal_pos[0] 
        new_pose.pose.position.y = goal_pos[1] 


        # hz = 10

        # rate = rospy.Rate(hz)
        # total = hz * 2
        # for i in range(0,total):
        #     new_pose.pose.position.x = start_x + x_val*((i+1.0)/total)
        #     new_pose.pose.position.y = start_y + y_val*((i+1.0)/total)
        #     new_pose.pose.position.z = self._endpose.pose.position.z
        #     # if i%(hz/2) == 0:
        #     #     if end_sensor.current_reading() > 950:
        #     #         new_pose.pose.position.z += 0.02
        #     #     elif end_sensor.current_reading() < 920:
        #     #         new_pose.pose.position.z -= 0.02
        #     #solve the IK
        #     joint_angles = self.solveIK(new_pose)
        #     #save the joint angles 
        #     self._last_joint_position = joint_angles
        
        #     #print(end_sensor.current_reading())
        #     #execute the movements
        #     self._limb.set_joint_positions(joint_angles, raw=True)
        #     rate.sleep()

        joint_angles = self.solveIK(new_pose)
        #save the joint angles 
        self._last_joint_position = joint_angles        
        self._limb.move_to_joint_positions(self._last_joint_position)
        rospy.sleep(1) #wait for one second for this to set in.

        #self._limb.set_joint_positions(joint_angles, raw=True)

class RotationOnlyInstructionController(BaseInstructionController):

    def init(self, **kwargs):
        self._homing_loop_thread = None
        self._homing_running_flag = False

    def start(self, initial_rotation=0):
        if np.abs(initial_rotation) > np.pi:
            self._curr_rot = alloy.math.clip_radian_rotation(initial_rotation)
        else:
            self._curr_rot = initial_rotation
        
        if self._curr_rot != 0:
            #rotate to that position
            self._rotate_to_curr_rot()
        self._dir_rot = 0
        self._limb.set_joint_position_speed(0.4)


    def rotation_action(self, rad, **kwargs):

        #we are assuming the rotation direction faces the robot (meaning at w2 = 0 position)
        #we want the rotation to be relative to the user as it is facing the robot, so it's the reverse
        self._curr_rot -= rad #it should be positive if it's from the robot's perspective

        #update the directional rotation
        self._dir_rot -= rad 
        self._dir_rot = alloy.math.clip_radian_rotation(self._dir_rot) 

        #clip the rotation
        if np.abs(self._curr_rot) > np.pi:
            self._curr_rot = alloy.math.clip_radian_rotation(self._curr_rot)
        self._rotate_to_curr_rot()

    def cue_action(self):
        #
        loop_times = 2
        loop_rad = 0.2
        pause_time = 0.25

        init_rot = self._curr_rot
        for i in range(0, loop_times):
            self._curr_rot += loop_rad
            self._rotate_to_curr_rot(move_type='set')
            rospy.sleep(pause_time)
            self._curr_rot -= loop_rad
            self._rotate_to_curr_rot(move_type='set')
            rospy.sleep(pause_time)
        rospy.sleep(pause_time)
        self._curr_rot = init_rot
        self._rotate_to_curr_rot()

    def _homing_loop(self):
        

        loop_rad = 0.1
        pulse_duration = 0.25
        pulse_width = 2

        init_rot = 0
        #init_rot = self._curr_rot
        while self._homing_running_flag and not rospy.is_shutdown():
            self._curr_rot += loop_rad
            self._rotate_to_curr_rot(move_type='set')
            rospy.sleep(pulse_duration)
            self._curr_rot = init_rot
            self._rotate_to_curr_rot(move_type='set')
            rospy.sleep(pulse_width)

        if not rospy.is_shutdown():
            self._curr_rot = init_rot
            self._rotate_to_curr_rot()        

    def start_homing_action(self):

        self._homing_running_flag = True
        self._homing_loop_thread = threading.Thread(target=self._homing_loop)
        self._homing_loop_thread.start()

    def end_homing_action(self):
        
        if self._homing_running_flag:
            self._homing_running_flag = False
            self._homing_loop_thread.join()

    def _rotate_to_curr_rot(self, move_type='block'):
        #now map the rotation to the w2 space and include the limits
        joint_angles = self._limb.joint_angles()
        val = joint_angles['{}_w2'.format(self._limb_name)]
        val = self._curr_rot + self._w2_offset
        val = np.min([3.059, val])
        val = np.max([-3.059, val])
        joint_angles['{}_w2'.format(self._limb_name)] = val
        self._last_joint_position = joint_angles
        #move to that joint position
        if move_type == 'block':
            self._limb.move_to_joint_positions(joint_angles)
        else:
            self._limb.set_joint_positions(joint_angles)


    def movement_action(self, dist):
        rospy.sleep(1)
        pass


class BasicInstructionController(RotationOnlyInstructionController):
    
    def movement_action_OLD(self, dist, end_sensor):

        #log scale the dist
        #log_dist = self._distance_scaler(dist)
        log_dist = dist * self._map_linear_scale

        #get the direction vector 
        x_val = np.cos(self._dir_rot) 
        x_val = -1 * x_val #because we have a flipped x, we mirror x
        y_val = np.sin(self._dir_rot)
        y_val = y_val
        #get current endpoint
        stamped_pose = self._endpose
        stamped_pose.pose.position.x += (x_val * log_dist)
        stamped_pose.pose.position.y += (y_val * log_dist)
        #IK it
        joint_angles = self.solveIK(stamped_pose)
        #save the joint angles 
        self._last_joint_position = joint_angles
        #execute the movements
        self._limb.move_to_joint_positions(joint_angles)

    def movement_action(self, next_chunk, end_sensor):

        pos = self.transform_map_to_movement_space(np.array([next_chunk['x'], next_chunk['y']]))

        #get current endpoint
        stamped_pose = self._endpose
        stamped_pose.pose.position.x = pos[0]
        stamped_pose.pose.position.y = pos[1]
        #IK it
        joint_angles = self.solveIK(stamped_pose)
        #save the joint angles 
        self._last_joint_position = joint_angles
        #execute the movements
        self._limb.move_to_joint_positions(joint_angles)