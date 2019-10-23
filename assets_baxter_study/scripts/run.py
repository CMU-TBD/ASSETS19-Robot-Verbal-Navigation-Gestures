#!/usr/bin/env python2

import rospy
from success_polly_speech import PollySpeech
from success_ros_aruco import ArucoTagModule

from instruction_generator import InstructionGenerator
from map_interface import MapInterface

from instruction_movement_controllers import (
    BasicInstructionController,
    PalmMovementController
)

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import (
    Image
)

from success_baxter_tools.motion.set_posture import(
    move_to_posture,
    move_arm_to_pose
)

from geometry_msgs.msg import (
    Pose
)

from std_msgs.msg import (
    UInt32
)

import alloy
import os
import copy
import inflect

import numpy as np

from joystick_interface import BaxterJoy

from input_sys import (
    BaxterGripperPressureInputSystem,
    BaxterGripperContactInputSystem
)
from arml_audio_common.sound_maker import SoundMaker


class DirectionService():
    """ Provide Navigation Directions to Users
    """

    def __init__(self, mode, hand="left"):

        #Interaction Flags
        self._first_time_interacting_flag = True

        #difference python modules
        self._speak_ctr = PollySpeech()
        self._aruco_tag =  ArucoTagModule()
        self._sound_maker = SoundMaker()
        self._inflect_eng = inflect.engine()
        self._joy_ctr = BaxterJoy(limb=hand)
        rospy.loginfo("Python Interface Loaded")

        #control signal for when the experimenter want to move on
        self._exp_control_flag = False
        def exp_btn_cb():
            self._exp_control_flag = True
        self._joy_ctr.set_button_callback(0, exp_btn_cb)

        self._mode = mode #voice(default), palm and hand

        # Map Interface
        self._map = MapInterface(os.path.join(alloy.ros.get_res_path('assets_baxter_study'),'routes'))

        # publisher for image
        self._head_img_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        self._cv_bridge = CvBridge()
        self._publish_head_image_color([0,255,0])

        self._default_pose = Pose()
        if hand == 'left':
            self._default_pose.position.y = 0.2
        else:
            self._default_pose.position.y = -0.2
    
        self._default_pose.position.x = 0.7
        
        self._default_pose.orientation.y = 1
        self._default_pose.orientation.w = 0

        #parameters for repeating information
        self._repeat_flag = False
        self._repeat_count = 0
        def btn_cb2():
            self._sound_maker.play_beep()
            self._repeat_flag = True
        self._joy_ctr.set_button_callback(2,btn_cb2)

        #For voice and movement, the squeezing of the hand will be for repeating instructions
        if self._mode == 'voice':
            self._inputs = BaxterGripperPressureInputSystem(sound=True)
            self._inputs.set_repeat_callback(self._repeat_cb)
            self._ic = BasicInstructionController(hand)
            self._default_pose.position.z = 0.15
        elif self._mode == 'movement':
            self._inputs = BaxterGripperPressureInputSystem(sound=True)
            self._inputs.set_repeat_callback(self._repeat_cb)
            self._ic = BasicInstructionController(hand)
            self._default_pose.position.z = 0.15
        elif self._mode == 'palm':
            self._ic = PalmMovementController(hand)
            self._inputs = BaxterGripperContactInputSystem()
            self._default_pose.position.z = 0.4

            self._ir_range = 1000
            rospy.Subscriber('/robot/analog_io/left_hand_range/value_uint32',UInt32, self._ir_callback)

        #move to start point
        if self._mode == 'palm':
            move_to_posture("assets_start-palm")
        else:
            move_to_posture("assets_start")
        rospy.loginfo("STUDY: Study system initialization compeleted")

    def _publish_head_image_color(self, color=[0,0,0]):
        img_data = np.ones((600,1024,3),dtype=np.uint8)
        img_data[:] = color
        try:
            self._head_img_pub.publish(self._cv_bridge.cv2_to_imgmsg(img_data, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def _ir_callback(self, msg):
        self._ir_range = msg.data

    def _repeat_cb(self):
        """This function is called when there is a repeat request
        """
        self._repeat_flag = True

    def _repeat_instruction_handler(self, i, block=True):
        rospy.loginfo("STUDY:repeat executed for instruction {}".format(i))
        self._repeat_flag = False
        if self._repeat_count < 2:
            sentence = "I'll repeat the previous instruction, which was the {} instruction".format(self._inflect_eng.ordinal(i+1))
        else:
            sentence = "I'm repeating the {} instruction".format(self._inflect_eng.ordinal(i+1))
        self._speak_ctr.speak(sentence, block=block)
        self._repeat_count += 1

    def _wait_for_experimenter_signal(self):
        self._publish_head_image_color([0,0,255])
        self._exp_control_flag = False
        rospy.logdebug("STUDY: waiting for experimenter signal")
        while not self._exp_control_flag and not rospy.is_shutdown():
            rospy.sleep(0.25)
        self._exp_control_flag = False
        self._publish_head_image_color([0,255,0])

    def initial_mode_configuration(self):

        self._wait_for_experimenter_signal()

        #first go to the initial position
        self._ic.go_to_start(pose=self._default_pose)
        self._ic.start(initial_rotation=0)

        if self._mode == 'movement' or self._mode == 'voice':
            self._speak_ctr.speak("Now I will explain how to find my hand.")
            self._speak_ctr.speak("My hand is positioned in front of you. " + 
                "I am now twisting my hand. Please follow the gear sounds to find my wrist. " + 
                "Once you find my wrist, follow it downwards until you find the sponge material. " +
                "Then move your hand upwards until you find the slightly elevated button on the side of the cylinder. " +
                "Give it a press to tell me you found my hand.", block=False)
            rospy.sleep(1)
            self._ic.start_homing_action()
            self._inputs.wait_for_contact()
            self._ic.end_homing_action()
            self._speak_ctr.speak("Great, you have found my hand. Please hold onto my hand, loosely, so you can feel it without gripping it.",cancel=True)
            
            #EXPERIMENTER ASK USER IF THEY WANT TO CHANGE HEIGHT ETC...
            self._wait_for_experimenter_signal()

            #save the current joint configuration + pose
            self._ic.save_default_pose()
        
        if self._mode == 'palm':
            # #move to high scanner pose
            # move_to_posture("side_high")
            # move_to_posture("finger_upper")
            # #scan for aruco ID 0
            # self._aruco_tag.wait_for_id(0)
            # id_pose = self._aruco_tag.get_pose_for_id(0,frame_id='base')
            # #move to hand capture position
            # self._default_pose.position = id_pose.pose.position
            # self._default_pose.position.z += 0.1
            # self._default_pose.position.x += 0.1

            #save the current position which is the above as a position
            self._ic.save_default_pose()
        

    def execute(self, route_num):

        # Step 1 : Search and find all the navigation information
        semantic_obj = self._map.get_route(route_num) 
        self._ic.chart_space(semantic_obj)

        # Step 2 : Start Interaction before giving instructions
        rospy.loginfo("STUDY: Start Pre-Direction Interaction")
        self._pre_direction_interaction(semantic_obj)

        #WAIT FOR EXPERIMENTER TO BE READY + DOUBLE CHECK WITH PARTICIPANT
        self._wait_for_experimenter_signal()

        rospy.loginfo("STUDY: Starting Direction Giving")
        self._speak_ctr.speak("I am now giving you the directions to {}.".format(semantic_obj['destination_name']))
        while not rospy.is_shutdown():

            #enable repeats
            self._repeat_flag = False
            self._inputs.enable()

            # Step 3 : Give the instructions
            if self._mode  == "voice":
                """Speak out the instruction without any movements on the robot
                """
                self._voice_instruction_giving(semantic_obj)
            else:
                """Speak out the instruction with physical guidance
                """                
                #DEBUG:
                rospy.logdebug('STUDY:Real world position of each node relative to baxter')
                for s in semantic_obj['route']:
                    rospy.logdebug(self._ic.transform_map_to_movement_space(np.array([s['x'],s['y']])))
                #give instructions
                self._baxter_hand_instruction_giving(semantic_obj)
            rospy.loginfo("STUDY: Completed Step 3")

            # Step 4: Ask if the user want to repeat instructions

            text = 'press the button' if self._mode != "palm" else "curl your fingers"
            self._speak_ctr.speak("I can repeat the instructions from the beginning if you {} again".format(text))
            while not self._repeat_flag and not self._exp_control_flag and not rospy.is_shutdown():
                rospy.sleep(0.5)
            self._inputs.disable()
            # experimenter issue control to exit the program/loop
            if self._exp_control_flag:
                self._exp_control_flag = False
                break
            rospy.sleep(1)
            self._speak_ctr.speak("I am now repeating the instructions.")
            rospy.loginfo("STUDY: User request repeat")
        #end of execution
        rospy.loginfo("STUDY:End of Execution")
        self._first_time_interacting_flag = False

    def _pre_direction_interaction(self, semantic_obj):
        """ Step (2) - Interactions that is repeated for each trial, mainly to guide user to
        """

        if self._mode == 'movement' or self._mode == 'voice':
            if self._mode == 'movement':
                
                if self._first_time_interacting_flag:
                    #Warn the user you are moving the hand
                    self._speak_ctr.speak("I'm now moving my hands to a comfortable starting location", block=False)
                    rospy.sleep(0.5)
                self._ic.move_to_starting_position(semantic_obj['route'][0])
                self._ic.start(initial_rotation=0)
                self._speak_ctr.wait()
            else:
                self._ic.go_to_start()
                self._ic.start(initial_rotation=0)
            
            if not self._first_time_interacting_flag: #only skip this for the first time of voice or finger
                #guide the user's hand there
                self._speak_ctr.speak("Please lightly hold onto my hand positioned in front of you and press the button on it.")
                self._speak_ctr.speak("I'm now twisting my fingers to make it easier to find.", block=False)
                self._ic.start_homing_action()
                self._inputs.wait_for_contact()
                self._ic.end_homing_action()
                self._speak_ctr.speak("Great",cancel=True)

            #mention how to ask for repeat
            if self._first_time_interacting_flag:
                self._speak_ctr.speak("Througout the interaction, you can lightly press the button to have me repeat a previous instruction.")
            else:
                self._speak_ctr.speak("Just a reminder, you can lightly press the button to repeat a previous instruction.")



        elif self._mode == 'palm':
            #go to initial position which is directly above
            self._ic.go_to_start()
            self._ic.start(initial_rotation=0)

            #TODO some fancy code that might be able to recognize the ARTAGs

            #ask the participant to place hand on the pedestal in front of them
            self._speak_ctr.speak("Please stretch out your fingers to make your hand as flat as possible. "+ 
                "With your palm facing up, rest the back of your hand on the pedestal in front of you. " +
                "Keep your fingers stretched out as much as you can.")

            #WIZARD
            # Move hand to be directly above it before the robot automatically descend.
            self._wait_for_experimenter_signal()
            self._speak_ctr.speak("Now I am moving my hand down to touch your palm. ")

            #get the current pose from IC
            pose_to_go = copy.deepcopy(self._ic._default_pose)
            self._exp_control_flag = False
            while self._ir_range >= 250 and not self._exp_control_flag:
                pose_to_go.position.z -= 0.05 if self._ir_range >= 350 else 0.03
                self._ic.go_to_start(pose=pose_to_go)
            self._exp_control_flag = False

            #WIZARD:
            # Move the hand for final adjustments
            self._wait_for_experimenter_signal()

            #Run charting code to figure out exact locations for where to draw the path.
            self._ic.chart_palm_space()
            self._ic.move_to_starting_position(semantic_obj['route'][0])

            self._speak_ctr.speak("Througout the interaction, you can slightly lift your fingers to have me repeat a previous instruction.")
            self._speak_ctr.speak("Now please give it a try.", block=False)

            #WIZARD:
            # wizard make sure the person flex their fingers
            self._repeat_flag = False 
            while not self._repeat_flag and not rospy.is_shutdown():
                rospy.sleep(0.10)
            self._repeat_flag = False
            #self._wait_for_experimenter_signal()

            #Acknowledgement
            self._speak_ctr.speak("Great!",cancel=True)

    def _voice_instruction_giving(self, semantic_obj):

        starting_instruction = semantic_obj['starting_description']
        self._speak_ctr.speak(starting_instruction)

        route_instructions = semantic_obj['route']
        i = 0
        while i <len(route_instructions):
            if self._repeat_flag:
                i = i - 1 if i > 0 else 0 
                self._repeat_instruction_handler(i)
                continue 
            r = route_instructions[i]            
            if i < len(route_instructions)-1:
                #normal directions:
                dist_text, turn_text, cue_text = InstructionGenerator.generate_text(r)
                self._speak_ctr.speak(turn_text)
                if self._repeat_flag:
                    i = i - 1 if i > 0 else 0 
                    self._repeat_instruction_handler(i)
                    continue 
                self._speak_ctr.speak(dist_text)
                if self._repeat_flag:
                    i = i - 1 if i > 0 else 0 
                    self._repeat_instruction_handler(i)
                    continue 
                if cue_text is not None and cue_text != "":
                    self._speak_ctr.speak(cue_text)
                    if self._repeat_flag:
                        i = i - 1 if i > 0 else 0 
                        self._repeat_instruction_handler(i)
                        continue 
                # self._speak_ctr.speak(turn_text + " and " + dist_text + cue_text)
            else:
                t1, t2 = InstructionGenerator.generate_destination_text(r, semantic_obj['destination_name'])
                self._speak_ctr.speak(t1 + t2)
            if self._repeat_flag:
                #i = i - 1 if i > 0 else 0 
                self._repeat_instruction_handler(i)
                continue 
            rospy.sleep(1)
            i += 1

    def _baxter_hand_instruction_giving(self, semantic_obj):
       

        #speak initial voice
        starting_instruction = semantic_obj['starting_description']
        self._speak_ctr.speak(starting_instruction,block=False)
        self._ic.cue_action()
        self._speak_ctr.wait()

        route_instructions = semantic_obj['route']
       

        #now go through each semantic
        i = 0
        #save the initial state
        self._ic.save_curr_state(i) #0
        while i < len(route_instructions):
            #get the current chunk
            semantic_chunk = route_instructions[i]

            if i < (len(route_instructions) - 1):
                #generate the text
                dist_text, turn_text, cue_text = InstructionGenerator.generate_text(semantic_chunk)

                ## STEP 1 ----- TURN INSTRUCTION

                #speak and then move
                self._speak_ctr.speak(turn_text, block=False)
                self._ic.rotation_action(semantic_chunk['rot'])
                self._speak_ctr.wait() # wait for speech to finish

                #check if they want it to be repeated
                if self._repeat_flag:
                    #calculate the correct i
                    i = i-1 if i > 0 else 0
                    self._repeat_instruction_handler(i, block=False)
                    #move back the hand
                    self._ic.roll_back_action(i)
                    self._speak_ctr.wait()
                    continue                

                ## STEP 2 ------ MOVEMENT
                self._speak_ctr.speak(dist_text, block=False)
                #self._ic.movement_action(semantic_chunk['dist'], end_sensor=self._inputs._ps)
                self._ic.movement_action(route_instructions[i+1], end_sensor=self._inputs._ps)
                self._speak_ctr.wait()

                #check if they want it to be repeated
                if self._repeat_flag:
                    #calculate the correct i
                    i = i-1 if i > 0 else 0
                    self._repeat_instruction_handler(i, block=False)
                    #move back the hand
                    self._ic.roll_back_action(i)
                    self._speak_ctr.wait()
                    continue

                ## STEP 3 ------ CUE movement
                if cue_text != "":
                    self._speak_ctr.speak(cue_text, block=False)
                    self._ic.cue_action()
                    self._speak_ctr.wait()

                #check if they want it to be repeated
                if self._repeat_flag:
                    #calculate the correct i
                    i = i-1 if i > 0 else 0
                    self._repeat_instruction_handler(i, block=False)
                    #move back the hand
                    self._ic.roll_back_action(i)
                    self._speak_ctr.wait()
                    continue     
                
            else:
                #generaete the destination text, the second part is the 
                ft1, ft2 = InstructionGenerator.generate_destination_text(semantic_chunk,",{},".format(semantic_obj['destination_name']))
                self._speak_ctr.speak(ft1, block=True)
                #gesture the rotation and movement
                self._speak_ctr.speak(ft2, block=False)
                self._ic.rotation_action(semantic_chunk['rot'],force=True)
                self._speak_ctr.wait()

            if self._repeat_flag:
                #calculate the correct i
                i = i-1 if i > 0 else 0
                self._repeat_instruction_handler(i, block=False)
                #move back the hand
                self._ic.roll_back_action(i)
                self._speak_ctr.wait()
            else:
                self._ic.save_curr_state(i+1)
                i += 1

        #sleep for a while and roll back to 1st instruction
        rospy.sleep(1)
        self._speak_ctr.speak("I am now moving my hand back", block=False)
        rospy.sleep(1)
        self._ic.roll_back_action(0) #roll back to initial configuration
        self._speak_ctr.wait()


if __name__ == "__main__":
    #initialize node
    rospy.init_node("assets_study_node", log_level=rospy.DEBUG)
    #get parameters of this particular condition
    mode = rospy.get_param('~mode')
    side = rospy.get_param('~side')
    #validate the parameters 
    if mode != 'movement' and mode != 'palm' and mode != 'voice':
        rospy.logerr('given mode:{} is unknown.'.format(mode))
        raise ValueError()
    if side != 'left' and side != 'right':
        rospy.logerr('invalid arm:{}'.format(side))
        raise ValueError()

    rospy.loginfo("STUDY: Starting condition {} on arm {}".format(mode, side))
    #initialize study object
    ds = DirectionService(mode, side)
    # #configure the arm height and save it as the default
    ds.initial_mode_configuration()
    #execute trajectory
    while not rospy.is_shutdown():
        ds._publish_head_image_color([255,0,0])
        route_num_str = raw_input("Next Route Number:")
        ds._publish_head_image_color([0,255,0])

        try:
            route_num = int(route_num_str)
            if route_num < 1 or route_num > 9:
                rospy.logerr("Invalid Route Number:{}".format(route_num))
                continue
        except ValueError as err:
            rospy.logerr("Invalid Route Input:{}".format(route_num_str))
            continue
        rospy.loginfo("STUDY: starting executing route: {} with mode {}".format(route_num, mode))
        ds.execute(route_num)
