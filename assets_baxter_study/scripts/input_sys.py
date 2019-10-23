#!/usr/bin/env python2

import rospy
import os
import copy
import numpy as np

from baxter_end_force_sensor.sensor import PressureSensor
from arml_audio_common.sound_maker import SoundMaker

class BaxterInputSystem():

    def __init__(self,contact_limit=930,sound=False):
        self._sound_maker = SoundMaker()
        self._sound_flag = sound
        self._enable_flag = False 
        self._contact_limit = contact_limit
        self._initialization()

    def _initialization(self):
        raise NotImplementedError()

    def set_repeat_callback(self, cb):
        """
        This function is called when a repeat is requested
        """
        raise NotImplementedError()

    def enable(self):
        self._enable_flag = True

    def disable(self):
        self._enable_flag = False

class BaxterGripperPressureInputSystem(BaxterInputSystem):

    def _initialization(self):
        self._ps = PressureSensor(contact_limit=self._contact_limit)
        self._ps.set_state_change_callback(self._state_cb)
        self._repeat_cb = None
        self._waiting_flag = False
        #first contact flag
        self._first_contact_flag = False

    def _state_cb(self, msg):
        if msg == 'contact' and not self._first_contact_flag:
            if self._repeat_cb is not None and self._enable_flag:
                #make the sound and call the callback
                if self._sound_flag:
                    self._sound_maker.play_beep()
                self._first_contact_flag = True
                self._repeat_cb()
        elif msg == 'open':
            self._first_contact_flag = False
                
                

    def wait_for_contact(self, timeout=None):
        #TODO add timeout
        while not rospy.is_shutdown():
            if self._ps.current_state() == 'contact':
                break
            rospy.sleep(0.1)
        if self._sound_flag:
            self._sound_maker.play_beep()
        

    def set_repeat_callback(self, cb):
        self._repeat_cb = cb


class BaxterGripperContactInputSystem(BaxterInputSystem):

    def __init__(self):
        self._ps = PressureSensor()

    def get_current_state(self):
        return self._ps.current_state()
