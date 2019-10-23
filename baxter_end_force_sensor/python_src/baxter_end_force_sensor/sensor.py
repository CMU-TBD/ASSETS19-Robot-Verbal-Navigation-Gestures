import rospy

from std_msgs.msg import(
    Float32
)

import Queue
import threading

class PressureSensor(object):

    def __init__(self, contact_limit=930, ema_alpha=0.4):
        rospy.Subscriber('baxter_force_sensor_left',Float32, self._sensor_cb)

        self._limit_ranges = {
            'contact': contact_limit
        }

        #values for the Exponential Moving Average
        self._ema_alpha = ema_alpha
        self._ema_value = None
        self._curr_reading = None

        self._curr_local_state = 'open'
        self._state_change_cb = None
        self._last_state_change = rospy.Time.now()
        self._cb_thread = None

    def _sensor_cb(self, msg):

        #Step 1 -- Get the smoothed reading
        if self._ema_value is None:
            self._ema_value = msg.data
        else:
            self._ema_value = self._ema_alpha * msg.data + (1 - self._ema_alpha) * self._ema_value
        #assign the current reading
        self._curr_reading = self._ema_value
        # Step 2 -- Progress the state machine and see what state we are in
        #rospy.logdebug('latest_reading:{}, smoothed reading:{}'.format(msg.data, self._curr_reading))

        state_change = False
        if self._curr_local_state == 'open':
            if self._curr_reading > self._limit_ranges['contact']:
                self._curr_local_state = 'contact'
                state_change = True
        elif self._curr_local_state == 'contact':
            if self._curr_reading < self._limit_ranges['contact']:
                self._curr_local_state = 'open'
                state_change = True             

        if state_change:
            self._trigger_state_change_callback(self._curr_local_state)
            self._last_state_change = rospy.Time.now()
            

    def set_state_change_callback(self, cb):
        self._state_change_cb = cb

    def _trigger_state_change_callback(self, type_name):
        if self._state_change_cb != None:
            #rospy.loginfo("state change to {}".format(type_name))
            self._cb_thread = threading.Thread(target=self._state_change_cb, args=(type_name,))
            self._cb_thread.start()


    def current_reading(self):
        return self._curr_reading


    def current_state(self):
        return self._curr_local_state        


    def wait_for_contact(self, timeout=0):
        return self._wait_for_range('contact', timeout)


    def _wait_for_range(self, type, timeout):
        rater = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._curr_reading >= self._limit_ranges[type]:
                break
            rater.sleep()
        return True