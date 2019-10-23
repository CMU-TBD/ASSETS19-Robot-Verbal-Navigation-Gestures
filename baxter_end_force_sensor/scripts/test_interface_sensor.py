#!/usr/bin/python2

import rospy
from baxter_end_force_sensor.sensor import PressureSensor


def cb_func(msg):
    print(msg)

def main():
    rospy.init_node('test_interface', log_level=rospy.DEBUG)
    ps = PressureSensor()
    #ps.set_state_callback(cb_func)
    rospy.spin()
    # print(ps.current_reading())
    # ps.wait_for_contact()
    # print("CONTACT")
    # print(ps.current_reading())



if __name__ == '__main__':
    main()