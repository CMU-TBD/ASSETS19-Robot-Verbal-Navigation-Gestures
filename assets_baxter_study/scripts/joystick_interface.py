import rospy
from success_ros_aruco import ArucoTagModule
from success_baxter_tools.camera import CameraController
from success_baxter_tools.motion.set_posture import(
    move_to_posture,
    move_arm_to_pose
)

from geometry_msgs.msg import(
    Pose
)

from sensor_msgs.msg import(
    Joy
)

import baxter_interface
from success_baxter_tools.motion.ik_solver import (
    solve_IK,
    # solve_IK_PseudoInverse
)
from baxter_core_msgs.msg import (
    EndpointState
)
import threading

from std_msgs.msg import (
    Empty
)

class BaxterJoy():

    def __init__(self, limb='left'):

        self._exp_pose = Pose()
        self._exp_pose.orientation.w = 0
        self._exp_pose.orientation.y = 1
        self._exp_pose.position.x = 0.7
        self._exp_pose.position.y = 0.2

        self._pose_lock = threading.RLock()
        self._last_command_time = rospy.Time.now()

        self._limb_name = limb
        self._limb = baxter_interface.limb.Limb(self._limb_name)
        #self._command_robot(self._exp_pose, force=True)

        self._cb_dict = {}

        rospy.Subscriber('joy',Joy, self._joy_cb, queue_size=1)
        rospy.Subscriber('/robot/limb/' + self._limb_name + '/endpoint_state',EndpointState,self._endpoint_cb, queue_size=1)
        self._estop_pub = rospy.Publisher('/robot/set_super_stop', Empty, queue_size=2)

    def _endpoint_cb(self, msg):
        with self._pose_lock:
            #just copy the position
            if (rospy.Time.now() - self._last_command_time).to_sec() > 0.5:
                self._exp_pose.position = msg.pose.position

    def set_button_callback(self, button_num, cb):
        
        if button_num not in self._cb_dict:
            self._cb_dict[button_num] = []
             
        self._cb_dict[button_num].append(cb)
        

    def _joy_cb(self, msg):

        for i,b in enumerate(msg.buttons):
            if b and i in self._cb_dict:
                for j in self._cb_dict[i]:
                    runthread = threading.Thread(target=j)
                    runthread.setDaemon(True)
                    runthread.start()

        with self._pose_lock:
            #print(msg)
            change_x = msg.axes[1] * 0.01 * -1
            change_y = msg.axes[0] * 0.01 * -1
            change_z = msg.axes[3] * 0.01

            change_x += msg.axes[5] * 0.01 * -1
            change_y += msg.axes[4] * 0.01 * -1

            if msg.buttons[3]:
                change_z += 0.005
            elif msg.buttons[1]:
                change_z -= 0.005
            elif msg.buttons[7] or msg.buttons[6]:
                #kill the robot
                rospy.logwarn("KILLING ROBOT")
                self._estop_pub.publish()

            self._exp_pose.position.x += change_x
            self._exp_pose.position.y += change_y
            self._exp_pose.position.z += change_z

            self._command_robot(self._exp_pose)
            self._last_command_time = rospy.Time.now()
            

    def _command_robot(self, pose, force=False):

            joint_state = solve_IK(self._limb_name,pose)
            if joint_state is None:
                rospy.logerror("UNABLE TO SOLVE IK")
                return None
            joint_angles = dict(zip(joint_state.name,joint_state.position))
            if not force:
                self._limb.set_joint_positions(joint_angles)
            else:
                self._limb.move_to_joint_positions(joint_angles)



def main():
    rospy.init_node("joystick_test")

    bj = BaxterJoy()
    rospy.spin()

if __name__ == "__main__":
    main()