#! /usr/bin/env python
'''
ROS Node - UR5 2 - Belt to Bin
'''

import sys
import math
import ast
import rospy

import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from hrwros_gazebo.msg import LogicalCameraImage
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import vacuumGripper
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import tf2_msgs.msg

from pkg_task5.msg import msgDictColours

from pkg_ros_iot_bridge.msg import msgMqttSub
from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotGoal
from pkg_ros_iot_bridge.msg import msgRosIotResult
from pkg_ros_iot_bridge.msg import msgRosIotFeedback

import cv2

class Ur52BeltToBin(object):
    '''Class to pick up the box and drop it in the bin'''
    # pylint: disable=too-many-instance-attributes
    # Twenty Eight is reasonable in this case.
    def __init__(self):
        '''Constructor'''

        rospy.init_node('node_ur5_2', anonymous=True)

        self._topic = '/ros_iot_bridge/mqtt/sub'
        self._inventory = []
        self._count = 0

        self._topic_dict_send = '/ur5_2_to_ur5_1/dict_colours'

        self._handle_ros_pub = rospy.Publisher(self._topic_dict_send,\
         msgDictColours, queue_size=10)

        self._ac = actionlib.ActionClient('/action_ros_iot', msgRosIotAction)

        self._bridge = CvBridge()
        self._image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image, self.callback)
        self._colours_dict = dict()

        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer)

        self._planning_group = "manipulator"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot_ns = '/'  + "ur5_2"

        self._robot = moveit_commander.RobotCommander(\
            robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group,\
            robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher(self._robot_ns +\
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._execute_trajectory_client = actionlib.SimpleActionClient(\
            self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._execute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        self.ur5_2_home_pose = [math.radians(11.2918523883),\
                        math.radians(-140.47291322),\
                        math.radians(-58.0557206423),\
                        math.radians(-71.4713544),\
                        math.radians(89.9999812372),\
                        math.radians(11.2918523729)]

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')

        #Fininshing the program
        self.picked = {
            "packagen00": False,\
            "packagen01": False,\
            "packagen02": False,\
            "packagen10": False,\
            "packagen11": False,\
            "packagen12": False,\
            "packagen20": False,\
            "packagen21": False,\
            "packagen22": False,\
            "packagen30": False,\
            "packagen31": False,\
            "packagen32": False}

        self.done = False
        self.red = False
        self.green = False
        self.yellow = False

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

    def get_dominant_colour(self, arg_img):
        '''
        Function to get the most dominant colour in the input image
        '''
        # pylint: disable-msg=no-else-return
        blue = arg_img[:, :, :1]
        green = arg_img[:, :, 1:2]
        red = arg_img[:, :, 2:]

        b_mean = np.mean(blue)
        g_mean = np.mean(green)
        r_mean = np.mean(red)

        if (b_mean > g_mean and b_mean > r_mean):
            return "Blue"
        if (g_mean > r_mean and g_mean > b_mean):
            return "Green"
        if (r_mean > g_mean and r_mean > b_mean):
            return "Red"
        else:
            return "Yellow"

    def unsubscribe(self):
        '''
        Function to unsubscribe from Camera_1 after Inventory Stock Taking
        '''
        self._image_sub.unregister()

    def callback(self, data):
        '''
        Camera_1 callback function which takes care of image processing with
        OpenCV and send the messages to ROSIOT bridge to update the spreadsheet
        '''
        # pylint: disable-msg=too-many-locals

        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as err:
            rospy.logerr(err)

        (_, _, _) = cv_image.shape

        image = cv_image

        # Resize a 720x1280 image to 360x640 to fit it on the screen
        resized_image = cv2.resize(image, (720/2, 1280/2))

        for i in range(0, 4):
            for j in range(0, 3):
                x_1 = (45 + (j * 90))
                x_2 = (130 + (j * 90))
                y_1 = (150 + (i * 80))
                y_2 = (220 + (i * 80))

                region_of_interest = resized_image[y_1:y_2, x_1:x_2]
                colour_detected = self.get_dominant_colour(region_of_interest)

                self._colours_dict['packagen'+str(i)+str(j)] = colour_detected

        self.unsubscribe()

        self.send_colours_to_ur5_1(str(self._colours_dict))

        goal = msgRosIotGoal()
        goal.protocol = 'mqtt'
        goal.mode = 'ss0'
        goal.topic = "eyrc/RrSsVvPp/ros_to_iot/task5"
        goal.message = str(self._colours_dict)

        self._ac.send_goal(goal, self.on_transition, None)

        cv2.waitKey(3)

    def priority_callback(self, msg):
        '''Callback function for ROS Topic'''
        self._inventory.append(ast.literal_eval(msg.message))
        rospy.loginfo(self._inventory)

    def on_transition(self, goal_handle):
        '''Function called when there is a change of state in the Action Client'''
        result = msgRosIotResult()

        if goal_handle.get_comm_state() == 2:
            rospy.loginfo(": Goal just went active.")

        if goal_handle.get_comm_state() == 7:
            rospy.loginfo(": Goal is DONE")
            rospy.loginfo(goal_handle.get_terminal_state())
            result = goal_handle.get_result()
            rospy.loginfo(result.flag_success)

            if result.flag_success:
                rospy.loginfo("Goal successfully completed.")
            else:
                rospy.loginfo("Goal failed.")

    def send_colours_to_ur5_1(self, message):
        '''Function to send the colours dictionary to the other node using ROS Topic'''
        msg_dict_colours = msgDictColours()
        msg_dict_colours.timestamp = rospy.Time.now()
        msg_dict_colours.message = message

        self._handle_ros_pub.publish(msg_dict_colours)

    def set_joint_angles(self, arg_list_joint_angles):
        '''Function to set joint angles of robot arm'''
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if flag_plan:
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def service_activate_deactivate(self, activate_vacuum_gripper):
        '''Function to call rosservice'''
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
        try:
            ser = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
            res = ser(activate_vacuum_gripper)
            return res.result
        except rospy.ServiceException as s_error:
            rospy.loginfo("Service call failed: %s"%s_error)

    def service_start_stop(self, conv):
        '''Function to call rosservice'''
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        try:
            ser = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
            res = ser(conv)
            return res.result
        except rospy.ServiceException as s_error:
            rospy.loginfo("Service call failed: %s"%s_error)

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):
        '''
        Function to hard set joint angles for the UR5 arm to avoid failed attempts
        '''
        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and  (flag_success is False)):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts))

    def pick_place_start(self, joint_angle_bin, joint_angle_home):
        '''
        Pick box place in bin and restart convyer
        '''
        # Pick it up
        pickup = False
        trials = 0
        rospy.loginfo("IN PICK IT AND PLACE")
        while (not pickup) and (trials < 100):
            pickup = self.service_activate_deactivate(True)

        # Move to bin

        rospy.loginfo("BEFORE JOINT ANGLES")
        self.hard_set_joint_angles(joint_angle_bin, 10)

        # Restart conveyor belt
        self.service_start_stop(100)

        # Drop in bin
        dropdown = True
        trials = 0
        while dropdown and (trials < 100):
            dropdown = self.service_activate_deactivate(False)

        goal = msgRosIotGoal()
        goal.protocol = 'mqtt'
        goal.mode = 'ss3'
        goal.topic = "eyrc/RrSsVvPp/ros_to_iot/task5"
        goal.message = str(self._inventory[self._count])

        self._ac.send_goal(goal, self.on_transition, None)

        self._count += 1

        #Go to home pose
        self.hard_set_joint_angles(joint_angle_home, 10)

    def camera_callback(self, msg):
        '''
        Callback function for /eyrc/vb/logical_camera_2 topic
        '''

        number_models = len(msg.models)

        for i in range(0, number_models):
            name_model = msg.models[i].type
            if name_model != 'ur5':
                #rospy.loginfo(self._colours_dict[name_model])
                #rospy.loginfo(name_model)
                if (self._colours_dict[name_model] == 'Red'\
                    and (not self.picked[name_model]) and (msg.models[i].pose.position.y <= 0)):
                    self.service_start_stop(0)
                    self.red = True
                    self.picked[name_model] = True
                elif (self._colours_dict[name_model] == 'Yellow'\
                    and (not self.picked[name_model]) and (msg.models[i].pose.position.y <= 0)):
                    self.service_start_stop(0)
                    self.yellow = True
                    self.picked[name_model] = True
                elif (self._colours_dict[name_model] == 'Green'\
                    and (not self.picked[name_model]) and (msg.models[i].pose.position.y <= 0)):
                    self.service_start_stop(0)
                    self.green = True
                    self.picked[name_model] = True

    def __del__(self):
        '''
        Destructor
        '''
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')


def main():
    '''Main Function'''
    rospy.sleep(30)
    ur52 = Ur52BeltToBin()
    rospy.Subscriber('/ur5_1_to_ur5_2/inventory', msgDictColours, ur52.priority_callback)

    ur52.set_joint_angles(ur52.ur5_2_home_pose)
    rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, ur52.camera_callback)

    lst_joint_anglesred = [math.radians(-90),\
                        math.radians(-112),\
                        math.radians(-85),\
                        math.radians(-71),\
                        math.radians(88),\
                        math.radians(0)]
    lst_joint_anglesyellow = [math.radians(-9),\
                        math.radians(0),\
                        math.radians(0),\
                        math.radians(0),\
                        math.radians(0),\
                        math.radians(0)]
    lst_joint_anglesgreen = [math.radians(90),\
                        math.radians(-112),\
                        math.radians(-85),\
                        math.radians(-71),\
                        math.radians(88),\
                        math.radians(0)]

    while not ur52.done:
        if ur52.red:
            ur52.pick_place_start(lst_joint_anglesred, ur52.ur5_2_home_pose)
            ur52.red = False
        elif ur52.yellow:
            ur52.pick_place_start(lst_joint_anglesyellow, ur52.ur5_2_home_pose)
            ur52.yellow = False
        elif ur52.green:
            ur52.pick_place_start(lst_joint_anglesgreen, ur52.ur5_2_home_pose)
            ur52.green = False

    ur52.service_start_stop(0)
    rospy.spin()

    cv2.destroyAllWindows()

    del ur52

    rospy.spin()

if __name__ == '__main__':
    main()
