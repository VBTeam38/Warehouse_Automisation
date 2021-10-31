#! /usr/bin/env python
'''
ROS Node - UR5 1 - Shelf to Belt
'''

import sys
import math
import ast
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import rospkg

import yaml

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import conveyorBeltPowerMsg

from pkg_task5.msg import msgDictColours

from pkg_ros_iot_bridge.msg import msgMqttSub
from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotGoal
from pkg_ros_iot_bridge.msg import msgRosIotResult

# pylint: disable-msg=protected-access
# pylint: disable-msg=too-many-nested-blocks
class Ur51ShelfToBelt(object):
    '''Class to pick up the box and drop it on the belt'''

    def __init__(self):
        '''Constructor'''

        rospy.init_node('node_ur5_1', anonymous=True)

        self._topic = '/ros_iot_bridge/mqtt/sub'

        self._topic_inventory_send = '/ur5_1_to_ur5_2/inventory'

        self._handle_ros_pub = rospy.Publisher(self._topic_inventory_send,\
         msgDictColours, queue_size=10)

        self._colours_dict = dict()
        self._colours = rospy.Subscriber('/ur5_2_to_ur5_1/dict_colours',\
         msgDictColours, self.got_the_colours)

        self._ac = actionlib.ActionClient('/action_ros_iot', msgRosIotAction)

        self._inventory = []
        self._num_boxes_picked = 0
        self._len_inventory = 0

        self._planning_group = "manipulator"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot_ns = '/'  + 'ur5_1'

        self._robot = moveit_commander.RobotCommander(\
        robot_description=self._robot_ns+"/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group,\
         robot_description=self._robot_ns+"/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher(\
        self._robot_ns+'/move_group/display_planned_path',\
         moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._execute_trajectory_client = actionlib.SimpleActionClient(\
        self._robot_ns+'/execute_trajectory',\
         moveit_msgs.msg.ExecuteTrajectoryAction)
        self._execute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        self._picked = {
            "packagen00": False,
            "packagen01": False,
            "packagen02": False,
            "packagen10": False,
            "packagen11": False,
            "packagen12": False,
            "packagen20": False,
            "packagen21": False,
            "packagen22": False,
            "packagen30": False,
            "packagen31": False,
            "packagen32": True}

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        ros_pkg = rospkg.RosPack()
        self._pkg_path = ros_pkg.get_path('pkg_task5')
        self._file_path = self._pkg_path+'/config/saved_trajectories/'
        rospy.loginfo("Package Path: {}".format(self._file_path))

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

        self._ac = actionlib.ActionClient('/action_ros_iot', msgRosIotAction)
        self._ac.wait_for_server()

    def send_inventory_to_ur5_2(self, message):
        '''Function to send the colours dictionary to the other node using ROS Topic'''
        msg_dict_colours = msgDictColours()
        msg_dict_colours.timestamp = rospy.Time.now()
        msg_dict_colours.message = message

        self._handle_ros_pub.publish(msg_dict_colours)

    def priority_callback(self, msg):
        '''Callback function for ROS Topic'''
        self._inventory.append(ast.literal_eval(msg.message))
        rospy.loginfo(self._inventory)
        self._len_inventory += 1

        goal = msgRosIotGoal()
        goal.protocol = 'mqtt'
        goal.mode = 'ss1'
        goal.topic = "eyrc/RrSsVvPp/ros_to_iot/task5"
        goal.message = str(msg.message)

        self._ac.send_goal(goal, self.on_transition, None)

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

    def got_the_colours(self, msg):
        '''Callback function for '/ur5_2_to_ur5_1/dict_colours' topic '''
        self._colours_dict = ast.literal_eval(msg.message)
        rospy.loginfo(self._colours_dict)

    def service_activate_deactivate(self, activate_vacuum_gripper):
        '''Function to call rosservice'''
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        try:
            ser = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
            res = ser(activate_vacuum_gripper)
            return res.result
        except rospy.ServiceException as s_error:
            rospy.loginfo("Service call failed: %s"%s_error)

    def service_start_stop(self, conv):
        '''Function to call rosservice'''
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        ser = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        ser(conv)

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        '''Function to play planned paths'''
        file_path = arg_file_path + arg_file_name

        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open, Loader=yaml.Loader)

        ret = self._group.execute(loaded_plan)
        return ret

    def moveit_hard_play_planned_path_from_file(\
    self, arg_file_path, arg_file_name, arg_max_attempts):
        '''Function to play planned paths with multiple attempts'''
        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and (flag_success is False)):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts))

        return True

    def pick_it_and_put_it(self, pos_file, drop_file, box_name):
        '''Function to pick package from shelf and put on conveyor belt'''
        rospy.loginfo('Going to pick up ' + box_name)

        self.moveit_hard_play_planned_path_from_file(self._file_path, pos_file, 20)

        success_pick = self.service_activate_deactivate(True)
        if success_pick:
            rospy.loginfo('PICKED UP OBJECT')
        else:
            rospy.loginfo("PICK UP DID NOT WORK")

        rospy.loginfo('Going to drop location')

        self.moveit_hard_play_planned_path_from_file(self._file_path, drop_file, 20)

        success_drop = self.service_activate_deactivate(False)
        if not success_drop:
            rospy.loginfo('DROPPED OBJECT')
        else:
            rospy.loginfo("DROP DID NOT WORK")

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

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):
        '''Function to set joint angles in multiple tries'''
        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and (flag_success is False)):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts))

def main():
    '''Main function'''
    ur51 = Ur51ShelfToBelt()
    rospy.Subscriber(ur51._topic, msgMqttSub, ur51.priority_callback)
    ur51.service_start_stop(100)

    lst_joint_angles_drop = [math.radians(0),
                             math.radians(-133),
                             math.radians(-60),
                             math.radians(-77),
                             math.radians(90),
                             math.radians(0)]

    lst_joint_angles_all_zeroes = [math.radians(0),
                                   math.radians(0),
                                   math.radians(0),
                                   math.radians(0),
                                   math.radians(0),
                                   math.radians(0)]

    ur51.hard_set_joint_angles(lst_joint_angles_drop, 20)


    while not rospy.is_shutdown():
        if ur51._len_inventory > ur51._num_boxes_picked:

            ur51._inventory = sorted(ur51._inventory, key=lambda i: i['item'], reverse=True)
            curr_box_info = ur51._inventory[0]
            rospy.logwarn(curr_box_info)
            curr_box_item = curr_box_info['item']

            if curr_box_item == 'Medicine':
                for box, colour in ur51._colours_dict.iteritems():
                    if colour == 'Red' and not ur51._picked[box]:
                        ur51._num_boxes_picked += 1

                        if box == 'packagen00':
                            ur51.hard_set_joint_angles(lst_joint_angles_all_zeroes, 20)
                        ur51._picked[box] = True

                        drop_pose_to_box = "dropPose_to_" + str(box) + '_trial_smallest'+ ".yaml"
                        box_to_drop_pose = str(box) + "_to_dropPose_trial_smallest.yaml"

                        ur51.pick_it_and_put_it(drop_pose_to_box,\
                         box_to_drop_pose, 'Box_' + str(ur51._num_boxes_picked))

                        goal = msgRosIotGoal()
                        goal.protocol = 'mqtt'
                        goal.mode = 'ss2'
                        goal.topic = "eyrc/RrSsVvPp/ros_to_iot/task5"
                        goal.message = str(ur51._inventory[0])
                        ur51._ac.send_goal(goal, ur51.on_transition, None)

                        ur51.send_inventory_to_ur5_2(str(ur51._inventory[0]))

                        ur51._inventory.pop(0)
                        break

            elif curr_box_item == 'Clothes':
                for box, colour in ur51._colours_dict.iteritems():
                    if colour == 'Green' and not ur51._picked[box]:
                        ur51._num_boxes_picked += 1

                        if box == 'packagen00':
                            ur51.hard_set_joint_angles(lst_joint_angles_all_zeroes, 20)

                        ur51._picked[box] = True

                        drop_pose_to_box = "dropPose_to_" + str(box) + '_trial_smallest'+ ".yaml"
                        box_to_drop_pose = str(box) + "_to_dropPose_trial_smallest.yaml"

                        ur51.pick_it_and_put_it(drop_pose_to_box,\
                         box_to_drop_pose, 'Box_' + str(ur51._num_boxes_picked))

                        goal = msgRosIotGoal()
                        goal.protocol = 'mqtt'
                        goal.mode = 'ss2'
                        goal.topic = "eyrc/RrSsVvPp/ros_to_iot/task5"
                        goal.message = str(ur51._inventory[0])

                        ur51._ac.send_goal(goal, ur51.on_transition, None)

                        ur51.send_inventory_to_ur5_2(str(ur51._inventory[0]))

                        ur51._inventory.pop(0)
                        break

            else:
                for box, colour in ur51._colours_dict.iteritems():
                    if colour == 'Yellow' and not ur51._picked[box]:
                        ur51._num_boxes_picked += 1

                        if box == 'packagen00':
                            ur51.hard_set_joint_angles(lst_joint_angles_all_zeroes, 20)

                        ur51._picked[box] = True

                        drop_pose_to_box = "dropPose_to_" + str(box) + '_trial_smallest'+ ".yaml"
                        box_to_drop_pose = str(box) + "_to_dropPose_trial_smallest.yaml"

                        ur51.pick_it_and_put_it(drop_pose_to_box,\
                         box_to_drop_pose, 'Box_' + str(ur51._num_boxes_picked))

                        goal = msgRosIotGoal()
                        goal.protocol = 'mqtt'
                        goal.mode = 'ss2'
                        goal.topic = "eyrc/RrSsVvPp/ros_to_iot/task5"
                        goal.message = str(ur51._inventory[0])

                        ur51._ac.send_goal(goal, ur51.on_transition, None)

                        ur51.send_inventory_to_ur5_2(str(ur51._inventory[0]))

                        ur51._inventory.pop(0)
                        break
        rospy.sleep(2)

    rospy.spin()

if __name__ == '__main__':
    main()
