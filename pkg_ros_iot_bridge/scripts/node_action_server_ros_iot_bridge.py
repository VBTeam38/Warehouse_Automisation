#!/usr/bin/env python
'''
ROS Node - Action Server - IoT ROS Bridge
'''

import time
import datetime
import ast
import threading
import requests
import rospy
import actionlib

from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotGoal
from pkg_ros_iot_bridge.msg import msgRosIotResult
from pkg_ros_iot_bridge.msg import msgRosIotFeedback

# Message class for MQTT subscription messages
from pkg_ros_iot_bridge.msg import msgMqttSub

# Custom python module to perform MQTT tasks
from pyiot import iot

class IotRosBridgeActionServer(object):
    '''Class RosIotBridgeActionServer'''
    # pylint: disable=too-many-instance-attributes
    # Ten is reasonable in this case.
    def __init__(self):
        '''Constructor for the class'''
        self._as = actionlib.ActionServer('/action_ros_iot',\
         msgRosIotAction, self.on_goal, self.on_cancel, auto_start=False)

        param_config_iot = rospy.get_param('config_pyiot')
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_topic_sub = param_config_iot['mqtt']['topic_sub']
        self._config_mqtt_topic_pub = param_config_iot['mqtt']['topic_pub']
        self._config_mqtt_qos = param_config_iot['mqtt']['qos']
        self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']
        self._config_google_apps_sheet_id_1 = param_config_iot['google_apps']['spread_sheet_id_1']
        self._config_google_apps_sheet_id_2 = param_config_iot['google_apps']['spread_sheet_id_2']

        self._handle_ros_pub = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic,\
         msgMqttSub, queue_size=10)

        ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback, self._config_mqtt_server_url,\
         self._config_mqtt_server_port, self._config_mqtt_topic_sub, self._config_mqtt_qos)

        if ret == 0:
            rospy.loginfo('MQTT Subscribe Thread Started')
        else:
            rospy.loginfo('Failed to start MQTT Subscribe Thread')

        self._as.start() # Start the action server
        rospy.loginfo('Started Ros-Iot Bridge Action Server')

    def mqtt_sub_callback(self, client, userdata, message):
        '''Callback function for mqtt subscription'''
        payload = str(message.payload.decode('utf-8'))

        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.message = payload

        rospy.loginfo(payload)

        self._handle_ros_pub.publish(msg_mqtt_sub)

    # This function will be called when Action Server receives a Goal
    def on_goal(self, goal_handle):
        '''This function will be called when Action Server receives a Goal'''
        goal = goal_handle.get_goal()
        rospy.loginfo('Received new goal from client')
        rospy.loginfo(goal)

        # Validate incoming goal parameters
        if goal.topic == 'eyrc/RrSsVvPp/ros_to_iot/task5':
            goal_handle.set_accepted()

            thread = threading.Thread(name='worker', target=self.process_goal,\
                args=(goal_handle,))
            thread.start()

        else:
            goal_handle.set_rejected()
            return

    # This function is called is a separate thread to process Goal.
    def process_goal(self, goal_handle):
        '''This function is called is a separate thread to process Goal'''
        msgRosIotResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo('Processing goal: %s', str(goal_id.id))

        goal = goal_handle.get_goal()

        # Goal Processing
        rospy.logwarn('MQTT PUB Goal ID: %s', str(goal_id.id))
        rospy.logwarn('%s > %s', str(goal.topic), str(goal.message))
        message = ast.literal_eval(goal.message)

        self.data_to_sheet(goal.mode, message)

        rospy.loginfo('Goal ID: %s Goal Processing Done!', str(goal_id.id))

    # This function will be called when Goal Cancel request is send to the Action Server
    def on_cancel(self, goal_handle):
        '''Called when Goal Cancel request is send to the Action Server'''
        rospy.loginfo('Received cancel request.')
        goal_handle.get_goal_id()

    def data_to_sheet(self, ss_num, msg):
        '''Publishes result to spreadsheet'''

        if ss_num == 'ss0':
            for package, colour in msg.iteritems():
                sku = ''
                if colour == 'Red':
                    sku += 'R'
                    item = "Medicines"
                    pri = "HP"
                    cost = "450"
                elif colour == 'Green':
                    sku += 'G'
                    item = "Clothes"
                    pri = "LP"
                    cost = "150"
                else:
                    sku += 'Y'
                    item = "Food"
                    pri = "MP"
                    cost = "300"

                sku += package[-2:] + str(datetime.datetime.now().strftime("%m"))\
                    + str(time.strftime("%y", time.localtime()))
                storage_no = 'R' + str(package[-2]) + ' C' + str(package[-1])

                parameters = {"id":"Inventory", "Team Id":'VB#0038',\
                "Unique Id":"RrSsVvPp", "SKU":sku, "Item":item,\
                    "Priority":pri, "Storage Number":storage_no, "Cost":cost,\
                        "Quantity":"1"}

                url = "https://script.google.com/macros/s/"\
                    + str(self._config_google_apps_sheet_id_1) + "/exec"
                # url_eyantra = "https://script.google.com/macros/s/"\
                #     + str(self._config_google_apps_sheet_id_2) + "/exec"
                # response_eyantra = requests.get(url_eyantra, params=parameters)
                response = requests.get(url, params=parameters)
                print response.content
                # print response_eyantra.content

        elif ss_num == 'ss1':
            if msg['item'] == 'Medicine':
                pri = "HP"
                cost = "450"
            elif msg['item'] == 'Clothes':
                pri = "LP"
                cost = "150"
            else:
                pri = "MP"
                cost = "300"

            parameters1 = {"id":"IncomingOrders", "Team Id":'VB#0038',\
                "Unique Id":"RrSsVvPp", "Order ID":msg['order_id'],\
                    "Order Date and Time":str(msg['order_time']),\
                        "Item":msg['item'], "Priority":pri, "Order Quantity":msg['qty'],\
                            "City":msg['city'], "Longitude":msg['lon'],\
                                "Latitude":msg['lat'],\
                                    "Cost":cost}

            parameters2 = {"id":"Dashboard", "Team Id":'VB#0038',\
                "Unique Id":"RrSsVvPp", "Order ID":msg['order_id'],\
                    "Item":msg['item'], "Priority":pri, "Quantity":msg['qty'],\
                        "City":msg['city'], "Longitude":msg['lon'],\
                            "Latitude":msg['lat'], "Order Dispatched":"NO", \
                                "Order Shipped":"NO", "Order Time":str(msg['order_time'])}

            url = "https://script.google.com/macros/s/"\
                + str(self._config_google_apps_sheet_id_1) + "/exec"
            # url_eyantra = "https://script.google.com/macros/s/"\
            #     + str(self._config_google_apps_sheet_id_2) + "/exec"
            # response_eyantra = requests.get(url_eyantra, params=parameters1)
            response1 = requests.get(url, params=parameters1)
            response2 = requests.get(url, params=parameters2)
            print response1.content
            print response2.content
            # print response_eyantra.content

        elif ss_num == 'ss2':
            if msg['item'] == 'Medicine':
                pri = "HP"
                cost = "450"
            elif msg['item'] == 'Clothes':
                pri = "LP"
                cost = "150"
            else:
                pri = "MP"
                cost = "300"

            curr_time = datetime.datetime.now()

            parameters1 = {"id":"OrdersDispatched", "Team Id":'VB#0038',\
                "Unique Id":"RrSsVvPp", "Order ID":msg['order_id'], "City":msg['city'],\
                    "Item":msg['item'], "Priority":pri, "Dispatch Quantity":msg['qty'],\
                        "Cost":cost, "Dispatch Status":"YES", \
                            "Dispatch Date and Time":str(curr_time)}

            parameters2 = {"id":"Dashboard", "Order ID":msg['order_id'], "Order Dispatched":"YES", \
                            "Dispatched Time":str(curr_time)}

            url = "https://script.google.com/macros/s/"\
                + str(self._config_google_apps_sheet_id_1) + "/exec"
            # url_eyantra = "https://script.google.com/macros/s/"\
            #     + str(self._config_google_apps_sheet_id_2) + "/exec"
            # response_eyantra = requests.get(url_eyantra, params=parameters1)
            response1 = requests.get(url, params=parameters1)
            response2 = requests.get(url, params=parameters2)
            print response1.content
            print response2.content
            # print response_eyantra.content

        elif ss_num == 'ss3':

            curr_time = datetime.datetime.now()
            curr_date = curr_time.date()

            if msg['item'] == 'Medicine':
                pri = "HP"
                cost = "450"
                eta = curr_date + datetime.timedelta(days=1)
            elif msg['item'] == 'Clothes':
                pri = "LP"
                cost = "150"
                eta = curr_date + datetime.timedelta(days=5)
            else:
                pri = "MP"
                cost = "300"
                eta = curr_date + datetime.timedelta(days=3)

            order_time = datetime.datetime.strptime(str(msg['order_time']), '%Y-%m-%d %H:%M:%S')
            time_taken = curr_time - order_time
            total_time = time_taken.total_seconds()

            parameters1 = {"id":"OrdersShipped", "Team Id":'VB#0038',\
                "Unique Id":"RrSsVvPp", "Order ID":msg['order_id'], "City":msg['city'],\
                    "Item":msg['item'], "Priority":pri, "Shipped Quantity":msg['qty'],\
                        "Cost":cost, "Shipped Status":"YES", \
                            "Shipped Date and Time":str(curr_time),\
                                "Estimated Time of Delivery": str(eta)}

            parameters2 = {"id":"Dashboard", "Order ID":msg['order_id'],\
                "Order Shipped":"YES", "Shipping Time": str(curr_time),\
                    "Time taken":total_time}

            url = "https://script.google.com/macros/s/"\
                + str(self._config_google_apps_sheet_id_1) + "/exec"
            # url_eyantra = "https://script.google.com/macros/s/"\
            #     + str(self._config_google_apps_sheet_id_2) + "/exec"
            # response_eyantra = requests.get(url_eyantra, params=parameters1)
            response1 = requests.get(url, params=parameters1)
            response2 = requests.get(url, params=parameters2)
            print response1.content
            print response2.content
            # print response_eyantra.content



def main():
    '''main function'''
    rospy.init_node('node_iot_ros_bridge_action_server')
    action_server = IotRosBridgeActionServer()
    rospy.spin()

if __name__ == '__main__':
    main()
