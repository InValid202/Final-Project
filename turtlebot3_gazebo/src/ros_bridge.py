#!/usr/bin/env python3
from flask import Flask, request, jsonify
from flask_cors import CORS
import rospy
import threading
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
# from pathlib import Path
# from actionlib_msgs.msg import GoalStatusArray
import xmlrpc.client
import os


class ROSBridgeServer:
    def __init__(self):
        # navigate parameter
        self.ongoing_order = []
        self.delivered_order = []
        self.table_locations = self._get_table_locations()

        # api parameter
        self.odoo_url = 'https://invalid202-pim-final-project.odoo.com/'
        self.db = 'invalid202-pim-final-project-main-20225624'
        # self.username = os.getenv('ODOO_USERNAME')
        # self.password = os.getenv('ODOO_PASSWORD')
        self.username = 'admin'
        self.password = '283570076f72f5ef815e6da6d438d96b1277f08c'
        self.common = xmlrpc.client.ServerProxy('{}/xmlrpc/2/common'.format(self.odoo_url))
        self.common.version()
        self.uid = self.common.authenticate(self.db, self.username, self.password, {})
        rospy.loginfo(f'UID: {self.uid}')
        self.models = xmlrpc.client.ServerProxy('{}/xmlrpc/2/object'.format(self.odoo_url))
        self.updated_data = {'stage_id': 3,}

        # api co with ros
        self.not_available = False
        self.app = Flask(__name__)
        CORS(self.app)

        # publisher list
        self.status_pub = rospy.Publisher('/robot_availability', String, queue_size=10)
        self.order_pub = rospy.Publisher('/order_stage_topic', String, queue_size=10)
        self.current_table_pub = rospy.Publisher('/current_table', String, queue_size=10)
        self.all_table_pub = rospy.Publisher('/all_table', String, queue_size=10)
        self.navigation_status_pub = rospy.Publisher('/navigation_status', Bool, queue_size=10)

        # subscriber list
        self.gui_sub = rospy.Subscriber("/gui_commands", String, self.gui_command_callback)

        # service for navigate
        self.move_action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_action_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server.")

        # Setup Routes
        self.app.add_url_rule("/", "home", self.home)
        self.app.add_url_rule("/ros/order_stage", "handle_order_stage", self.handle_order_stage, methods=["POST"])

    def _get_table_locations(self):
        return {
            "original": {'x_position': 0, 'y_position': 0, 'z_position': 0.0, 'x_orientation': 0.0, 'y_orientation': 0.0, 'z_orientation': 0.0, 'w_orientation': 1.0},
            "1": {'x_position': 0.77, 'y_position': -8.9, 'z_position': 0.0, 'x_orientation': 0.0, 'y_orientation': 0.0, 'z_orientation': 0.0, 'w_orientation': 1.0},
            "2": {'x_position': 0.77, 'y_position': -11.47, 'z_position': 0.0, 'x_orientation': 0.0, 'y_orientation': 0.0, 'z_orientation': 0.0, 'w_orientation': 1.0},
            "3": {'x_position': 0.77, 'y_position': -13.66, 'z_position': 0.0, 'x_orientation': 0.0, 'y_orientation': 0.0, 'z_orientation': 0.0, 'w_orientation': 1.0},
            "4": {'x_position': -1.88, 'y_position': -8.8, 'z_position': 0.0, 'x_orientation': 0.0, 'y_orientation': 0.0, 'z_orientation': 1.0, 'w_orientation': 0.0},
            "5": {'x_position': -1.88, 'y_position': -11.22, 'z_position': 0.0, 'x_orientation': 0.0, 'y_orientation': 0.0, 'z_orientation': 1.0, 'w_orientation': 0.0},
            "6": {'x_position': -1.88, 'y_position': -13.66, 'z_position': 0.0, 'x_orientation': 0.0, 'y_orientation': 0.0, 'z_orientation': 1.0, 'w_orientation': 0.0},
        }

    def home(self):
        return "Hello, This is Local Host for ROS to receive Message From Odoo."

    def gui_command_callback(self, msg):
        if msg.data == "start_delivery" and len(self.ongoing_order) > 0:
            table_number = self.ongoing_order[0]["table_number"]
            self.current_table_pub.publish(f'{table_number}')
            self.not_available = True
            self.status_pub.publish("False")
            table_number = self.ongoing_order[0]["table_number"]
            self.current_table_pub.publish(f'{table_number}')
            self.send_goal(str(table_number))

        elif msg.data == "confirm_receive":
            current_delivering_table = self.ongoing_order[0]["table_number"]
            new_order = []
            new_delivered = []
            for order in self.ongoing_order:
                if order["table_number"] != current_delivering_table:
                    new_order.append(order)
                else:
                    new_delivered.append(order)
            self.ongoing_order = new_order
            self.delivered_order = new_delivered
            if len(self.ongoing_order) > 0:
                self.not_available = True
                self.status_pub.publish("False")
                next_table = self.ongoing_order[0]["table_number"]
                self.current_table_pub.publish(f'{next_table}')
                self.send_goal(str(next_table))
            else:
                self.current_table_pub.publish('0')
                self.send_goal('original')
                self.not_available = False
                self.status_pub.publish("True")
                record_ids = self.models.execute_kw(
                    self.db, self.uid, self.password,
                    'pos_preparation_display.order.stage', 'search',
                    [[['stage_id', '=', 5]]]
                )
                rospy.loginfo(f'record_ids:{record_ids}')
                result = self.models.execute_kw(self.db, self.uid, self.password, 'pos_preparation_display.order.stage',
                                                'write', [record_ids, self.updated_data])
                rospy.loginfo(f'Result for update stage: {result}')

    def handle_order_stage(self):
        if self.not_available:
            return jsonify({
                'status': 'OK',
                'published': 'None',
                'robot_status': 'Not Available'
            })
        data = request.get_json()
        order_id = data.get('id')
        stage_id = data.get('newStageId')
        table_number = data.get('tableNumber')

        msg = f"Order {order_id}, Stage {stage_id}, Table {table_number}"
        self.all_table_pub.publish(f'{table_number}')
        self.ongoing_order.append({
            'order_id': order_id,
            'stage_id': stage_id,
            'table_number': table_number
        })

        # self.send_goal(str(table_number))
        rospy.loginfo(f'order store: {self.ongoing_order}')
        self.order_pub.publish(msg)

        return jsonify({
            'status': 'OK',
            'published': msg,
            'robot_status': 'Available',
        })

    def send_goal(self, table_number):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()

        location = self.table_locations.get(table_number)
        rospy.logwarn(f"[ROS] Location: {location}")
        if not location:
            rospy.logwarn(f"[ROS] Invalid table: {table_number}")
            return

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # assign pose
        pose = Pose()
        pose.position = Point(location['x_position'], location['y_position'], location['z_position'])
        pose.orientation = Quaternion(0, 0, location['z_orientation'], location['w_orientation'])

        goal.target_pose.pose = pose

        rospy.loginfo(f"[ROS] Sending goal to move_base: {table_number}")
        self.navigation_status_pub.publish(True)
        self.move_action_client.send_goal(goal)
        self.move_action_client.wait_for_result()
        result = self.move_action_client.get_result()
        self.navigation_status_pub.publish(False)
        print("Goal reached:", result)

    def run(self, ssl_cert=None, ssl_key=None):
        if ssl_cert and ssl_key:
            self.app.run(host="0.0.0.0", port=5000, ssl_context=(ssl_cert, ssl_key))
        else:
            self.app.run(host="0.0.0.0", port=5000)

def init_ros_node():
    rospy.init_node('rosbridge_from_odoo', disable_signals=True)

if __name__ == '__main__':
    ros_thread = threading.Thread(target=init_ros_node)
    ros_thread.start()
    ros_thread.join()

    server = ROSBridgeServer()
    server.run(
        ssl_cert="/home/beer/tumpoom_ws/src/turtlebot3_simulations/turtlebot3_gazebo/src/cert.pem",
        ssl_key="/home/beer/tumpoom_ws/src/turtlebot3_simulations/turtlebot3_gazebo/src/key.pem"
    )


