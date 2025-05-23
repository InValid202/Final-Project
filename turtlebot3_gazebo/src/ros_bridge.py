#!/usr/bin/env python3
from flask import Flask, request, jsonify
from flask_cors import CORS
import rospy
import threading
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PoseWithCovarianceStamped, Twist
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseActionFeedback
# from pathlib import Path
from actionlib_msgs.msg import GoalStatusArray


class ROSBridgeServer:
    def __init__(self):
        self.stored_orders = []
        self.not_available = False
        self.table_locations = self._get_table_locations()
        self.app = Flask(__name__)
        CORS(self.app)

        self.order_pub = rospy.Publisher('/order_stage_topic', String, queue_size=10)
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

        self.stored_orders.append({
            'order_id': order_id,
            'stage_id': stage_id,
            'table_number': table_number
        })

        self.send_goal(str(table_number))
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
        # goal.target_pose.pose.position.x = location['x_position']
        # goal.target_pose.pose.position.y = location['y_position']
        # goal.target_pose.pose.orientation.z = location['z_orientation']
        # goal.target_pose.pose.orientation.w = location['w_orientation']

        rospy.loginfo(f"[ROS] Sending goal to move_base: {table_number}")
        self.move_action_client.send_goal(goal)
        self.move_action_client.wait_for_result()
        result = self.move_action_client.get_result()
        print("Goal reached:", result)

        # goal.pose.position.x = location['x_position']
        # goal.pose.position.y = location['y_position']
        # goal.pose.orientation.z = location['z_orientation']
        # goal.pose.orientation.w = location['w_orientation']

        # self.nav_pub.publish(goal)

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


