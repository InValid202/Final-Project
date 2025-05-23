#!/usr/bin/env python3
from flask import Flask, request, jsonify
from flask_cors import CORS
import rospy
import threading
from std_msgs.msg import String

app = Flask(__name__)
CORS(app)

threading.Thread(target=lambda: rospy.init_node('rosbrige_from_odoo', disable_signals=True)).start()
pub = rospy.Publisher('/order_stage_topic', String, queue_size=10)

@app.route("/")
def home():
    return "Hello, This is Local Host for ROS to receive Message From ROS."

@app.route('/ros/order_stage', methods=['POST'])
def handle_order_stage():
    data = request.get_json()

    order_id = data.get('id')
    stage_id = data.get('newStageId')
    table_number = data.get('tableNumber')

    msg = f"Order {order_id}, Stage {stage_id}, Table {table_number}"
    rospy.loginfo(f"[ROS HTTP] Received: {msg}")

    pub.publish(msg)

    return jsonify({
        'status': 'OK',
        'published': f'{msg}'
    })

if __name__ == '__main__':
    # app.run(host='0.0.0.0', port=5000)
    app.run(host="0.0.0.0", port=5000, ssl_context=("cert.pem", "key.pem"))

