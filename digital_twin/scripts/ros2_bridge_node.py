#!/usr/bin/env python3
"""ROS2 → ZMQ Bridge Node (Python 3.10, ROS2 side)

Subscribes to ROS2 topics and forwards data to Isaac Sim via ZMQ socket.

Run with system Python (ROS2):
    python3 ros2_bridge_node.py

Topics subscribed:
    /dsr01/curobo/obstacles  (String)     → vision-detected objects
    /dsr01/joint_states      (JointState) → robot arm joints
    /dsr01/gripper/stroke    (Int32)      → gripper position (0=open, 700=closed)
"""

import json
import zmq
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int32


class BridgeNode(Node):
    def __init__(self):
        super().__init__("digital_twin_bridge")

        # ZMQ publisher (sends to Isaac Sim)
        self._ctx = zmq.Context()
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.bind("tcp://*:5557")

        # ROS2 subscribers
        self.create_subscription(
            String, "/dsr01/curobo/obstacles", self._obstacles_cb, 10
        )
        self.create_subscription(
            JointState, "/dsr01/joint_states", self._joints_cb, 10
        )
        self.create_subscription(
            Int32, "/dsr01/gripper/stroke", self._gripper_cb, 10
        )
        self.create_subscription(
            String, "/digital_twin/command", self._cmd_cb, 10
        )

        self.get_logger().info("Bridge started → tcp://*:5557")
        self.get_logger().info("  /dsr01/curobo/obstacles → obstacles")
        self.get_logger().info("  /dsr01/joint_states     → joints")
        self.get_logger().info("  /dsr01/gripper/stroke   → gripper")
        self.get_logger().info("  /digital_twin/command   → command")

    def _obstacles_cb(self, msg: String):
        self._pub.send_multipart([b"obstacles", msg.data.encode()])

    def _joints_cb(self, msg: JointState):
        data = json.dumps({
            "name": list(msg.name),
            "position": list(msg.position),
        })
        self._pub.send_multipart([b"joints", data.encode()])

    def _gripper_cb(self, msg: Int32):
        # stroke: 0=fully open, 700=fully closed
        self._pub.send_multipart([b"gripper", str(msg.data).encode()])

    def _cmd_cb(self, msg: String):
        self._pub.send_multipart([b"command", msg.data.encode()])

    def destroy_node(self):
        self._pub.close()
        self._ctx.term()
        super().destroy_node()


def main():
    rclpy.init()
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
