import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import VehicleOdometry
import tf2_ros
import numpy as np

class PX4OdomToBaseLinkBroadcaster(Node):
    def __init__(self):
        super().__init__('px4_odom_tf_broadcaster')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile)

    def odom_callback(self, msg):
        if np.isnan(msg.position[0]) or np.isnan(msg.q[0]):
            return  

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = float(msg.position[1])   
        t.transform.translation.y = float(msg.position[0])   
        t.transform.translation.z = -float(msg.position[2])  
        
        q_w = float(msg.q[0])
        q_x = float(msg.q[1])
        q_y = float(msg.q[2])
        q_z = float(msg.q[3])
        
        t.transform.rotation.w = 0.70710678 * (q_w + q_z)
        t.transform.rotation.x = 0.70710678 * (q_x + q_y)
        t.transform.rotation.y = 0.70710678 * (q_x - q_y)
        t.transform.rotation.z = 0.70710678 * (q_w - q_z)

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PX4OdomToBaseLinkBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
