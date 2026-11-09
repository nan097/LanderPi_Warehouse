#!/usr/bin/env python3
"""Publish color detections from synchronized Gazebo RGB-D, never entity poses."""
from time import monotonic
import cv2
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String,Bool
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
import tf2_ros
import tf2_geometry_msgs  # registers PointStamped transforms
from color_geometry import COLORS, colored_cloud, transform_cloud, upright_center


class ColorVision(Node):
    def __init__(self):
        super().__init__('mtc_color_vision')
        self.declare_parameter('target_color', 'red')
        self.color = self.get_parameter('target_color').value
        if self.color not in COLORS:
            raise ValueError('target_color must be red, green or blue')
        self.ready_time=0.
        self.ready=self.create_publisher(Bool,'/mtc_grasp/vision_ready',10)
        self.create_timer(.1,lambda:self.ready.publish(Bool(data=monotonic()-self.ready_time<1.)))
        self.bridge = CvBridge()
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf, self)
        self.info = None
        self.detect_enabled = False
        self.create_subscription(String, "/mtc_grasp/status", self.status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.last_warning = 0.
        self.reported = False
        self.create_subscription(CameraInfo, '/warehouse_camera/camera/camera_info', self.camera, qos_profile_sensor_data)
        self.rgb = Subscriber(self, Image, '/warehouse_camera/camera/image_raw', qos_profile=qos_profile_sensor_data)
        self.depth = Subscriber(self, Image, '/warehouse_camera/camera/depth/image_raw', qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([self.rgb, self.depth], 10, .03)
        self.sync.registerCallback(self.detect)
        self.target = self.create_publisher(PoseStamped, '/mtc_grasp/target_pose', 10)
        self.scene = self.create_publisher(PlanningScene, '/mtc_grasp/observed_scene', 10)
        self.debug = self.create_publisher(Image, '/mtc_grasp/debug_image', 10)

    def status(self, msg):
        self.detect_enabled = msg.data.startswith('WAITING_FOR_RGBD')

    def camera(self, msg):
        self.info = msg

    def detect(self, rgb_msg, depth_msg):
        rgb = None
        try:
            if self.info is None:
                return
            now = self.get_clock().now().nanoseconds
            stamp = rclpy.time.Time.from_msg(rgb_msg.header.stamp).nanoseconds
            if not 0 <= now-stamp < 2_000_000_000:
                return
            if rgb_msg.header.frame_id != depth_msg.header.frame_id or self.info.header.frame_id != rgb_msg.header.frame_id:
                raise ValueError('RGB-D camera frames differ')
            if rgb_msg.width!=depth_msg.width or rgb_msg.height!=depth_msg.height or self.info.width!=rgb_msg.width or self.info.height!=rgb_msg.height:
                raise ValueError('RGB-D images are not registered to the same calibrated resolution')
            if depth_msg.encoding not in ('16UC1','32FC1') or min(self.info.k[0],self.info.k[4])<=0:
                raise ValueError('invalid RGB-D depth encoding or intrinsics')
            transform=self.tf.lookup_transform('map',rgb_msg.header.frame_id,
                rclpy.time.Time.from_msg(rgb_msg.header.stamp),timeout=Duration(seconds=.25)).transform
            self.ready_time=monotonic()
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            if not self.detect_enabled:
                return
            depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
            scene = PlanningScene(is_diff=True)
            scene.robot_state.is_diff = True
            selected = None
            # TF may arrive a few milliseconds after the image. A second executor
            # thread keeps the listener running while this bounded lookup waits.
            t, q = transform.translation, transform.rotation
            for color in COLORS:
                try:
                    points, pixel = colored_cloud(rgb, depth, depth_msg.encoding, self.info.k, color)
                except ValueError:
                    if color == self.color:
                        raise
                    continue
                world = transform_cloud(points, [t.x,t.y,t.z], [q.x,q.y,q.z,q.w])
                center = upright_center(world)
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = rgb_msg.header.stamp
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, center)
                pose.pose.orientation.w = 1.
                primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.04, .04, .05])
                obj = CollisionObject(header=pose.header, id='cargo_'+color,
                    primitives=[primitive], primitive_poses=[pose.pose], operation=CollisionObject.ADD)
                scene.world.collision_objects.append(obj)
                cv2.circle(rgb, pixel, 4, (255, 255, 255), 1)
                cv2.putText(rgb, color, pixel, cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 255, 255), 1)
                if color == self.color:
                    selected = pose
            if selected:
                if not self.reported:
                    self.get_logger().info(f'RGBD target {self.color}: {selected.pose.position}; camera={rgb_msg.header.frame_id}, camera_map_xyz=({t.x:.3f},{t.y:.3f},{t.z:.3f})')
                    cv2.imwrite('/tmp/landerpi_mtc_'+self.color+'.png', rgb)
                    self.reported = True
                self.scene.publish(scene)
                self.target.publish(selected)
        except Exception as exc:
            if monotonic()-self.last_warning > 5.:
                self.get_logger().warning(str(exc))
                self.last_warning = monotonic()
        finally:
            if rgb is not None:
                debug = self.bridge.cv2_to_imgmsg(rgb, 'bgr8')
                debug.header = rgb_msg.header
                self.debug.publish(debug)


def main():
    rclpy.init()
    node = ColorVision()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
