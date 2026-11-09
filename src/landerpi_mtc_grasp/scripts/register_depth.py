#!/usr/bin/env python3
"""Real-camera depth registration using CameraInfo and vendor extrinsic TF."""
from time import monotonic
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge
import tf2_ros
from depth_registration import register

class Registration(Node):
    def __init__(self):
        super().__init__('mtc_depth_registration')
        self.declare_parameter('already_aligned',False)
        self.depth_info=None;self.rgb_info=None;self.bridge=CvBridge();self.warn=0.
        self.tf=tf2_ros.Buffer();self.listener=tf2_ros.TransformListener(self.tf,self)
        self.create_subscription(CameraInfo,'depth_info',lambda m:setattr(self,'depth_info',m),qos_profile_sensor_data)
        self.create_subscription(CameraInfo,'rgb_info',lambda m:setattr(self,'rgb_info',m),qos_profile_sensor_data)
        self.create_subscription(Image,'depth',self.image,qos_profile_sensor_data)
        self.output=self.create_publisher(Image,'aligned_depth',qos_profile_sensor_data)
    def image(self,msg):
        try:
            d,r=self.depth_info,self.rgb_info
            if r is None:return
            if self.get_parameter('already_aligned').value:
                if msg.width!=r.width or msg.height!=r.height or msg.encoding not in ('16UC1','32FC1'):
                    raise ValueError('vendor aligned depth and RGB calibration dimensions differ')
                out=self.bridge.cv2_to_imgmsg(self.bridge.imgmsg_to_cv2(msg,'passthrough'),msg.encoding)
                out.header=msg.header;out.header.frame_id=r.header.frame_id;self.output.publish(out);return
            if d is None:return
            if d.header.frame_id!=msg.header.frame_id:raise ValueError('depth image and intrinsics frames differ')
            if d.width!=msg.width or d.height!=msg.height:raise ValueError('depth calibration dimensions differ')
            if msg.encoding not in ('16UC1','32FC1'):raise ValueError('unsupported depth encoding')
            depth=np.asarray(self.bridge.imgmsg_to_cv2(msg,'passthrough'),dtype=np.float32)
            if msg.encoding=='16UC1':depth*=.001
            transform=self.tf.lookup_transform(r.header.frame_id,msg.header.frame_id,rclpy.time.Time.from_msg(msg.header.stamp),timeout=Duration(seconds=.2)).transform
            q=transform.rotation;t=transform.translation
            qv=np.array([q.x,q.y,q.z],dtype=float);w=q.w
            if abs(np.linalg.norm([q.x,q.y,q.z,w])-1)>1e-3:raise ValueError('invalid camera calibration quaternion')
            cross=np.array([[0,-qv[2],qv[1]],[qv[2],0,-qv[0]],[-qv[1],qv[0],0]])
            rotation=np.eye(3)+2*w*cross+2*cross@cross
            aligned=register(depth,np.array(d.k),np.array(r.k),rotation,[t.x,t.y,t.z],r.height,r.width)
            out=self.bridge.cv2_to_imgmsg(aligned,'32FC1');out.header=msg.header;out.header.frame_id=r.header.frame_id
            self.output.publish(out)
        except Exception as exc:
            if monotonic()-self.warn>5:self.get_logger().warning(str(exc));self.warn=monotonic()
def main():
    rclpy.init();node=Registration();executor=MultiThreadedExecutor(num_threads=2);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
