#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rclpy
import signal
import threading
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import Twist, Point
from math import copysign, pi, atan2, asin, degrees
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

def qua2rpy(quat):
    # 四元数转欧拉角，四元数格式为ros(Quaternion to Euler angles, where the quaternion is in ROS format)
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w

    roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = asin(2 * (w * y - x * z))
    yaw = atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))

    # 将角度转换到0-360范围(convert the angle to the range of 0 to 360 degrees)
    if roll < 0:
        roll += 2 * pi
    
    if pitch < 0:
        pitch += 2 * pi
    
    if yaw < 0:
        yaw += 2 * pi
    
    return roll, pitch, yaw
# 将角度标准化到[-π, π]范围内，避免角度跳跃
def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
        
    return res

class CalibrateAngular(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        signal.signal(signal.SIGINT, self.shutdown)

        #declare parameters，设置参数
        self.declare_parameter('test_angle', 360.0)# 旋转测试角度
        self.declare_parameter('speed', 0.5)# 旋转速度        
        self.declare_parameter('tolerance', 1.5)# 允许误差范围
        self.declare_parameter('odom_angular_scale_correction', 1.0)# 里程计角度比例修正系数
        self.declare_parameter('start_test', False)# 是否开始测试
        self.update_param()
        
        #TF配置
        self.odom_frame = 'odom'#世界坐标系
        self.base_frame = 'base_footprint'#机器人坐标系
        self.cmd_vel = self.create_publisher(Twist,"/controller/cmd_vel", 1)# 速度命令发布器
       
       # TF监听器
        self.tf_buffer = Buffer()# TF缓存
        self.tf_listener = TransformListener(self.tf_buffer, self)#变换监听器
        
        # 状态变量初始化
        self.position = Point()# 当前位置
        self.x_start = self.position.x# 起始位置X坐标
        self.y_start = self.position.y # 起始位置Y坐标   
        
        self.last_angle = 0# 上次测量的角度
        self.reverse = 1# 旋转方向
        self.calib_angle = 0# 累计校准角度
        
        # 启动主线程
        self.clock = self.get_clock()
        threading.Thread(target=self.main, daemon=True).start()

        self.get_logger().info('\033[1;32m%s\033[0m' % 'Bring up rqt_reconfigure to control the test')  
    # 更新参数
    def update_param(self):
        self.start_test = self.get_parameter('start_test').value
        self.test_angle = self.get_parameter('test_angle').value
        self.speed = self.get_parameter('speed').value
        self.tolerance = self.get_parameter('tolerance').value
        self.odom_angular_scale_correction = self.get_parameter('odom_angular_scale_correction').value

    def main(self):
        while True:#无限循环
            self.update_param()# 更新参数
            move_cmd = Twist()# 速度命令消息，
            if self.start_test:# 只有当测试标志为True时执行
                # Get the current rotation angle from tf，获取当前机器人朝向角度
                original_angle = degrees(self.get_odom_angle())
                self.test_angle = copysign(self.test_angle, self.reverse)#确定目标旋转方向

                # Compute how far we have gone since the last measurement，计算角度增量
                delta_angle = self.odom_angular_scale_correction * normalize_angle(original_angle - self.last_angle)

                # Add to our total angle so far,
                self.calib_angle += delta_angle#累加总旋转角度

                error = self.test_angle - self.calib_angle#计算角度误差
                # self.get_logger().info('\033[1;32moriginal:%f calib:%f\033[0m' % (original_angle, self.calib_angle))

                self.last_angle = original_angle#更新上一次角度记录
                #控制决策
                if abs(error) > self.tolerance and self.start_test:#误差超过容差范围，继续旋转
                    # Rotate the robot to reduce the error#向误差减小的方向旋转
                    move_cmd.angular.z = copysign(self.speed, error)
                else:
                    #测试完成，重置状态
                    self.calib_angle = 0.0
                    self.start_test  = rclpy.parameter.Parameter('start_test', rclpy.Parameter.Type.BOOL, False)#自动关闭测试标志
                    all_new_parameters = [self.start_test]
                    self.set_parameters(all_new_parameters)
                    self.reverse = -self.reverse
                    self.last_angle = 0
                    self.get_logger().info('\033[1;32m%s\033[0m' % 'finish')
            # 发布速度命令
            self.cmd_vel.publish(move_cmd)
            time.sleep(0.05)
    #TF角度获取                
    def get_odom_angle(self):
        # Get the current transform between the odom and base frames
        try:
            # 查询从odom到base_footprint的坐标变换
            trans = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1))
            return qua2rpy(trans.transform.rotation)[2]#提取变换中的旋转四元数，转换为欧拉角，返回yaw分量
        except (LookupException, ConnectivityException, ExtrapolationException):# TF异常处理
            self.get_logger().info('\033[1;32m%s\033[0m' % 'TF Exception')
            return
        
    def shutdown(self, signum, frame): 
        # Always stop the robot when shutting down the node
        self.get_logger().info('\033[1;32m%s\033[0m' % 'Stopping the robot...')
        self.start_test = False
        self.cmd_vel.publish(Twist())
        rclpy.shutdown()

def main():
    node = CalibrateAngular('calibrate_angular')
    rclpy.spin(node)  # 循环等待ROS2退出(loop waiting for ROS2 to exit)

if __name__ == "__main__":
    main()
