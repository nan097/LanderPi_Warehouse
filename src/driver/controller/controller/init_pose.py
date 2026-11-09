#!/usr/bin/env python3
# encoding: utf-8
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState# 关节状态消息
from servo_controller import action_group_controller# 自定义动作组控制器
from servo_controller_msgs.msg import ServosPosition, ServoPosition# 舵机位置消息

class InitPose(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)  
        
        # 获取命名空间
        namespace = self.get_namespace()
        if namespace == '/':
            namespace = ''
       
        # self.servo_state_pub = self.create_publisher(SetPWMServoState, 'ros_robot_controller/pwm_servo/set_state', 1)

        self.servo_controller_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)# 舵机控制器发布器
        self.joint_controller_pub = self.create_publisher(JointState, '/joint_controller', 1)# 关节控制器发布器  

        self.client = self.create_client(Trigger, namespace + '/controller_manager/init_finish')# 初始化完成检测客户端
        self.client.wait_for_service()#阻塞直到服务可用

        # Pro
        self.type = self.get_parameter('type').value# 获取初始化类型参数
        if self.type == 'action':# 动作组初始化
            action = self.get_parameters_by_prefix(self.type)# 获取动作组参数
            # 执行动作组
            acg = action_group_controller.ActionGroupController(self.servo_controller_pub, '/home/ubuntu/software/arm_pc/ActionGroups')
            acg.run_action(action['action_name'].value)# 执行动作组
        elif self.type == 'servo':# 舵机位置初始化
            pulse = self.get_parameters_by_prefix(self.type)
            msg = ServosPosition()# 舵机位置消息
            msg.duration = float(pulse['duration'].value)# 持续时间
            data = []   
            # 设置各舵机位置
            for i in ['id1', 'id2', 'id3', 'id4', 'id5', 'id10']:
                servo = ServoPosition()
                servo.id = int(i[2:]) 
                servo.position = float(pulse[i].value)
                data.append(servo)
            msg.position = data
            self.servo_controller_pub.publish(msg)# 发布舵机位置消息
       
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
    # 提供简单的服务接口，返回初始化完成状态
    def get_node_state(self, request, response):
        response.success = True
        return response
# 主函数
def main():
    node = InitPose('init_pose')
    rclpy.spin(node)  
if __name__ == "__main__":
    main()


