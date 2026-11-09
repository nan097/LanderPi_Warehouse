#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2023/08/28
# stm32 ros2 package

import os
import math
import time
import rclpy
import signal
import threading
import yaml  # 已导入 PyYAML
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import UInt16, Bool
from ros_robot_controller.ros_robot_controller_sdk import Board, PacketReportKeyEvents
from ros_robot_controller_msgs.srv import GetBusServoState, GetPWMServoState
from ros_robot_controller_msgs.msg import (
    ButtonState, BuzzerState, MotorsState, BusServoState, LedState,
    SetBusServoState, ServosPosition, SetPWMServoState, Sbus, OLEDState,
    RGBStates, PWMServoState
)

# 机器人控制器节点(Robot Controller Node)
class RosRobotController(Node):
    gravity = 9.80665

    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.board = Board()# STM32控制板实例
        self.board.enable_reception()# 启用数据接收
        self.running = True

        # 参数声明(Parameter declaration)
        self.declare_parameter('motor_command_timeout',0.0)
        self.motor_timeout=float(self.get_parameter('motor_command_timeout').value)
        self.last_motor_time=time.monotonic();self.motor_active=False
        self.create_timer(.1,self.motor_watchdog)
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('init_finish', False)# 初始化完成参数
        self.IMU_FRAME = self.get_parameter('imu_frame').value

        # 发布者和订阅者(Publishers and Subscribers)
        self.imu_pub = self.create_publisher(Imu, '~/imu_raw', 1)# IMU数据发布者
        self.joy_pub = self.create_publisher(Joy, '~/joy', 1)# 游戏手柄数据发布者
        self.sbus_pub = self.create_publisher(Sbus, '~/sbus', 1)# SBUS数据发布者
        self.button_pub = self.create_publisher(ButtonState, '~/button', 1)# 按键数据发布者
        self.battery_pub = self.create_publisher(UInt16, '~/battery', 1)# 电池电量数据发布者
        self.create_subscription(LedState, '~/set_led', self.set_led_state, 5)# LED状态订阅
        self.create_subscription(BuzzerState, '~/set_buzzer', self.set_buzzer_state, 5)# 蜂鸣器状态订阅
        self.create_subscription(OLEDState, '~/set_oled', self.set_oled_state, 5)# OLED状态订阅
        self.create_subscription(MotorsState, '~/set_motor', self.set_motor_state, 10)# 电机状态订阅
        self.create_subscription(Bool, '~/enable_reception', self.enable_reception, 1)# 启用数据接收订阅
        self.create_subscription(SetBusServoState, '~/bus_servo/set_state', self.set_bus_servo_state, 10)# 舵机状态订阅
        self.create_subscription(ServosPosition, '~/bus_servo/set_position', self.set_bus_servo_position, 10)# 舵机位置订阅
        self.create_subscription(SetPWMServoState, '~/pwm_servo/set_state', self.set_pwm_servo_state, 10)# PWM舵机状态订阅
        self.create_service(GetBusServoState, '~/bus_servo/get_state', self.get_bus_servo_state)# 舵机状态服务
        self.create_service(GetPWMServoState, '~/pwm_servo/get_state', self.get_pwm_servo_state)# PWM舵机状态服务
        self.create_subscription(RGBStates, '~/set_rgb', self.set_rgb_states, 10)# RGB状态订阅

        self.create_service(Trigger,'~/set_machine_type',self.set_machine_type)# 机器类型设置服务
        # 加载并设置舵机偏移量从 YAML 文件
        self.load_servo_offsets()

        self.machine_type = os.environ['MACHINE_TYPE']
        self.motor_type = None
        self.battery_level = None
        
        self.send_machine_type()# 发送机器类型
        self.networdamode_set_led()# 设置网络模式LED指示灯

        # 初始化电机速度
        self.board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])

        self.clock = self.get_clock()# 获取时钟(Get clock)
        threading.Thread(target=self.pub_callback, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)

        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def networdamode_set_led(self):
        network_file = '/home/ubuntu/shared/.networkmode.txt'
        if not os.path.exists(network_file):
            return None
        with open(network_file, 'r') as f:# 读取网络模式(Read network mode)
            mode = f.read().strip()# 去除空白字符(Remove whitespace characters)
            # self.get_logger().info(f'\033[1;32m network mode: [{mode}]\033[0m')
            if mode == 'AP':# 访问点模式(Access Point mode)
                # msg.on_time, msg.off_time, msg.repeat, msg.id
                self.board.set_led(0.5, 0.5, 0, 2)# 蓝色常亮(Blue steady)
            elif mode == 'STA':# 站点模式(Station mode)
                self.board.set_led(1, 0, 0, 2)# 绿色常亮(Green steady)

    def load_servo_offsets(self):
        """
        从 YAML 文件中读取舵机偏差设置。
        """
        config_path = '/home/ubuntu/software/Servo_upper_computer/servo_config.yaml'
        try:
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)

            # 确保config是字典
            if not isinstance(config, dict):
                self.get_logger().error(f"YAML 配置文件格式错误: {config_path}，应为字典格式。")
                return

            # 遍历ID1到ID4并设置偏移量
            for servo_id in range(1, 5):
                offset = config.get(servo_id, 0)  # 如果未找到，默认偏移量为0
                try:
                    self.board.pwm_servo_set_offset(servo_id, offset)
                    self.get_logger().info(f"已设置舵机 {servo_id} 的偏移量为 {offset}")
                except Exception as e:
                    self.get_logger().error(f"设置舵机 {servo_id} 偏移量时出错: {e}")

        except FileNotFoundError:
            self.get_logger().error(f"配置文件未找到: {config_path}")
        except yaml.YAMLError as e:
            self.get_logger().error(f"解析 YAML 文件时出错: {e}")
        except Exception as e:
            self.get_logger().error(f"读取配置文件时出错: {e}")

    def get_node_state(self, request, response):# 获取节点状态(Get node state)
        response.success = True
        return response

    def set_machine_type(self, request, response):# 设置机器类型(Set machine type)
        self.send_machine_type()
        response.success = True
        response.message = 'FINISH!!!'
        return response

    def pub_callback(self):# 数据发布回调(Data publishing callback)
        while self.running:
            if getattr(self, 'enable_reception', False):
                self.pub_button_data(self.button_pub)# 按键数据发布(Publish button data)
                self.pub_joy_data(self.joy_pub)# 游戏手柄数据发布(Publish gamepad data)
                self.pub_imu_data(self.imu_pub)# IMU数据发布(Publish IMU data)
                self.pub_sbus_data(self.sbus_pub)# SBUS数据发布(Publish SBUS data)
                self.pub_battery_data(self.battery_pub)# 电池数据发布(Publish battery data)
                time.sleep(0.02)
            else:
                time.sleep(0.02)
        rclpy.shutdown()

    def enable_reception(self, msg):# 启用数据接收(Enable data reception)
        # self.get_logger().info('\033[1;32m%s\033[0m' % ('enable_reception ' + str(msg.data)))
        self.enable_reception = msg.data
        self.board.enable_reception(msg.data)

    def set_led_state(self, msg):# 设置LED状态(Set LED state)
        self.send_machine_type()
        self.board.set_led(msg.on_time, msg.off_time, msg.repeat, msg.id)

    def set_buzzer_state(self, msg):# 设置蜂鸣器状态(Set buzzer state)
        self.board.set_buzzer(msg.freq, msg.on_time, msg.off_time, msg.repeat)
    
    def set_rgb_states(self, msg):# 设置RGB状态(Set RGB states)
        pixels = []
        for state in msg.states:
            pixels.append((state.index, state.red, state.green, state.blue))
        self.board.set_rgb(pixels)

    def motor_watchdog(self):
        if self.motor_timeout>0 and self.motor_active and time.monotonic()-self.last_motor_time>self.motor_timeout:
            self.board.set_motor_speed([[1,0],[2,0],[3,0],[4,0]])
            self.motor_active=False

    def set_motor_state(self, msg):# 设置电机状态(Set motor state)
        data = []
        for i in msg.data:
            data.extend([[i.id, i.rps]])
        self.board.set_motor_speed(data)
        self.last_motor_time=time.monotonic();self.motor_active=any(speed!=0 for _,speed in data)

    def set_oled_state(self, msg):# 设置OLED状态(Set OLED state)
        self.board.set_oled_text(int(msg.index), msg.text)

    def set_pwm_servo_state(self, msg):# 设置PWM舵机状态(Set PWM servo state)
        data = []
        for i in msg.state:
            if i.id and i.position:
                data.extend([[i.id[0], i.position[0]]])
            if i.id and i.offset:
                self.board.pwm_servo_set_offset(i.id[0], i.offset[0])

        if data != []:
            self.board.pwm_servo_set_position(msg.duration, data)



    def send_machine_type(self):# 发送机器类型(Send machine type)
        if 'Tank' in self.machine_type:
            self.motor_type = 0x01
            # origin
            # self.battery_level = 0x2af8
            self.battery_level = 0x1af4 # 调整电池电压等级

        elif 'Acker' in self.machine_type or 'Mecanum' in self.machine_type:
            self.motor_type = 0x02# 麦克纳姆轮和阿克曼底盘
            self.battery_level = 0x1af4# 调整电池电压等级
        else:
            self.motor_type = 0x09
            self.battery_level = 0x1af4
        # 发送电机类型和电池配置到STM32
        if self.motor_type is not None:
            for i in range(2):# 发送两次确保接收
                '''
                MOTOR_TYPE_JGB520 ox00
                MOTOR_TYPE_JGB37  0x01
                MOTOR_TYPE_JGB27  0x02
                MOTOR_TYPE_JGB528 0x03
                '''
                self.board.set_motor_type(self.motor_type)
                self.board.set_battery_level(self.battery_level)
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'Please Set the machine_type')

    def get_pwm_servo_state(self, msg):# 获取PWM舵机状态(Get PWM servo state)
        states = []
        for i in msg.cmd:
            data = PWMServoState()# PWM舵机状态(PWM servo state)
            if i.get_position:
                state = self.board.pwm_servo_read_position(i.id)
                if state is not None:
                    data.position = state# 位置(Position)
            if i.get_offset:
                state = self.board.pwm_servo_read_offset(i.id)
                if state is not None:
                    data.offset = state
            states.append(data)
        return [True, states]
    
    def set_bus_servo_position(self, msg):
        data = []
        for i in msg.position:
            data.extend([[i.id, i.position]])
        if data:
            self.board.bus_servo_set_position(msg.duration, data)
    #总线舵机控制
    def set_bus_servo_state(self, msg):
        data = []
        servo_id = []
        for i in msg.state:
            if i.present_id:
                if i.present_id[0]:
                    if i.target_id:
                        if i.target_id[0]:
                            self.board.bus_servo_set_id(i.present_id[1], i.target_id[1])
                    if i.position:# 位置(Position)
                        if i.position[0]:
                            data.extend([[i.present_id[1], i.position[1]]])
                    if i.offset:# 偏移(Offset)
                        if i.offset[0]:
                            self.board.bus_servo_set_offset(i.present_id[1], i.offset[1])
                    if i.position_limit:# 位置限位(Position limit)
                        if i.position_limit[0]:
                            self.board.bus_servo_set_angle_limit(i.present_id[1], i.position_limit[1:])
                    if i.voltage_limit:# 电压限位(Voltage limit)
                        if i.voltage_limit[0]:
                            self.board.bus_servo_set_vin_limit(i.present_id[1], i.voltage_limit[1:])
                    if i.max_temperature_limit:# 最高温度限位(Max temperature limit)
                        if i.max_temperature_limit[0]:
                            self.board.bus_servo_set_temp_limit(i.present_id[1], i.max_temperature_limit[1])
                    if i.enable_torque:# 扭矩使能(Torque enable)
                        if i.enable_torque[0]:
                            self.board.bus_servo_enable_torque(i.present_id[1], i.enable_torque[1])
                    if i.save_offset:# 保存偏移(Save offset)
                        if i.save_offset[0]:
                            self.board.bus_servo_save_offset(i.present_id[1])
                    if i.stop:# 停止(Stop)
                        if i.stop[0]:
                            servo_id.append(i.present_id[1])
        if data != []:
            self.board.bus_servo_set_position(msg.duration, data)
        if servo_id != []:    
            self.board.bus_servo_stop(servo_id)
    # 获取总线舵机状态(Get bus servo state)
    def get_bus_servo_state(self, request, response):
        states = []
        for i in request.cmd:
            data = BusServoState()
            if i.get_id:
                state = self.board.bus_servo_read_id(i.id)
                if state is not None:
                    i.id = state[0]
                    data.present_id = state
            if i.get_position:
                state = self.board.bus_servo_read_position(i.id)
                if state is not None:
                    data.position = state
            if i.get_offset:
                state = self.board.bus_servo_read_offset(i.id)
                if state is not None:
                    data.offset = state
            if i.get_voltage:
                state = self.board.bus_servo_read_voltage(i.id)
                if state is not None:
                    data.voltage = state
            if i.get_temperature:
                state = self.board.bus_servo_read_temp(i.id)
                if state is not None:
                    data.temperature = state
            if i.get_position_limit:
                state = self.board.bus_servo_read_angle_limit(i.id)
                if state is not None:
                    data.position_limit = state
            if i.get_voltage_limit:
                state = self.board.bus_servo_read_vin_limit(i.id)
                if state is not None:
                    data.voltage_limit = state
            if i.get_max_temperature_limit:
                state = self.board.bus_servo_read_temp_limit(i.id)
                if state is not None:
                    data.max_temperature_limit = state
            if i.get_torque_state:
                state = self.board.bus_servo_read_torque(i.id)
                if state is not None:
                    data.enable_torque = state
            states.append(data)
        response.state = states
        response.success = True
        return response
    # 电池数据发布(Publish battery data)
    def pub_battery_data(self, pub):
        data = self.board.get_battery()# 获取电池数据(Get battery data)
        if data is not None:
            msg = UInt16()
            msg.data = data
            pub.publish(msg)
    # 按键数据发布(Publish button data)
    def pub_button_data(self, pub):
        data = self.board.get_button()
        if data is not None:
            key_id, key_event = data
            state_map = {
                PacketReportKeyEvents.KEY_EVENT_PRESSED: 1,# 按下(Pressed)
                PacketReportKeyEvents.KEY_EVENT_LONGPRESS: 2,# 长按(Long press)
                PacketReportKeyEvents.KEY_EVENT_LONGPRESS_REPEAT: 3,# 长按重复(Long press repeat)
                PacketReportKeyEvents.KEY_EVENT_RELEASE_FROM_LP: 4,# 释放自长按(Release from long press)
                PacketReportKeyEvents.KEY_EVENT_RELEASE_FROM_SP: 0,# 释放自短按(Release from short press)
                PacketReportKeyEvents.KEY_EVENT_CLICK: 5,# 点击(Click)
                PacketReportKeyEvents.KEY_EVENT_DOUBLE_CLICK: 6,## 双击(Double click)
                PacketReportKeyEvents.KEY_EVENT_TRIPLE_CLICK: 7,### 三击(Triple click)
            }
            state = state_map.get(key_event, -1)# 未处理的按键事件(Unhandled button event)

            if state != -1:
                msg = ButtonState()# 按键状态(Button state)
                msg.id = key_id# 按键ID(Button ID)
                msg.state = state#  按键状态(Button state)
                pub.publish(msg)# 发布(Publish)
            else:
                self.get_logger().error(f"Unhandled button event: {key_event}")
    # 游戏手柄数据发布(Publish gamepad data)
    def pub_joy_data(self, pub):
        data = self.board.get_gamepad()
        if data is not None:
            msg = Joy()
            msg.axes = data[0]# 轴数据(Axis data)
            msg.buttons = data[1]# 按钮数据(Button data)
            msg.header.stamp = self.clock.now().to_msg()# 时间戳(Timestamp)
            pub.publish(msg)

    def pub_sbus_data(self, pub):
        data = self.board.get_sbus()
        if data is not None:
            msg = Sbus()
            msg.channel = data
            msg.header.stamp = self.clock.now().to_msg()
            pub.publish(msg)

    def pub_imu_data(self, pub):
        data = self.board.get_imu()
        if data is not None:
            ax, ay, az, gx, gy, gz = data
            msg = Imu()
            msg.header.frame_id = self.IMU_FRAME
            msg.header.stamp = self.clock.now().to_msg()

            msg.orientation.w = 0.0
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0

            msg.linear_acceleration.x = ax * self.gravity
            msg.linear_acceleration.y = ay * self.gravity
            msg.linear_acceleration.z = az * self.gravity

            msg.angular_velocity.x = math.radians(gx)
            msg.angular_velocity.y = math.radians(gy)
            msg.angular_velocity.z = math.radians(gz)

            msg.orientation_covariance = [0.01, 0.0, 0.0,
                                          0.0, 0.01, 0.0,
                                          0.0, 0.0, 0.01]
            msg.angular_velocity_covariance = [0.01, 0.0, 0.0,
                                              0.0, 0.01, 0.0,
                                              0.0, 0.0, 0.01]
            msg.linear_acceleration_covariance = [0.0004, 0.0, 0.0,
                                                 0.0, 0.0004, 0.0,
                                                 0.0, 0.0, 0.004]
            pub.publish(msg)

def main():
    node = RosRobotController('ros_robot_controller')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Direct serial stop still works when the ROS context has shut down.
        node.board.set_motor_speed([[1,0],[2,0],[3,0],[4,0]])
        node.board.bus_servo_stop([1,2,3,4,5,9,10])
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__ == '__main__':
    main()
