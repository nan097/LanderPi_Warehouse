#!/usr/bin/env python3
"""Simulation adapters implementing the public task contracts against real ROS feedback.

The camera is a stationary overhead RGB-D testbench camera. Localization is
perfect odometry; grasping uses an explicitly ideal Gazebo fixed joint.
"""
import csv
import math
from pathlib import Path
from time import monotonic, time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from sensor_msgs.msg import JointState, Image, CameraInfo, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, PointStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from nav2_msgs.action import NavigateToPose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetPositionIK, GetMotionPlan, ApplyPlanningScene, GetPlanningScene, GetStateValidity
from moveit_msgs.msg import Constraints, JointConstraint, CollisionObject, AttachedCollisionObject, PlanningScene, AllowedCollisionEntry
from shape_msgs.msg import SolidPrimitive
from gazebo_msgs.srv import GetEntityState
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
from landerpi_task_interfaces.srv import ResetFaults
from landerpi_task_interfaces.msg import RobotStatus
from landerpi_task_manager.error_codes import ErrorCode
from landerpi_task_interfaces.action import ExecuteSortTask, NavigateStation, DetectObject, PickObject, PlaceObject, FineAlign, MoveToJoint, MoveToPose, MoveNamedPose
ARM = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
STATIONS = {'home': (0.0, 0.0), 'pick': (0.4, 0.0), 'place': (0.4, 1.5)}

class Interrupted(RuntimeError):
    pass

def error_code(exc, phase=''):
    text = str(exc).lower()
    if 'cancel' in text:
        return int(ErrorCode.TASK_CANCELLED)
    if 'timeout' in text:
        return int(ErrorCode.NAV_TIMEOUT if phase.startswith('NAVIGATE') else ErrorCode.TASK_TIMEOUT)
    for (keyword, code) in [('ik failed', ErrorCode.IK_FAILED), ('plan failed', ErrorCode.MOVEIT_PLAN_FAILED), ('execution failed', ErrorCode.TRAJECTORY_EXEC_FAILED), ('arm feedback', ErrorCode.TRAJECTORY_EXEC_FAILED), ('grasp', ErrorCode.GRASP_VERIFY_FAILED), ('follow gripper', ErrorCode.GRASP_VERIFY_FAILED), ('confidence', ErrorCode.LOW_CONFIDENCE), ('joint names', ErrorCode.CONFIG_INVALID), ('joint position count', ErrorCode.CONFIG_INVALID), ('scales', ErrorCode.CONFIG_INVALID), ('gripper', ErrorCode.GRIPPER_FAILED), ('depth', ErrorCode.DEPTH_INVALID), ('red object', ErrorCode.OBJECT_NOT_FOUND), ('scene', ErrorCode.COLLISION_SCENE_FAILED), ('grasp', ErrorCode.GRASP_VERIFY_FAILED), ('follow gripper', ErrorCode.GRASP_VERIFY_FAILED), ('outside place', ErrorCode.PLACE_FAILED), ('joint feedback', ErrorCode.JOINT_STATE_STALE), ('alignment', ErrorCode.FINE_ALIGN_FAILED), ('navigate_to_pose', ErrorCode.NAV_FAILED)]:
        if keyword in text:
            return int(code)
    return int(ErrorCode.UNKNOWN_ERROR)

class Runtime(Node):

    def __init__(self):
        super().__init__('simulation_task_runtime')
        self.group = ReentrantCallbackGroup()
        self.declare_parameter('csv_path', '/tmp/landerpi_experiments/simulation_tasks_v2.csv')
        self.csv_path = Path(self.get_parameter('csv_path').value)
        self.stations = dict(STATIONS)
        self.object_name = 'box'
        self.pick_support = 'pick_table'
        self.release_clearance = .04
        self.object_size = [.04,.04,.05]
        self.data = {}
        self.stamps = {}
        self.busy = set()
        self.bridge = CvBridge()
        self.attached = False
        self.world_box_pose = None
        self.phase = 'IDLE'
        self.fault = ''
        self.ready = False
        self.last_error = 0
        self.deadline = None
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf, self)
        for (typ, topic, key) in [(JointState, '/joint_states', 'joints'), (Odometry, '/odom', 'odom'), (Image, '/camera/camera/image_raw', 'rgb'), (Image, '/camera/camera/depth/image_raw', 'depth'), (CameraInfo, '/camera/camera/camera_info', 'info'), (Bool, '/simulation/object_attached', 'attached'), (LaserScan, '/scan', 'scan')]:
            self.create_subscription(typ, topic, lambda msg, k=key: self.receive(k, msg), qos_profile_sensor_data, callback_group=self.group)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.action_clients = {}
        for (kind, name) in [(NavigateToPose, '/navigate_to_pose'), (ExecuteTrajectory, '/execute_trajectory'), (FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')]:
            self.action_clients[name] = ActionClient(self, kind, name, callback_group=self.group)
        self.service_clients = {}
        for (kind, name) in [(GetStateValidity, '/check_state_validity'), (GetPositionIK, '/compute_ik'), (GetMotionPlan, '/plan_kinematic_path'), (ApplyPlanningScene, '/apply_planning_scene'), (GetPlanningScene, '/get_planning_scene'), (Trigger, '/simulation/attach_object'), (Trigger, '/simulation/detach_object'), (Trigger, '/simulation/hold_base'), (Trigger, '/simulation/release_base'), (GetEntityState, '/gazebo/get_entity_state')]:
            self.service_clients[name] = self.create_client(kind, name, callback_group=self.group)
        self.servers = []
        for (kind, name, callback) in [(ExecuteSortTask, '/task/execute_sort', self.task), (NavigateStation, '/navigation/navigate_station', self.navigation), (DetectObject, '/perception/detect_object', self.detection), (PickObject, '/manipulation/pick_object', self.pick_action), (PlaceObject, '/manipulation/place_object', self.place_action), (FineAlign, '/fine_alignment/align', self.align_action), (MoveToJoint, '/manipulation/move_to_joint', self.joint_action), (MoveToPose, '/manipulation/move_to_pose', self.pose_action), (MoveNamedPose, '/manipulation/move_named_pose', self.named_action)]:
            self.servers.append(ActionServer(self, kind, name, execute_callback=callback, goal_callback=lambda goal, n=name: self.accept(n, goal), cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group))
        self.reset_service = self.create_service(ResetFaults, '/robot/reset_faults', self.reset_faults, callback_group=self.group)
        self.status_pub = self.create_publisher(RobotStatus, '/robot/status', 10)
        self.status_timer = self.create_timer(0.5, self.publish_status, callback_group=self.group)
        self.get_logger().info('SIMULATION runtime: real Nav2, RGB-D, MoveIt and ideal rigid attachment; quantity=1')

    def reset_faults(self, request, response):
        if self.busy or self.attached or self.data.get('attached', Bool()).data:
            response.success = False
            response.error_code = int(ErrorCode.GRASP_VERIFY_FAILED)
            response.message = 'cannot reset during motion or with an attached object; restart/reset simulation scene'
        else:
            self.fault = ''
            self.last_error = 0
            self.phase = 'IDLE'
            response.success = True
            response.message = 'simulation fault reset: ' + request.reason
        return response

    def publish_status(self):
        status = RobotStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        fresh = lambda k: monotonic() - self.stamps.get(k, 0) < 3
        status.base_ready = fresh('odom') and self.action_clients['/navigate_to_pose'].server_is_ready()
        status.arm_ready = fresh('joints')
        status.gripper_ready = status.arm_ready and self.action_clients['/gripper_controller/follow_joint_trajectory'].server_is_ready()
        status.camera_ready = all((fresh(k) for k in ['rgb', 'depth', 'info']))
        status.lidar_ready = fresh('scan')
        status.localization_ready = bool(self.tf.can_transform('map', 'base_footprint', rclpy.time.Time()))
        status.moveit_ready = self.service_clients['/compute_ik'].service_is_ready() and self.action_clients['/execute_trajectory'].server_is_ready()
        status.robot_ready = not bool(self.fault) and all([status.base_ready, status.arm_ready, status.gripper_ready, status.camera_ready, status.lidar_ready, status.localization_ready, status.moveit_ready])
        self.ready = status.robot_ready
        status.task_state = self.phase
        status.active_error_code = self.last_error if self.fault else 0
        status.active_error_message = self.fault or ('' if status.robot_ready else 'waiting for simulation feedback and action servers')
        self.status_pub.publish(status)

    def receive(self, key, msg):
        self.data[key] = msg
        self.stamps[key] = monotonic()

    def accept(self, name, goal):
        if self.busy or self.fault or not self.ready or (name == '/task/execute_sort' and (goal.quantity != 1 or goal.object_class != 'red_box' or goal.pick_station != 'pick' or (goal.place_station != 'place') or (not goal.task_id))):
            return GoalResponse.REJECT
        self.busy.add(name)
        return GoalResponse.ACCEPT

    async def pause(self, seconds=0.05):
        future = Future()
        timer = self.create_timer(seconds, lambda : future.set_result(True) if not future.done() else None, callback_group=self.group, clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))
        try:
            await future
        finally:
            self.destroy_timer(timer)

    def check(self, h, deadline):
        if h and h.is_cancel_requested:
            raise Interrupted('task cancelled')
        if h is not None and self.deadline is not None:
            deadline = min(deadline, self.deadline)
        if monotonic() > deadline:
            raise Interrupted('operation timeout')

    async def wait(self, future, h=None, timeout=30):
        deadline = monotonic() + timeout
        self.check(h, deadline)
        while not future.done():
            self.check(h, deadline)
            await self.pause()
        return future.result()

    async def service(self, name, request, h=None, timeout=20):
        client = self.service_clients[name]
        deadline = monotonic() + timeout
        self.check(h, deadline)
        while not client.service_is_ready():
            self.check(h, deadline)
            await self.pause()
        return await self.wait(client.call_async(request), h, timeout)

    async def action(self, name, goal, h=None, timeout=90):
        client = self.action_clients[name]
        deadline = monotonic() + timeout
        self.check(h, deadline)
        while not client.server_is_ready():
            self.check(h, deadline)
            await self.pause()
        # Resolve the send handshake even if the parent is cancelled meanwhile.
        # Otherwise an accepted child could keep moving without a goal handle.
        handle = await self.wait(client.send_goal_async(goal), timeout=10)
        if not handle.accepted:
            self.check(h, deadline)
            raise RuntimeError(name + ' goal rejected')
        result_future = handle.get_result_async()
        try:
            result = await self.wait(result_future, h, timeout)
        except Interrupted:
            await self.wait(handle.cancel_goal_async(), timeout=5)
            self.cmd.publish(Twist())
            await self.wait(result_future, timeout=5)
            self.cmd.publish(Twist())
            raise
        if result.status != 4:
            raise RuntimeError(name + ' failed: ' + str(result.result))
        return result.result

    def joints(self):
        if monotonic() - self.stamps.get('joints', 0) > 2:
            raise RuntimeError('joint feedback missing or stale')
        source = self.data['joints']
        indices = [i for (i, n) in enumerate(source.name) if not n.endswith('_mimic')]
        return JointState(header=source.header, name=[source.name[i] for i in indices], position=[source.position[i] for i in indices])

    def pose(self, x, y, z, frame='map'):
        p = PoseStamped()
        p.header.frame_id = frame
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.position.z = float(z)
        p.pose.orientation.w = 1.0
        return p

    async def transform(self, p, frame, h):
        deadline = monotonic() + 5
        while not self.tf.can_transform(frame, p.header.frame_id, rclpy.time.Time()):
            self.check(h, deadline)
            await self.pause()
        return self.tf.transform(p, frame)

    async def move_joints(self, values, h, execute=True, velocity_scale=None, acceleration_scale=None, planner_id=''):
        if velocity_scale is None: velocity_scale=getattr(self,'arm_velocity_scale',.4)
        if acceleration_scale is None: acceleration_scale=getattr(self,'arm_acceleration_scale',.4)
        if len(values) != 5 or not all(math.isfinite(v) for v in values):
            raise RuntimeError('joint goal must contain five finite positions')
        if not (0 < velocity_scale <= 1 and 0 < acceleration_scale <= 1):
            raise RuntimeError('velocity and acceleration scales must be in (0,1]')
        if not (await self.service('/simulation/hold_base', Trigger.Request(), h)).success:
            raise RuntimeError('base brake failed')
        await self.refresh_fixtures(h)
        req = GetMotionPlan.Request()
        m = req.motion_plan_request
        m.group_name = 'arm'
        m.allowed_planning_time = 5.0
        m.max_velocity_scaling_factor = float(velocity_scale)
        m.max_acceleration_scaling_factor = float(acceleration_scale)
        m.planner_id = planner_id
        m.start_state.joint_state = self.joints()
        m.start_state.is_diff = True
        c = Constraints()
        c.joint_constraints = [JointConstraint(joint_name=n, position=float(v), tolerance_above=getattr(self,'joint_goal_tolerance',.006), tolerance_below=getattr(self,'joint_goal_tolerance',.006), weight=1.0) for (n, v) in zip(ARM, values)]
        m.goal_constraints = [c]
        answer = (await self.service('/plan_kinematic_path', req, h)).motion_plan_response
        if answer.error_code.val != 1:
            query=GetStateValidity.Request(group_name='arm')
            query.robot_state.joint_state=self.joints()
            query.robot_state.is_diff=True
            for name,value in zip(ARM,values):
                query.robot_state.joint_state.position[query.robot_state.joint_state.name.index(name)]=float(value)
            validity=await self.service('/check_state_validity',query,h)
            self.get_logger().error(f'Planning goal valid={validity.valid} contacts={[(c.contact_body_1,c.contact_body_2) for c in validity.contacts]} joints={values}')
            raise RuntimeError('MoveIt plan failed ' + str(answer.error_code.val))
        if not execute:
            return (answer.planning_time, 0.)
        execution_started = monotonic()
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = answer.trajectory
        result = await self.action('/execute_trajectory', goal, h, 60)
        if result.error_code.val != 1:
            raise RuntimeError('MoveIt execution failed ' + str(result.error_code.val))
        deadline = monotonic() + 3.
        while True:
            state = dict(zip(self.joints().name, self.joints().position))
            errors = {n:state[n]-v for n,v in zip(ARM, values)}
            if max(abs(v) for v in errors.values()) <= .03:break
            if monotonic() > deadline:
                raise RuntimeError('arm feedback differs from executed target: '+str(errors))
            self.check(h, deadline)
            await self.pause(.05)
        return (answer.planning_time, monotonic() - execution_started)

    async def move_pose(self, pose, h, **options):
        p = await self.transform(pose, 'base_footprint', h)
        req = GetPositionIK.Request()
        ik = req.ik_request
        ik.group_name = 'arm'
        ik.ik_link_name = 'end_effector_link'
        ik.pose_stamped = p
        ik.robot_state.joint_state = self.joints()
        ik.robot_state.is_diff = True
        ik.avoid_collisions = True
        ik.timeout.sec = 2
        answer = await self.service('/compute_ik', req, h)
        for seed in getattr(self, 'ik_seeds', [[0.0, -0.5, -0.8, -0.8, 0.0], [0.0, -0.8, -0.5, -0.8, 0.0], [0.0, -1.0, 0.3, -1.4, 0.0]]):
            if answer.error_code.val == 1:
                break
            state = self.joints()
            positions = list(state.position)
            for (n, v) in zip(ARM, seed):
                positions[state.name.index(n)] = v
            state.position = positions
            ik.robot_state.joint_state = state
            answer = await self.service('/compute_ik', req, h)
        if answer.error_code.val != 1:
            raise RuntimeError(f'IK failed {answer.error_code.val} at {p.pose.position}')
        state = dict(zip(answer.solution.joint_state.name, answer.solution.joint_state.position))
        return await self.move_joints([state[n] for n in ARM], h, **options)

    async def gripper(self, value, h):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['r_joint']
        point = JointTrajectoryPoint(positions=[float(value)])
        duration=getattr(self,'gripper_duration',1.)
        point.time_from_start.sec=int(duration)
        point.time_from_start.nanosec=round((duration%1)*1e9)
        goal.trajectory.points = [point]
        result = await self.action('/gripper_controller/follow_joint_trajectory', goal, h, 20)
        if result.error_code:
            raise RuntimeError('gripper trajectory failed')

    async def entity(self, name, h=None):
        req = GetEntityState.Request()
        req.name = name
        req.reference_frame = 'world'
        res = await self.service('/gazebo/get_entity_state', req, h)
        if not res.success:
            raise RuntimeError('Gazebo entity unavailable: ' + name)
        return res.state.pose

    def collision(self, name, p, dimensions):
        obj = CollisionObject()
        obj.id = name
        obj.header = p.header
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=dimensions)]
        obj.primitive_poses = [p.pose]
        obj.operation = CollisionObject.ADD
        return obj

    async def refresh_fixtures(self, h):
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects = [self.collision(name, await self.transform(self.pose(.75, y, .08), 'base_footprint', h), [.30, .40, .16]) for name, y in [('pick_table', 0.), ('place_table', 1.5)]]
        if self.world_box_pose is not None and not self.attached:
            request.scene.world.collision_objects.append(self.collision(self.object_name, await self.transform(self.world_box_pose, 'base_footprint', h), self.object_size))
        if not (await self.service('/apply_planning_scene', request, h)).success:
            raise RuntimeError('fixture scene update failed')

    async def scene(self, pose, h):
        self.world_box_pose = await self.transform(pose, 'map', h)
        req = ApplyPlanningScene.Request()
        s = req.scene
        s.is_diff = True
        s.robot_state.is_diff = True
        s.world.collision_objects = [self.collision('pick_table', await self.transform(self.pose(0.75, 0, 0.08), 'base_footprint', h), [0.3, 0.4, 0.16]), self.collision('place_table', await self.transform(self.pose(0.75, 1.5, 0.08), 'base_footprint', h), [0.3, 0.4, 0.16]), self.collision(self.object_name, await self.transform(pose, 'base_footprint', h), self.object_size)]
        current = await self.service('/get_planning_scene', GetPlanningScene.Request(components=__import__('moveit_msgs.msg', fromlist=['PlanningSceneComponents']).PlanningSceneComponents(components=128)), h)
        acm = current.scene.allowed_collision_matrix
        if self.object_name not in acm.entry_names:
            for row in acm.entry_values:
                row.enabled.append(False)
            acm.entry_names.append(self.object_name)
            acm.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(acm.entry_names)))
        index = acm.entry_names.index(self.object_name)
        for (i, name) in enumerate(acm.entry_names):
            if name in ['link5', 'gripper_link', 'gripper_base_link', 'guide_rod_link', 'gripper_left_link', 'gripper_right_link', 'left_jaw_link', 'right_jaw_link', 'end_effector_link']:
                acm.entry_values[i].enabled[index] = True
                acm.entry_values[index].enabled[i] = True
        s.allowed_collision_matrix = acm
        if not (await self.service('/apply_planning_scene', req, h)).success:
            raise RuntimeError('planning scene update failed')

    async def support_contact(self, table, allowed, h):
        from moveit_msgs.msg import PlanningSceneComponents
        query = GetPlanningScene.Request(components=PlanningSceneComponents(components=128))
        acm = (await self.service('/get_planning_scene', query, h)).scene.allowed_collision_matrix
        for name in [self.object_name, table]:
            if name not in acm.entry_names:
                for row in acm.entry_values:
                    row.enabled.append(False)
                acm.entry_names.append(name)
                acm.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(acm.entry_names)))
        i, j = acm.entry_names.index(self.object_name), acm.entry_names.index(table)
        acm.entry_values[i].enabled[j] = allowed
        acm.entry_values[j].enabled[i] = allowed
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.allowed_collision_matrix = acm
        if not (await self.service('/apply_planning_scene', request, h)).success:
            raise RuntimeError('support contact scene update failed')

    async def attached_scene(self, attach, h):
        req = ApplyPlanningScene.Request()
        s = req.scene
        s.is_diff = True
        s.robot_state.is_diff = True
        attached = AttachedCollisionObject()
        attached.link_name = 'link5'
        attached.object.id = self.object_name
        attached.object.operation = CollisionObject.ADD if attach else CollisionObject.REMOVE
        if attach:
            world = await self.entity(self.object_name, h)
            p = self.pose(world.position.x, world.position.y, world.position.z)
            p.pose = world
            local = await self.transform(p, 'link5', h)
            attached.object = self.collision(self.object_name, local, self.object_size)
            attached.touch_links = ['link5', 'gripper_link', 'gripper_base_link', 'guide_rod_link', 'gripper_left_link', 'gripper_right_link', 'left_jaw_link', 'right_jaw_link', 'end_effector_link']
        if not attach:
            world = await self.entity(self.object_name, h)
            pose = self.pose(world.position.x, world.position.y, world.position.z)
            pose.pose = world
            self.world_box_pose = pose
            s.world.collision_objects = [self.collision(self.object_name, await self.transform(pose, 'base_footprint', h), self.object_size)]
        s.robot_state.attached_collision_objects = [attached]
        if not (await self.service('/apply_planning_scene', req, h)).success:
            raise RuntimeError('attached scene failed')

    async def navigate(self, station, h, timeout=160.):
        if station not in self.stations:
            raise RuntimeError('unknown station ' + station)
        (x, y) = self.stations[station]
        state = dict(zip(self.joints().name, self.joints().position))
        if any(abs(state[n]) > .08 for n in ARM):
            raise RuntimeError('transport arm pose required before navigation')
        if self.attached and state['r_joint'] > -.35:
            raise RuntimeError('grasp lost before transport')
        goal = NavigateToPose.Goal()
        goal.pose = self.pose(x, y, 0)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        if not (await self.service('/simulation/release_base', Trigger.Request(), h)).success:
            raise RuntimeError('base brake release failed')
        await self.action('/navigate_to_pose', goal, h, timeout)
        odom = self.data.get('odom')
        position = odom.pose.pose.position
        if math.hypot(position.x - x, position.y - y) > 0.065:
            raise RuntimeError('station arrival verification failed')
        await self.align(station, h)
        if self.attached:
            box = await self.entity(self.object_name, h)
            tip = self.tool_pose().pose.position
            if math.dist([box.position.x, box.position.y, box.position.z], [tip.x, tip.y, tip.z]) > .06:
                raise RuntimeError('grasp lost during transport')
        position = self.data['odom'].pose.pose.position
        return self.pose(position.x, position.y, position.z)

    async def align(self, station, h, xy_tolerance=.006, yaw_tolerance=.015, timeout=30.):
        if station not in self.stations:
            raise RuntimeError("unknown station " + station)
        (x, y) = self.stations[station]
        deadline = monotonic() + timeout
        last_alignment_report = monotonic()
        try:
            while True:
                self.check(h, deadline)
                if monotonic() - self.stamps.get('odom', 0) > 2:
                    raise RuntimeError('alignment odometry stale')
                p = self.data['odom'].pose.pose
                q = p.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                dx = x - p.position.x
                dy = y - p.position.y
                if monotonic() - last_alignment_report > 5.:
                    self.get_logger().info(f'Alignment {station}: xy_error={math.hypot(dx,dy):.4f}, yaw_error={yaw:.4f}, pose=({p.position.x:.4f},{p.position.y:.4f})')
                    last_alignment_report = monotonic()
                if math.hypot(dx, dy) < xy_tolerance and abs(yaw) < yaw_tolerance:
                    break
                if math.hypot(dx, dy) > 0.1:
                    raise RuntimeError('alignment requested outside station capture radius')
                cmd = Twist()
                gain=getattr(self,'alignment_gain',.8)
                linear=getattr(self,'alignment_max_linear',.06)
                angular=getattr(self,'alignment_max_angular',.15)
                cmd.linear.x = max(-linear, min(linear, gain * (math.cos(yaw) * dx + math.sin(yaw) * dy)))
                cmd.linear.y = max(-linear, min(linear, gain * (-math.sin(yaw) * dx + math.cos(yaw) * dy)))
                cmd.angular.z = max(-angular, min(angular, -gain * yaw))
                self.cmd.publish(cmd)
                await self.pause(0.05)
        finally:
            self.cmd.publish(Twist())

    async def align_action(self, h):
        async def operation():
            if h.request.station not in self.stations:
                raise RuntimeError('unknown station ' + h.request.station)
            if h.request.mode == 0:
                return
            if h.request.mode != 1:
                raise RuntimeError('simulation fine alignment supports NAV_POSE_ONLY mode 1')
            if h.request.xy_tolerance <= 0 or h.request.yaw_tolerance <= 0:
                raise RuntimeError('alignment tolerances must be positive')
            if not (await self.service('/simulation/release_base', Trigger.Request(), h)).success:
                raise RuntimeError('base brake release failed')
            timeout = h.request.timeout.sec + h.request.timeout.nanosec * 1e-9
            await self.align(h.request.station, h, h.request.xy_tolerance,
                             h.request.yaw_tolerance, timeout if timeout > 0 else 30.)
        return await self.finish_action(h, '/fine_alignment/align', FineAlign, operation)

    async def detect(self, h):
        deadline = monotonic() + 15
        requested_stamp = self.get_clock().now().nanoseconds / 1e9
        def fresh_pair():
            if not all(k in self.data for k in ['rgb','depth','info']):return False
            stamp = lambda key:self.data[key].header.stamp.sec + self.data[key].header.stamp.nanosec/1e9
            return min(stamp('rgb'),stamp('depth')) > requested_stamp and abs(stamp('rgb')-stamp('depth')) < .05
        while not (fresh_pair() and all((monotonic() - self.stamps.get(k, 0) < 2 for k in ['rgb', 'depth', 'info']))):
            self.check(h, deadline)
            await self.pause()
        rgb = self.bridge.imgmsg_to_cv2(self.data['rgb'], 'bgr8')
        cv2.imwrite('/tmp/landerpi_rgbd_frame.png', rgb)
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 130, 60), (12, 255, 255)) | cv2.inRange(hsv, (170, 130, 60), (180, 255, 255))
        (contours, _) = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            raise RuntimeError('red object not found in actual RGB image')
        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < 8:
            raise RuntimeError('red candidate too small')
        moments = cv2.moments(contour)
        u = int(moments['m10'] / moments['m00'])
        v = int(moments['m01'] / moments['m00'])
        depth = self.bridge.imgmsg_to_cv2(self.data['depth'], 'passthrough')
        patch = depth[max(0, v - 2):v + 3, max(0, u - 2):u + 3]
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if not valid.size:
            raise RuntimeError('invalid target depth')
        z = float(np.median(valid))
        info = self.data['info']
        point = PointStamped()
        point.header.frame_id = info.header.frame_id
        point.point.x = (u - info.k[2]) * z / info.k[0]
        point.point.y = (v - info.k[5]) * z / info.k[4]
        point.point.z = z
        deadline = monotonic() + 5
        while not self.tf.can_transform('map', point.header.frame_id, rclpy.time.Time()):
            self.check(h, deadline)
            await self.pause()
        world = self.tf.transform(point, 'map').point
        pose = self.pose(world.x, world.y, world.z - 0.025)
        self.get_logger().info(f'RGB-D centroid ({u},{v}), depth={z:.3f}, object={pose.pose.position}')
        return pose

    async def pick(self, pose, h):
        await self.scene(pose, h)
        await self.support_contact(self.pick_support, True, h)
        await self.gripper(0, h)
        p = await self.transform(pose, 'map', h)
        pos = p.pose.position
        await self.move_pose(self.pose(pos.x, pos.y, pos.z + 0.075), h)
        await self.move_pose(p, h)
        measured = await self.entity(self.object_name, h)
        tool = self.tf.lookup_transform('map', 'end_effector_link', rclpy.time.Time()).transform.translation
        self.get_logger().info(f'Grasp verification before close: box={measured.position}, tool={tool}')
        self.last_grasp_pose = self.tool_pose()
        await self.gripper(-0.5, h)
        res = await self.service('/simulation/attach_object', Trigger.Request(), h)
        if not res.success:
            raise RuntimeError(res.message)
        self.attached = True
        await self.attached_scene(True, h)
        before = await self.entity(self.object_name, h)
        await self.move_pose(self.pose(pos.x, pos.y, pos.z + 0.09), h)
        after = await self.entity(self.object_name, h)
        if after.position.z - before.position.z < 0.045:
            raise RuntimeError('box did not follow gripper during lift')
        await self.support_contact(self.pick_support, False, h)
        await self.move_joints([0.0] * 5, h)

    def support_for_pose(self, position):
        return 'pick_table' if abs(position.y) < .2 else 'place_table'

    async def place(self, pose, h):
        await self.refresh_fixtures(h)
        p = await self.transform(pose, 'map', h)
        pos = p.pose.position
        await self.move_pose(self.pose(pos.x, pos.y, pos.z + 0.09), h)
        support = self.support_for_pose(pos)
        await self.support_contact(support, True, h)
        await self.move_pose(self.pose(pos.x, pos.y, pos.z + self.release_clearance), h)
        self.last_place_pose = self.tool_pose()
        await self.gripper(0, h)
        res = await self.service('/simulation/detach_object', Trigger.Request(), h)
        if not res.success:
            raise RuntimeError(res.message)
        self.attached = False
        # The released cargo is still falling. Keep the arm stationary until
        # it settles before reintroducing it as a world collision object.
        released_stamp = self.get_clock().now().nanoseconds
        settling_deadline = monotonic() + 15.
        while self.get_clock().now().nanoseconds - released_stamp < 500_000_000:
            self.check(h, settling_deadline)
            await self.pause(.05)
        await self.attached_scene(False, h)
        await self.support_contact(support, False, h)
        await self.move_pose(self.pose(pos.x, pos.y, pos.z + 0.09), h)
        await self.move_joints([0.0] * 5, h)
        await self.pause(1.0)
        actual = await self.entity(self.object_name, h)
        self.world_box_pose = self.pose(actual.position.x, actual.position.y, actual.position.z)
        self.world_box_pose.pose = actual
        await self.refresh_fixtures(h)
        if math.hypot(actual.position.x - pos.x, actual.position.y - pos.y) > 0.09 or abs(actual.position.z - pos.z) > 0.04:
            raise RuntimeError('released object outside place table: ' + str(actual.position))

    async def finish_action(self, h, name, kind, operation):
        result = kind.Result()
        try:
            feedback = kind.Feedback()
            feedback.stage = 'EXECUTING'
            feedback.detail = 'actual simulation backend'
            if hasattr(feedback, 'progress'):feedback.progress = 0.
            h.publish_feedback(feedback)
            outcome = await operation()
            if hasattr(result, 'planning_time') and isinstance(outcome, tuple):
                for field, seconds in zip(['planning_time', 'execution_time'], outcome):
                    duration = getattr(result, field)
                    duration.sec = int(seconds)
                    duration.nanosec = int(seconds % 1 * 1e9)
            if hasattr(result, 'reached_pose') and isinstance(outcome, PoseStamped):
                result.reached_pose = outcome
            if hasattr(result, 'final_pose'):
                pose = self.data['odom'].pose.pose
                result.final_pose = self.pose(pose.position.x, pose.position.y, pose.position.z)
                result.final_pose.pose.orientation = pose.orientation
                x,y = self.stations[h.request.station]
                result.final_xy_error = float(math.hypot(pose.position.x-x, pose.position.y-y))
                q = pose.orientation
                result.final_yaw_error = float(abs(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))
            if hasattr(result, 'backend'):
                result.backend = 'moveit_simulation'
            if hasattr(result, 'executed_grasp_pose'):
                result.executed_grasp_pose = self.last_grasp_pose
            if hasattr(result, 'executed_place_pose'):
                result.executed_place_pose = self.last_place_pose
            result.success = True
            result.message = 'simulation operation verified'
            h.succeed()
        except Exception as exc:
            result.success = False
            result.error_code = error_code(exc, self.phase)
            result.message = str(exc)
            h.canceled() if h.is_cancel_requested else h.abort()
        finally:
            self.busy.discard(name)
        return result

    async def navigation(self, h):
        return await self.finish_action(h, '/navigation/navigate_station', NavigateStation, lambda : self.navigate(h.request.station, h, h.request.timeout.sec + h.request.timeout.nanosec/1e9 or 160.))

    async def detection(self, h):
        result = DetectObject.Result()
        try:
            if h.request.min_confidence > 0.95:
                raise RuntimeError('requested confidence exceeds color detector confidence')
            if h.request.target_class != 'red_box' or h.request.require_orientation:
                raise RuntimeError('simulation supports red_box position with known upright orientation only')
            result.pose = await self.transform(await self.detect(h), h.request.output_frame or 'map', h)
            result.success = True
            result.object_id = 'box'
            result.class_name = 'red_box'
            result.confidence = 0.95
            result.size.x = 0.04
            result.size.y = 0.04
            result.size.z = 0.05
            result.detector_backend = 'gazebo_rgbd_hsv'
            h.succeed()
        except Exception as exc:
            result.error_code = error_code(exc, self.phase)
            result.message = str(exc)
            h.canceled() if h.is_cancel_requested else h.abort()
        finally:
            self.busy.discard('/perception/detect_object')
        return result

    async def joint_action(self, h):
        async def operation():
            if len(h.request.joint_names) != 5 or set(h.request.joint_names) != set(ARM):
                raise RuntimeError('joint names must name the five arm joints exactly once')
            if len(h.request.positions) != 5:
                raise RuntimeError('joint position count mismatch')
            state = dict(zip(h.request.joint_names, h.request.positions))
            return await self.move_joints([state[n] for n in ARM], h, execute=h.request.execute,
                velocity_scale=h.request.velocity_scale, acceleration_scale=h.request.acceleration_scale)
        return await self.finish_action(h, '/manipulation/move_to_joint', MoveToJoint, operation)

    async def pose_action(self, h):
        async def operation():
            if h.request.planning_group not in ['', 'arm']:
                raise RuntimeError('only the arm group is supported')
            return await self.move_pose(h.request.target_pose, h, execute=h.request.execute,
                velocity_scale=h.request.velocity_scale, acceleration_scale=h.request.acceleration_scale,
                planner_id=h.request.planner_id)
        return await self.finish_action(h, '/manipulation/move_to_pose', MoveToPose, operation)

    async def named_action(self, h):
        async def operation():
            if h.request.name not in ['home', 'transport', 'observe']:
                raise RuntimeError('unknown simulation named pose')
            return await self.move_joints([0.] * 5, h, execute=h.request.execute,
                velocity_scale=h.request.velocity_scale, acceleration_scale=h.request.acceleration_scale)
        return await self.finish_action(h, '/manipulation/move_named_pose', MoveNamedPose, operation)

    def tool_pose(self):
        transform = self.tf.lookup_transform('map', 'end_effector_link', rclpy.time.Time()).transform
        pose = self.pose(transform.translation.x, transform.translation.y, transform.translation.z)
        pose.pose.orientation = transform.rotation
        return pose

    async def pick_action(self, h):
        return await self.finish_action(h, '/manipulation/pick_object', PickObject, lambda : self.pick(h.request.object_pose, h))

    async def place_action(self, h):
        return await self.finish_action(h, '/manipulation/place_object', PlaceObject, lambda : self.place(h.request.place_pose, h))

    async def task(self, h):
        result = ExecuteSortTask.Result()
        started, wall = monotonic(), time()
        self.deadline = started + 480.
        states, retries = [], {'navigation': 0, 'detection': 0}
        completed = 0

        async def step(name, operation, attempts=1):
            self.phase = name
            beginning = monotonic()
            try:
                for attempt in range(attempts):
                    self.check(h, self.deadline)
                    feedback = ExecuteSortTask.Feedback(current_state=name, progress=len(states)/7.,
                        detail='actual simulation operation', current_index=1,
                        navigation_retry=retries['navigation'], detection_retry=retries['detection'])
                    h.publish_feedback(feedback)
                    self.get_logger().info(f'[TASK] {name}, attempt={attempt+1}')
                    try:
                        return await operation()
                    except Interrupted:
                        raise
                    except RuntimeError:
                        if attempt+1 == attempts:
                            raise
                        key = 'navigation' if name.startswith('NAVIGATE') else 'detection'
                        retries[key] += 1
                        await self.pause(.3)
            finally:
                states.append((name, monotonic()-beginning))

        try:
            await step('TRANSPORT', lambda: self.move_joints([0.]*5, h))
            await step('NAVIGATE_PICK', lambda: self.navigate(h.request.pick_station, h), 2)
            pose = await step('DETECT_OBJECT', lambda: self.detect(h), 3)
            await step('PICK_OBJECT', lambda: self.pick(pose, h))
            await step('NAVIGATE_PLACE', lambda: self.navigate(h.request.place_station, h), 2)
            await step('PLACE_OBJECT', lambda: self.place(self.pose(.67, 1.5, .185), h))
            completed = 1
            if h.request.return_home:
                await step('RETURN_HOME', lambda: self.navigate('home', h), 2)
            await self.refresh_fixtures(h)
            result.success = True
            result.message = 'Nav2 -> RGB-D -> MoveIt pick -> carry -> place verified'
        except Exception as exc:
            result.error_code = int(ErrorCode.TASK_CANCELLED) if h.is_cancel_requested else error_code(exc, self.phase)
            result.message = str(exc)
            self.get_logger().error(result.message)
            self.cmd.publish(Twist())
        duration = monotonic()-started
        result.completed_quantity = completed
        result.total_duration.sec = int(duration)
        result.total_duration.nanosec = int(duration%1*1e9)
        try:
            self.csv_path.parent.mkdir(parents=True, exist_ok=True)
            exists = self.csv_path.exists()
            with self.csv_path.open('a', newline='') as stream:
                writer = csv.writer(stream)
                if not exists:
                    writer.writerow(['task_id','execution_mode','software_version','start_time','duration_s',
                        'success','completed_quantity','error_code','message','phase_seconds'])
                writer.writerow([h.request.task_id,'simulation','0.2.0',wall,duration,result.success,
                    completed,result.error_code,result.message,str(states)])
        except OSError as exc:
            result.success = False
            result.error_code = int(ErrorCode.CONFIG_INVALID)
            result.message = 'CSV write failed: '+str(exc)
        finally:
            self.deadline = None
            self.phase = 'IDLE' if result.success else 'FAULT'
            self.fault = '' if result.success else result.message
            self.last_error = int(result.error_code)
            self.busy.discard('/task/execute_sort')
        if h.is_cancel_requested:
            h.canceled()
        elif result.success:
            h.publish_feedback(ExecuteSortTask.Feedback(current_state='FINISHED', progress=1., current_index=1, detail=result.message))
            h.succeed()
        else:
            h.abort()
        return result

def main():
    rclpy.init()
    node = Runtime()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.cmd.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()
