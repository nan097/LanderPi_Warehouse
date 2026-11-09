#!/usr/bin/env python3
"""Multi-cargo warehouse sorting, using the verified simulation adapters."""
import csv
import json
import math
from pathlib import Path
from time import monotonic, time
import cv2
import numpy as np
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Bool
from scipy.spatial.transform import Rotation
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.msg import AllowedCollisionEntry, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.action import GoalResponse
from sort_runtime import Runtime, Interrupted, error_code
from landerpi_task_interfaces.action import ExecuteSortTask, DetectObject
from landerpi_task_manager.error_codes import ErrorCode
from warehouse_layout import inventory, fixture_parts, slot, inside, validate, dock, planning_parts


class WarehouseRuntime(Runtime):
    def __init__(self):
        share=Path(get_package_share_directory('landerpi_simulation'))
        self.config=yaml.safe_load((share/'config/warehouse.yaml').read_text())
        self.goods=validate(self.config)
        self.completed={}
        self.poses={}
        self.active_camera='A1'
        self.current_item=None
        self.current_output=None
        self.current_slot=None
        self.operator_enabled = False
        self.operator_stamp = 0.0
        super().__init__()
        for name,default in dict(joint_goal_tolerance=.006,arm_velocity_scale=.4,arm_acceleration_scale=.4,
                gripper_duration=1.,alignment_gain=.8,alignment_max_linear=.06,
                alignment_max_angular=.15).items():
            self.declare_parameter(name,default)
            value=float(self.get_parameter(name).value)
            if not math.isfinite(value) or value<=0 or (name.endswith('_scale') and value>1):
                raise ValueError(name+' is outside its valid range')
            setattr(self,name,value)
        self.get_logger().info(f'Warehouse motion arm={self.arm_velocity_scale:.2f}/{self.arm_acceleration_scale:.2f} gripper={self.gripper_duration:.2f}s')
        self.declare_parameter('require_operator', False)
        self.require_operator = self.get_parameter('require_operator').value
        self.create_subscription(Bool, "/warehouse/operator_enabled", self.operator_enable, 10, callback_group=self.group)
        # With position-only five-axis IK, keep the forearm above the resting
        # cargo envelope while gravity completes the final 80 mm descent.
        self.release_clearance=.08
        self.object_size=list(self.config['box_size'])
        self.ik_seeds=[[0.,-.8,-1.8,-2.,0.],[0.,-.8,-.6,-2.,0.],[0.,-.8,-.6,-1.,0.],[0.,-.5,-.8,-.8,0.]]
        self.stations={'home':tuple(self.config['start'])}
        self.stations.update({name:(x-self.config['dock_offset'],y) for name,(x,y) in self.config['racks'].items()})
        self.stations.update({name:(v['center'][0]-self.config['dock_offset'],v['center'][1]) for name,v in self.config['outputs'].items()})
        from sensor_msgs.msg import Image,CameraInfo
        from rclpy.qos import qos_profile_sensor_data
        self.active_rgb_pub=self.create_publisher(Image,'/warehouse/camera/image_raw',10)
        for typ,suffix,key in [(Image,'image_raw','rgb'),(Image,'depth/image_raw','depth'),(CameraInfo,'camera_info','info')]:
            self.create_subscription(typ,f'/warehouse_camera/camera/{suffix}',lambda msg,k=key:self.camera_receive(k,msg),qos_profile_sensor_data,callback_group=self.group)
        self.progress_pub=self.create_publisher(String,'/warehouse/progress',10)
        self.markers_pub=self.create_publisher(MarkerArray,'/warehouse/labels',10)
        self.create_timer(.5,self.publish_warehouse)
        self.get_logger().info(f'WAREHOUSE: six source boxes, three output boxes, {len(self.goods)} physical cargo')

    async def pick(self, pose, h):
        # Grip the exposed upper part. The tool frame lies above the jaw tips;
        # keep the gripper body over the rim while the fingers enter the cavity.
        p=await self.transform(pose,'map',h)
        q=p.pose.position
        await super().pick(self.pose(q.x,q.y,self.config['support_height']+self.config['container_inner'][2]+self.config['grasp_tool_clearance']),h)

    def camera_receive(self,key,msg):
        self.receive(key,msg)
        if key=='rgb':self.active_rgb_pub.publish(msg)

    def select(self,item):
        self.current_item=item
        self.object_name=item['id']
        self.pick_support=f"{item['rack']}_support"
        self.active_camera=item['rack']

    def check(self, h, deadline):
        super().check(h, deadline)
        if self.require_operator and h is not None and (not self.operator_enabled or monotonic()-self.operator_stamp > 0.6):
            raise Interrupted("gamepad authorization lost")

    def operator_enable(self, msg):
        self.operator_enabled = msg.data
        self.operator_stamp = monotonic()

    def accept(self,name,goal):
        if self.busy or self.fault or not self.ready:return GoalResponse.REJECT
        if name=='/task/execute_sort':
            if self.require_operator and (not self.operator_enabled or monotonic()-self.operator_stamp > 0.6):
                return GoalResponse.REJECT
            valid=(bool(goal.task_id) and goal.object_class in ('all',*self.config['colors']) and
                   goal.pick_station=='warehouse' and goal.place_station=='by_color' and
                   goal.quantity<=len(self.goods))
            if not valid:return GoalResponse.REJECT
        self.busy.add(name)
        return GoalResponse.ACCEPT

    def publish_warehouse(self):
        if not hasattr(self,'progress_pub'):return
        by_color={c:sum(self.completed.get(g['id'])==c for g in self.goods) for c in self.config['colors']}
        state=dict(total=len(self.goods),completed=len(self.completed),remaining=len(self.goods)-len(self.completed),
                   phase=self.phase,current_object=self.object_name if self.current_item else '',
                   source=self.current_item['rack'] if self.current_item else '',destination=self.current_output or '',
                   slot=self.current_slot,outputs=by_color,fault=self.fault,
                   camera='warehouse_camera_optical_frame', observed_rack=self.active_camera,
                   completed_ids=sorted(self.completed))
        self.progress_pub.publish(String(data=json.dumps(state)))
        markers=MarkerArray()
        centers={**self.config['racks'],**{k:v['center'] for k,v in self.config['outputs'].items()}}
        for idx,(name,(x,y)) in enumerate(centers.items()):
            m=Marker();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg()
            m.ns='warehouse_names';m.id=idx;m.type=Marker.TEXT_VIEW_FACING;m.action=Marker.ADD
            m.pose.position.x=x;m.pose.position.y=y;m.pose.position.z=.52;m.pose.orientation.w=1.
            m.scale.z=.105;m.color.r=m.color.g=m.color.b=m.color.a=1.
            if name in self.config['racks']:
                left=sum(g['rack']==name and g['id'] not in self.completed for g in self.goods)
                m.text=f'{name} [{left}/{sum(g["rack"]==name for g in self.goods)}]'
            else:
                color=self.config['outputs'][name]['color'];m.text=f'{name} [{by_color[color]}/{len(self.config["output_offsets"])}]'
                m.color.r,m.color.g,m.color.b=self.config['colors'][color]
            markers.markers.append(m)
        self.markers_pub.publish(markers)

    async def refresh_fixtures(self,h):
        req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.robot_state.is_diff=True
        collisions=[]
        for name,position,size in planning_parts(self.config):
            collisions.append(self.collision(name,await self.transform(self.pose(*position),'base_footprint',h),size))
        for item in self.goods:
            actual=await self.entity(item['id'],h)
            self.poses[item['id']]=actual
            if self.attached and item['id']==self.object_name:continue
            pose=self.pose(actual.position.x,actual.position.y,actual.position.z);pose.pose=actual
            collisions.append(self.collision(item['id'],await self.transform(pose,'base_footprint',h),self.config['box_size']))
        req.scene.world.collision_objects=collisions
        if not (await self.service('/apply_planning_scene',req,h)).success:raise RuntimeError('warehouse scene update failed')

    async def scene(self,pose,h):
        self.world_box_pose=await self.transform(pose,'map',h)
        await self.refresh_fixtures(h)
        await self.touch_contact(True,h)

    async def touch_contact(self,allowed,h):
        query=GetPlanningScene.Request(components=PlanningSceneComponents(components=128))
        acm=(await self.service('/get_planning_scene',query,h)).scene.allowed_collision_matrix
        if self.object_name not in acm.entry_names:
            for row in acm.entry_values:row.enabled.append(False)
            acm.entry_names.append(self.object_name)
            acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(acm.entry_names)))
        index=acm.entry_names.index(self.object_name)
        for i,name in enumerate(acm.entry_names):
            if name in ['link5','gripper_link','gripper_base_link','guide_rod_link','gripper_left_link','gripper_right_link','left_jaw_link','right_jaw_link','end_effector_link']:
                acm.entry_values[i].enabled[index]=allowed;acm.entry_values[index].enabled[i]=allowed
        req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.robot_state.is_diff=True;req.scene.allowed_collision_matrix=acm
        if not (await self.service('/apply_planning_scene',req,h)).success:raise RuntimeError('warehouse grasp scene failed')

    async def navigate(self,station,h,timeout=300.):
        return await super().navigate(station,h,timeout)

    async def align(self,station,h,xy_tolerance=.010,yaw_tolerance=.015,timeout=90.):
        # Physics and rendering may run below real time in a VM. Keep a bounded
        # docking deadline with adequate time for measured feedback convergence.
        return await super().align(station,h,xy_tolerance,yaw_tolerance,timeout)

    def support_for_pose(self,position):
        if not self.current_output:raise RuntimeError('no reserved destination')
        return f'{self.current_output}_support'

    async def detect(self,h):
        if not self.current_item:raise RuntimeError('select a warehouse cargo before detection')
        requested=self.get_clock().now().nanoseconds/1e9;deadline=monotonic()+25
        def fresh():
            if not all(k in self.data for k in ('rgb','depth','info')):return False
            stamps=[self.data[k].header.stamp.sec+self.data[k].header.stamp.nanosec/1e9 for k in ('rgb','depth')]
            return min(stamps)>requested and abs(stamps[0]-stamps[1])<.05 and all(monotonic()-self.stamps.get(k,0)<3 for k in ('rgb','depth','info'))
        while not fresh():self.check(h,deadline);await self.pause()
        rgb=self.bridge.imgmsg_to_cv2(self.data['rgb'],'bgr8');hsv=cv2.cvtColor(rgb,cv2.COLOR_BGR2HSV)
        color=self.current_item['color']
        ranges={'red':[(0,12),(170,180)],'green':[(35,85)],'blue':[(95,135)]}[color]
        mask=np.zeros(hsv.shape[:2],np.uint8)
        for lo,hi in ranges:mask|=cv2.inRange(hsv,(lo,110,50),(hi,255,255))
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        if not contours:raise RuntimeError(f'{color} object not found in RGB-D at {self.active_camera}')
        contour=max(contours,key=cv2.contourArea)
        if cv2.contourArea(contour)<8:raise RuntimeError('color candidate too small')
        selected=np.zeros_like(mask);cv2.drawContours(selected,[contour],-1,255,-1)
        selected=cv2.erode(selected,np.ones((3,3),np.uint8))
        depth=self.bridge.imgmsg_to_cv2(self.data['depth'],'passthrough')
        v,u=np.nonzero((selected>0)&np.isfinite(depth)&(depth>.08)&(depth<4))
        if len(u)<12:raise RuntimeError('insufficient valid cargo depth pixels')
        distance=depth[v,u];info=self.data['info']
        points=np.column_stack(((u-info.k[2])*distance/info.k[0],(v-info.k[5])*distance/info.k[4],distance))
        stamp=rclpy.time.Time.from_msg(self.data['rgb'].header.stamp)
        deadline=monotonic()+8
        while not self.tf.can_transform('map',info.header.frame_id,stamp):self.check(h,deadline);await self.pause()
        transform=self.tf.lookup_transform('map',info.header.frame_id,stamp).transform
        q=transform.rotation;translation=transform.translation
        world=Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(points)+[translation.x,translation.y,translation.z]
        sx,sy,sz=self.config['box_size']
        # Box walls obscure the front face; estimate center from the visible top.
        top=world[world[:,2]>=np.percentile(world[:,2],85)-.002]
        center=[float((np.percentile(top[:,0],5)+np.percentile(top[:,0],95))/2),float((np.percentile(top[:,1],5)+np.percentile(top[:,1],95))/2),float(np.median(top[:,2])-sz/2)]
        pose=self.pose(*center)
        if math.dist(center,self.current_item['position'])>.045:raise RuntimeError(f'RGB-D cargo outside selected rack slot: {center}')
        self.get_logger().info(f"[WAREHOUSE RGB-D] {self.object_name} color={color} center={center}, camera=onboard")
        return pose

    async def detection(self,h):
        result=DetectObject.Result()
        try:
            color=h.request.target_class.removesuffix('_box')
            if color not in self.config['colors'] or h.request.require_orientation:raise RuntimeError('warehouse supports red/green/blue box positions')
            if h.request.min_confidence>.95:raise RuntimeError('requested confidence exceeds color detector confidence')
            candidates=[g for g in self.goods if g['rack']==self.active_camera and g['color']==color and g['id'] not in self.completed]
            if not candidates:raise RuntimeError('selected color already sorted or unavailable')
            self.select(candidates[0]);result.pose=await self.transform(await self.detect(h),h.request.output_frame or 'map',h)
            result.success=True;result.object_id=self.object_name;result.class_name=color+'_box';result.confidence=.95
            result.size.x,result.size.y,result.size.z=self.config['box_size'];result.detector_backend='warehouse_rgbd_hsv';h.succeed()
        except Exception as exc:
            result.error_code=error_code(exc,self.phase);result.message=str(exc);h.canceled() if h.is_cancel_requested else h.abort()
        finally:self.busy.discard('/perception/detect_object')
        return result

    async def verify_inventory(self,h):
        # Physical model positions are the authority, including after runtime restart.
        completed={}
        for item in self.goods:
            actual=await self.entity(item['id'],h);self.poses[item['id']]=actual
            position=[actual.position.x,actual.position.y,actual.position.z]
            for output,description in self.config['outputs'].items():
                if inside(self.config,output,position):
                    if description['color']!=item['color']:raise RuntimeError('cargo found in wrong destination: '+item['id'])
                    completed[item['id']]=item['color'];break
        self.completed=completed
        for output,description in self.config['outputs'].items():
            actual=[self.poses[g['id']].position for g in self.goods if self.completed.get(g['id'])==description['color']]
            for i,p in enumerate(actual):
                for q in actual[i+1:]:
                    if math.dist([p.x,p.y],[q.x,q.y])<self.config['cargo_min_spacing']:raise RuntimeError('destination cargo overlap or insufficient spacing')
        self.publish_warehouse()

    def reserve_slot(self,output):
        used=[]
        for item in self.goods:
            if self.completed.get(item['id'])==self.config['outputs'][output]['color']:
                used.append(self.poses[item['id']].position)
        for index in range(len(self.config['output_offsets'])):
            target=slot(self.config,output,index)
            if all(math.dist([p.x,p.y],target[:2])>self.config['cargo_min_spacing'] for p in used):return index,target
        raise RuntimeError('destination capacity exhausted')

    async def task(self,h):
        result=ExecuteSortTask.Result();started=monotonic();wall=time();states=[];completed=0
        self.deadline=started+max(1,len(self.goods))*420.
        async def step(name,operation,index,total,attempts=1):
            self.phase=name;begin=monotonic()
            try:
                for attempt in range(attempts):
                    self.check(h,self.deadline)
                    detail=f"{self.object_name}: {self.current_item['rack'] if self.current_item else 'START'} -> {self.current_output or 'HOME'}; {completed}/{total}"
                    h.publish_feedback(ExecuteSortTask.Feedback(current_state=name,progress=completed/max(total,1),current_index=index,detail=detail))
                    self.get_logger().info(f'[WAREHOUSE] {name} {detail} attempt={attempt+1}')
                    try:return await operation()
                    except Interrupted:raise
                    except RuntimeError:
                        if attempt+1==attempts:raise
                        await self.pause(.3)
            finally:states.append((index,name,monotonic()-begin))
        try:
            await self.verify_inventory(h)
            pending=[g for g in self.goods if g['id'] not in self.completed and h.request.object_class in ('all',g['color'])]
            wanted=h.request.quantity or len(pending)
            if wanted>len(pending):raise RuntimeError('requested quantity exceeds unsorted inventory')
            # Nearest-source scheduling reduces empty travel; color determines output.
            total=wanted
            processed=[]
            await step('TRANSPORT',lambda:self.move_joints([0.]*5,h),0,total)
            for index in range(1,wanted+1):
                position=self.data['odom'].pose.pose.position
                item=min(pending,key=lambda g:math.dist([position.x,position.y],[g['position'][0]-self.config['dock_offset']-self.config['cargo_x_offset'],g['position'][1]]))
                pending.remove(item);self.select(item)
                output=next(k for k,v in self.config['outputs'].items() if v['color']==item['color'])
                self.current_output=output;self.current_slot,target=self.reserve_slot(output)
                source=f"{item['rack']}_{item['color']}";self.stations[source]=tuple(dock(self.config,item['position']))
                destination=f'{output}_{self.current_slot}';self.stations[destination]=tuple(dock(self.config,target,self.config['outputs'][output]['center']))
                await step('NAVIGATE_PICK',lambda:self.navigate(source,h),index,total,2)
                await self.refresh_fixtures(h)
                await step('OBSERVE_OBJECT',lambda:self.move_joints(self.config['observation_joints'],h),index,total)
                pose=await step('DETECT_OBJECT',lambda:self.detect(h),index,total,3)
                await step('PICK_OBJECT',lambda:self.pick(pose,h),index,total)
                await step('NAVIGATE_PLACE',lambda:self.navigate(destination,h),index,total,2)
                await step('PLACE_OBJECT',lambda:self.place(self.pose(*target),h),index,total)
                # Released cargo remains an obstacle for subsequent objects.
                await self.touch_contact(False,h)
                actual=await self.entity(item['id'],h)
                if not inside(self.config,output,[actual.position.x,actual.position.y,actual.position.z]):raise RuntimeError('released cargo outside physical destination')
                self.completed[item['id']]=item['color'];self.poses[item['id']]=actual;completed+=1;self.publish_warehouse()
                processed.append(item['id'])
                self.get_logger().info(f'[WAREHOUSE VERIFIED] {item["id"]} -> {output} slot={self.current_slot}, completed={len(self.completed)}/{len(self.goods)}')
            if h.request.return_home:await step('RETURN_HOME',lambda:self.navigate('home',h),wanted,total,2)
            await self.refresh_fixtures(h);await self.verify_inventory(h)
            if any(name not in self.completed for name in processed):
                raise RuntimeError('previously placed cargo moved outside its destination')
            if h.request.object_class=='all' and h.request.quantity==0 and len(self.completed)!=len(self.goods):
                raise RuntimeError('warehouse still contains unsorted cargo after full task')
            result.success=True;result.message=f'Physically verified {completed} sorted cargo; {len(self.completed)}/{len(self.goods)} warehouse complete'
        except Exception as exc:
            result.error_code=int(ErrorCode.TASK_CANCELLED) if h.is_cancel_requested else error_code(exc,self.phase)
            result.message=str(exc);self.get_logger().error(result.message)
            from geometry_msgs.msg import Twist
            self.cmd.publish(Twist())
        duration=monotonic()-started;result.completed_quantity=completed
        result.total_duration.sec=int(duration);result.total_duration.nanosec=int(duration%1*1e9)
        try:
            self.csv_path.parent.mkdir(parents=True,exist_ok=True);exists=self.csv_path.exists()
            with self.csv_path.open('a',newline='') as f:
                writer=csv.writer(f)
                if not exists:writer.writerow(['task_id','execution_mode','software_version','start_time','duration_s','success','completed_quantity','error_code','message','phase_seconds'])
                writer.writerow([h.request.task_id,'warehouse_simulation','0.3.0',wall,duration,result.success,completed,result.error_code,result.message,str(states)])
        except OSError as exc:result.success=False;result.error_code=int(ErrorCode.CONFIG_INVALID);result.message='CSV write failed: '+str(exc)
        finally:
            self.deadline=None;self.phase='FINISHED' if result.success else 'FAULT';self.fault='' if result.success else result.message
            self.last_error=int(result.error_code);self.busy.discard('/task/execute_sort');self.publish_warehouse()
        if h.is_cancel_requested:h.canceled()
        elif result.success:
            h.publish_feedback(ExecuteSortTask.Feedback(current_state='FINISHED',progress=1.,current_index=completed,detail=result.message));h.succeed()
        else:h.abort()
        return result


def main():
    rclpy.init();node=WarehouseRuntime()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
