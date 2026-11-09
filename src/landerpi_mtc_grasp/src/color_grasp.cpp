// Simulation-only visual grasp. MTC owns trajectory planning and execution.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/introspection.h>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <atomic>
#include <std_srvs/srv/trigger.hpp>
#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <chrono>
#include <mutex>
#include <thread>
#include <cmath>
#include <map>
#include <vector>
#include <iostream>

namespace mtc = moveit::task_constructor;
using namespace std::chrono_literals;
using Joints = std::map<std::string, double>;
struct PlanningFailure : std::runtime_error {using std::runtime_error::runtime_error;};

class ColorGrasp {
  rclcpp::Node::SharedPtr node_;
  std::mutex mutex_;
  geometry_msgs::msg::PoseStamped target_;
  moveit_msgs::msg::PlanningScene observed_;
  sensor_msgs::msg::JointState joints_;
  bool got_target_=false, got_scene_=false;
  std::string color_, object_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr scene_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  std::unique_ptr<mtc::Task> task_;
  moveit::core::RobotModelConstPtr model_;
  std::shared_ptr<mtc::solvers::PipelinePlanner> arm_planner_;
  std::shared_ptr<mtc::solvers::JointInterpolationPlanner> gripper_planner_;
  double velocity_scale_, acceleration_scale_, planned_seconds_=0.;

  size_t executed_segments_=0;
  std::chrono::steady_clock::time_point cycle_wall_;
  double cycle_sim_=0.;
  bool hardware_=false;
  std::atomic<bool> requested_{false},running_{false},cancelled_{false},operator_enabled_{false},vision_ready_{false};
  std::atomic<double> operator_time_{0.},joint_time_{0.},vision_time_{0.};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr operator_sub_,vision_ready_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_,cancel_service_;
  static double steadySeconds() {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  void checkControl() {
    if (hardware_ && running_ && (cancelled_ || !operator_enabled_ || steadySeconds()-operator_time_>.6 || steadySeconds()-joint_time_>1.))
      throw std::runtime_error("operator cancelled, gamepad disconnected or actual joint feedback stale");
  }
  const std::vector<std::string> touch_{"link5", "end_effector_link", "gripper_link", "gripper_base_link",
      "guide_rod_link", "gripper_left_link", "gripper_right_link", "left_jaw_link", "right_jaw_link"};

  template<class Service>
  typename Service::Response::SharedPtr call(const std::string& name,
      const std::shared_ptr<typename Service::Request>& req, std::chrono::seconds timeout=30s) {
    checkControl();
    auto client=node_->create_client<Service>(name);
    if (!client->wait_for_service(timeout)) throw std::runtime_error("service unavailable: "+name);
    auto future=client->async_send_request(req);
    const auto deadline=std::chrono::steady_clock::now()+timeout;
    while (future.wait_for(50ms)!=std::future_status::ready) {
      checkControl();
      if (std::chrono::steady_clock::now()>deadline) throw std::runtime_error("service timeout: "+name);
    }
    checkControl();
    return future.get();
  }
  void trigger(const std::string& name) {
    auto res=call<std_srvs::srv::Trigger>(name,std::make_shared<std_srvs::srv::Trigger::Request>());
    if (!res->success) throw std::runtime_error(name+": "+res->message);
  }
  void report(const std::string& text) {
    RCLCPP_INFO(node_->get_logger(), "%s", text.c_str());
    std_msgs::msg::String msg; msg.data=text; status_->publish(msg);
  }
  geometry_msgs::msg::Pose entity(const std::string& frame="world") {
    auto req=std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    req->name=object_; req->reference_frame=frame;
    auto res=call<gazebo_msgs::srv::GetEntityState>("/gazebo/get_entity_state",req);
    if (!res->success) throw std::runtime_error("verification entity unavailable");
    return res->state.pose;
  }
  Joints ik(geometry_msgs::msg::PoseStamped pose) {
    auto req=std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    auto& ik=req->ik_request; ik.group_name="arm"; ik.ik_link_name="end_effector_link";
    ik.pose_stamped=pose; ik.avoid_collisions=true; ik.timeout.sec=2;
    {std::lock_guard<std::mutex> lock(mutex_); ik.robot_state.joint_state=joints_;}
    ik.robot_state.is_diff=true;
    const std::vector<std::vector<double>> seeds{{},{0.,-.8,-1.8,-2.,0.},{0.,-.8,-.6,-2.,0.},{0.,-.8,-.6,-1.,0.},{0.,-.5,-.8,-.8,0.},{0.,-.8,-.5,-.8,0.},{0.,-1.,.3,-1.4,0.}};
    for (const auto& seed:seeds) {
      if (!seed.empty()) {
        for (size_t i=0;i<ik.robot_state.joint_state.name.size();++i)
          for (size_t j=0;j<5;++j)
            if (ik.robot_state.joint_state.name[i]=="joint"+std::to_string(j+1))
              ik.robot_state.joint_state.position[i]=seed[j];
      }
      auto res=call<moveit_msgs::srv::GetPositionIK>("/compute_ik",req);
      if (res->error_code.val!=1) continue;
      Joints result;
      for (size_t i=0;i<res->solution.joint_state.name.size();++i)
        for (size_t j=1;j<=5;++j)
          if (res->solution.joint_state.name[i]=="joint"+std::to_string(j))
            result[res->solution.joint_state.name[i]]=res->solution.joint_state.position[i];
      if (result.size()==5) {
        return result;
      }
    }
    throw std::runtime_error("No collision-free position IK for visual target");
  }
  void startTask(const std::string& name) {
    task_=std::make_unique<mtc::Task>(); task_->stages()->setName(name);
    if (model_) task_->setRobotModel(model_);
    else {
      task_->loadRobotModel(node_);model_=task_->getRobotModel();
      arm_planner_=std::make_shared<mtc::solvers::PipelinePlanner>(node_);
      arm_planner_->setPlannerId("RRTConnect");
      arm_planner_->setMaxVelocityScalingFactor(velocity_scale_);
      arm_planner_->setMaxAccelerationScalingFactor(acceleration_scale_);
      gripper_planner_=std::make_shared<mtc::solvers::JointInterpolationPlanner>();
      gripper_planner_->setMaxVelocityScalingFactor(node_->get_parameter("gripper_velocity_scale").as_double());
      gripper_planner_->setMaxAccelerationScalingFactor(node_->get_parameter("gripper_acceleration_scale").as_double());
    }
    task_->setProperty("group",std::string("arm"));
    task_->setProperty("eef",std::string("gripper"));
    task_->setProperty("ik_frame",std::string("end_effector_link"));
    task_->add(std::make_unique<mtc::stages::CurrentState>("current simulation state"));
  }
  void armStage(const std::string& name, const Joints& goal) {
    auto stage=std::make_unique<mtc::stages::MoveTo>(name,arm_planner_);
    mtc::TrajectoryExecutionInfo execution;execution.controller_names={"arm_controller"};
    stage->setTrajectoryExecutionInfo(execution);
    stage->setGroup("arm"); stage->setGoal(goal); stage->setTimeout(10.);
    task_->add(std::move(stage));
  }
  void gripperStage(const std::string& name, const std::string& goal) {
    auto stage=std::make_unique<mtc::stages::MoveTo>(name,gripper_planner_);
    mtc::TrajectoryExecutionInfo execution;execution.controller_names={"gripper_controller"};
    stage->setTrajectoryExecutionInfo(execution);
    stage->setGroup("gripper"); stage->setGoal(goal); task_->add(std::move(stage));
  }
  void execute() {
    checkControl();
    task_->init();
    if (!task_->plan(1) || task_->solutions().empty()) {task_->explainFailure(std::cerr);throw PlanningFailure("MTC task planning failed");}
    task_->introspection().publishSolution(*task_->solutions().front());
    using Execute=moveit_task_constructor_msgs::action::ExecuteTaskSolution;
    Execute::Goal goal;
    task_->solutions().front()->toMsg(goal.solution,&task_->introspection());
    // CurrentState serializes a complete scene as its zero-motion first step.
    // The live scene already matches that snapshot. Replaying it with an
    // attached cube in Humble detaches/re-adds the same ID into the world.
    // Keep every planned motion and intentional attach/detach stage intact.
    auto& parts=goal.solution.sub_trajectory;
    if (!parts.empty() && parts.front().trajectory.joint_trajectory.points.empty() &&
        parts.front().trajectory.multi_dof_joint_trajectory.points.empty()) parts.erase(parts.begin());
    for (const auto& part:goal.solution.sub_trajectory) {
      const auto& points=part.trajectory.joint_trajectory.points;
      if (!points.empty()) {
        const auto duration=points.back().time_from_start;
        planned_seconds_+=duration.sec+duration.nanosec*1e-9;
        ++executed_segments_;
      }
    }
    auto client=rclcpp_action::create_client<Execute>(node_,"/execute_task_solution");
    if (!client->wait_for_action_server(10s)) throw std::runtime_error("MTC execution capability unavailable");
    checkControl();
    auto sent=client->async_send_goal(goal);
    if (sent.wait_for(15s)!=std::future_status::ready) throw std::runtime_error("MTC goal handshake timeout");
    auto handle=sent.get();if (!handle) throw std::runtime_error("MTC solution rejected");
    auto result=client->async_get_result(handle);
    const auto deadline=std::chrono::steady_clock::now()+180s;
    try {
      while (result.wait_for(50ms)!=std::future_status::ready) {
        checkControl();
        if (std::chrono::steady_clock::now()>deadline) throw std::runtime_error("MTC execution timeout");
      }
      checkControl();
    } catch (...) {
      client->async_cancel_goal(handle);result.wait_for(3s);throw;
    }
    auto response=result.get();
    if (response.code!=rclcpp_action::ResultCode::SUCCEEDED || response.result->error_code.val!=1)
      throw std::runtime_error("MTC task execution failed");
  }

public:
  explicit ColorGrasp(const rclcpp::Node::SharedPtr& node):node_(node) {
    for (const auto& parameter:std::map<std::string,double>{{"velocity_scale",.7},{"acceleration_scale",.65},
         {"gripper_velocity_scale",.8},{"gripper_acceleration_scale",.8}}) {
      if (!node_->has_parameter(parameter.first)) node_->declare_parameter(parameter.first,parameter.second);
      const double value=node_->get_parameter(parameter.first).as_double();
      if (!std::isfinite(value) || value<=0. || value>1.) throw std::runtime_error(parameter.first+" must be in (0,1]");
    }
    velocity_scale_=node_->get_parameter("velocity_scale").as_double();
    acceleration_scale_=node_->get_parameter("acceleration_scale").as_double();
    if (!node_->has_parameter("hardware_mode")) node_->declare_parameter("hardware_mode",false);
    hardware_=node_->get_parameter("hardware_mode").as_bool();
    for(const auto& [name,value]:std::map<std::string,double>{{"place_x",.08},{"place_y",.24},{"place_z",.07},{"observe_joint2",.1},{"observe_joint3",-.8},{"observe_joint4",-2.}})
      if(!node_->has_parameter(name))node_->declare_parameter(name,value);
    color_=node_->get_parameter("target_color").as_string(); object_="cargo_"+color_;
    if (color_!="red" && color_!="green" && color_!="blue") throw std::runtime_error("invalid target_color");
    status_=node_->create_publisher<std_msgs::msg::String>("/mtc_grasp/status",rclcpp::QoS(1).transient_local());
    target_sub_=node_->create_subscription<geometry_msgs::msg::PoseStamped>("/mtc_grasp/target_pose",10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg){std::lock_guard<std::mutex> lock(mutex_);target_=*msg;got_target_=true;});
    scene_sub_=node_->create_subscription<moveit_msgs::msg::PlanningScene>("/mtc_grasp/observed_scene",10,
      [this](moveit_msgs::msg::PlanningScene::SharedPtr msg){std::lock_guard<std::mutex> lock(mutex_);observed_=*msg;got_scene_=true;});
    joints_sub_=node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex_);
        joints_=sensor_msgs::msg::JointState(); joints_.header=msg->header; joint_time_=steadySeconds();
        for (size_t i=0;i<msg->name.size() && i<msg->position.size();++i) {
          const auto& name=msg->name[i];
          if (name.size()>=6 && name.compare(name.size()-6,6,"_mimic")==0) continue;
          joints_.name.push_back(name);joints_.position.push_back(msg->position[i]);
        }
      });
  }
  void hardwareLoop() {
    report("WAITING_FOR_REAL_CONTROLLERS");
    auto planning_scene=node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    auto execution=rclcpp_action::create_client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>(node_,"/execute_task_solution");
    if (!planning_scene->wait_for_service(90s) || !execution->wait_for_action_server(90s))
      throw std::runtime_error("real MoveIt planning/execution stack not ready");
    operator_sub_=node_->create_subscription<std_msgs::msg::Bool>("/mtc_grasp/operator_enabled",10,
      [this](std_msgs::msg::Bool::SharedPtr msg){operator_enabled_=msg->data;operator_time_=steadySeconds();});
    vision_ready_sub_=node_->create_subscription<std_msgs::msg::Bool>("/mtc_grasp/vision_ready",10,
      [this](std_msgs::msg::Bool::SharedPtr msg){vision_ready_=msg->data;vision_time_=steadySeconds();});
    start_service_=node_->create_service<std_srvs::srv::Trigger>("/mtc_grasp/start",
      [this](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr res){
        res->success=!running_ && !requested_ && !cancelled_ && operator_enabled_ && steadySeconds()-operator_time_<.6 &&
          steadySeconds()-joint_time_<1. && vision_ready_ && steadySeconds()-vision_time_<1.;
        if (res->success) requested_=true;
        res->message=res->success?"MTC real hardware task queued":"busy, faulted or real sensors/operator not ready";
      });
    cancel_service_=node_->create_service<std_srvs::srv::Trigger>("/mtc_grasp/cancel",
      [this](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr res){
        if (running_) cancelled_=true;
        requested_=false;if (!running_) report("WAITING_FOR_START real_hardware=true");res->success=true;res->message="cancel requested";
      });
    while (rclcpp::ok()) {
      report("WAITING_FOR_START real_hardware=true");
      while (rclcpp::ok() && !requested_) std::this_thread::sleep_for(50ms);
      if (!rclcpp::ok()) return;
      running_=true;requested_=false;
      try {runHardware();running_=false;}
      catch (const std::exception& error) {
        running_=false;cancelled_=true;report(std::string("FAULT real_hardware=true: ")+error.what());
        while (rclcpp::ok()) std::this_thread::sleep_for(100ms);
      }
    }
  }
  bool hardwareMode() const {return hardware_;}
  void runHardware() {
    checkControl();planned_seconds_=0.;executed_segments_=0;
    for (const auto& name:{"/arm_controller/follow_joint_trajectory","/gripper_controller/follow_joint_trajectory"}) {
      auto client=rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(node_,name);
      if (!client->wait_for_action_server(5s)) throw std::runtime_error(std::string("real controller unavailable: ")+name);
    }
    report("HARDWARE_OBSERVE");
    startTask("real arm camera observation");
    armStage("observe real ground cubes",{{"joint1",0.},{"joint2",.1},{"joint3",-.8},{"joint4",-2.},{"joint5",0.}});execute();
    {std::lock_guard<std::mutex> lock(mutex_);got_target_=false;got_scene_=false;}
    const auto stamp=node_->now();report("WAITING_FOR_RGBD real_hardware=true color="+color_);
    geometry_msgs::msg::PoseStamped target;moveit_msgs::msg::PlanningScene scene;
    const auto deadline=std::chrono::steady_clock::now()+30s;
    while (rclcpp::ok()) {
      checkControl();
      {std::lock_guard<std::mutex> lock(mutex_);
       if (got_target_ && got_scene_ && rclcpp::Time(target_.header.stamp)>stamp &&
           (node_->now()-rclcpp::Time(target_.header.stamp)).seconds()<1.) {target=target_;scene=observed_;break;}}
      if (std::chrono::steady_clock::now()>deadline) throw std::runtime_error("fresh real RGB-D target timeout");
      std::this_thread::sleep_for(50ms);
    }
    checkControl();
    moveit_msgs::msg::CollisionObject ground;ground.header.frame_id="map";ground.id="mtc_ground";ground.operation=ground.ADD;
    shape_msgs::msg::SolidPrimitive floor;floor.type=floor.BOX;floor.dimensions={4.,4.,.02};
    geometry_msgs::msg::Pose floor_pose;floor_pose.orientation.w=1.;floor_pose.position.z=-.012;
    ground.primitives={floor};ground.primitive_poses={floor_pose};scene.world.collision_objects.push_back(ground);
    collision_detection::AllowedCollisionMatrix acm(taskModelACM());
    for (const auto& link:touch_) acm.setEntry(object_,link,true);
    acm.setEntry(object_,"mtc_ground",true);acm.getMessage(scene.allowed_collision_matrix);
    auto update=std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();update->scene=scene;
    if (!call<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene",update)->success) throw std::runtime_error("real visual scene rejected");
    target.pose.position.z+=.024;auto approach=target;approach.pose.position.z+=.075;
    const auto approach_joints=ik(approach),grasp_joints=ik(target);
    startTask("real color cube approach and grip");gripperStage("open real gripper","open");
    armStage("approach real cube",approach_joints);armStage("grasp real cube",grasp_joints);gripperStage("close real gripper","close");
    auto attach=std::make_unique<mtc::stages::ModifyPlanningScene>("model presumed held cube");
    attach->attachObject(object_,"end_effector_link");task_->add(std::move(attach));execute();
    auto held_query=std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();held_query->components.components=0xffff;
    const auto held=call<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene",held_query)->scene;
    bool modeled=false;
    for (const auto& body:held.robot_state.attached_collision_objects) if (body.object.id==object_) modeled=true;
    if (!modeled) throw std::runtime_error("real planning attachment missing");
    for (const auto& object:held.world.collision_objects) if (object.id==object_) {
      auto remove=std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
      remove->scene.is_diff=true;remove->scene.robot_state.is_diff=true;
      moveit_msgs::msg::CollisionObject erased;erased.id=object_;erased.operation=erased.REMOVE;remove->scene.world.collision_objects={erased};
      if (!call<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene",remove)->success) throw std::runtime_error("real attachment scene synchronization failed");
    }
    auto lift=target;lift.pose.position.z+=.09;
    auto lift_joints=ik(lift);startTask("real cube lift");armStage("lift real gripper",lift_joints);execute();
    report("HARDWARE_LIFT_EXECUTED grasp_verified=false");
    geometry_msgs::msg::PoseStamped storage;storage.header.frame_id="base_footprint";storage.pose.orientation.w=1.;
    storage.pose.position.x=.08;storage.pose.position.y=.24;storage.pose.position.z=.07;
    auto above=storage;above.pose.position.z+=.08;
    auto above_joints=ik(above),release_joints=ik(storage);
    startTask("real left ground placement");armStage("above left ground",above_joints);armStage("lower to left ground",release_joints);
    gripperStage("release real gripper","open");
    auto detach=std::make_unique<mtc::stages::ModifyPlanningScene>("remove presumed held cube");
    detach->detachObject(object_,"end_effector_link");task_->add(std::move(detach));execute();
    startTask("real arm retreat");armStage("retreat from real ground",above_joints);
    armStage("home real arm",{{"joint1",0.},{"joint2",0.},{"joint3",0.},{"joint4",0.},{"joint5",0.}});execute();
    report("SUCCESS color="+color_+" real_hardware=true execution_complete=true grasp_verified=false");
  }
  void run() {
    report("WAITING_FOR_CONTROLLERS");
    for (const auto& name:{"/arm_controller/follow_joint_trajectory", "/gripper_controller/follow_joint_trajectory"}) {
      auto client=rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(node_,name);
      if (!client->wait_for_action_server(120s)) throw std::runtime_error(std::string("controller not ready: ")+name);
    }
    cycle_wall_=std::chrono::steady_clock::now();cycle_sim_=node_->now().seconds();
    report("CYCLE_START");
    // Move the arm-mounted camera into the calibrated ground observation pose.
    const auto feedback_deadline=std::chrono::steady_clock::now()+30s;
    while (rclcpp::ok()) {
      {std::lock_guard<std::mutex> lock(mutex_);if (joints_.name.size()>=6) break;}
      if (std::chrono::steady_clock::now()>feedback_deadline) throw std::runtime_error("joint feedback unavailable");
      std::this_thread::sleep_for(100ms);
    }
    trigger("/simulation/hold_base");
    startTask("position arm-mounted camera for ground observation");
    armStage("observe ground cubes",{{"joint1",0.},{"joint2",node_->get_parameter("observe_joint2").as_double()},{"joint3",node_->get_parameter("observe_joint3").as_double()},{"joint4",node_->get_parameter("observe_joint4").as_double()},{"joint5",0.}});
    execute();
    {std::lock_guard<std::mutex> lock(mutex_);got_target_=false;got_scene_=false;}
    const auto observation_stamp=node_->now();
    report("WAITING_FOR_RGBD color="+color_);
    auto deadline=std::chrono::steady_clock::now()+180s;
    geometry_msgs::msg::PoseStamped target;
    moveit_msgs::msg::PlanningScene scene;
    while (rclcpp::ok()) {
      {std::lock_guard<std::mutex> lock(mutex_);
       auto age=(node_->now()-rclcpp::Time(target_.header.stamp)).seconds();
       if (got_target_ && got_scene_ && joints_.name.size()>=6 && age>=0 && age<2. && rclcpp::Time(target_.header.stamp)>observation_stamp) {
         target=target_;scene=observed_;break;
       }}
      if (std::chrono::steady_clock::now()>deadline) throw std::runtime_error("fresh RGB-D or joint feedback timeout");
      std::this_thread::sleep_for(100ms);
    }
    if (!rclcpp::ok()) return;
    trigger("/simulation/hold_base");
    moveit_msgs::msg::CollisionObject ground;
    ground.header.frame_id="map";ground.id="mtc_ground";ground.operation=ground.ADD;
    shape_msgs::msg::SolidPrimitive box;box.type=box.BOX;box.dimensions={4.,4.,.02};
    geometry_msgs::msg::Pose floor;floor.orientation.w=1.;floor.position.z=-.012;
    ground.primitives={box};ground.primitive_poses={floor};scene.world.collision_objects.push_back(ground);
    // Preserve SRDF self-collision rules. Only the target may touch the gripper and support.
    auto acm=std::make_unique<collision_detection::AllowedCollisionMatrix>(taskModelACM());
    for (const auto& link:touch_) acm->setEntry(object_,link,true);
    acm->setEntry(object_,"mtc_ground",true);
    acm->getMessage(scene.allowed_collision_matrix);
    auto apply=std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();apply->scene=scene;
    if (!call<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene",apply)->success)
      throw std::runtime_error("visual planning scene rejected");
    // Use the upper part of the cube so the jaws remain above the floor.
    // The camera may move after this snapshot: all detections are already in map.
    target.pose.position.z+=.024;
    auto pre=target;pre.pose.position.z+=.075;
    auto pre_joints=ik(pre);auto grasp_joints=ik(target);
    report("MTC_PLAN_APPROACH color="+color_);
    startTask("visual "+color_+" cube: approach and grip");
    gripperStage("open gripper","open");
    armStage("move above visual target",pre_joints);
    armStage("approach visual target",grasp_joints);
    gripperStage("close gripper","close");
    auto attach=std::make_unique<mtc::stages::ModifyPlanningScene>("attach target in planning scene");
    attach->attachObject(object_,"end_effector_link");task_->add(std::move(attach));
    execute();
    // MTC scene attachment is distinct from Gazebo physical attachment.
    trigger("/simulation/attach_object");
    // Explicitly synchronize the live monitor: a held cube must not also exist
    // as a world obstacle when the next independent MTC task begins.
    auto held_query=std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    held_query->components.components=0xffff;
    auto held=call<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene",held_query)->scene;
    bool attached=false;
    for (const auto& body:held.robot_state.attached_collision_objects) if (body.object.id==object_) attached=true;
    if (!attached) throw std::runtime_error("MTC did not attach selected object in live scene");
    for (const auto& obstacle:held.world.collision_objects) if (obstacle.id==object_) {
      auto remove=std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
      remove->scene.is_diff=true;remove->scene.robot_state.is_diff=true;
      moveit_msgs::msg::CollisionObject obj;obj.id=object_;obj.operation=obj.REMOVE;
      remove->scene.world.collision_objects={obj};
      if (!call<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene",remove)->success)
        throw std::runtime_error("held scene synchronization failed");
    }
    const auto before=entity();
    auto lift=target;lift.pose.position.z+=.09;
    report("MTC_PLAN_LIFT");
    auto lift_joints=ik(lift);
    startTask("visual "+color_+" cube: lift verification");
    armStage("lift attached cube",lift_joints);execute();
    auto after=entity();
    const auto lift_deadline=std::chrono::steady_clock::now()+2s;
    while (after.position.z-before.position.z<.045 && std::chrono::steady_clock::now()<lift_deadline) {
      std::this_thread::sleep_for(50ms);after=entity();
    }
    if (after.position.z-before.position.z<.045) throw std::runtime_error("target did not physically follow gripper by 45 mm");
    const double lifted=after.position.z-before.position.z;
    report("LIFT_VERIFIED color="+color_+" lift_m="+std::to_string(lifted));
    // Place on the ground to the vehicle's left (base_footprint +Y).
    geometry_msgs::msg::PoseStamped storage;
    storage.header.frame_id="base_footprint";storage.pose.orientation.w=1.;
    storage.pose.position.x=node_->get_parameter("place_x").as_double();storage.pose.position.y=node_->get_parameter("place_y").as_double();storage.pose.position.z=node_->get_parameter("place_z").as_double();
    auto above=storage;above.pose.position.z+=.08;
    const auto above_joints=ik(above);

    const auto release_joints=ik(storage);

    report("MTC_PLAN_LEFT_GROUND");
    startTask("place "+color_+" cube on vehicle left ground");
    armStage("transport upright",{{"joint1",0.},{"joint2",0.},{"joint3",0.},{"joint4",0.},{"joint5",0.}});
    armStage("move above left ground",above_joints);
    armStage("lower to left ground",release_joints);
    gripperStage("open gripper to release","open");
    auto detach=std::make_unique<mtc::stages::ModifyPlanningScene>("detach target in planning scene");
    detach->detachObject(object_,"end_effector_link");task_->add(std::move(detach));execute();
    trigger("/simulation/detach_object");
    const auto settling_deadline=std::chrono::steady_clock::now()+15s;
    geometry_msgs::msg::Pose stored;
    bool settled=false;
    while (rclcpp::ok() && std::chrono::steady_clock::now()<settling_deadline) {
      std::this_thread::sleep_for(100ms);stored=entity("landerpi");
      const double ground_height=entity().position.z;
      if (std::abs(stored.position.x-storage.pose.position.x)<.075 && std::abs(stored.position.y-storage.pose.position.y)<.065 &&
          ground_height>.012 && ground_height<.045) {settled=true;break;}
    }
    if (!settled) throw std::runtime_error("released cube did not settle on the left ground");
    // Refresh released world geometry from physics for collision-safe retreat only.
    const auto world=entity();
    moveit_msgs::msg::CollisionObject released;released.header.frame_id="map";released.id=object_;
    shape_msgs::msg::SolidPrimitive shape;shape.type=shape.BOX;shape.dimensions={.04,.04,.05};
    released.primitives={shape};released.primitive_poses={world};released.operation=released.ADD;
    auto update=std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    update->scene.is_diff=true;update->scene.robot_state.is_diff=true;update->scene.world.collision_objects={released};
    moveit_msgs::msg::AttachedCollisionObject remove_attached;
    remove_attached.object.id=object_;remove_attached.object.operation=remove_attached.object.REMOVE;
    auto released_scene=call<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene",held_query)->scene;
    collision_detection::AllowedCollisionMatrix resting_contact(released_scene.allowed_collision_matrix);
    resting_contact.setEntry(object_,"mtc_ground",true);
    resting_contact.getMessage(update->scene.allowed_collision_matrix);
    for (const auto& body:released_scene.robot_state.attached_collision_objects)
      if (body.object.id==object_) update->scene.robot_state.attached_collision_objects={remove_attached};
    if (!call<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene",update)->success)
      throw std::runtime_error("released collision scene update failed");
    startTask("retreat from left ground");
    armStage("retreat above stored cube",above_joints);
    armStage("return arm home",{{"joint1",0.},{"joint2",0.},{"joint3",0.},{"joint4",0.},{"joint5",0.}});execute();
    stored=entity("landerpi");
    const double final_ground_height=entity().position.z;
    if (std::abs(stored.position.x-storage.pose.position.x)>.075 || std::abs(stored.position.y-storage.pose.position.y)>.065 ||
        final_ground_height<.012 || final_ground_height>.045) throw std::runtime_error("cube left ground during retreat");
    report("PERFORMANCE cycle_wall_s="+std::to_string(std::chrono::duration<double>(std::chrono::steady_clock::now()-cycle_wall_).count())+
      " cycle_sim_s="+std::to_string(node_->now().seconds()-cycle_sim_)+" planned_motion_s="+std::to_string(planned_seconds_)+
      " executed_segments="+std::to_string(executed_segments_));
    report("SUCCESS color="+color_+" lift_m="+std::to_string(lifted)+" placed_left_ground=true base_xyz="+
      std::to_string(stored.position.x)+","+std::to_string(stored.position.y)+","+std::to_string(stored.position.z));
  }
  collision_detection::AllowedCollisionMatrix taskModelACM() {
    // Current scene includes SRDF defaults; retrieving the ACM also preserves external rules.
    auto req=std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    req->components.components=moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    return collision_detection::AllowedCollisionMatrix(call<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene",req)->scene.allowed_collision_matrix);
  }
};

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("mtc_color_grasp",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor executor;executor.add_node(node);
  // Subscription callbacks capture ColorGrasp::this. Construct before spinning,
  // and keep it alive until every callback has stopped, including error paths.
  std::unique_ptr<ColorGrasp> grasp;
  std::thread spinner;
  int code=0;
  try {
    grasp=std::make_unique<ColorGrasp>(node);
    spinner=std::thread([&]{executor.spin();});
    if(grasp->hardwareMode())grasp->hardwareLoop();else grasp->run();
  }
  catch(const std::exception& error) {RCLCPP_ERROR(node->get_logger(),"GRASP_FAILED: %s",error.what());code=1;}
  executor.cancel();if(spinner.joinable())spinner.join();
  grasp.reset();rclcpp::shutdown();return code;
}
