// Simulation-only ideal fixed attachment; contact/friction is not modelled.
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <gazebo_ros/node.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/bool.hpp>
#include <mutex>
#include <condition_variable>
namespace landerpi {
class GraspPlugin : public gazebo::ModelPlugin {
 gazebo::physics::ModelPtr robot_, held_;
 gazebo::physics::JointPtr fixed_, brake_;
 gazebo_ros::Node::SharedPtr node_;
 gazebo::event::ConnectionPtr update_;
 rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr attach_, detach_, hold_, release_;
 rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
 std::mutex mutex_; std::condition_variable cv_;
 double last_publish_=0;
 int pending_=0; bool ok_=false; std::string message_;
 void request(int operation, std_srvs::srv::Trigger::Response::SharedPtr res) {
   std::unique_lock<std::mutex> lock(mutex_);
   pending_=operation;
   if (!cv_.wait_for(lock,std::chrono::seconds(3),[this]{return pending_==0;})) {
     pending_=0; res->success=false; res->message="physics update timeout"; return;
   }
   res->success=ok_; res->message=message_;
 }
 void step() {
   std::lock_guard<std::mutex> lock(mutex_);
   if (pending_) {
     auto object=held_;
     auto tip=robot_->GetLink("end_effector_link");
     // URDF fixed joint reduction may merge the tip into link5.
     if (!tip) tip=robot_->GetLink("link5");
     auto grip=robot_->GetJoint("r_joint");
     ok_=false;
     if (pending_==3) {
       if (!brake_) {
         auto ground=robot_->GetWorld()->ModelByName("ground");
         auto root=robot_->GetLink("base_footprint"); if (!root) root=robot_->GetLinks().front();
         brake_=robot_->GetWorld()->Physics()->CreateJoint("fixed",robot_);
         brake_->SetName("landerpi_base_brake");brake_->Load(ground->GetLinks().front(),root,ignition::math::Pose3d::Zero);brake_->Init();
       }
       ok_=true;message_="simulation base brake held";
     } else if (pending_==4) {
       if (brake_) {brake_->Detach();brake_->Fini();brake_.reset();}
       ok_=true;message_="simulation base brake released";
     } else if (pending_==1 && !fixed_ && tip && grip) {
       // Only a physically reachable cargo may attach; never relocate cargo.
       auto grasp_position=tip->WorldPose().Pos();
       if (tip->GetName()=="link5") grasp_position += tip->WorldPose().Rot().RotateVector(ignition::math::Vector3d(0,0,0.076));
       double nearest=0.030;
       for (const auto & candidate:robot_->GetWorld()->Models()) {
         const auto name=candidate->GetName();
         if (name!="box" && name.rfind("cargo_",0)!=0) continue;
         auto link=candidate->GetLink("box"); if (!link) continue;
         const double d=(grasp_position-link->WorldPose().Pos()).Length();
         if (d<nearest) {object=candidate;nearest=d;}
       }
       if (!object) {message_="grasp rejected: no cargo within 30 mm";pending_=0;cv_.notify_all();return;}
       const auto target=object->GetLink("box");
       auto grasp=tip->WorldPose().Pos();
       if (tip->GetName()=="link5") grasp += tip->WorldPose().Rot().RotateVector(ignition::math::Vector3d(0,0,0.076));
       const double distance=(grasp-target->WorldPose().Pos()).Length();
       if (distance>0.030 || grip->Position(0)>-0.35) {
         message_="grasp rejected: distance="+std::to_string(distance)+", gripper="+std::to_string(grip->Position(0));
       } else {
         fixed_=robot_->GetWorld()->Physics()->CreateJoint("fixed",robot_);
         fixed_->SetName("landerpi_object_grasp");fixed_->Load(tip,target,ignition::math::Pose3d::Zero);
         fixed_->Init(); target->SetGravityMode(false);
         held_=object; ok_=true; message_="ideal rigid grasp attached: "+object->GetName();
       }
     } else if (pending_==2 && fixed_ && object) {
       fixed_->Detach(); fixed_->Fini(); fixed_.reset(); object->GetLink("box")->SetGravityMode(true);
       held_.reset(); ok_=true; message_="object released";
     } else { message_="invalid attachment state or missing model/link"; }
     pending_=0; cv_.notify_all();
   }
   double now=robot_->GetWorld()->SimTime().Double();
   if (now-last_publish_>=0.05 || now<last_publish_) {
     std_msgs::msg::Bool msg; msg.data=static_cast<bool>(fixed_); pub_->publish(msg);last_publish_=now;
   }
 }
 public:
 void Load(gazebo::physics::ModelPtr model,sdf::ElementPtr sdf) override {
   robot_=model; node_=gazebo_ros::Node::Get(sdf);
   // This ideal grasp deliberately excludes robot/object contact impulses.
   // Fixture and ground contacts remain physical; MoveIt validates approach.
   for (const auto & link:robot_->GetLinks()) for (const auto & collision:link->GetCollisions()) {
     collision->SetCategoryBits(0x01); collision->SetCollideBits(0xfffd);
   }
   for (const auto & object:robot_->GetWorld()->Models()) {
     const auto name=object->GetName();
     if (name!="box" && name.rfind("cargo_",0)!=0) continue;
     for (const auto & link:object->GetLinks()) for (const auto & collision:link->GetCollisions()) {
       collision->SetCategoryBits(0x02); collision->SetCollideBits(0xfffe);
     }
   }
   attach_=node_->create_service<std_srvs::srv::Trigger>("/simulation/attach_object",[this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res){request(1,res);});
   detach_=node_->create_service<std_srvs::srv::Trigger>("/simulation/detach_object",[this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res){request(2,res);});
   hold_=node_->create_service<std_srvs::srv::Trigger>("/simulation/hold_base",[this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res){request(3,res);});
   release_=node_->create_service<std_srvs::srv::Trigger>("/simulation/release_base",[this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res){request(4,res);});
   pub_=node_->create_publisher<std_msgs::msg::Bool>("/simulation/object_attached",10);
   update_=gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&GraspPlugin::step,this));
 }
};
GZ_REGISTER_MODEL_PLUGIN(GraspPlugin)
}
