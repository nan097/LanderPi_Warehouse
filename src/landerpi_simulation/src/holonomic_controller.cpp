// Deterministic, low-cost holonomic path follower for the simulation testbench.
// Nav2 still owns planning, lifecycle, progress checking and action cancellation.
#include <nav2_core/controller.hpp>
#include <nav2_core/exceptions.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <cmath>
namespace landerpi {
class HolonomicController : public nav2_core::Controller {
 std::shared_ptr<tf2_ros::Buffer> tf_;
 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
 rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
 nav_msgs::msg::Path path_;
 double limit_=0.18, cruise_=0.18, lookahead_=0.15, gain_=0.9;
 double angular_=0.3,yaw_gain_=0.8,acceleration_=0.5,angular_acceleration_=1.,period_=0.125;
 public:
 void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,std::string name,
   std::shared_ptr<tf2_ros::Buffer> tf,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override {
   node_=parent.lock();tf_=tf;map_=map;
   auto parameter=[&](const std::string& key,double fallback) {
     const auto full=name+"."+key;
     if(!node_->has_parameter(full))node_->declare_parameter(full,fallback);
     const double value=node_->get_parameter(full).as_double();
     if(!std::isfinite(value)||value<=0.)throw std::invalid_argument(full+" must be finite and positive");
     return value;
   };
   cruise_=parameter("cruise_speed",.18);limit_=cruise_;
   lookahead_=parameter("lookahead_distance",.15);gain_=parameter("position_gain",.9);
   angular_=parameter("max_angular_speed",.3);yaw_gain_=parameter("yaw_gain",.8);
   acceleration_=parameter("linear_acceleration",.5);angular_acceleration_=parameter("angular_acceleration",1.);
   period_=1./node_->get_parameter("controller_frequency").as_double();
   RCLCPP_INFO(node_->get_logger(),"Warehouse follower speed=%.2f m/s lookahead=%.2f m acceleration=%.2f m/s2",cruise_,lookahead_,acceleration_);
 }
 void cleanup() override {path_.poses.clear();}
 void activate() override {}
 void deactivate() override {}
 void setPlan(const nav_msgs::msg::Path & path) override {path_=path;}
 void setSpeedLimit(const double & value,const bool & percentage) override {
   limit_=value<=0?cruise_:std::min(cruise_,percentage?cruise_*value/100.:value);
 }
 geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose,
   const geometry_msgs::msg::Twist & measured,nav2_core::GoalChecker *) override {
   if(path_.poses.empty())throw nav2_core::PlannerException("empty path");
   std::vector<geometry_msgs::msg::PoseStamped> local;
   for(auto p:path_.poses){p.header.stamp=rclcpp::Time(0);local.push_back(tf_->transform(p,pose.header.frame_id,tf2::durationFromSec(0.1)));}
   size_t nearest=0;double best=1e10;
   for(size_t i=0;i<local.size();++i){const auto & p=local[i].pose.position;const double d=std::hypot(p.x-pose.pose.position.x,p.y-pose.pose.position.y);if(d<best){best=d;nearest=i;}}
   size_t target=nearest;
   while(target+1<local.size() && std::hypot(local[target].pose.position.x-pose.pose.position.x,local[target].pose.position.y-pose.pose.position.y)<lookahead_)++target;
   const auto & p=local[target].pose.position;
   auto latest=tf_->lookupTransform(pose.header.frame_id,map_->getBaseFrameID(),tf2::TimePointZero);
   if((node_->now()-rclcpp::Time(latest.header.stamp)).seconds()>0.5)
     throw nav2_core::PlannerException("navigation odometry TF is stale");
   RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
     "Follower pose=(%.3f,%.3f) latest_tf=(%.3f,%.3f) target=(%.3f,%.3f) frame=%s",
     pose.pose.position.x,pose.pose.position.y,latest.transform.translation.x,latest.transform.translation.y,
     p.x,p.y,pose.header.frame_id.c_str());
   const double dx=p.x-pose.pose.position.x,dy=p.y-pose.pose.position.y;
   const double distance=std::hypot(dx,dy),speed=std::min(limit_,gain_*distance);
   double vx=distance>1e-6?speed*dx/distance:0.,vy=distance>1e-6?speed*dy/distance:0.;
   const double yaw=tf2::getYaw(pose.pose.orientation);
   const double goal_yaw=tf2::getYaw(local.back().pose.orientation);
   const double yaw_error=std::atan2(std::sin(goal_yaw-yaw),std::cos(goal_yaw-yaw));
   double wz=std::clamp(yaw_gain_*yaw_error,-angular_,angular_);
   const double measured_x=std::cos(yaw)*measured.linear.x-std::sin(yaw)*measured.linear.y;
   const double measured_y=std::sin(yaw)*measured.linear.x+std::cos(yaw)*measured.linear.y;
   const double change=std::hypot(vx-measured_x,vy-measured_y);
   if(change>acceleration_*period_) {
     const double ratio=acceleration_*period_/change;
     vx=measured_x+ratio*(vx-measured_x);vy=measured_y+ratio*(vy-measured_y);
   }
   wz=std::clamp(wz,measured.angular.z-angular_acceleration_*period_,measured.angular.z+angular_acceleration_*period_);
   // Check the commanded footprint through at least one braking horizon.
   const double horizon=std::max({.4,std::hypot(vx,vy)/acceleration_+period_,std::abs(wz)/angular_acceleration_+period_});
   const int samples=std::max(8,static_cast<int>(std::ceil(horizon/.025)));
   std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map_->getCostmap()->getMutex());
   nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(map_->getCostmap());
   bool valid=false;double scale=1.;
   for(double candidate:{1.,0.5,0.25}) {
     bool free=true;
     for(int k=1;k<=samples;++k){double dt=horizon*k/samples;double cost=checker.footprintCostAtPose(pose.pose.position.x+vx*candidate*dt,pose.pose.position.y+vy*candidate*dt,yaw+wz*candidate*dt,map_->getRobotFootprint());if(cost<0 || cost>=254){free=false;break;}}
     if(free){scale=candidate;valid=true;break;}
   }
   if(!valid)throw nav2_core::PlannerException("holonomic swept footprint collides with costmap");
   geometry_msgs::msg::TwistStamped out;out.header.stamp=node_->now();out.header.frame_id=map_->getBaseFrameID();
   out.twist.linear.x=scale*(std::cos(yaw)*vx+std::sin(yaw)*vy);out.twist.linear.y=scale*(-std::sin(yaw)*vx+std::cos(yaw)*vy);out.twist.angular.z=scale*wz;return out;
 }
};
}
PLUGINLIB_EXPORT_CLASS(landerpi::HolonomicController,nav2_core::Controller)
