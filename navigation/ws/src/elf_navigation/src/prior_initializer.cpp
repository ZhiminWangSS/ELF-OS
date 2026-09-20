#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Geometry>
using Cloud=pcl::PointCloud<pcl::PointXYZ>;
using Pose=geometry_msgs::msg::PoseWithCovarianceStamped;
class Initializer: public rclcpp::Node {
 Cloud::Ptr map_{new Cloud}; Eigen::Matrix4f guess_=Eigen::Matrix4f::Identity(),previous_=guess_;
 Eigen::Matrix3f extrinsic_; Eigen::Vector3f translation_;
 int stable_=0; bool done_=false; double last_=0;
 rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr scan_;
 rclcpp::Subscription<Pose>::SharedPtr initial_;
 rclcpp::Publisher<Pose>::SharedPtr result_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
 void status(const std::string &s){std_msgs::msg::String m;m.data=s;status_->publish(m);RCLCPP_INFO(get_logger(),"%s",s.c_str());}
 public:
 Initializer():Node("prior_initializer") {
   auto path=declare_parameter<std::string>("map_path","");
   auto xyz=declare_parameter<std::vector<double>>("initial_xyz",{0.,0.,.45});
   auto rpy=declare_parameter<std::vector<double>>("initial_rpy",{0.,.5235987756,0.});
   if(xyz.size()!=3||rpy.size()!=3)throw std::runtime_error("initial_xyz/rpy must have 3 values");
   guess_.block<3,3>(0,0)=(Eigen::AngleAxisf(rpy[2],Eigen::Vector3f::UnitZ())*Eigen::AngleAxisf(rpy[1],Eigen::Vector3f::UnitY())*Eigen::AngleAxisf(rpy[0],Eigen::Vector3f::UnitX())).toRotationMatrix();
   for(int i=0;i<3;++i)guess_(i,3)=xyz[i];
   // Livox driver rotates points +30deg about Y; IMU is native.
   extrinsic_=Eigen::AngleAxisf(-M_PI/6,Eigen::Vector3f::UnitY()).toRotationMatrix();
   translation_={-.011f,-.02329f,.04412f};
   Cloud::Ptr raw(new Cloud);
   if(pcl::io::loadPCDFile(path,*raw)<0||raw->size()<1000)throw std::runtime_error("Missing/empty prior map");
   pcl::VoxelGrid<pcl::PointXYZ> voxel;voxel.setInputCloud(raw);voxel.setLeafSize(.2,.2,.2);voxel.filter(*map_);
   result_=create_publisher<Pose>("/icp_result",rclcpp::QoS(1).transient_local());
   status_=create_publisher<std_msgs::msg::String>("/navigation/initialization",rclcpp::QoS(1).transient_local());
   initial_=create_subscription<Pose>("/navigation/initial_imu_pose",10,[this](Pose::SharedPtr p){
     if(done_){status("Already initialized: restart localization to reset");return;}
     if(p->header.frame_id!="map")return;
     auto o=p->pose.pose.orientation;Eigen::Quaternionf q(o.w,o.x,o.y,o.z);
     auto t=p->pose.pose.position;
     if(!q.coeffs().allFinite()||q.norm()<.9||!std::isfinite(t.x+t.y+t.z))return;
     guess_.block<3,3>(0,0)=q.normalized().toRotationMatrix();guess_.block<3,1>(0,3)=Eigen::Vector3f(t.x,t.y,t.z);stable_=0;
   });
   scan_=create_subscription<livox_ros_driver2::msg::CustomMsg>("/livox/lidar",rclcpp::SensorDataQoS(),[this](livox_ros_driver2::msg::CustomMsg::SharedPtr m){align(m);});
   status("Waiting for lidar; initial guess must be near actual map pose");
 }
 void align(livox_ros_driver2::msg::CustomMsg::SharedPtr msg){
   if(done_||now().seconds()-last_<1.)return;last_=now().seconds();
   double age=now().seconds()-rclcpp::Time(msg->header.stamp).seconds();
   if(age<-.1||age>.5){stable_=0;status("Rejected stale scan");return;}
   Cloud::Ptr cloud(new Cloud),down(new Cloud);
   for(const auto &p:msg->points){Eigen::Vector3f v(p.x,p.y,p.z);if(!v.allFinite()||v.norm()<.5||v.norm()>25)continue;v=extrinsic_*v+translation_;cloud->push_back(pcl::PointXYZ(v.x(),v.y(),v.z()));}
   if(cloud->size()<300){stable_=0;return;}
   pcl::VoxelGrid<pcl::PointXYZ> voxel;voxel.setInputCloud(cloud);voxel.setLeafSize(.2,.2,.2);voxel.filter(*down);
   auto estimate=guess_;Cloud aligned;bool converged=true;
   for(double distance:{.8,.3}){
     pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;icp.setInputSource(down);icp.setInputTarget(map_);
     icp.setMaxCorrespondenceDistance(distance);icp.setMaximumIterations(40);icp.setTransformationEpsilon(1e-7);icp.align(aligned,estimate);
     estimate=icp.getFinalTransformation();converged=converged&&icp.hasConverged();
   }
   pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(map_);size_t inliers=0;double squared=0;
   std::vector<int> index(1);std::vector<float>d(1);
   for(auto &p:aligned){if(tree.nearestKSearch(p,1,index,d)>0&&d[0]<.09){++inliers;squared+=d[0];}}
   double overlap=double(inliers)/std::max(size_t(1),aligned.size());double rmse=std::sqrt(squared/std::max(size_t(1),inliers));
   double shift=(estimate.block<3,1>(0,3)-guess_.block<3,1>(0,3)).norm();
   auto relative=estimate.block<3,3>(0,0)*guess_.block<3,3>(0,0).transpose();
   // The map is level while the IMU has its fixed mounting pitch.  Navigation
   // uses the horizontal base pose, so reject on map-frame yaw disagreement.
   double angle=std::abs(std::atan2(relative(1,0),relative(0,0)));
   bool acceptable=converged&&estimate.allFinite()&&overlap>=.65&&rmse<.15&&shift<1.5&&angle<.5;
   bool consistent=(estimate.block<3,1>(0,3)-previous_.block<3,1>(0,3)).norm()<.08&&Eigen::AngleAxisf(estimate.block<3,3>(0,0)*previous_.block<3,3>(0,0).transpose()).angle()<.05;
   stable_=acceptable?(consistent?stable_+1:1):0;previous_=estimate;
   status("overlap="+std::to_string(overlap)+" rmse="+std::to_string(rmse)+" shift="+std::to_string(shift)+" angle="+std::to_string(angle)+" stable="+std::to_string(stable_));
   if(stable_<5)return;
   Pose out;out.header.frame_id="map";out.header.stamp=msg->header.stamp;
   out.pose.pose.position.x=estimate(0,3);out.pose.pose.position.y=estimate(1,3);out.pose.pose.position.z=estimate(2,3);
   Eigen::Quaternionf q(estimate.block<3,3>(0,0));q.normalize();out.pose.pose.orientation.x=q.x();out.pose.pose.orientation.y=q.y();out.pose.pose.orientation.z=q.z();out.pose.pose.orientation.w=q.w();
   result_->publish(out);done_=true;status("INITIALIZED: keep stationary until frontend reports healthy tracking");
 }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<Initializer>());}catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}rclcpp::shutdown();}
