#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <ryusei/common/defs.hpp>
#include <ryusei/navi/obstacle_detector.hpp>
#include <ryusei/navi/potential.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ros2_rs_interfaces/srv/way_points.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/odometry.hpp>

#include <string>
#include <thread>
#include <mutex>
#include <chrono>

namespace rs = project_ryusei;
using namespace std;
using namespace std_srvs::srv;
using namespace std::chrono_literals;

namespace project_ryusei
{

class ROS2LocalPathPlanner : public rclcpp::Node
{
public:
  ROS2LocalPathPlanner(rclcpp::NodeOptions options);
  ~ROS2LocalPathPlanner();
private:
  void getWayPoint();
  void onPoseSubscribed(geometry_msgs::msg::Pose::SharedPtr msg);
  void onBlokingSignSubscribed(std_msgs::msg::Bool::SharedPtr msg);

  geometry_msgs::msg::Pose pose_;
  geometry_msgs::msg::PoseArray way_points_;
  std::mutex mutex_;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_pose_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_bloking_sign_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_pose_;
  rclcpp::Client<ros2_rs_interfaces::srv::WayPoints>::SharedPtr way_point_client_;

  bool is_way_point_ = false;
};

/*** コンストラクタ ***/
ROS2LocalPathPlanner::ROS2LocalPathPlanner(rclcpp::NodeOptions options) : Node("rs_local_path_planner", options)
{
  using std::placeholders::_1;
  sub_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("/pose", 10,
                          std::bind(&ROS2LocalPathPlanner::onPoseSubscribed, this, _1));
  sub_bloking_sign_ = this->create_subscription<std_msgs::msg::Bool>("/bloking_sign", 10,
                          std::bind(&ROS2LocalPathPlanner::onBlokingSignSubscribed, this, _1));
  pub_pose_ = this->create_publisher<geometry_msgs::msg::Pose>("~/pose", 10);
  way_point_client_ = this->create_client<ros2_rs_interfaces::srv::WayPoints>("way_points");


  /*** 別スレッドでway point要請ループ ***/
  std::thread{&ROS2LocalPathPlanner::getWayPoint, this}.detach();
}

/*** デストラクタ ***/
ROS2LocalPathPlanner::~ROS2LocalPathPlanner()
{

}

/*** way_pointのサービス要請 (別スレッド) ***/
void ROS2LocalPathPlanner::getWayPoint()
{
  while(1)
  {
    if(!is_way_point_)
    {
      /*** サービスが有効になるまで待機 ***/
      while(!way_point_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()){
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
      }

      /*** way pointのサービスサーバに要請を送る ***/
      auto request = std::make_shared<ros2_rs_interfaces::srv::WayPoints::Request>();
      auto result = way_point_client_->async_send_request(request);

      /*** サービスからの返答があるまで待機 ***/
      if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) 
         == rclcpp::FutureReturnCode::SUCCESS){
        is_way_point_ = true;
        way_points_ = result.get()-> way_points;
        RCLCPP_INFO(this->get_logger(), "This service was called successfully");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call the service");
      }
    }
  }
}

/*** 自己位置に対してのコールバック関数 ***/
void ROS2LocalPathPlanner::onPoseSubscribed(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(is_way_point_)
  {
    
  }
}

/*** 経路封鎖看板に対してのコールバック関数 ***/
void ROS2LocalPathPlanner::onBlokingSignSubscribed(const std_msgs::msg::Bool::SharedPtr msg)
{
  is_way_point_ = false;
  
}

}
/*** PotentialAngleクラスをコンポーネントとして登録 ***/
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(project_ryusei::ROS2LocalPathPlanner)