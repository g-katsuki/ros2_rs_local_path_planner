#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <ryusei/common/defs.hpp>
#include <ryusei/navi/obstacle_detector.hpp>
#include <ryusei/navi/potential.hpp>
#include <std_srvs/srv/empty.hpp>
#include <ros2_rs_interfaces/srv/way_points_with_max_velocity.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ros_mercury_interfaces/msg/mercury_state_msg.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <string>
#include <thread>
#include <mutex>
#include <chrono>

namespace rs = project_ryusei;
using namespace std;
using namespace std_srvs::srv;
using namespace std::chrono_literals;


/*** orientation から euler(roll, pitch, yaw)へ変換・保持するための構造体 ***/
struct RobotEuler {
  double roll_, pitch_, yaw_;

  RobotEuler(){
    roll_ = 0.; pitch_ = 0.; yaw_ = 0.;
  }

  RobotEuler(geometry_msgs::msg::Quaternion robot_orientation){
    update(robot_orientation);
  }

  void update(geometry_msgs::msg::Quaternion robot_orientation){
    tf2::Quaternion q(
      robot_orientation.x,
      robot_orientation.y,
      robot_orientation.z,
      robot_orientation.w
    );

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw); 
    
    this->update(roll, pitch, yaw);
  }

  void update(double roll, double pitch, double yaw){
    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;
  };
};

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
  void onMercuryStateSubscribed(ros_mercury_interfaces::msg::MercuryStateMsg::SharedPtr msg);
  void onBlokingSignSubscribed(std_msgs::msg::Bool::SharedPtr msg);

  rs::ObstacleDetector detector_;
  vector<rs::Obstacle> obstacles_;
  rs::Potential potential_;

  geometry_msgs::msg::Pose robot_pose_;
  geometry_msgs::msg::Pose way_point_;
  geometry_msgs::msg::Pose target_pub_;
  geometry_msgs::msg::PoseArray way_points_;
  std_msgs::msg::Float64 dis_to_nearest_obs_;
  std_msgs::msg::Float64 max_velocity_;
  std::string RESOURCE_DIRECTORY_NAME_;
  std::string INIT_FILE_NAME_;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_pose_;
  rclcpp::Subscription<ros_mercury_interfaces::msg::MercuryStateMsg>::SharedPtr sub_mercury_state_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_bloking_sign_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_pose_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_dis_to_obs_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_max_velocity_;
  rclcpp::Client<ros2_rs_interfaces::srv::WayPointsWithMaxVelocity>::SharedPtr way_point_client_;

  vector<double> max_velocities_;
  struct RobotEuler robot_euler_;
  std::unique_ptr<struct RobotEuler> robot_euler_ptr_;
  std::mutex mutex_;
  bool is_way_point_ = false;
  bool is_blocking_sign_ = false;
};

/*** コンストラクタ ***/
ROS2LocalPathPlanner::ROS2LocalPathPlanner(rclcpp::NodeOptions options) : Node("rs_local_path_planner", options)
{
  using std::placeholders::_1;
  sub_pose_          = this->create_subscription<geometry_msgs::msg::Pose>("/pose", 10,
                          std::bind(&ROS2LocalPathPlanner::onPoseSubscribed, this, _1));
  sub_mercury_state_ = this->create_subscription<ros_mercury_interfaces::msg::MercuryStateMsg>("/mercury/state",
                          10, std::bind(&ROS2LocalPathPlanner::onMercuryStateSubscribed, this, _1));
  sub_bloking_sign_  = this->create_subscription<std_msgs::msg::Bool>("/bloking_sign", 10,
                          std::bind(&ROS2LocalPathPlanner::onBlokingSignSubscribed, this, _1));
  pub_pose_          = this->create_publisher<geometry_msgs::msg::Pose>("~/pose", 10);
  pub_dis_to_obs_    = this->create_publisher<std_msgs::msg::Float64>("~/dis_to_obs", 10);
  pub_max_velocity_  = this->create_publisher<std_msgs::msg::Float64>("~/max_velocity", 10);
  way_point_client_  = this->create_client<ros2_rs_interfaces::srv::WayPointsWithMaxVelocity>
                             ("/way_points_with_max_velocities");
  RESOURCE_DIRECTORY_NAME_ = this->declare_parameter<std::string>("resource_directory_name", "");
  INIT_FILE_NAME_    = this->declare_parameter<std::string>("init_file_name", "");

  std::string init_path;
    init_path = ament_index_cpp::get_package_share_directory("ros2_rs_local_path_planner");
    init_path += "/" + RESOURCE_DIRECTORY_NAME_ + "/";
    init_path += INIT_FILE_NAME_;

  /*** ObstacleDetectorの設定ファイル読み込み ***/
  if(!detector_.init("data/navigation_cfg/navigation.ini"))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load config file");
  }

  /*** potentialの設定ファイル読み込み ***/
  if(!potential_.init("data/navigation_cfg/navigation.ini"))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load config file");
  }

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
        RCLCPP_INFO_STREAM(this->get_logger(), "waiting for service to appear...");
      }

      /*** way pointのサービスサーバに要請を送る ***/
      auto request = std::make_shared<ros2_rs_interfaces::srv::WayPointsWithMaxVelocity::Request>();
      request->blocking_sign = is_blocking_sign_;
      auto result = way_point_client_->async_send_request(request);

      // /*** サービスからの返答があるまで待機 ***/
      // if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) 
      //    == rclcpp::FutureReturnCode::SUCCESS){
      //   RCLCPP_INFO(this->get_logger(), "This service was called successfully");
        RCLCPP_INFO(this->get_logger(), "This service was called successfully");
        is_way_point_ = true;
        way_points_     = result.get()->way_points;
        max_velocities_ = result.get()->max_velocities;
        if(!way_points_.poses.empty()){
          is_way_point_     = true;
          is_blocking_sign_ = false;
        }
      // } 
      // else
      // {
      //   RCLCPP_ERROR(this->get_logger(), "Failed to call the service");
      // }
    }
  }
}

/*** 自己位置に対してのコールバック関数 ***/
/*** 2つの絶対座標ウェイポイントから相対座標だしてpotentialよんで進行方向座標の計算 ***/
void ROS2LocalPathPlanner::onPoseSubscribed(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(is_way_point_)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "rest waysize: " << way_points_.poses.size());
    RCLCPP_INFO_STREAM(this->get_logger(), "rest velosize: " << max_velocities_.size());
    /****************************************************************************/
    /*** poseのorientationをyawに変換(後に関数化される) ***/
    robot_pose_.position    = msg->position;
    robot_pose_.orientation = msg->orientation;
    robot_euler_ptr_.reset(new struct RobotEuler(robot_pose_.orientation));
    /****************************************************************************/

    /****************************************************************************************/
    /*** waypointの相対位置計算(後に関数化される) ***/
    rs::Pose2D rel_way_point;
    way_point_         = way_points_.poses[0];
    max_velocity_.data = max_velocities_[0];

    /*** 絶対座標でのrobotからみたway_pointの座標 ***/
    double dis_to_way_point_x = way_point_.position.x - robot_pose_.position.x;
    double dis_to_way_point_y = way_point_.position.y - robot_pose_.position.y;

    /*** 絶対位置でのロボット(ロボットは0度とされる)とway_pointの角度 ***/
    double angle_integrated = std::atan2(dis_to_way_point_y, dis_to_way_point_x);

    /*** ロボットからみたway_pointへの角度 ***/
    double angle_to_way_point = angle_integrated - robot_euler_ptr_->yaw_;

    /*** 直線距離(r)から、ロボットからway_pointへのx,yの距離を算出 ***/
    double r = sqrt(dis_to_way_point_x * dis_to_way_point_x + dis_to_way_point_y * dis_to_way_point_y);
    rel_way_point.x = r * cos(angle_to_way_point);
    rel_way_point.y = r * sin(angle_to_way_point);
    /******************************************************************************************/

    RCLCPP_INFO_STREAM(this->get_logger(), "rel_x: " << rel_way_point.x);
    RCLCPP_INFO_STREAM(this->get_logger(), "rel_y: " << rel_way_point.y);

    if((rel_way_point.x < 0.2 && rel_way_point.x > -0.2) && 
       (rel_way_point.y < 0.2 && rel_way_point.y > -0.2))
    {
      /*** 目的地の一定距離内で次のway_pointに向かう ***/
      way_points_.poses.erase(way_points_.poses.begin());
      max_velocities_.erase(max_velocities_.begin());
      if(way_points_.poses.empty()){  // way_pointsが空になったらサービス要請に切り替え
        is_way_point_ = false;
        RCLCPP_INFO_STREAM(this->get_logger(), "reached to the check point");
      }
    }
    else
    {
      /*** 最適方向の計算 ***/
      rs::Pose2D robot_pose;
      robot_pose.x   = robot_pose_.position.x;
      robot_pose.y   = robot_pose_.position.y;
      robot_pose.yaw = robot_euler_ptr_->yaw_;
      double target_dir = potential_.findOptimalWay(robot_pose, obstacles_, rel_way_point);
      target_pub_.position.x = 1 * cos(target_dir);
      target_pub_.position.y = 1 * sin(target_dir);

      /*** 自己位置、最近障害物距離、最大速度をパブリッシュ ***/
      dis_to_nearest_obs_.data = potential_.dis_to_nearest_obs_;
      pub_pose_         -> publish(target_pub_);
      pub_dis_to_obs_   -> publish(dis_to_nearest_obs_);
      pub_max_velocity_ -> publish(max_velocity_);
      
      /*** ループ待機 ***/
      rclcpp::WallRate loop_rate(100ms);
      loop_rate.sleep();
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "way_x  : " << way_points_.poses[0].position.x);
    RCLCPP_INFO_STREAM(this->get_logger(), "way_y  : " << way_points_.poses[0].position.y);
  }
}

void ROS2LocalPathPlanner::onMercuryStateSubscribed
     (const ros_mercury_interfaces::msg::MercuryStateMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(is_way_point_)
  {
      RCLCPP_INFO_STREAM(this->get_logger(), "-----------------------------------------");
    /*** 2つの絶対座標ウェイポイントから相対座標だしてpotentialよんで進行方向かえす ***/
  
  /****************************************************************************/
  /*** poseのorientationをyawに変換(後に関数化される) ***/
  robot_pose_.position   = msg->odometry.pose.pose.position;
  robot_pose_.orientation = msg->odometry.pose.pose.orientation;
  robot_euler_ptr_.reset(new struct RobotEuler(robot_pose_.orientation));
  /****************************************************************************/

  /****************************************************************************************/
  /*** waypointの相対位置計算(後に関数化される) ***/
  rs::Pose2D rel_way_point;
  way_point_         = way_points_.poses[0];
  max_velocity_.data = max_velocities_[0];

  /*** 絶対座標でのrobotからみたway_pointの座標 ***/
  double dis_to_way_point_x = way_point_.position.x - robot_pose_.position.x;
  double dis_to_way_point_y = way_point_.position.y - robot_pose_.position.y;

  /*** 絶対位置でのロボット(ロボットは0度とされる)とway_pointの角度 ***/
  double angle_integrated = std::atan2(dis_to_way_point_y, dis_to_way_point_x);

  /*** ロボットからみたway_pointへの角度 ***/
  double angle_to_way_point = angle_integrated - robot_euler_ptr_->yaw_;

  /*** 直線距離(r)から、ロボットからgoalへのx,yの距離を算出 ***/
  double r = sqrt(dis_to_way_point_x * dis_to_way_point_x + dis_to_way_point_y * dis_to_way_point_y);
  rel_way_point.x = r * cos(angle_to_way_point);
  rel_way_point.y = r * sin(angle_to_way_point);
  /******************************************************************************************/

  RCLCPP_INFO_STREAM(this->get_logger(), "goal_to_x: " << rel_way_point.x);
  RCLCPP_INFO_STREAM(this->get_logger(), "goal_to_y: " << rel_way_point.y);

  if((rel_way_point.x < 0.2 && rel_way_point.x > -0.2) && 
     (rel_way_point.y < 0.2 && rel_way_point.y > -0.2))
    {
      /*** 目的地の一定距離内で次のway_pointに向かう ***/
      way_points_.poses.erase(way_points_.poses.begin());
      max_velocities_.erase(max_velocities_.begin());
      if(way_points_.poses.empty()){  // way_pointsが空になったらサービス要請に切り替え
        is_way_point_ = false;
        target_pub_.position.x = 0.0;
        target_pub_.position.y = 0.0;
        pub_pose_ -> publish(target_pub_);
      }
    }
    else
    {
      /*** 最適方向の計算 ***/
      rs::Pose2D robot_pose;
      robot_pose.x   = robot_pose_.position.x;
      robot_pose.y   = robot_pose_.position.y;
      robot_pose.yaw = robot_euler_ptr_->yaw_;

      double target_dir = potential_.findOptimalWay(robot_pose, obstacles_, rel_way_point);

      target_pub_.position.x = 1 * cos(target_dir);
      target_pub_.position.y = 1 * sin(target_dir);

      /*** 自己位置、最近障害物距離、最大速度をパブリッシュ ***/
      dis_to_nearest_obs_.data = potential_.dis_to_nearest_obs_;
      pub_pose_         -> publish(target_pub_);
      pub_dis_to_obs_   -> publish(dis_to_nearest_obs_);
      pub_max_velocity_ -> publish(max_velocity_);
      
      RCLCPP_INFO_STREAM(this->get_logger(), "way_point_x  : " << way_points_.poses[0].position.x);
      RCLCPP_INFO_STREAM(this->get_logger(), "way_point_y  : " << way_points_.poses[0].position.y);

      target_dir = target_dir * 180/CV_PI;
      RCLCPP_INFO_STREAM(this->get_logger(), "angle  : " << target_dir);


      /*** ループ待機 ***/
      rclcpp::WallRate loop_rate(100ms);
      loop_rate.sleep();
    }
  }
}

/*** 経路封鎖看板に対してのコールバック関数 ***/
void ROS2LocalPathPlanner::onBlokingSignSubscribed(const std_msgs::msg::Bool::SharedPtr msg)
{
  is_way_point_ = false;
  is_blocking_sign_ = true;
  // これから記述 目標座標(0,0)をパブリッシュ
}

}
/*** PotentialAngleクラスをコンポーネントとして登録 ***/
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(project_ryusei::ROS2LocalPathPlanner)