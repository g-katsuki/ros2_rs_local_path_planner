#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <ryusei/common/defs.hpp>
#include <ryusei/navi/obstacle_detector.hpp>
#include <ryusei/navi/potential.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <ros2_rs_interfaces/srv/way_points_with_max_velocity.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ros_mercury_interfaces/msg/mercury_state_msg.hpp>
#include <ros2_rs_interfaces/msg/obstacles.hpp>
#include <ros2_rs_type_potential_viewer/msg/potential2_d.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <string>
#include <thread>
#include <mutex>
#include <chrono>

namespace rs = project_ryusei;
using namespace std;
using namespace std_srvs::srv;
using namespace std::chrono_literals;
using sensor_msgs::msg::Joy;

#define DEBUG 0
#define MERCURY_STATE 0
#define OBSTACLE 1
#define POTENTIAL_VIEWER 0
#define TWO_WAYPOINTS 1

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
  void execute();
  void getParameter();
  void getWayPoint();
  void onJoystickSubscribed(Joy::SharedPtr joy);
  void onPoseSubscribed(geometry_msgs::msg::Pose::SharedPtr msg);
  #if MERCURY_STATE
  void onMercuryStateSubscribed(ros_mercury_interfaces::msg::MercuryStateMsg::SharedPtr msg);
  #endif
  void onCorrectedPoseSubscribed(geometry_msgs::msg::Pose::SharedPtr msg);
  void onBlokingSignSubscribed(std_msgs::msg::Bool::SharedPtr msg);
  void onSignalSubscribed(std_msgs::msg::Bool::SharedPtr msg);
  void onObstaclesSubscribed(ros2_rs_interfaces::msg::Obstacles::SharedPtr msg);

  bool checkWayPointPassed(const geometry_msgs::msg::Pose &past_pose,
                           const geometry_msgs::msg::Pose &robot_pose,
                           const geometry_msgs::msg::Pose &way_point);

  rs::ObstacleDetector detector_;
  vector<rs::Obstacle> obstacles_;
  rs::Potential potential_;

  geometry_msgs::msg::Pose robot_pose_;
  geometry_msgs::msg::Pose way_point_;
  geometry_msgs::msg::Pose past_point_;
  geometry_msgs::msg::Pose target_pub_;
  geometry_msgs::msg::PoseArray way_points_;
  std_msgs::msg::Float64 dis_to_nearest_obs_;
  std_msgs::msg::Float64 max_velocity_;
  ros2_rs_interfaces::msg::Obstacles obs_msg_;
  std::string RESOURCE_DIRECTORY_NAME_;
  std::string INIT_FILE_NAME_;
  std::string POSE_NAME_;
  std::string LOCATOR_NAME_;
  double WAY_POINT_DISTANCE_;

  rclcpp::Subscription<Joy>::SharedPtr sub_joy_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_pose_;
  #if MERCURY_STATE
  rclcpp::Subscription<ros_mercury_interfaces::msg::MercuryStateMsg>::SharedPtr sub_mercury_state_;
  #endif
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr corrected_pose_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_bloking_sign_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_signal_;
  rclcpp::Subscription<ros2_rs_interfaces::msg::Obstacles>::SharedPtr sub_obstacles_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_pose_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_dis_to_obs_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_max_velocity_;
  rclcpp::Publisher<ros2_rs_type_potential_viewer::msg::Potential2D>::SharedPtr pub_potential_;
  rclcpp::Client<ros2_rs_interfaces::srv::WayPointsWithMaxVelocity>::SharedPtr way_point_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_usb_client_;

  vector<double> max_velocities_;
  struct RobotEuler robot_euler_;
  std::unique_ptr<struct RobotEuler> robot_euler_ptr_;
  std::mutex mutex_;
  std::mutex mutex_pose_;
  bool local_path_planner_ = false;
  bool joy_                = false;
  bool pre_joy_button_     = false;
  bool first_request_      = true;
  bool is_pose_            = false;
  bool is_way_point_       = false;
  bool is_blocking_sign_   = false;
  bool is_signal_          = false;
  bool detect_red_signal_  = false;

  /*** ポテンシャル場描画用 ***/
  ros2_rs_type_potential_viewer::msg::Potential2D potential_msg_;
};

/*** コンストラクタ ***/
ROS2LocalPathPlanner::ROS2LocalPathPlanner(rclcpp::NodeOptions options) : Node("rs_local_path_planner", options)
{
  using std::placeholders::_1;

  /*** パラメータ ***/
  local_path_planner_      = this->declare_parameter<bool>("local_path_planner", true);
  RESOURCE_DIRECTORY_NAME_ = this->declare_parameter<std::string>("resource_directory_name", "");
  INIT_FILE_NAME_          = this->declare_parameter<std::string>("init_file_name", "");
  POSE_NAME_               = this->declare_parameter<std::string>("pose_name", "");
  LOCATOR_NAME_            = this->declare_parameter<std::string>("locator_name", "");
  WAY_POINT_DISTANCE_      = this->declare_parameter<double>("way_point_distance", 0.2);

  /*** トピック、サービス ***/
  sub_joy_           = this->create_subscription<Joy>("/joystick/state", 1, 
                          std::bind(&ROS2LocalPathPlanner::onJoystickSubscribed, this, _1));
  sub_pose_          = this->create_subscription<geometry_msgs::msg::Pose>(POSE_NAME_, 10,
                          std::bind(&ROS2LocalPathPlanner::onPoseSubscribed, this, _1));
  #if MERCURY_STATE
  sub_mercury_state_ = this->create_subscription<ros_mercury_interfaces::msg::MercuryStateMsg>("/mercury/state",
                         10, std::bind(&ROS2LocalPathPlanner::onMercuryStateSubscribed, this, _1));
  #endif
  corrected_pose_    = this->create_subscription<geometry_msgs::msg::Pose>(LOCATOR_NAME_ + "/corrected_pose", 10,
                          std::bind(&ROS2LocalPathPlanner::onCorrectedPoseSubscribed, this, _1));
  sub_bloking_sign_  = this->create_subscription<std_msgs::msg::Bool>("/bloking_sign", 10,
                          std::bind(&ROS2LocalPathPlanner::onBlokingSignSubscribed, this, _1));
  sub_signal_        = this->create_subscription<std_msgs::msg::Bool>("/signal", 10,
                          std::bind(&ROS2LocalPathPlanner::onSignalSubscribed, this, _1));
  sub_obstacles_     = this->create_subscription<ros2_rs_interfaces::msg::Obstacles>("/rs_obstacle_detector/obstacles", 10,
                          std::bind(&ROS2LocalPathPlanner::onObstaclesSubscribed, this, _1));
  pub_pose_          = this->create_publisher<geometry_msgs::msg::Pose>("~/pose", 10);
  pub_dis_to_obs_    = this->create_publisher<std_msgs::msg::Float64>("~/dis_to_obs", 10);
  pub_max_velocity_  = this->create_publisher<std_msgs::msg::Float64>("~/max_velocity", 10);
  pub_potential_     = this->create_publisher<ros2_rs_type_potential_viewer::msg::Potential2D>("potential2d", 10);
  set_usb_client_    = this->create_client<std_srvs::srv::SetBool>("/mercury_run/set_active");
  way_point_client_  = this->create_client<ros2_rs_interfaces::srv::WayPointsWithMaxVelocity>
                             ("/way_points_with_max_velocities");

  /*** parameterからファイル名を取得 ***/
  std::string init_path;
  init_path = ament_index_cpp::get_package_share_directory("ros2_rs_local_path_planner");
  init_path += "/" + RESOURCE_DIRECTORY_NAME_ + "/";
  init_path += INIT_FILE_NAME_;

  /*** 点群の発生元のTFの名前 ***/
  potential_msg_.header.frame_id = "base_link";
  /*** ポテンシャルの描画の際の高さ[m] 最大と最小 ***/
  potential_msg_.normalize_potential_max = 1.0; 
  potential_msg_.normalize_potential_min = 0.0;

  /*** potentialの設定ファイル読み込み ***/
  if(!potential_.init(init_path))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load config file");
  }

  /*** 別スレッドでparameterを取得 ***/
  std::thread t1{&ROS2LocalPathPlanner::getParameter, this};
  t1.detach();

  /*** 別スレッドでway point要請ループ ***/
  std::thread t2{&ROS2LocalPathPlanner::getWayPoint, this};
  t2.detach();

  /*** 別スレッドで実行 ***/
  std::thread t3{&ROS2LocalPathPlanner::execute, this};
  t3.detach();
}

/*** デストラクタ ***/
ROS2LocalPathPlanner::~ROS2LocalPathPlanner()
{

}

/************************************************
* parameterの取得 (別スレッド1) 
*************************************************/
void ROS2LocalPathPlanner::getParameter()
{
  while(1)
  {
    this->get_parameter("local_path_planner", local_path_planner_);
    // RCLCPP_INFO_STREAM(this->get_logger(), "flag: " << local_path_planner_);

    /*** flagがoffの時,目標座標(0,0)をパブリッシュすることでロボットを停止させる ***/
    if(!local_path_planner_){
        target_pub_.position.x = 0.0;
        target_pub_.position.y = 0.0;
        pub_pose_ -> publish(target_pub_);
    }

    /*** ループ待機 ***/
    rclcpp::WallRate loop_rate(1s);
    loop_rate.sleep();
  }
}

/************************************************
* way_pointのサービス要請 (別スレッド2) 
*************************************************/
void ROS2LocalPathPlanner::getWayPoint()
{
  while(1)
  {
    if(is_pose_ && !is_way_point_ && !is_signal_)
    {
      /*** サービスが有効になるまで待機 ***/
      while(!way_point_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()){
        RCLCPP_INFO_STREAM(this->get_logger(), "waiting for service to appear...");
      }

      /*** ループ待機 ***/
      rclcpp::WallRate loop_rate(1s);
      loop_rate.sleep();

      mutex_.lock();
      /*** way pointのサービスサーバに要請を送る ***/
      auto request = std::make_shared<ros2_rs_interfaces::srv::WayPointsWithMaxVelocity::Request>();
      request->first_request = first_request_;
      request->blocking_sign = is_blocking_sign_;
      auto result = way_point_client_->async_send_request(request);
      result.wait();

      RCLCPP_INFO(this->get_logger(), "This service was called successfully");

      if(!local_path_planner_){
        RCLCPP_INFO(this->get_logger(), "to activate, use the command below to switch the flag to True");
        RCLCPP_INFO(this->get_logger(), "ros2 param set rs_local_path_planner local_path_planner True");
      }

      /*** 初回は一個前のway_pointがないので現在のposeを使用 ***/
      if(first_request_) past_point_ = robot_pose_;

      /*** サービスから受け取った変数を格納 ***/
      way_points_     = result.get()->way_points;
      max_velocities_ = result.get()->max_velocities;

      // 初回リクエストが終わったのでフラグを折る
      first_request_ = false;

      if(!way_points_.poses.empty()){
        RCLCPP_INFO_STREAM(this->get_logger(), "WSIZE: " << way_points_.poses.size());
        is_way_point_     = true;
        is_blocking_sign_ = false;
      }

      /*** 初回のウェイポイントが後ろにあるかの判定 ***/
      if(checkWayPointPassed(way_points_.poses[1], robot_pose_, way_points_.poses[0])){
        /*** 後ろにあったら削除 ***/
        way_points_.poses.erase(way_points_.poses.begin());
      }

      mutex_.unlock();
      RCLCPP_INFO_STREAM(this->get_logger(), "WSIZE: " << way_points_.poses.size());
    }
  }
}

/******************************************************************************
* 自己位置に対してのコールバック関数
********************************************************************************/
void ROS2LocalPathPlanner::onPoseSubscribed(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  mutex_pose_.lock();
  // サービスを受け取るときにposeが必要なのでそのためのフラグ
  is_pose_ = true;

  /****************************************************************************/
    /*** poseのorientationをyawに変換(後に関数化される) ***/
    robot_pose_.position    = msg->position;
    robot_pose_.orientation = msg->orientation;
    robot_euler_ptr_.reset(new struct RobotEuler(robot_pose_.orientation));
  /****************************************************************************/

  mutex_pose_.unlock();

  RCLCPP_INFO_STREAM(this->get_logger(), "POSE GET " );
}

/*******************************************************************************
* 自己位置に対してのコールバック関数(mercury)
*********************************************************************************/
#if MERCURY_STATE
void ROS2LocalPathPlanner::onMercuryStateSubscribed
     (const ros_mercury_interfaces::msg::MercuryStateMsg::SharedPtr msg)
{
  mutex_pose_.lock();
  // サービスを受け取るときにposeが必要なのでそのためのフラグ
  is_pose_ = true;

  /****************************************************************************/
  /*** poseのorientationをyawに変換(後に関数化される) ***/
  robot_pose_.position   = msg->odometry.pose.pose.position;
  robot_pose_.orientation = msg->odometry.pose.pose.orientation;
  robot_euler_ptr_.reset(new struct RobotEuler(robot_pose_.orientation));
  /****************************************************************************/

  mutex_pose_.unlock();
}
#endif

/*******************************************************************************
* 自己位置に対してのコールバック関数(corrected pose)
*********************************************************************************/
void ROS2LocalPathPlanner::onCorrectedPoseSubscribed(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  mutex_pose_.lock();
  // サービスを受け取るときにposeが必要なのでそのためのフラグ
  is_pose_ = true;

  /****************************************************************************/
  /*** poseのorientationをyawに変換(後に関数化される) ***/
  robot_pose_.position    = msg->position;
  robot_pose_.orientation = msg->orientation;
  robot_euler_ptr_.reset(new struct RobotEuler(robot_pose_.orientation));
  /****************************************************************************/

  mutex_pose_.unlock();
}

/*****************************************************
* 経路封鎖看板に対してのコールバック関数
******************************************************/
void ROS2LocalPathPlanner::onBlokingSignSubscribed(const std_msgs::msg::Bool::SharedPtr msg)
{
  if(msg->data == true)
  {
    /*** 目標座標(0,0)をパブリッシュすることでロボットを停止させる ***/
    target_pub_.position.x = 0.0;
    target_pub_.position.y = 0.0;
    pub_pose_ -> publish(target_pub_);
    /*** way_pointsを破棄し、看板発見フラグを立ててサービス要請に切り替え ***/
    is_way_point_ = false;
    way_points_.poses.clear();
    is_blocking_sign_ = true;
  }
}

/*****************************************************
* 信号に対してのコールバック関数
******************************************************/
void ROS2LocalPathPlanner::onSignalSubscribed(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  /*** 信号状態に切り替え ***/
  is_signal_ = true;

  /*** 目標座標(0,0)をパブリッシュすることでロボットを停止させる ***/
  target_pub_.position.x = 0.0;
  target_pub_.position.y = 0.0;
  pub_pose_ -> publish(target_pub_);
  /*** way_pointsを破棄し、サービス要請に切り替え ***/
  is_way_point_ = false;
  way_points_.poses.clear();

  /*** 赤を検出した場合のフラグを立てる ***/
  if(msg->data == true) detect_red_signal_ = true;

  /*** 赤から青に変わった場合のみ信号状態を解除 ***/
  if(detect_red_signal_ && !msg->data){
    is_signal_ = false;
  }
}

/*****************************************************
* 障害物に対してのコールバック関数
******************************************************/
void ROS2LocalPathPlanner::onObstaclesSubscribed(const ros2_rs_interfaces::msg::Obstacles::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  obs_msg_ = *msg;
}

/******************************************************************
* way_pointを通過したか判定する関数
* (way_pointから見たロボットの位置をpoint_1, way_pointから見た前の
* way_pointをpoint_2とし,point_1とpoint_2のなす角で判定)
*******************************************************************/
bool ROS2LocalPathPlanner::checkWayPointPassed(const geometry_msgs::msg::Pose &past_pose,
                                               const geometry_msgs::msg::Pose &robot_pose,
                                               const geometry_msgs::msg::Pose &way_point)
{
  Pose2D point_1, point_2;
  double numerator, denominator;  // 分子、分母
  double cosine_rad, rad, theta;
  
  // way_pointから見たロボット位置と1つ前のway_point
  point_1.x =  robot_pose.position.x - way_point.position.x;
  point_1.y =  robot_pose.position.y - way_point.position.y;
  point_2.x =  past_pose.position.x - way_point.position.x;
  point_2.y =  past_pose.position.y - way_point.position.y;

  // 内積の式からcos(rad)を求めるときの分子分母
  numerator   = (point_1.x * point_2.x) + (point_1.y * point_2.y);
  denominator = sqrt((point_1.x * point_1.x) + (point_1.y * point_1.y))
                * sqrt((point_2.x * point_2.x) + (point_2.y * point_2.y));

  // arccos(cos(rad))でradを求めthetaに変換
  cosine_rad = numerator/denominator;
  rad        = acos(cosine_rad);
  theta      = rad * 180/CV_PI;

  return theta >= 90 ? true : false;
}

/*****************************************************
* JoyStickに対してのコールバック関数
******************************************************/
void ROS2LocalPathPlanner::onJoystickSubscribed(Joy::SharedPtr joy)
{
  if(joy->buttons.empty()) return;
  /*** 自立走行がオン状態でボタンが押されたらコントローラーに切り替え ***/
  if(!joy_){
    if(joy->buttons[0] == 1 && !pre_joy_button_)
    {
      joy_ = true;
      /*** サービスが有効になるまで待機 ***/
      while(!set_usb_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()){
        RCLCPP_INFO_STREAM(this->get_logger(), "waiting for SETUSB service to appear...");
      }
      /*** サービスサーバに要請を送る ***/
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = true;
      auto result = set_usb_client_->async_send_request(request);
      // result.wait();
      RCLCPP_INFO_STREAM(this->get_logger(), "USB ON");
      RCLCPP_INFO_STREAM(this->get_logger(), "USB ON");
      RCLCPP_INFO_STREAM(this->get_logger(), "USB ON");
    }
  }
  /*** 自立走行がオフ状態でボタンが押されたらコントローラーをオフにする ***/
  else if(joy){
    if(joy->buttons[0] == 1 && !pre_joy_button_)
    {
      joy_ = false;
      /*** サービスが有効になるまで待機 ***/
      while(!set_usb_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()){
        RCLCPP_INFO_STREAM(this->get_logger(), "waiting for SETUSB service to appear...");
      }
      /*** サービスサーバに要請を送る ***/
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = false;
      auto result = set_usb_client_->async_send_request(request);
      // result.wait();
      RCLCPP_INFO_STREAM(this->get_logger(), "USB OFF");
      RCLCPP_INFO_STREAM(this->get_logger(), "USB OFF");
      RCLCPP_INFO_STREAM(this->get_logger(), "USB OFF");
    }
  }
  pre_joy_button_ = joy->buttons[0];
}

/***************************************
* 実行関数 
***************************************/
void ROS2LocalPathPlanner::execute()
{
  while(1)
  {
    // 自立走行オンオフのフラグがオンからオフなら停止
    if(local_path_planner_){
      this->get_parameter("local_path_planner", local_path_planner_);
      if(!local_path_planner_){
        /*** 目標座標(0,0)をパブリッシュすることでロボットを停止させる ***/
        target_pub_.position.x = 0.0;
        target_pub_.position.y = 0.0;
        pub_pose_ -> publish(target_pub_);
      }
    }

    this->get_parameter("local_path_planner", local_path_planner_);

    if(is_way_point_ && local_path_planner_)
    {
      # if !DEBUG
        RCLCPP_INFO_STREAM(this->get_logger(), "-----------------------------------------");
      # endif
      # if DEBUG
        RCLCPP_INFO_STREAM(this->get_logger(), "rest waysize: " << way_points_.poses.size());
      # endif

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

      /*------------------------*/
      /*** 2つ先のウェイポイントを目指す ***/
      # if TWO_WAYPOINTS
      bool aim2 = false;
      rs::Pose2D rel_way_point2;
      if(way_points_.poses.size() >= 2)
      {
        geometry_msgs::msg::Pose next_way_point;
        next_way_point = way_points_.poses[1];
        if(checkWayPointPassed(past_point_, next_way_point, way_point_)) // 2つ先のway_pointが1つ先より奥にあるか
        {
          aim2 = true;
          double dis_to_way_point_x2 = next_way_point.position.x - robot_pose_.position.x;
          double dis_to_way_point_y2 = next_way_point.position.y - robot_pose_.position.y;
          double angle_integrated2 = std::atan2(dis_to_way_point_y2, dis_to_way_point_x2);
          double angle_to_way_point2 = angle_integrated2 - robot_euler_ptr_->yaw_;
          double r = sqrt(dis_to_way_point_x * dis_to_way_point_x + dis_to_way_point_y * dis_to_way_point_y);
          rel_way_point2.x = r * cos(angle_to_way_point2);
          rel_way_point2.y = r * sin(angle_to_way_point2);
        }
      }
      # endif
      /*--------------------------*/
      /******************************************************************************************/
      RCLCPP_INFO_STREAM(this->get_logger(), "goal_to_x: " << rel_way_point.x);
      RCLCPP_INFO_STREAM(this->get_logger(), "goal_to_y: " << rel_way_point.y);

      double distance_to_goal = sqrt((rel_way_point.x * rel_way_point.x)
                                   + (rel_way_point.y * rel_way_point.y));

      /*** 目的地の一定距離内で次のway_pointに向かう ***/
      if(checkWayPointPassed(past_point_, robot_pose_, way_point_) ||
         WAY_POINT_DISTANCE_ > distance_to_goal)
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "heading to the next way point");
        past_point_ = way_point_;
        way_points_.poses.erase(way_points_.poses.begin());
        max_velocities_.erase(max_velocities_.begin());
        if(way_points_.poses.empty()){  // way_pointsが空になったらサービス要請に切り替え
          /*** 目標座標(0,0)をパブリッシュすることでロボットを停止させる ***/
          target_pub_.position.x = 0.0;
          target_pub_.position.y = 0.0;
          pub_pose_ -> publish(target_pub_);
          RCLCPP_INFO_STREAM(this->get_logger(), "FINISH WAYPOINTS!!!!!!!!!");
          is_way_point_ = false;
        }
      }
      else
      {
        /*** 障害物情報の変換 ***/
        # if OBSTACLE
        mutex_.lock();
        cv::Point3f obstacle_point;
        rs::Obstacle obstacle;

        for(int i = 0; i < obs_msg_.abs.size(); i++)
        {
          /*** 直線障害物を格納 ***/
          if(obs_msg_.abs[i].points.size() == 2)
          {
            for(int j = 0; j < 2; j++){
              /*** 直線障害物の絶対座標 ***/
              obstacle_point.x = obs_msg_.abs[i].points[j].x;
              obstacle_point.y = obs_msg_.abs[i].points[j].y;
              obstacle.abs.push_back(obstacle_point);
              /*** 直線障害物の相対座標 ***/
              obstacle_point.x = obs_msg_.rel[i].points[j].x;
              obstacle_point.y = obs_msg_.rel[i].points[j].y;
              obstacle.rel.push_back(obstacle_point);
            }
            obstacles_.push_back(obstacle);
            obstacle.abs.clear();
            obstacle.rel.clear();
          }

          /*** 矩形障害物を格納 ***/
          if(obs_msg_.abs[i].points.size() == 4)
          {
            for(int j = 0; j < 4; j++){
              /*** 矩形障害物の絶対座標 ***/
              obstacle_point.x = obs_msg_.abs[i].points[j].x;
              obstacle_point.y = obs_msg_.abs[i].points[j].y;
              obstacle.abs.push_back(obstacle_point);
              /*** 矩形障害物の相対座標 ***/
              obstacle_point.x = obs_msg_.rel[i].points[j].x;
              obstacle_point.y = obs_msg_.rel[i].points[j].y;
              obstacle.rel.push_back(obstacle_point);
            }
            obstacles_.push_back(obstacle);
            obstacle.abs.clear();
            obstacle.rel.clear();
          }
        }
        mutex_.unlock();
        # endif

        /*** 最適方向の計算 ***/
        rs::Pose2D robot_pose;
        robot_pose.x   = robot_pose_.position.x;
        robot_pose.y   = robot_pose_.position.y;
        robot_pose.yaw = robot_euler_ptr_->yaw_;

        /*** ポテンシャル場描画用 ***/
        #if POTENTIAL_VIEWER
        float range_x = 1.0;
        float range_y = 1.0;
        float unit    = 0.1;
        int potential_map_cols = 2 * range_x / unit;
        int potential_map_rows = 2 * range_y / unit;
        cv::Mat potential_map = cv::Mat::zeros(potential_map_cols, potential_map_rows, CV_32FC1);
        # endif

        double target_dir;
        # if TWO_WAYPOINTS
        if(aim2){  // 2つ先のウェイポイントに向かう
          target_dir = potential_.findOptimalWay(robot_pose, obstacles_, rel_way_point2);
          /*** 描画用 ***/
          #if POTENTIAL_VIEWER
          potential_.createPotentialMap(robot_pose, obstacles_, rel_way_point2, range_x, range_y, unit, potential_map);
          # endif
        }
        else
        {  // 2つ先のウェイポイントがない時1つ先に向かう
        # endif
          target_dir = potential_.findOptimalWay(robot_pose, obstacles_, rel_way_point);
          /*** 描画用 ***/
          #if POTENTIAL_VIEWER
          potential_.createPotentialMap(robot_pose, obstacles_, rel_way_point, range_x, range_y, unit, potential_map);
          # endif
          # if TWO_WAYPOINTS
        }
        # endif

        /*** potential描画ノードに渡す ***/
        #if POTENTIAL_VIEWER
        if(obstacles_.size() != 0)
        {
          geometry_msgs::msg::Point potential_point;
          int size = potential_map.cols * potential_map.rows;
          potential_msg_.points.resize(size);
          int count = 0;
          for(int x = 0; x < potential_map.cols; x++){
            for(int y = 0; y < potential_map.rows; y++){
              potential_point.x = ((potential_map.rows/2) - y) * unit;
              potential_point.y = ((potential_map.cols/2) - x) * unit;
              potential_point.z = potential_map.at<float>(x,y);
              potential_msg_.points[count] = potential_point;
              count ++;
            }
          }
          count = 0;
          pub_potential_->publish(potential_msg_);
          potential_msg_.points.clear();
        }
        # endif

        /*** 目標方向を目標地点に変換 ***/
        if(target_dir == 100){  // ポテンシャル場が0だった場合
          target_pub_.position.x = 0.0;
          target_pub_.position.y = 0.0;
          RCLCPP_INFO_STREAM(this->get_logger(), "no potential values");
        }
        else
        {
          target_pub_.position.x = 3 * cos(target_dir);
          target_pub_.position.y = 3 * sin(target_dir);
        }

        /*** 前方の障害物で停止 ***/
        if(potential_.checkFront())
        {
          target_pub_.position.x = 0.0;
          target_pub_.position.y = 0.0;
          /*** 自己位置、最近障害物距離、最大速度をパブリッシュ ***/
          dis_to_nearest_obs_.data = potential_.dis_to_nearest_obs_;
          pub_pose_         -> publish(target_pub_);
          pub_dis_to_obs_   -> publish(dis_to_nearest_obs_);
          pub_max_velocity_ -> publish(max_velocity_);
          RCLCPP_INFO_STREAM(this->get_logger(), "OBSTACLE AHEAD!!!");
          RCLCPP_INFO_STREAM(this->get_logger(), "OBSTACLE AHEAD!!!");
          RCLCPP_INFO_STREAM(this->get_logger(), "OBSTACLE AHEAD!!!");
          /*** 待機 ***/
          rclcpp::WallRate loop_rate(1000ms);
          loop_rate.sleep();
        }
        else
        {
          /*** 自己位置、最近障害物距離、最大速度をパブリッシュ ***/
          dis_to_nearest_obs_.data = potential_.dis_to_nearest_obs_;
          pub_pose_         -> publish(target_pub_);
          pub_dis_to_obs_   -> publish(dis_to_nearest_obs_);
          pub_max_velocity_ -> publish(max_velocity_);
        }
        
        # if !DEBUG
        RCLCPP_INFO_STREAM(this->get_logger(), "robot_pose_x  : " << robot_pose_.position.x);
        RCLCPP_INFO_STREAM(this->get_logger(), "robot_pose_y  : " << robot_pose_.position.y);

        RCLCPP_INFO_STREAM(this->get_logger(), "way_point_x  : " << way_points_.poses[0].position.x);
        RCLCPP_INFO_STREAM(this->get_logger(), "way_point_y  : " << way_points_.poses[0].position.y);

        target_dir = target_dir * 180/CV_PI;
        RCLCPP_INFO_STREAM(this->get_logger(), "angle  : " << target_dir);
        # endif
      }

      #if DEBUG
      RCLCPP_INFO_STREAM(this->get_logger(), "past_x  : " << past_point_.position.x);
      RCLCPP_INFO_STREAM(this->get_logger(), "past_y  : " << past_point_.position.y);

      RCLCPP_INFO_STREAM(this->get_logger(), "way_x  : " << way_points_.poses[0].position.x);
      RCLCPP_INFO_STREAM(this->get_logger(), "way_y  : " << way_points_.poses[0].position.y);
      #endif

      if(joy_) RCLCPP_INFO_STREAM(this->get_logger(), "USB ON!!!");
    }

    /*** 障害物情報を消す ***/
    potential_.potential_points_.clear();
    obstacles_.clear();

    /*** ループ待機 ***/
    rclcpp::WallRate loop_rate(100ms);
    loop_rate.sleep();
  }
}

}  // project_ryusei

/*** PotentialAngleクラスをコンポーネントとして登録 ***/
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(project_ryusei::ROS2LocalPathPlanner)