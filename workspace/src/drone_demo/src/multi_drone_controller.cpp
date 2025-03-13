#include "drone_demo/multi_drone_controller.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

//------------------------------------------------------------------------------
// ドローンの姿勢→yaw角を取り出すヘルパー
//------------------------------------------------------------------------------
static double getYaw(const geometry_msgs::Pose &pose)
{
  tf2::Quaternion q(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w
  );
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

//------------------------------------------------------------------------------
// コンストラクタ
//------------------------------------------------------------------------------
MultiDroneController::MultiDroneController()
: joy_x_(0.0)
, joy_y_(0.0)
, joy_z_(0.0)
, joy_yaw_(0.0)
, formation_angle_(0.0)
, all_received_(false)
{
  // 三角形のオフセットを固定で設定
  // (unit: [m], ドローンの中心(0,0)に対して)
  drone1_offset_x_ = 0.0;   drone1_offset_y_ =  0.5;
  drone2_offset_x_ = -0.5;  drone2_offset_y_ = -0.5;
  drone3_offset_x_ =  0.5;  drone3_offset_y_ = -0.5;

  // 編隊の中心
  formation_center_x_ = 0.0;
  formation_center_y_ = 0.0;

  // 編隊回転速度 & 高さ
  rotation_speed_ = 1.0;  // ボタン押し中に 1[rad/s]回転
  formation_z_    = 2.0;  // 全機高さ 2[m]でホバリングするイメージ

  // 中心移動用スティックスケール
  move_scale_ = 0.5;

  // ジョイスティック購読
  joy_sub_ = nh_.subscribe("/joy", 1, &MultiDroneController::joyCallback, this);

  // ドローン1
  drone1_.name = "drone1";
  drone1_.pose_sub = nh_.subscribe(
      "/drone1/mavros/local_position/pose", 
      1, 
      &MultiDroneController::poseCallback1, 
      this
  );
  drone1_.vel_pub = nh_.advertise<geometry_msgs::Twist>(
      "/drone1/mavros/setpoint_velocity/cmd_vel_unstamped", 1
  );

  // ドローン2
  drone2_.name = "drone2";
  drone2_.pose_sub = nh_.subscribe(
      "/drone2/mavros/local_position/pose", 
      1, 
      &MultiDroneController::poseCallback2, 
      this
  );
  drone2_.vel_pub = nh_.advertise<geometry_msgs::Twist>(
      "/drone2/mavros/setpoint_velocity/cmd_vel_unstamped", 1
  );

  // ドローン3
  drone3_.name = "drone3";
  drone3_.pose_sub = nh_.subscribe(
      "/drone3/mavros/local_position/pose", 
      1, 
      &MultiDroneController::poseCallback3, 
      this
  );
  drone3_.vel_pub = nh_.advertise<geometry_msgs::Twist>(
      "/drone3/mavros/setpoint_velocity/cmd_vel_unstamped", 1
  );

  // PID初期化: [drone_index][0:x,1:y,2:z,3:yaw]
  double P = 1.0;
  double I = 0.0;
  double D = 0.0;
  double I_max = 0.5; 
  double I_min = 0.0;

  pid_controllers_.resize(3);
  for(int i=0; i<3; ++i)
  {
    pid_controllers_[i].push_back(SimplePID(P,I,D,I_max,I_min)); // X
    pid_controllers_[i].push_back(SimplePID(P,I,D,I_max,I_min)); // Y
    pid_controllers_[i].push_back(SimplePID(P,I,D,I_max,I_min)); // Z
    pid_controllers_[i].push_back(SimplePID(P,I,D,I_max,I_min)); // Yaw
  }

  prev_time_ = ros::Time::now();
}

//------------------------------------------------------------------------------
// joyコールバック
//------------------------------------------------------------------------------
void MultiDroneController::joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
  // 軸番号は実機に合わせて
  joy_x_   = msg->axes[0] * -1.0; 
  joy_y_   = msg->axes[1];
  joy_z_   = msg->axes[4];
  joy_yaw_ = msg->axes[3];

  buttons_ = msg->buttons;
}

//------------------------------------------------------------------------------
// poseCallback1
//------------------------------------------------------------------------------
void MultiDroneController::poseCallback1(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  drone1_.current_pose = msg->pose;
  if(!drone1_.first_pose_received)
  {
    drone1_.first_pose_received = true;
    ROS_INFO("Drone1 pose arrived");
  }
}

//------------------------------------------------------------------------------
// poseCallback2
//------------------------------------------------------------------------------
void MultiDroneController::poseCallback2(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  drone2_.current_pose = msg->pose;
  if(!drone2_.first_pose_received)
  {
    drone2_.first_pose_received = true;
    ROS_INFO("Drone2 pose arrived");
  }
}

//------------------------------------------------------------------------------
// poseCallback3
//------------------------------------------------------------------------------
void MultiDroneController::poseCallback3(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  drone3_.current_pose = msg->pose;
  if(!drone3_.first_pose_received)
  {
    drone3_.first_pose_received = true;
    ROS_INFO("Drone3 pose arrived");
  }
}

//------------------------------------------------------------------------------
// publishVelocityCommands
//------------------------------------------------------------------------------
void MultiDroneController::publishVelocityCommands(double dt)
{
  //=== まずは3機とも初回姿勢を受信しているか？ ===
  if(!all_received_)
  {
    if(drone1_.first_pose_received && drone2_.first_pose_received && drone3_.first_pose_received)
    {
      all_received_ = true;
      ROS_INFO("All drone poses are received. Start controlling formation.");
    }
  }
  if(!all_received_) return; // まだなら何もしない

  //========================================================================
  // A) ボタン1/2 で三角形全体を回転
  //========================================================================
  if(!buttons_.empty())
  {
    bool btn1 = (buttons_.size()>0)? (buttons_[0] == 1) : false; // ボタン1
    bool btn2 = (buttons_.size()>1)? (buttons_[1] == 1) : false; // ボタン2
    // ボタン1: 時計回り → angleを減らす
    if(btn1) formation_angle_ -= rotation_speed_ * dt;
    // ボタン2: 反時計回り → angleを増やす
    if(btn2) formation_angle_ += rotation_speed_ * dt;
  }

  //========================================================================
  // B) ジョイスティック (axes[0], axes[1]) で編隊の中心を移動
  //========================================================================
  // 例えば: “スティックの倒れ具合×move_scale_” を dt秒かけ続ける → そのぶん中心が移動
  formation_center_x_ += joy_x_ * move_scale_ * dt;
  formation_center_y_ += joy_y_ * move_scale_ * dt;

  //========================================================================
  // C) Z方向(axes[4]) を使って高度 (formation_z_) を上下
  //========================================================================
  formation_z_ += joy_z_ * 0.3 * dt; 
  if(formation_z_ < 0.5) formation_z_ = 0.5; // 下限

  //========================================================================
  // D) 各ドローンに対して「三角形の回転座標 + 中心 + 高さ + Yaw」をPID制御
  //========================================================================
  // ---------- Drone1 ----------
  {
    double cosA = std::cos(formation_angle_);
    double sinA = std::sin(formation_angle_);
    // もともとのオフセットに回転行列を掛ける
    double rx = drone1_offset_x_ * cosA - drone1_offset_y_ * sinA;
    double ry = drone1_offset_x_ * sinA + drone1_offset_y_ * cosA;

    // 目標位置 = (編隊中心 + 回転後オフセット, formation_z_)
    double desired_x = formation_center_x_ + rx;
    double desired_y = formation_center_y_ + ry;
    double desired_z = formation_z_;

    double current_x = drone1_.current_pose.position.x;
    double current_y = drone1_.current_pose.position.y;
    double current_z = drone1_.current_pose.position.z;

    double err_x = desired_x - current_x;
    double err_y = desired_y - current_y;
    double err_z = desired_z - current_z;

    double vx = pid_controllers_[0][0].computeCommand(err_x, dt);
    double vy = pid_controllers_[0][1].computeCommand(err_y, dt);
    double vz = pid_controllers_[0][2].computeCommand(err_z, dt);

    // yaw: “三角形の向き(formation_angle_) を全機で共有する” としてみる
    double yaw_now = getYaw(drone1_.current_pose);
    double yaw_err = formation_angle_ - yaw_now;
    while(yaw_err >  M_PI) yaw_err -= 2.0*M_PI;
    while(yaw_err < -M_PI) yaw_err += 2.0*M_PI;

    double wz = pid_controllers_[0][3].computeCommand(yaw_err, dt);

    // clip
    double max_xy = 1.0;
    if(vx >  max_xy) vx =  max_xy;
    if(vx < -max_xy) vx = -max_xy;
    if(vy >  max_xy) vy =  max_xy;
    if(vy < -max_xy) vy = -max_xy;

    double max_z = 0.5;
    if(vz >  max_z) vz =  max_z;
    if(vz < -max_z) vz = -max_z;

    double max_wz = 1.0;
    if(wz >  max_wz) wz =  max_wz;
    if(wz < -max_wz) wz = -max_wz;

    geometry_msgs::Twist cmd;
    cmd.linear.x  = vx;
    cmd.linear.y  = vy;
    cmd.linear.z  = vz;
    cmd.angular.z = wz;

    drone1_.vel_pub.publish(cmd);
  }

  // ---------- Drone2 ----------
  {
    double cosA = std::cos(formation_angle_);
    double sinA = std::sin(formation_angle_);
    double rx = drone2_offset_x_ * cosA - drone2_offset_y_ * sinA;
    double ry = drone2_offset_x_ * sinA + drone2_offset_y_ * cosA;

    double desired_x = formation_center_x_ + rx;
    double desired_y = formation_center_y_ + ry;
    double desired_z = formation_z_;

    double current_x = drone2_.current_pose.position.x;
    double current_y = drone2_.current_pose.position.y;
    double current_z = drone2_.current_pose.position.z;

    double err_x = desired_x - current_x;
    double err_y = desired_y - current_y;
    double err_z = desired_z - current_z;

    double vx = pid_controllers_[1][0].computeCommand(err_x, dt);
    double vy = pid_controllers_[1][1].computeCommand(err_y, dt);
    double vz = pid_controllers_[1][2].computeCommand(err_z, dt);

    double yaw_now = getYaw(drone2_.current_pose);
    double yaw_err = formation_angle_ - yaw_now;
    while(yaw_err >  M_PI) yaw_err -= 2.0*M_PI;
    while(yaw_err < -M_PI) yaw_err += 2.0*M_PI;

    double wz = pid_controllers_[1][3].computeCommand(yaw_err, dt);

    // clip
    double max_xy = 1.0;
    if(vx >  max_xy) vx =  max_xy;
    if(vx < -max_xy) vx = -max_xy;
    if(vy >  max_xy) vy =  max_xy;
    if(vy < -max_xy) vy = -max_xy;

    double max_z = 0.5;
    if(vz >  max_z) vz =  max_z;
    if(vz < -max_z) vz = -max_z;

    double max_wz = 1.0;
    if(wz >  max_wz) wz =  max_wz;
    if(wz < -max_wz) wz = -max_wz;

    geometry_msgs::Twist cmd;
    cmd.linear.x  = vx;
    cmd.linear.y  = vy;
    cmd.linear.z  = vz;
    cmd.angular.z = wz;

    drone2_.vel_pub.publish(cmd);
  }

  // ---------- Drone3 ----------
  {
    double cosA = std::cos(formation_angle_);
    double sinA = std::sin(formation_angle_);
    double rx = drone3_offset_x_ * cosA - drone3_offset_y_ * sinA;
    double ry = drone3_offset_x_ * sinA + drone3_offset_y_ * cosA;

    double desired_x = formation_center_x_ + rx;
    double desired_y = formation_center_y_ + ry;
    double desired_z = formation_z_;

    double current_x = drone3_.current_pose.position.x;
    double current_y = drone3_.current_pose.position.y;
    double current_z = drone3_.current_pose.position.z;

    double err_x = desired_x - current_x;
    double err_y = desired_y - current_y;
    double err_z = desired_z - current_z;

    double vx = pid_controllers_[2][0].computeCommand(err_x, dt);
    double vy = pid_controllers_[2][1].computeCommand(err_y, dt);
    double vz = pid_controllers_[2][2].computeCommand(err_z, dt);

    double yaw_now = getYaw(drone3_.current_pose);
    double yaw_err = formation_angle_ - yaw_now;
    while(yaw_err >  M_PI) yaw_err -= 2.0*M_PI;
    while(yaw_err < -M_PI) yaw_err += 2.0*M_PI;

    double wz = pid_controllers_[2][3].computeCommand(yaw_err, dt);

    // clip
    double max_xy = 1.0;
    if(vx >  max_xy) vx =  max_xy;
    if(vx < -max_xy) vx = -max_xy;
    if(vy >  max_xy) vy =  max_xy;
    if(vy < -max_xy) vy = -max_xy;

    double max_z = 0.5;
    if(vz >  max_z) vz =  max_z;
    if(vz < -max_z) vz = -max_z;

    double max_wz = 1.0;
    if(wz >  max_wz) wz =  max_wz;
    if(wz < -max_wz) wz = -max_wz;

    geometry_msgs::Twist cmd;
    cmd.linear.x  = vx;
    cmd.linear.y  = vy;
    cmd.linear.z  = vz;
    cmd.angular.z = wz;

    drone3_.vel_pub.publish(cmd);
  }
}

//------------------------------------------------------------------------------
// メインループ
//------------------------------------------------------------------------------
void MultiDroneController::run()
{
  ros::Rate rate(20.0);

  while(ros::ok())
  {
    ros::spinOnce();

    ros::Time now = ros::Time::now();
    double dt = (now - prev_time_).toSec();
    if(dt <= 0) dt = 0.05;
    prev_time_ = now;

    publishVelocityCommands(dt);

    rate.sleep();
  }
}

//------------------------------------------------------------------------------
// main
//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_drone_controller");

  MultiDroneController controller;
  controller.run();

  return 0;
}
