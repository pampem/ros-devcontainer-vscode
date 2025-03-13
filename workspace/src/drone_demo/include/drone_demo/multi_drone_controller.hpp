#ifndef MULTI_DRONE_CONTROLLER_HPP
#define MULTI_DRONE_CONTROLLER_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <vector>
#include <string>

// PIDクラス（簡易版）
class SimplePID
{
public:
  SimplePID(double p, double i, double d, double i_max, double i_min)
  : p_gain_(p), i_gain_(i), d_gain_(d),
    i_max_(i_max), i_min_(i_min), prev_error_(0.0), integral_(0.0)
  {}

  double computeCommand(double error, double dt)
  {
    double p_term = p_gain_ * error;

    // I項
    integral_ += error * dt;
    if(integral_ > i_max_) integral_ = i_max_;
    if(integral_ < i_min_) integral_ = i_min_;
    double i_term = i_gain_ * integral_;

    // D項
    double derivative = (error - prev_error_) / dt;
    double d_term = d_gain_ * derivative;

    prev_error_ = error;
    return p_term + i_term + d_term;
  }

private:
  double p_gain_;
  double i_gain_;
  double d_gain_;
  double i_max_;
  double i_min_;

  double prev_error_;
  double integral_;
};

struct DroneInfo
{
  std::string name;
  geometry_msgs::Pose current_pose;
  bool first_pose_received = false;

  ros::Subscriber pose_sub;
  ros::Publisher vel_pub;
};

//----------------------------------------------------------------------
// メインクラス
//----------------------------------------------------------------------
class MultiDroneController
{
public:
  MultiDroneController();
  ~MultiDroneController(){}

  // メインループ
  void run();

private:
  // コールバック
  void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
  void poseCallback1(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void poseCallback2(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void poseCallback3(const geometry_msgs::PoseStamped::ConstPtr &msg);

  // 速度コマンド発行
  void publishVelocityCommands(double dt);

  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;

  DroneInfo drone1_, drone2_, drone3_;

  // ジョイスティック
  double joy_x_;   // x移動
  double joy_y_;   // y移動
  double joy_z_;   // 高さ変更(必要なら)
  double joy_yaw_; // ここでは使っていない例
  std::vector<int> buttons_;

  // フォーメーション用PID : DroneごとにX,Y,Z,Yawを制御するとして4軸ぶん確保
  std::vector<std::vector<SimplePID>> pid_controllers_; // [drone_index][0:x,1:y,2:z,3:yaw]

  //=== 三角形フォーメーションの設定 ===
  // 例: center = (0,0) に対して
  // Drone1=(0, +0.5), Drone2=(-0.5, -0.5), Drone3=(+0.5, -0.5)
  double drone1_offset_x_, drone1_offset_y_;
  double drone2_offset_x_, drone2_offset_y_;
  double drone3_offset_x_, drone3_offset_y_;

  //=== 編隊の中心と回転角 ===
  double formation_center_x_;
  double formation_center_y_;
  double formation_angle_;

  // 回転速度[rad/s]
  double rotation_speed_;

  //======================
  // その他パラメータ
  //======================
  double formation_z_;   // 全機の高さ
  double move_scale_;    // x,yスティックで中心を動かすスケール
  bool   all_received_;  // 全機姿勢を受信したかどうか

  // タイミング計測
  ros::Time prev_time_;
};

#endif // MULTI_DRONE_CONTROLLER_HPP
