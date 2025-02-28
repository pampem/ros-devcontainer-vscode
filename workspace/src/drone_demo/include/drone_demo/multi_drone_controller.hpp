#ifndef MULTI_DRONE_CONTROLLER_HPP
#define MULTI_DRONE_CONTROLLER_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

// PID制御用に簡易的なクラス（P制御のみ例示）
class SimplePID
{
public:
  SimplePID(double p_gain, double i_gain, double d_gain, double i_max, double i_min)
  : p_gain_(p_gain), i_gain_(i_gain), d_gain_(d_gain),
    i_max_(i_max), i_min_(i_min), prev_error_(0.0), integral_(0.0)
  {
  }

  double computeCommand(double error, double dt)
  {
    // P項
    double p_term = p_gain_ * error;

    // I項
    integral_ += error * dt;
    // windup防止
    if (integral_ > i_max_) integral_ = i_max_;
    if (integral_ < i_min_) integral_ = i_min_;
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

// ドローン1機分の状態を管理するための構造体
struct DroneInfo
{
  std::string name;                // 例: "drone1"
  geometry_msgs::Pose current_pose;
  geometry_msgs::Pose home_pose;   // “基準位置” (離陸後、あるいは初期位置とする)
  ros::Subscriber pose_sub;
  ros::Publisher vel_pub;
  ros::ServiceClient set_mode_client;
  bool first_pose_received = false;
};

class MultiDroneController
{
public:
  MultiDroneController();
  ~MultiDroneController(){}

  void run(); // メインループを回す

private:
  // コールバック
  void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
  void poseCallback1(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void poseCallback2(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void poseCallback3(const geometry_msgs::PoseStamped::ConstPtr &msg);

  // 各ドローンに対して速度指令を送る
  void publishVelocityCommands(double dt);

  // PIDコントローラ配列
  // 本例では x, y, z それぞれの制御にSimplePIDを用いる想定
  // [drone_index][axis_index(0:x,1:y,2:z)]
  std::vector<std::vector<SimplePID>> pid_controllers_;

  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;

  // ドローン情報 (3機分)
  DroneInfo drone1_, drone2_, drone3_;

  // ジョイスティックからの入力値を格納
  double joy_x_; // 左右スティック (例: axes[0])
  double joy_y_; // 前後スティック (例: axes[1])

  // ジョイスティック入力をどの程度のオフセット(目標位置変位)に変換するかのスケール
  double joy_scale_;

  // 動作範囲の制限 (±何m動けるか)
  double max_offset_;

  // 前回ループの時刻
  ros::Time prev_time_;

  // 各ドローンの「三角形配置」での相対位置(ホーム座標との差分)を設定
  // 例: drone1(0,0), drone2(-0.5, -0.5), drone3(0.5, -0.5) 高度は1mと仮定
  // 実際にはパラメータ等から読んでも良い
};

#endif // MULTI_DRONE_CONTROLLER_HPP
