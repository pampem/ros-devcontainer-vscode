#include "drone_demo/multi_drone_controller.hpp"

MultiDroneController::MultiDroneController()
: joy_x_(0.0), joy_y_(0.0)
{
  //***********************
  // パラメータの読み込み
  //***********************
  // 例えばパラメータサーバーから読み込みたい場合は以下のようにする
  // nh_.param("joy_scale", joy_scale_, 1.0);
  // nh_.param("max_offset", max_offset_, 1.0);

  // ここでは直接値を代入する例にします
  joy_scale_  = 0.5;   // ジョイスティック最大倒しで、±0.5[m]くらい動くイメージ
  max_offset_ = 1.0;   // 目標オフセットは最大±1.0[m]に制限

  joy_sub_ = nh_.subscribe("/joy", 1, &MultiDroneController::joyCallback, this);

  // ドローン1
  drone1_.name = "drone1";
  drone1_.pose_sub = nh_.subscribe("/drone1/mavros/local_position/pose", 1, 
                                   &MultiDroneController::poseCallback1, this);
  drone1_.vel_pub = nh_.advertise<geometry_msgs::Twist>("/drone1/mavros/setpoint_velocity/cmd_vel_unstamped", 1);
  drone1_.set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("/drone1/mavros/set_mode");

  // ドローン2
  drone2_.name = "drone2";
  drone2_.pose_sub = nh_.subscribe("/drone2/mavros/local_position/pose", 1, 
                                   &MultiDroneController::poseCallback2, this);
  drone2_.vel_pub = nh_.advertise<geometry_msgs::Twist>("/drone2/mavros/setpoint_velocity/cmd_vel_unstamped", 1);
  drone2_.set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("/drone2/mavros/set_mode");

  // ドローン3
  drone3_.name = "drone3";
  drone3_.pose_sub = nh_.subscribe("/drone3/mavros/local_position/pose", 1, 
                                   &MultiDroneController::poseCallback3, this);
  drone3_.vel_pub = nh_.advertise<geometry_msgs::Twist>("/drone3/mavros/setpoint_velocity/cmd_vel_unstamped", 1);
  drone3_.set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("/drone3/mavros/set_mode");

  //***********************
  // PIDコントローラの初期化
  //***********************
  // 例: P=1.0, I=0.0, D=0.0, 積分制限±0.0

  double P = 1.0;
  double I = 0.0;
  double D = 0.0;
  double I_max = 0.0;
  double I_min = 0.0;

  // [drone_index][x,y,z]で3機×3軸 = 3x3
  pid_controllers_.resize(3);
  for (int i = 0; i < 3; ++i)
  {
    pid_controllers_[i].push_back(SimplePID(P,I,D,I_max,I_min)); // X軸
    pid_controllers_[i].push_back(SimplePID(P,I,D,I_max,I_min)); // Y軸
    pid_controllers_[i].push_back(SimplePID(P,I,D,I_max,I_min)); // Z軸
  }

  prev_time_ = ros::Time::now();
}

void MultiDroneController::joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
  // 例として、axes[0] = 左右, axes[1] = 前後 として使用
  // 実際のコントローラに合わせてインデックスを調整してください
  joy_x_ = msg->axes[0];
  joy_y_ = msg->axes[1];
}

void MultiDroneController::poseCallback1(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  drone1_.current_pose = msg->pose;
  if(!drone1_.first_pose_received)
  {
    // 初回受信時にhome_poseを設定
    drone1_.home_pose = msg->pose;
    drone1_.first_pose_received = true;
  }
}

void MultiDroneController::poseCallback2(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  drone2_.current_pose = msg->pose;
  if(!drone2_.first_pose_received)
  {
    drone2_.home_pose = msg->pose;
    drone2_.first_pose_received = true;
  }
}

void MultiDroneController::poseCallback3(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  drone3_.current_pose = msg->pose;
  if(!drone3_.first_pose_received)
  {
    drone3_.home_pose = msg->pose;
    drone3_.first_pose_received = true;
  }
}

// 各ドローンの速度コマンドを発行
void MultiDroneController::publishVelocityCommands(double dt)
{
  // それぞれのドローンについて、現在姿勢と目標位置を計算 → PID → Twistで速度指令をパブリッシュ
  // --------------------------------------------------
  // ドローン1の計算
  // --------------------------------------------------
  if(drone1_.first_pose_received)
  {
    // ジョイスティック入力から、目標位置(home + offset) を決定
    // xy平面方向のみ: joy_x_, joy_y_ をscaleし、±max_offset_にクリップ
    double target_offset_x = joy_x_ * joy_scale_;
    double target_offset_y = joy_y_ * joy_scale_;

    if(target_offset_x >  max_offset_) target_offset_x =  max_offset_;
    if(target_offset_x < -max_offset_) target_offset_x = -max_offset_;
    if(target_offset_y >  max_offset_) target_offset_y =  max_offset_;
    if(target_offset_y < -max_offset_) target_offset_y = -max_offset_;

    double desired_x = drone1_.home_pose.position.x + target_offset_x;
    double desired_y = drone1_.home_pose.position.y + target_offset_y;
    double desired_z = drone1_.home_pose.position.z; // 高度は維持(初期値)

    double current_x = drone1_.current_pose.position.x;
    double current_y = drone1_.current_pose.position.y;
    double current_z = drone1_.current_pose.position.z;

    double error_x = desired_x - current_x;
    double error_y = desired_y - current_y;
    double error_z = desired_z - current_z;

    // PIDから速度コマンド算出
    double vx = pid_controllers_[0][0].computeCommand(error_x, dt);
    double vy = pid_controllers_[0][1].computeCommand(error_y, dt);
    double vz = pid_controllers_[0][2].computeCommand(error_z, dt);

    // 安全のため速度をクリップ
    double max_vel = 0.5; // [m/s] 例: 0.5
    if(vx >  max_vel) vx =  max_vel;
    if(vx < -max_vel) vx = -max_vel;
    if(vy >  max_vel) vy =  max_vel;
    if(vy < -max_vel) vy = -max_vel;
    if(vz >  max_vel) vz =  max_vel;
    if(vz < -max_vel) vz = -max_vel;

    // Twistメッセージ
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vx;
    cmd_vel.linear.y = vy;
    cmd_vel.linear.z = vz;
    cmd_vel.angular.z = 0.0;

    drone1_.vel_pub.publish(cmd_vel);
  }

  // --------------------------------------------------
  // ドローン2の計算
  // --------------------------------------------------
  if(drone2_.first_pose_received)
  {
    double target_offset_x = joy_x_ * joy_scale_;
    double target_offset_y = joy_y_ * joy_scale_;
    if(target_offset_x >  max_offset_) target_offset_x =  max_offset_;
    if(target_offset_x < -max_offset_) target_offset_x = -max_offset_;
    if(target_offset_y >  max_offset_) target_offset_y =  max_offset_;
    if(target_offset_y < -max_offset_) target_offset_y = -max_offset_;

    double desired_x = drone2_.home_pose.position.x + target_offset_x;
    double desired_y = drone2_.home_pose.position.y + target_offset_y;
    double desired_z = drone2_.home_pose.position.z;

    double current_x = drone2_.current_pose.position.x;
    double current_y = drone2_.current_pose.position.y;
    double current_z = drone2_.current_pose.position.z;

    double error_x = desired_x - current_x;
    double error_y = desired_y - current_y;
    double error_z = desired_z - current_z;

    double vx = pid_controllers_[1][0].computeCommand(error_x, dt);
    double vy = pid_controllers_[1][1].computeCommand(error_y, dt);
    double vz = pid_controllers_[1][2].computeCommand(error_z, dt);

    double max_vel = 0.5;
    if(vx >  max_vel) vx =  max_vel;
    if(vx < -max_vel) vx = -max_vel;
    if(vy >  max_vel) vy =  max_vel;
    if(vy < -max_vel) vy = -max_vel;
    if(vz >  max_vel) vz =  max_vel;
    if(vz < -max_vel) vz = -max_vel;

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vx;
    cmd_vel.linear.y = vy;
    cmd_vel.linear.z = vz;
    drone2_.vel_pub.publish(cmd_vel);
  }

  // --------------------------------------------------
  // ドローン3の計算
  // --------------------------------------------------
  if(drone3_.first_pose_received)
  {
    double target_offset_x = joy_x_ * joy_scale_;
    double target_offset_y = joy_y_ * joy_scale_;
    if(target_offset_x >  max_offset_) target_offset_x =  max_offset_;
    if(target_offset_x < -max_offset_) target_offset_x = -max_offset_;
    if(target_offset_y >  max_offset_) target_offset_y =  max_offset_;
    if(target_offset_y < -max_offset_) target_offset_y = -max_offset_;

    double desired_x = drone3_.home_pose.position.x + target_offset_x;
    double desired_y = drone3_.home_pose.position.y + target_offset_y;
    double desired_z = drone3_.home_pose.position.z;

    double current_x = drone3_.current_pose.position.x;
    double current_y = drone3_.current_pose.position.y;
    double current_z = drone3_.current_pose.position.z;

    double error_x = desired_x - current_x;
    double error_y = desired_y - current_y;
    double error_z = desired_z - current_z;

    double vx = pid_controllers_[2][0].computeCommand(error_x, dt);
    double vy = pid_controllers_[2][1].computeCommand(error_y, dt);
    double vz = pid_controllers_[2][2].computeCommand(error_z, dt);

    double max_vel = 0.5;
    if(vx >  max_vel) vx =  max_vel;
    if(vx < -max_vel) vx = -max_vel;
    if(vy >  max_vel) vy =  max_vel;
    if(vy < -max_vel) vy = -max_vel;
    if(vz >  max_vel) vz =  max_vel;
    if(vz < -max_vel) vz = -max_vel;

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vx;
    cmd_vel.linear.y = vy;
    cmd_vel.linear.z = vz;
    drone3_.vel_pub.publish(cmd_vel);
  }
}

void MultiDroneController::run()
{
  ros::Rate rate(20.0); // 20Hz

  while(ros::ok())
  {
    ros::spinOnce();

    ros::Time now = ros::Time::now();
    double dt = (now - prev_time_).toSec();
    if (dt <= 0) dt = 0.05;
    prev_time_ = now;

    // 各ドローンに速度コマンドを送る
    publishVelocityCommands(dt);

    rate.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_drone_controller");

  MultiDroneController controller;
  controller.run();

  return 0;
}
