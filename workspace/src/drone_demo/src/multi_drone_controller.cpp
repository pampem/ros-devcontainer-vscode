#include "drone_demo/multi_drone_controller.hpp"

MultiDroneController::MultiDroneController()
: joy_x_(0.0), joy_y_(0.0), joy_z_(0.0), joy_yaw_(0.0) // ← 追加した変数を初期化
{
  //***********************
  // パラメータの読み込み
  //***********************
  // ここでは直接値を代入するサンプル
  joy_scale_  = 0.5;   
  max_offset_ = 1.0;   

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
  double P = 1.0;
  double I = 0.0;
  double D = 0.0;
  double I_max = 0.0;
  double I_min = 0.0;

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
  // 例として、axes[0] = 左右(x), axes[1] = 前後(y) を既存で使っていた
  // ここで axes[3] = yaw, axes[4] = 上下(z) として受け取る例
  // コントローラによっては軸番号が違う場合もあるので注意
  joy_x_   = (msg->axes[0])*-1;
  joy_y_   = msg->axes[1];
  joy_yaw_ = msg->axes[3];  // 右スティック 左右
  joy_z_   = msg->axes[4];  // 右スティック 上下
}

void MultiDroneController::poseCallback1(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  drone1_.current_pose = msg->pose;
  if(!drone1_.first_pose_received)
  {
    drone1_.home_pose = msg->pose;
    drone1_.first_pose_received = true;
    ROS_INFO("First pose of drone1 arrived");
  }
}

void MultiDroneController::poseCallback2(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  drone2_.current_pose = msg->pose;
  if(!drone2_.first_pose_received)
  {
    drone2_.home_pose = msg->pose;
    drone2_.first_pose_received = true;
    ROS_INFO("First pose of drone2 arrived");
  }
}

void MultiDroneController::poseCallback3(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  drone3_.current_pose = msg->pose;
  if(!drone3_.first_pose_received)
  {
    drone3_.home_pose = msg->pose;
    drone3_.first_pose_received = true;
    ROS_INFO("First pose of drone3 arrived");
  }
}

void MultiDroneController::publishVelocityCommands(double dt)
{
  // --------------------------------------------------
  // ドローン1
  // --------------------------------------------------
  if(drone1_.first_pose_received)
  {
    // ★ Z（上下）方向のオフセットもジョイスティックから計算
    double target_offset_x = joy_x_ * joy_scale_;
    double target_offset_y = joy_y_ * joy_scale_;
    double target_offset_z = joy_z_ * joy_scale_; // 追加: 上下オフセット

    // クリップ
    if(target_offset_x >  max_offset_) target_offset_x =  max_offset_;
    if(target_offset_x < -max_offset_) target_offset_x = -max_offset_;
    if(target_offset_y >  max_offset_) target_offset_y =  max_offset_;
    if(target_offset_y < -max_offset_) target_offset_y = -max_offset_;
    if(target_offset_z >  max_offset_) target_offset_z =  max_offset_;
    if(target_offset_z < -max_offset_) target_offset_z = -max_offset_;

    double desired_x = drone1_.home_pose.position.x + target_offset_x;
    double desired_y = drone1_.home_pose.position.y + target_offset_y;
    double desired_z = drone1_.home_pose.position.z + target_offset_z; // ← ここが変更点

    double current_x = drone1_.current_pose.position.x;
    double current_y = drone1_.current_pose.position.y;
    double current_z = drone1_.current_pose.position.z;

    double error_x = desired_x - current_x;
    double error_y = desired_y - current_y;
    double error_z = desired_z - current_z;

    double vx = pid_controllers_[0][0].computeCommand(error_x, dt);
    double vy = pid_controllers_[0][1].computeCommand(error_y, dt);
    double vz = pid_controllers_[0][2].computeCommand(error_z, dt);

    double max_vel = 0.5; // 速度上限
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

    // ★ yawは角速度で直接コマンド送る（シンプルにスケール値かけるだけなど）
    double yaw_scale = 1.0; // 好きに調整可(最大何[rad/s]にするか等)
    cmd_vel.angular.z = joy_yaw_ * yaw_scale;

    drone1_.vel_pub.publish(cmd_vel);
  }

  // --------------------------------------------------
  // ドローン2
  // --------------------------------------------------
  if(drone2_.first_pose_received)
  {
    double target_offset_x = joy_x_ * joy_scale_;
    double target_offset_y = joy_y_ * joy_scale_;
    double target_offset_z = joy_z_ * joy_scale_;

    if(target_offset_x >  max_offset_) target_offset_x =  max_offset_;
    if(target_offset_x < -max_offset_) target_offset_x = -max_offset_;
    if(target_offset_y >  max_offset_) target_offset_y =  max_offset_;
    if(target_offset_y < -max_offset_) target_offset_y = -max_offset_;
    if(target_offset_z >  max_offset_) target_offset_z =  max_offset_;
    if(target_offset_z < -max_offset_) target_offset_z = -max_offset_;

    double desired_x = drone2_.home_pose.position.x + target_offset_x;
    double desired_y = drone2_.home_pose.position.y + target_offset_y;
    double desired_z = drone2_.home_pose.position.z + target_offset_z;

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

    double yaw_scale = 1.0;
    cmd_vel.angular.z = joy_yaw_ * yaw_scale;

    drone2_.vel_pub.publish(cmd_vel);
  }

  // --------------------------------------------------
  // ドローン3
  // --------------------------------------------------
  if(drone3_.first_pose_received)
  {
    double target_offset_x = joy_x_ * joy_scale_;
    double target_offset_y = joy_y_ * joy_scale_;
    double target_offset_z = joy_z_ * joy_scale_;

    if(target_offset_x >  max_offset_) target_offset_x =  max_offset_;
    if(target_offset_x < -max_offset_) target_offset_x = -max_offset_;
    if(target_offset_y >  max_offset_) target_offset_y =  max_offset_;
    if(target_offset_y < -max_offset_) target_offset_y = -max_offset_;
    if(target_offset_z >  max_offset_) target_offset_z =  max_offset_;
    if(target_offset_z < -max_offset_) target_offset_z = -max_offset_;

    double desired_x = drone3_.home_pose.position.x + target_offset_x;
    double desired_y = drone3_.home_pose.position.y + target_offset_y;
    double desired_z = drone3_.home_pose.position.z + target_offset_z;

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

    double yaw_scale = 1.0;
    cmd_vel.angular.z = joy_yaw_ * yaw_scale;

    drone3_.vel_pub.publish(cmd_vel);
  }
}

void MultiDroneController::run()
{
  ros::Rate rate(20.0);

  while(ros::ok())
  {
    ros::spinOnce();

    ros::Time now = ros::Time::now();
    double dt = (now - prev_time_).toSec();
    if (dt <= 0) dt = 0.05;
    prev_time_ = now;

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
