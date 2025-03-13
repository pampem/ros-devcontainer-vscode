#include "drone_demo/formation_control.hpp"
#include <std_msgs/Float32MultiArray.h>
#include <string>
#include <cmath>

//==============================================================
// MyDrone クラス実装
//==============================================================
MyDrone::MyDrone(const std::string &drone_ns, ros::NodeHandle &nh)
{
    setpoint_pub_ = nh.advertise<mavros_msgs::PositionTarget>(
        drone_ns + "mavros/setpoint_raw/local", 1);

    // PositionTarget メッセージ初期設定 (FRAME_LOCAL_NED)
    position_target_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    // デフォルトでは位置以外を無視
    position_target_.type_mask =
        (   mavros_msgs::PositionTarget::IGNORE_VX
          | mavros_msgs::PositionTarget::IGNORE_VY
          | mavros_msgs::PositionTarget::IGNORE_VZ
          | mavros_msgs::PositionTarget::IGNORE_AFX
          | mavros_msgs::PositionTarget::IGNORE_AFY
          | mavros_msgs::PositionTarget::IGNORE_AFZ
          | mavros_msgs::PositionTarget::IGNORE_YAW_RATE
        );
}

void MyDrone::setTargetPosition(double x, double y, double z, double yaw)
{
    position_target_.position.x = x;
    position_target_.position.y = y;
    position_target_.position.z = z;
    position_target_.yaw        = yaw;

    setpoint_pub_.publish(position_target_);
}

void MyDrone::stopPubPosVelAcc()
{
    // すべての成分を無視
    position_target_.type_mask =
        (   mavros_msgs::PositionTarget::IGNORE_PX
          | mavros_msgs::PositionTarget::IGNORE_PY
          | mavros_msgs::PositionTarget::IGNORE_PZ
          | mavros_msgs::PositionTarget::IGNORE_VX
          | mavros_msgs::PositionTarget::IGNORE_VY
          | mavros_msgs::PositionTarget::IGNORE_VZ
          | mavros_msgs::PositionTarget::IGNORE_AFX
          | mavros_msgs::PositionTarget::IGNORE_AFY
          | mavros_msgs::PositionTarget::IGNORE_AFZ
          | mavros_msgs::PositionTarget::IGNORE_YAW
        );
    setpoint_pub_.publish(position_target_);
}


//==============================================================
// FormationController クラス実装
//==============================================================
FormationController::FormationController(ros::NodeHandle &nh)
    : drone1_("/drone1/", nh)
    , drone2_("/drone2/", nh)
    , drone3_("/drone3/", nh)
    , base_waypoint_index_(0)
    , target_z_(1.5)   // デフォ高度
    , target_yaw_(0.0) // デフォヨー角
    , button1_pressed_(false)
    , button2_pressed_(false)
{
    // Joy購読
    joy_sub_ = nh.subscribe<sensor_msgs::Joy>(
        "/joy", 10, &FormationController::joyCallback, this
    );

    formation_timer_ = nh.createTimer(
        ros::Duration(0.2),
        &FormationController::timerCallback,
        this
    );

    // 円形ウェイポイント生成 (例: 半径2.0, 分割数36)
    initCircleWaypoints(1.0, 60);

    // Joyの axes / buttons を念のため初期化
    axes_.resize(8, 0.0f);
    buttons_.resize(8, 0);

    drone1_home.x = 0.0;
    drone1_home.y = 0.0;
    drone1_home.z = 0.0;

    drone2_home.x = -2.0;
    drone2_home.y = 2.0;
    drone2_home.z = 0.0;

    drone3_home.x = 2.0;
    drone3_home.y = 2.0;
    drone3_home.z = 0.0;
}

void FormationController::initCircleWaypoints(double radius, int num_points)
{
    circle_waypoints_.clear();
    circle_waypoints_.reserve(num_points);

    for(int i = 0; i < num_points; ++i)
    {
        double theta = 2.0 * M_PI * i / num_points;
        geometry_msgs::Point p;
        p.x = radius * std::cos(theta);
        p.y = radius * std::sin(theta);
        p.z = 0.0;  // XY平面
        circle_waypoints_.push_back(p);
    }
}

void FormationController::joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
    // 軸とボタンをメンバにコピー
    axes_   = msg->axes;
    buttons_ = msg->buttons;

    // ボタンが押されているかどうかを保持
    // 例: button1 => buttons[0], button2 => buttons[1] (環境に応じて変更)
    if(buttons_.size() >= 2) {
        button1_pressed_ = (buttons_[0] == 1);
        button2_pressed_ = (buttons_[1] == 1);
    }

    // 高度とヨー角だけスティック入力から拾う（必要なら変更）
    // 例: axes_[4] => 高度, axes_[3] => ヨー
    if(axes_.size() > 4) {
        target_z_   = 1.5 + axes_[4] * 1.0;  // ±1 の範囲なら高度を 1.5±1.0 にする例
        target_yaw_ = axes_[3];             // -1 ~ +1 をそのままラジアンとして使う例
    }
}

void FormationController::timerCallback(const ros::TimerEvent &)
{
    // ボタン1 or 2 が押されていれば円周のインデックスを更新
    if(button1_pressed_ || button2_pressed_)
    {
        // ボタン1 => 時計回り: index++
        // ボタン2 => 反時計回り: index--
        // (両方押したらどうする？などは適宜調整)

        if(button1_pressed_) base_waypoint_index_++;
        if(button2_pressed_) base_waypoint_index_--;

        // インデックスが範囲外にならないようにラップ
        int n = circle_waypoints_.size();
        if(base_waypoint_index_ < 0) base_waypoint_index_ = n - 1;
        if(base_waypoint_index_ >= n) base_waypoint_index_ = 0;
    }

    // 円形フォーメーションを適用 (毎回実行し、高度やヨー角の変更にも追従)
    applyFormation();
}

void FormationController::applyFormation()
{
    int n = circle_waypoints_.size();
    if(n == 0) return;

    int offset2 = n / 3;
    int offset3 = (2 * n) / 3;

    int idx1 = (base_waypoint_index_ + 0) % n;
    int idx2 = (base_waypoint_index_ + offset2) % n;
    int idx3 = (base_waypoint_index_ + offset3) % n;

    // グローバルなウェイポイント
    geometry_msgs::Point wp_global1 = circle_waypoints_[idx1];
    geometry_msgs::Point wp_global2 = circle_waypoints_[idx2];
    geometry_msgs::Point wp_global3 = circle_waypoints_[idx3];

    // 各ドローンのローカル座標系に変換
    geometry_msgs::Point wp_local1, wp_local2, wp_local3;
    wp_local1.x = wp_global1.x - drone1_home.x;
    wp_local1.y = wp_global1.y - drone1_home.y;
    wp_local1.z = target_z_;

    wp_local2.x = wp_global2.x - drone2_home.x;
    wp_local2.y = wp_global2.y - drone2_home.y;
    wp_local2.z = target_z_;

    wp_local3.x = wp_global3.x - drone3_home.x;
    wp_local3.y = wp_global3.y - drone3_home.y;
    wp_local3.z = target_z_;

    drone1_.setTargetPosition(wp_local1.x, wp_local1.y, wp_local1.z, target_yaw_);
    drone2_.setTargetPosition(wp_local2.x, wp_local2.y, wp_local2.z, target_yaw_);
    drone3_.setTargetPosition(wp_local3.x, wp_local3.y, wp_local3.z, target_yaw_);

    ROS_INFO_STREAM_THROTTLE(1.0, "[applyFormation] idx1=" << idx1
        << ", idx2=" << idx2
        << ", idx3=" << idx3
        << " (Z=" << target_z_ << ", Yaw=" << target_yaw_ << ")");
}


//==============================================================
// メイン関数
//==============================================================
int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation_controller_cpp");
    ros::NodeHandle nh("~");

    FormationController controller(nh);

    ros::spin();
    return 0;
}
