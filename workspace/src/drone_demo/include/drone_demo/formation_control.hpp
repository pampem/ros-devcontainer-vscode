#pragma once
#ifndef FORMATION_CONTROL_HPP
#define FORMATION_CONTROL_HPP

#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <vector>

/**
 * @brief 単一ドローンを制御するクラス
 */
class MyDrone
{
public:
    /**
     * @brief コンストラクタ
     * @param drone_ns mavros ノードの名前空間 (例: "/drone1/")
     * @param nh        ROS NodeHandle
     */
    MyDrone(const std::string &drone_ns, ros::NodeHandle &nh);

    /**
     * @brief 目標位置+ヨー角を指定して /mavros/setpoint_raw/local に投げる
     * @param x   ターゲット X[m]
     * @param y   ターゲット Y[m]
     * @param z   ターゲット Z[m]
     * @param yaw ターゲットヨー[rad]
     */
    void setTargetPosition(double x, double y, double z, double yaw);

    /**
     * @brief 位置や速度、加速度などをすべて無視した Setpoint を送る(ホバリング継続など)
     */
    void stopPubPosVelAcc();

private:
    ros::Publisher setpoint_pub_;
    mavros_msgs::PositionTarget position_target_;
};


/**
 * @brief 3機のドローンを円形フォーメーションで飛行させるコントローラー
 */
class FormationController
{
public:
    FormationController(ros::NodeHandle &nh);
    ~FormationController() = default;

private:
    // ドローン3機
    MyDrone drone1_;
    MyDrone drone2_;
    MyDrone drone3_;

    geometry_msgs::Point drone1_home;

    geometry_msgs::Point drone2_home;

    geometry_msgs::Point drone3_home;

    // Joy の購読
    ros::Subscriber joy_sub_;

    // 定期的にフォーメーション更新するTimer
    ros::Timer formation_timer_;

    // Joyの状態を保持
    std::vector<float> axes_;
    std::vector<int>   buttons_;

    bool button1_pressed_; // ボタン1（例: buttons[0]）
    bool button2_pressed_; // ボタン2（例: buttons[1]）

    // 円形ウェイポイント配列
    std::vector<geometry_msgs::Point> circle_waypoints_;

    // ベースとなるウェイポイントインデックス
    // (これにドローンごとのオフセットを加算して使う)
    int base_waypoint_index_;

    // ヨー角と高度
    double target_z_;
    double target_yaw_;

    /**
     * @brief 円形ウェイポイント生成
     * @param radius     半径
     * @param num_points 分割数
     */
    void initCircleWaypoints(double radius, int num_points);

    /**
     * @brief ジョイメッセージのコールバック
     * @details ボタンの押下状態を更新し、高度やヨー角を更新する
     */
    void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);

    /**
     * @brief タイマーコールバック(周期的に呼ばれる)
     * @details ボタンが押されていればウェイポイントを進めてフォーメーションを更新
     */
    void timerCallback(const ros::TimerEvent &);

    /**
     * @brief 現在の base_waypoint_index_ & オフセットで3機を円周上に配置
     */
    void applyFormation();
};

#endif // FORMATION_CONTROL_HPP
