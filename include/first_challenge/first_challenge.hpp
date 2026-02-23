#ifndef FIRST_CHALLENGE_HPP
#define FIRST_CHALLENGE_HPP

#include <rclcpp/rclcpp.hpp>
#include <functional>  // bind & placeholders用
#include <memory>      // SharedPtr用
#include <optional>    // has_value()用

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

class FirstChallenge : public rclcpp::Node
{
    public:
        FirstChallenge();
        // void process();

        // コールバック関数
        void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        // 関数
        bool can_move();                        // センサ情報（今回はodom）を取得できているかの確認用
        bool is_goal();                         // 終了判定
        double calc_distance();                 // 進んだ距離を計算
        void run(float velocity, float omega);  // roombaの制御入力を決定
        void set_cmd_vel();                     // 並進速度と旋回速度を計算

        // 変数
        int hz_ = 10;             // 制御周期
        double goal_dist_ = 0.0;  // ゴールまでの距離
        double velocity_ = 0.0;   // 直進時の並進速度
        std::optional<nav_msgs::msg::Odometry> odom_;  // optional型で定義することによりodomをsubできたかの判定も同時に行う
        geometry_msgs::msg::Twist cmd_vel_;

        // Pub & Sub
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;                  // odom
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;  // 制御入力
};


#endif  // FIRST_CHALLENGE_HPP
