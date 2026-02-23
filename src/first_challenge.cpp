#include "first_challenge/first_challenge.hpp"
#include <cmath>

FirstChallenge::FirstChallenge() : Node("FC_node")
{
  // --- 修正ポイント：速度を 0.1 から 0.25〜0.3 程度に引き上げ ---
  goal_dist_ = 1.0; 
  velocity_  = 0.25; // 0.1から0.25へ増量（必要に応じて0.4程度まで上げてもOK）

  // Subscriber
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom",
    10,
    std::bind(&FirstChallenge::odometry_callback, this, std::placeholders::_1)
  );

  // Publisher
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  RCLCPP_INFO(this->get_logger(), "FirstChallenge Started with Velocity: %f", velocity_);
}

void FirstChallenge::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_ = *msg; 
  // Odomが届くたびに制御計算を実行
  set_cmd_vel();
}

bool FirstChallenge::can_move()
{
  return odom_.has_value();
} 

bool FirstChallenge::is_goal()
{
  return calc_distance() >= goal_dist_;
}

double FirstChallenge::calc_distance()
{
  static double start_x = 0.0;
  static double start_y = 0.0;
  static bool initialized = false;

  if (!odom_.has_value()) return 0.0;

  double x = odom_->pose.pose.position.x;
  double y = odom_->pose.pose.position.y;

  if (!initialized) {
    start_x = x;
    start_y = y;
    initialized = true;
    RCLCPP_INFO(this->get_logger(), "Starting position: x=%f, y=%f", x, y);
  }

  double dx = x - start_x;
  double dy = y - start_y;

  return std::sqrt(dx * dx + dy * dy);
}

void FirstChallenge::run(float velocity, float omega)
{
  cmd_vel_.linear.x = velocity;
  cmd_vel_.angular.z = omega;
  cmd_vel_pub_->publish(cmd_vel_);
}

void FirstChallenge::set_cmd_vel()
{
  if (!can_move()) return;

  if (is_goal()) {
    run(0.0, 0.0);
    RCLCPP_INFO_ONCE(this->get_logger(), "Goal Reached!");
  } else {
    // 現在の進捗を確認するためのログ（デバッグ用）
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                         "Moving... Distance: %f / %f", calc_distance(), goal_dist_);
    
    run(velocity_, 0.0);
  }
}