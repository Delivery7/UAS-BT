#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <thread>
#include <atomic>
#include <iostream>

class BTMission : public rclcpp::Node
{
public:
  BTMission() : Node("bt_mission_node")
  {
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&BTMission::odomCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&BTMission::tick, this));

    keyboard_thread_ = std::thread(&BTMission::keyboardLoop, this);

    RCLCPP_INFO(get_logger(), "BT INTERACTIVE MISSION STARTED");
  }

  ~BTMission()
  {
    running_ = false;
    if (keyboard_thread_.joinable())
      keyboard_thread_.join();
  }

private:
  /* =========================
     STATE
     ========================= */
  enum State
  {
    SEQ1_BATTERY,
    GO_MERAH,
    SEQ2_ORDER,
    GO_KUNING,
    SEQ3_BOX,
    GO_HIJAU,
    SEQ5_PASSWORD,
    DONE
  };

  State state_ = SEQ1_BATTERY;

  /* =========================
     ROS
     ========================= */
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  /* =========================
     ODOM
     ========================= */
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  bool odom_ready_{false};

  /* =========================
     WAYPOINT
     ========================= */
  struct Waypoint { double x; double y; };

  Waypoint merah_  {0.08, 9.54};
  Waypoint kuning_ {8.47, 9.19};
  Waypoint hijau_  {7.92, 0.14};

  const double GOAL_TOL = 0.15;

  /* =========================
     MOTION
     ========================= */
  const double SPEED_FORWARD = 0.25;
  const double SPEED_TURN    = 0.4;

  /* =========================
     KEYBOARD
     ========================= */
  std::atomic<char> last_key_{0};
  std::atomic<bool> running_{true};
  std::thread keyboard_thread_;

  void keyboardLoop()
  {
    char c;
    while (running_)
    {
      std::cin >> c;              // pakai ENTER
      last_key_ = toupper(c);
    }
  }

  /* =========================
     BASIC MOTION
     ========================= */
  void moveForward(geometry_msgs::msg::Twist &cmd)
  {
    cmd.linear.x = SPEED_FORWARD;
    cmd.angular.z = 0.0;
  }

  void turn(geometry_msgs::msg::Twist &cmd, double dir)
  {
    cmd.linear.x = 0.0;
    cmd.angular.z = dir * SPEED_TURN;
  }

  void stopRobot()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
  }

  /* =========================
     ODOM CALLBACK
     ========================= */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;

    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w
    );

    double roll, pitch;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_);

    odom_ready_ = true;
  }

  /* =========================
     GO TO GOAL
     ========================= */
  bool goToGoal(const Waypoint &g, geometry_msgs::msg::Twist &cmd)
  {
    double dx = g.x - x_;
    double dy = g.y - y_;
    double dist = std::hypot(dx, dy);

    double target_yaw = std::atan2(dy, dx);
    double yaw_err = target_yaw - yaw_;

    while (yaw_err > M_PI) yaw_err -= 2 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2 * M_PI;

    if (std::fabs(yaw_err) > 0.25)
    {
      turn(cmd, yaw_err > 0 ? 1.0 : -1.0);
      return false;
    }

    if (dist > GOAL_TOL)
    {
      moveForward(cmd);
      return false;
    }

    stopRobot();
    return true;
  }

  /* =========================
     MAIN LOOP
     ========================= */
  void tick()
  {
    if (!odom_ready_) return;

    geometry_msgs::msg::Twist cmd;

    switch (state_)
    {
      case SEQ1_BATTERY:
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "SEQ1: Tekan B (cukup) / C (charge)");
        if (last_key_ == 'B')
        {
          state_ = SEQ2_ORDER;
          last_key_ = 0;
        }
        else if (last_key_ == 'C')
        {
          state_ = GO_MERAH;
          last_key_ = 0;
        }
        break;

      case GO_MERAH:
        if (goToGoal(merah_, cmd))
        {
          RCLCPP_INFO(get_logger(), "Charging 3 detik...");
          std::this_thread::sleep_for(std::chrono::seconds(3));
          state_ = SEQ2_ORDER;
        }
        cmd_pub_->publish(cmd);
        break;

      case SEQ2_ORDER:
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "SEQ2: Tekan O Pesanan Diterima");
        if (last_key_ == 'O')
        {
          state_ = GO_KUNING;
          last_key_ = 0;
        }
        break;

      case GO_KUNING:
        if (goToGoal(kuning_, cmd))
        {
          RCLCPP_INFO(get_logger(), "Pesanan Siap untuk Diantar");
          state_ = SEQ3_BOX;
        }
        cmd_pub_->publish(cmd);
        break;

      case SEQ3_BOX:
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "SEQ3: Tekan F Mengantar Pesanan");
        if (last_key_ == 'F')
        {
          state_ = GO_HIJAU;
          last_key_ = 0;
        }
        break;

      case GO_HIJAU:
        if (goToGoal(hijau_, cmd))
        {
          state_ = SEQ5_PASSWORD;
        }
        cmd_pub_->publish(cmd);
        break;

      case SEQ5_PASSWORD:
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "SEQ5: Tekan P (benar) / W (salah)");
        if (last_key_ == 'P')
        {
          RCLCPP_INFO(get_logger(), "MISSION COMPLETE");
          state_ = DONE;
          last_key_ = 0;
        }
        else if (last_key_ == 'W')
        {
          RCLCPP_WARN(get_logger(), "Password salah");
          last_key_ = 0;
        }
        break;

      case DONE:
        stopRobot();
        break;
    }
  }
};

/* =========================
   MAIN
   ========================= */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BTMission>());
  rclcpp::shutdown();
  return 0;
}
