//////////////////////////////////////////////////////////
// Node: pid_control
// Created by Nelson Durrant, Sep 2024
//
// PUBLISHERS:
// - drive_command (agrobot_interfaces/msg/DriveCommand)
// SUBSCRIBERS:
// - init (std_msgs/msg/Empty)
// - desired_distance (agrobot_interfaces/msg/DesiredDistance)
// - range_data (agrobot_interfaces/msg/RangeData)
//////////////////////////////////////////////////////////

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "agrobot_cpp/pid.h"
#include "agrobot_interfaces/msg/desired_distance.hpp"
#include "agrobot_interfaces/msg/range_data.hpp"
#include "agrobot_interfaces/msg/drive_command.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

rmw_qos_profile_t sensor_qos_profile = rmw_qos_profile_sensor_data;
auto sensor_qos = rclcpp::QoS(
    rclcpp::QoSInitialization(sensor_qos_profile.history, sensor_qos_profile.depth),
    sensor_qos_profile);

rmw_qos_profile_t default_qos_profile = rmw_qos_profile_default;
auto default_qos = rclcpp::QoS(
    rclcpp::QoSInitialization(default_qos_profile.history, default_qos_profile.depth),
    default_qos_profile);

class PIDControl : public rclcpp::Node {
public:
  PIDControl() : Node("pid_control") {

    // declare ros params
    this->declare_parameter("base_speed", 0);
    this->declare_parameter("timer_period", 100);
    this->declare_parameter("kp", 0.0);
    this->declare_parameter("ki", 0.0);
    this->declare_parameter("kd", 0.0);
    this->declare_parameter("min_output", 0);
    this->declare_parameter("max_output", 0);
    this->declare_parameter("bias", 0);

    // calibrate PID controller
    myPID.calibrate(this->get_parameter("kp").as_double(),
                    this->get_parameter("ki").as_double(),
                    this->get_parameter("kd").as_double(),
                    this->get_parameter("min_output").as_int(),
                    this->get_parameter("max_output").as_int(),
                    this->get_parameter("timer_period").as_int(),
                    this->get_parameter("bias").as_int());

    // declare ros publishers
    drive_cmd_publisher_ =
        this->create_publisher<agrobot_interfaces::msg::DriveCommand>(
            "drive_command", default_qos);

    // declare ros subscribers
    init_subscription_ = 
        this->create_subscription<std_msgs::msg::Empty>(
            "init", default_qos,
            std::bind(&PIDControl::init_callback, this, _1));

    desired_dist_subscription_ =
        this->create_subscription<agrobot_interfaces::msg::DesiredDistance>(
            "desired_distance", default_qos,
            std::bind(&PIDControl::desired_dist_callback, this, _1));

    range_data_subscription_ =
        this->create_subscription<agrobot_interfaces::msg::RangeData>(
            "range_data", sensor_qos,
            std::bind(&PIDControl::range_data_callback, this, _1));

    // declare ros timers
    pid_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(
            this->get_parameter("timer_period").as_int()),
        std::bind(&PIDControl::timer_callback, this));
  }

private:
  void init_callback(const std_msgs::msg::Empty::SharedPtr msg) {
    (void)msg; // suppress unused parameter warning
    this->init_flag = true;
  }

  void desired_dist_callback(
      const agrobot_interfaces::msg::DesiredDistance::SharedPtr msg) {
    this->last_msg_time = this->now().seconds();
    this->desired_dist = msg->desired_distance;
  }

  void range_data_callback(
      const agrobot_interfaces::msg::RangeData::SharedPtr msg) {
    this->right_range = msg->right_us;
  }

  void timer_callback() {
    
    if (!(this->init_flag)) {
      return;
    }

    if (this->now().seconds() - last_msg_time > 2) {
      return;
    }

    //////////////////////////////////////////////////////////
    // LOW-LEVEL CONTROLLER CODE STARTS HERE
    //////////////////////////////////////////////////////////

    auto drive_cmd_msg = agrobot_interfaces::msg::DriveCommand();

    int speed_offset = myPID.compute(this->desired_dist, right_range);

    drive_cmd_msg.fl_motor = this->get_parameter("base_speed").as_int() + speed_offset;
    drive_cmd_msg.fr_motor = this->get_parameter("base_speed").as_int() - speed_offset;
    drive_cmd_msg.bl_motor = this->get_parameter("base_speed").as_int() + speed_offset;
    drive_cmd_msg.br_motor = this->get_parameter("base_speed").as_int() - speed_offset;

    drive_cmd_publisher_->publish(drive_cmd_msg);

    RCLCPP_INFO(this->get_logger(),
                "Drive Command: FL: %f, FR: %f, BL: %f, BR: %f",
                drive_cmd_msg.fl_motor, drive_cmd_msg.fr_motor,
                drive_cmd_msg.bl_motor, drive_cmd_msg.br_motor);

    //////////////////////////////////////////////////////////
    // LOW-LEVEL CONTROLLER CODE ENDS HERE
    //////////////////////////////////////////////////////////
  }

  // micro-ROS objects
  rclcpp::TimerBase::SharedPtr pid_timer_;
  rclcpp::Publisher<agrobot_interfaces::msg::DriveCommand>::SharedPtr
      drive_cmd_publisher_;
  rclcpp::Subscription<agrobot_interfaces::msg::DesiredDistance>::SharedPtr
      desired_dist_subscription_;
  rclcpp::Subscription<agrobot_interfaces::msg::RangeData>::SharedPtr
      range_data_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr init_subscription_;

  // flags
  bool init_flag = false;
  int last_msg_time = 0;

  // class desired value variables
  float desired_dist = 0.0;

  // control objects
  PID myPID;

  // class sensor variables
  float right_range = 0.0;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDControl>());
  rclcpp::shutdown();
  return 0;
}
