#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <functional>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "interactive_markers/interactive_marker_server.hpp"

#include "abb_irb4600_ikfast/abb_irb4600_ikfast.h"

class PoseTeacher : public rclcpp::Node
{
public:
  PoseTeacher()
  : Node("pose_teacher"),
    server_(std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "pose_teacher_marker", this))
  {
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    joint_names_ = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    current_joints_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Стартова поза маркера з FK від нульової конфігурації.
    // Так marker одразу з’явиться біля TCP.
    const Eigen::Affine3d fk_pose = ikfast_abb::computeFk(current_joints_);

    make_marker(fk_pose);
    publish_joint_state(current_joints_);

    // Періодично перепубліковуємо JointState, щоб robot_state_publisher стабільно оновлювався
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PoseTeacher::republish_current_state, this));

    RCLCPP_INFO(this->get_logger(), "pose_teacher started");
    RCLCPP_INFO(this->get_logger(), "Move the 6-DOF marker in RViz. On mouse release, joint values are printed.");
  }

private:
  void republish_current_state()
  {
    publish_joint_state(current_joints_);
  }

  void make_marker(const Eigen::Affine3d & pose_eigen)
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    int_marker.header.stamp = this->now();
    int_marker.name = "tcp_marker";
    int_marker.description = "TCP Teacher";
    int_marker.scale = 0.25;

    const Eigen::Quaterniond q(pose_eigen.rotation());

    int_marker.pose.position.x = pose_eigen.translation().x();
    int_marker.pose.position.y = pose_eigen.translation().y();
    int_marker.pose.position.z = pose_eigen.translation().z();

    int_marker.pose.orientation.x = q.x();
    int_marker.pose.orientation.y = q.y();
    int_marker.pose.orientation.z = q.z();
    int_marker.pose.orientation.w = q.w();

    // Маленький sphere marker, щоб було видно центр TCP
    visualization_msgs::msg::Marker sphere;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.scale.x = 0.04;
    sphere.scale.y = 0.04;
    sphere.scale.z = 0.04;
    sphere.color.r = 0.1f;
    sphere.color.g = 0.8f;
    sphere.color.b = 0.1f;
    sphere.color.a = 1.0f;

    visualization_msgs::msg::InteractiveMarkerControl visual_control;
    visual_control.always_visible = true;
    visual_control.markers.push_back(sphere);
    int_marker.controls.push_back(visual_control);

    add_6dof_controls(int_marker);

    server_->insert(
      int_marker,
      std::bind(&PoseTeacher::process_feedback, this, std::placeholders::_1));
    server_->applyChanges();
  }

  void add_6dof_controls(visualization_msgs::msg::InteractiveMarker & int_marker)
  {
    using visualization_msgs::msg::InteractiveMarkerControl;

    InteractiveMarkerControl control;

    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  Eigen::Affine3d pose_msg_to_eigen(const geometry_msgs::msg::Pose & pose_msg) const
  {
    Eigen::Translation3d t(
      pose_msg.position.x,
      pose_msg.position.y,
      pose_msg.position.z);

    Eigen::Quaterniond q(
      pose_msg.orientation.w,
      pose_msg.orientation.x,
      pose_msg.orientation.y,
      pose_msg.orientation.z);

    Eigen::Affine3d pose = t * q.normalized();
    return pose;
  }

  double squared_distance(
    const ikfast_abb::JointValues & a,
    const ikfast_abb::JointValues & b) const
  {
    double sum = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
      const double d = a[i] - b[i];
      sum += d * d;
    }
    return sum;
  }

  bool choose_best_solution(
    const ikfast_abb::Solutions & solutions,
    const ikfast_abb::JointValues & q_prev,
    ikfast_abb::JointValues & q_best) const
  {
    if (solutions.empty()) {
      return false;
    }

    double best_dist = std::numeric_limits<double>::max();
    for (const auto & sol : solutions) {
      const double dist = squared_distance(sol, q_prev);
      if (dist < best_dist) {
        best_dist = dist;
        q_best = sol;
      }
    }
    return true;
  }

  void publish_joint_state(const ikfast_abb::JointValues & joints)
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = joint_names_;
    msg.position.assign(joints.begin(), joints.end());
    joint_pub_->publish(msg);
  }

  void print_joint_values(const ikfast_abb::JointValues & joints, const std::string & label)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "%s: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
      label.c_str(),
      joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
  }

  void process_feedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
  {
    using visualization_msgs::msg::InteractiveMarkerFeedback;

    if (feedback->event_type != InteractiveMarkerFeedback::POSE_UPDATE &&
        feedback->event_type != InteractiveMarkerFeedback::MOUSE_UP)
    {
      return;
    }

    const Eigen::Affine3d target_pose = pose_msg_to_eigen(feedback->pose);
    const ikfast_abb::Solutions solutions = ikfast_abb::computeIK(target_pose);

    if (solutions.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "No IK solutions found for current marker pose");
      return;
    }

    ikfast_abb::JointValues best_solution;
    if (!choose_best_solution(solutions, current_joints_, best_solution)) {
      RCLCPP_WARN(this->get_logger(), "Failed to choose IK solution");
      return;
    }

    current_joints_ = best_solution;
    publish_joint_state(current_joints_);

    if (feedback->event_type == InteractiveMarkerFeedback::MOUSE_UP) {
      print_joint_values(current_joints_, "Saved point");
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  std::vector<std::string> joint_names_;
  ikfast_abb::JointValues current_joints_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseTeacher>());
  rclcpp::shutdown();
  return 0;
}