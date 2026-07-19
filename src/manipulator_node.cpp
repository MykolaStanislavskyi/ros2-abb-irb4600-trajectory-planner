#include <chrono>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int8.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "block1_stanislavskyi/manipulator.hpp"
#include "block1_stanislavskyi/srv/execute_machining.hpp"

using block1_stanislavskyi::Manipulator;
using block1_stanislavskyi::TrajectoryPoint;

enum ManipulatorState : int8_t
{
  IDLE = 0,
  PTP = 1,
  APPROACHING = 2,
  MACHINING = 3,
  RETRACTING = 4,
  DONE = 5,
  ERROR = 6
};

class ManipulatorNode : public rclcpp::Node
{
public:
  ManipulatorNode() : Node("manipulator_node")
  {
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    path_pub_ = create_publisher<visualization_msgs::msg::Marker>("tool_path", 10);
    state_pub_ = create_publisher<std_msgs::msg::Int8>("manipulator/state", 10);

    service_ = create_service<block1_stanislavskyi::srv::ExecuteMachining>(
      "execute_machining",
      std::bind(&ManipulatorNode::handle_execute, this, std::placeholders::_1, std::placeholders::_2));

    joint_names_ = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

    q_home_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    T1_ = {0.619370, 0.487012, 0.757542, -2.488436, 1.243505, -0.170737};
    T2_ = {0.077469, 0.385220, 0.985691, -3.066491, 1.307897, 0.010746};

    T3_ = {-0.198636, 0.380018, 0.964956, -0.215498, -1.113423, 0.059486};
    T4_ = {-0.393189, 0.416336, 0.901440, -0.430651, -1.102375, 0.117917};

    Tvia_ = {-0.653819, -0.267951, 1.287689, -0.835480, -0.922878, 0.421086};

    current_q_ = q_home_;

    const char * home_directory = std::getenv("HOME");

const std::string log_path =
  std::string(home_directory ? home_directory : "/tmp") +
  "/trajectory_log.csv";

log_file_.open(log_path);

if (log_file_.is_open()) {
  log_file_ << "motion,t,"
            << "j1,j2,j3,j4,j5,j6,"
            << "dj1,dj2,dj3,dj4,dj5,dj6,"
            << "ddj1,ddj2,ddj3,ddj4,ddj5,ddj6\n";

  RCLCPP_INFO(
    get_logger(),
    "Trajectory log: %s",
    log_path.c_str());
} else {
  RCLCPP_WARN(
    get_logger(),
    "Could not open trajectory log: %s",
    log_path.c_str());
}

    publish_joint_state(current_q_);
    publish_state(IDLE);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&ManipulatorNode::timer_callback, this));

    RCLCPP_INFO(get_logger(), "manipulator_node ready");
  }

  ~ManipulatorNode()
  {
    if (log_file_.is_open()) {
      log_file_.close();
    }
  }

private:
  void timer_callback()
  {
    publish_joint_state(current_q_);
  }

  void publish_state(int8_t state)
  {
    std_msgs::msg::Int8 msg;
    msg.data = state;
    state_pub_->publish(msg);
  }

  void publish_joint_state(const ikfast_abb::JointValues & q)
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = joint_names_;
    msg.position.assign(q.begin(), q.end());
    joint_pub_->publish(msg);
  }

  Eigen::Affine3d liftZ(const ikfast_abb::JointValues & q, double dz)
  {
    Eigen::Affine3d pose = ikfast_abb::computeFk(q);
    pose.translation().z() += dz;
    return pose;
  }

  void log_point(const std::string & motion, const TrajectoryPoint & p)
  {
    if (!log_file_.is_open()) {
      return;
    }

    log_file_ << motion << "," << p.t;

    for (double v : p.q) {
      log_file_ << "," << v;
    }

    for (double v : p.dq) {
      log_file_ << "," << v;
    }

    for (double v : p.ddq) {
      log_file_ << "," << v;
    }

    log_file_ << "\n";
  }

  void clear_path_markers()
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = now();
  marker.ns = "tool_path";
  marker.action = visualization_msgs::msg::Marker::DELETEALL;

  path_pub_->publish(marker);

  marker_id_counter_ = 0;
}

  void publish_path_point(const ikfast_abb::JointValues & q, float r, float g, float b, int id)
  {
    Eigen::Affine3d pose = ikfast_abb::computeFk(q);

    visualization_msgs::msg::Marker m;
    m.header.frame_id = "base_link";
    m.header.stamp = now();
    m.ns = "tool_path";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x = pose.translation().x();
    m.pose.position.y = pose.translation().y();
    m.pose.position.z = pose.translation().z();
    m.pose.orientation.w = 1.0;

    m.scale.x = 0.01;
    m.scale.y = 0.01;
    m.scale.z = 0.01;

    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0;

    path_pub_->publish(m);
  }

  void execute_trajectory(
    const std::vector<TrajectoryPoint> & traj,
    float r,
    float g,
    float b,
    int8_t state,
    const std::string & motion_name)
  {
    RCLCPP_INFO(
    get_logger(),
    "Starting trajectory '%s' with %zu points",
    motion_name.c_str(),
    traj.size());
    publish_state(state);

    int marker_id = marker_id_counter_;

    for (const auto & p : traj) {
      current_q_ = p.q;
      publish_joint_state(current_q_);
      publish_path_point(current_q_, r, g, b, marker_id++);
      log_point(motion_name, p);
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }

    marker_id_counter_ = marker_id;
    RCLCPP_INFO(
    get_logger(),
    "Finished trajectory '%s'",
    motion_name.c_str());
  }

  void handle_execute(
    const std::shared_ptr<block1_stanislavskyi::srv::ExecuteMachining::Request>,
    std::shared_ptr<block1_stanislavskyi::srv::ExecuteMachining::Response> response)
  {
    
    try {
      clear_path_markers();
      
      const double dt = 0.01;
      const double lift_height = 0.35;

      const auto above_T1 = liftZ(T1_, lift_height);
      const auto above_T2 = liftZ(T2_, lift_height);
      const auto above_T3 = liftZ(T3_, lift_height);
      const auto above_T4 = liftZ(T4_, lift_height);

      const auto ik_above_T1 = ikfast_abb::computeIK(above_T1);
      const auto ik_above_T3 = ikfast_abb::computeIK(above_T3);

      if (ik_above_T1.empty() || ik_above_T3.empty()) {
        throw std::runtime_error("No IK for approach points");
      }

      ikfast_abb::JointValues q_above_T1;
      ikfast_abb::JointValues q_above_T3;

      if (!manip_.chooseBestSolution(ik_above_T1, current_q_, q_above_T1)) {
        throw std::runtime_error("Could not choose IK for above T1");
      }

      if (!manip_.chooseBestSolution(ik_above_T3, current_q_, q_above_T3)) {
        throw std::runtime_error("Could not choose IK for above T3");
      }

      auto ptp_to_T1 = manip_.generatePTP(current_q_, q_above_T1, 3.0, dt);
      execute_trajectory(ptp_to_T1, 0.0f, 0.0f, 1.0f, PTP, "ptp_to_T1");

      auto approach1 = manip_.generateLIN(current_q_, ikfast_abb::computeFk(T1_), 2.0, dt);
      execute_trajectory(approach1, 1.0f, 0.0f, 0.0f, APPROACHING, "approach1");

      auto machining1 = manip_.generateLIN(current_q_, ikfast_abb::computeFk(T2_), 3.0, dt);
      execute_trajectory(machining1, 0.0f, 1.0f, 0.0f, MACHINING, "machining1");

      auto retract1 = manip_.generateLIN(current_q_, above_T2, 2.0, dt);
      execute_trajectory(retract1, 1.0f, 0.0f, 0.0f, RETRACTING, "retract1");

      auto via_motion = manip_.generatePTPVia(current_q_, Tvia_, q_above_T3, 2.0, 2.0, dt);
      execute_trajectory(via_motion, 0.0f, 0.0f, 1.0f, PTP, "via_motion");

      auto approach2 = manip_.generateLIN(current_q_, ikfast_abb::computeFk(T3_), 2.0, dt);
      execute_trajectory(approach2, 1.0f, 0.0f, 0.0f, APPROACHING, "approach2");

      auto machining2 = manip_.generateLIN(current_q_, ikfast_abb::computeFk(T4_), 3.0, dt);
      execute_trajectory(machining2, 0.0f, 1.0f, 0.0f, MACHINING, "machining2");

      auto retract2 = manip_.generateLIN(current_q_, above_T4, 2.0, dt);
      execute_trajectory(retract2, 1.0f, 0.0f, 0.0f, RETRACTING, "retract2");

      auto home = manip_.generatePTP(current_q_, q_home_, 3.0, dt);
      execute_trajectory(home, 0.0f, 0.0f, 1.0f, PTP, "home");

      if (log_file_.is_open()) {
        log_file_.flush();
      }
      
      publish_state(DONE);

      response->success = true;
      response->message = "Machining sequence executed";
    } catch (const std::exception & e) {
      publish_state(ERROR);
      response->success = false;
      response->message = e.what();
      RCLCPP_ERROR(get_logger(), "Execution failed: %s", e.what());
    }
  }

  Manipulator manip_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_; //joint_states
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_pub_; //tool_path
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr state_pub_; //manipulator/state

  rclcpp::Service<block1_stanislavskyi::srv::ExecuteMachining>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::string> joint_names_;

  ikfast_abb::JointValues q_home_;
  ikfast_abb::JointValues current_q_;

  ikfast_abb::JointValues T1_;
  ikfast_abb::JointValues T2_;
  ikfast_abb::JointValues T3_;
  ikfast_abb::JointValues T4_;
  ikfast_abb::JointValues Tvia_;

  std::ofstream log_file_;

  int marker_id_counter_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManipulatorNode>());
  rclcpp::shutdown();
  return 0;
}