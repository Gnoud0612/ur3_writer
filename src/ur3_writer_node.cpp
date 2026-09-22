#include <memory>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "ur3_writer_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("ur3_writer_node");

  // Publisher để xuất nét vẽ lên RViz
  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("written_letter_marker", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  move_group_interface.setPlanningTime(10.0);
  // TĂNG TỐC ĐỘ DI CHUYỂN (35% tốc độ tối đa của UR3)
  move_group_interface.setMaxVelocityScalingFactor(0.35);
  move_group_interface.setMaxAccelerationScalingFactor(0.35);

  RCLCPP_INFO(logger, ">>> Bat dau dieu khien robot UR3...");

  geometry_msgs::msg::Quaternion down_orientation;
  down_orientation.x = 1.0;
  down_orientation.y = 0.0;
  down_orientation.z = 0.0;
  down_orientation.w = 0.0;

  // Bộ thông số chữ D đẹp ngang tầm nhìn
  double center_x = 0.25;
  double start_y  = -0.06;
  double top_z    = 0.35;
  double bot_z    = 0.21;
  double center_z = 0.28;
  double radius   = 0.07;

  // 1. Di chuyển về điểm bắt đầu
  geometry_msgs::msg::Pose start_pose;
  start_pose.orientation = down_orientation;
  start_pose.position.x = center_x;
  start_pose.position.y = start_y;
  start_pose.position.z = top_z;

  move_group_interface.setPoseTarget(start_pose);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  if (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    move_group_interface.execute(my_plan);
  } else {
    RCLCPP_ERROR(logger, "Khong the di chuyen ve vi tri xuat phat!");
    rclcpp::shutdown();
    return 1;
  }

  // 2. Tạo tập Waypoints cho chữ D
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose pose = start_pose;

  // --- NÉT 1: Dọc thẳng ---
  pose.position.z = bot_z;
  waypoints.push_back(pose);

  // --- NÂNG ĐẦU CÔNG TÁC (Chuyển nét) ---
  pose.position.x = center_x - 0.03;
  waypoints.push_back(pose);

  pose.position.z = top_z;
  waypoints.push_back(pose);

  pose.position.x = center_x;
  waypoints.push_back(pose);

  // --- NÉT 2: Cong bán nguyệt ---
  int num_arc_points = 20;
  for (int i = 0; i <= num_arc_points; ++i) {
    double theta = (M_PI / 2.0) - (double)i * (M_PI / (double)num_arc_points);
    pose.position.x = center_x;
    pose.position.y = start_y + radius * std::cos(theta);
    pose.position.z = center_z + radius * std::sin(theta);
    waypoints.push_back(pose);
  }

  // Cấu hình Marker
  visualization_msgs::msg::Marker line_strip;
  line_strip.header.frame_id = "world";
  line_strip.header.stamp = node->now();
  line_strip.ns = "letter_d";
  line_strip.action = visualization_msgs::msg::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 0;
  line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_strip.scale.x = 0.01; // Độ dày nét 10mm
  line_strip.color.r = 1.0;  // Màu đỏ
  line_strip.color.g = 0.0;
  line_strip.color.b = 0.0;
  line_strip.color.a = 1.0;

  geometry_msgs::msg::Point p;
  p.x = start_pose.position.x; p.y = start_pose.position.y; p.z = start_pose.position.z;
  line_strip.points.push_back(p);
  for (const auto & wp : waypoints) {
    p.x = wp.position.x;
    p.y = wp.position.y;
    p.z = wp.position.z;
    line_strip.points.push_back(p);
  }

  // 3. Thực thi vẽ chữ D
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group_interface.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);

  if (fraction > 0.8) {
    RCLCPP_INFO(logger, ">>> Dang thuc thi ve chu D...");
    
    for (int i = 0; i < 5; ++i) {
      marker_pub->publish(line_strip);
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }

    move_group_interface.execute(trajectory);
    RCLCPP_INFO(logger, ">>> Da ve xong chu D!");

    // 4. THU TAY ROBOT SANG BÊN HÔNG (Dùng Joint Targets để posture đẹp và tự nhiên)
    // Xoay khớp đế (Base) -70 độ (-1.2 rad) để quay arm sang bên cạnh, nhấc cao elbow lên
    std::vector<double> retract_joints = {-1.2, -1.2, 1.2, -1.57, -1.57, 0.0};
    move_group_interface.setJointValueTarget(retract_joints);

    if (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger, ">>> Dang thu tay robot sang ben hong...");
      move_group_interface.execute(my_plan);
      RCLCPP_INFO(logger, ">>> Hoan thanh! Chu D da lo ro hoan toan.");
    }

    // Giữ Marker cố định trên RViz
    while (rclcpp::ok()) {
      marker_pub->publish(line_strip);
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
}
  rclcpp::shutdown();
  return 0;
}
