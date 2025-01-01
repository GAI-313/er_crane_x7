#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "er_crane_x7_srvs/srv/set_ee_pose.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "angles/angles.h"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class RobotControlServer : public rclcpp::Node
{
public:
  RobotControlServer(const rclcpp::NodeOptions & options)
  : Node("robot_control_server", options)
  {
    // サービスの作成
    ee_pose_service_ = this->create_service<er_crane_x7_srvs::srv::SetEEPose>(
      "set_ee_pose", std::bind(&RobotControlServer::set_ee_pose_callback, this, std::placeholders::_1, std::placeholders::_2));
  }

  void set_move_group_interface(std::shared_ptr<MoveGroupInterface> arm)
  {
    move_group_arm_ = arm;
  }

private:
  void set_ee_pose_callback(
    const std::shared_ptr<er_crane_x7_srvs::srv::SetEEPose::Request> request,
    std::shared_ptr<er_crane_x7_srvs::srv::SetEEPose::Response> response)
  {
    geometry_msgs::msg::Pose target_pose;
    tf2::Quaternion q;
    q.setRPY(angles::from_degrees(request->roll), angles::from_degrees(request->pitch), angles::from_degrees(request->yaw));

    target_pose.position.x = request->x;
    target_pose.position.y = request->y;
    target_pose.position.z = request->z;
    target_pose.orientation = tf2::toMsg(q);

    move_group_arm_->setPoseTarget(target_pose);
    bool success = (move_group_arm_->move() == moveit::core::MoveItErrorCode::SUCCESS);
    
    response->success = success;
  }

  std::shared_ptr<MoveGroupInterface> move_group_arm_;
  rclcpp::Service<er_crane_x7_srvs::srv::SetEEPose>::SharedPtr ee_pose_service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  // ノードオプションを設定
  auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
  auto node = std::make_shared<RobotControlServer>(options);

  // MoveGroupInterface の初期化
  auto move_group_arm = std::make_shared<MoveGroupInterface>(node, "arm");
  node->set_move_group_interface(move_group_arm);

  // メインスレッドを適切にブロックするための無限ループ
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
