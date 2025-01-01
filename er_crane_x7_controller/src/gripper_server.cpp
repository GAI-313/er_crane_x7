#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "er_crane_x7_srvs/srv/set_gripper.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "angles/angles.h"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class GripperControlServer : public rclcpp::Node
{
public:
  GripperControlServer(const rclcpp::NodeOptions & options)
  : Node("gripper_control_server", options)
  {
    // サービスの作成
    gripper_service_ = this->create_service<er_crane_x7_srvs::srv::SetGripper>(
      "set_gripper", std::bind(&GripperControlServer::set_gripper_callback, this, std::placeholders::_1, std::placeholders::_2));
  }

  void set_move_group_interface(std::shared_ptr<MoveGroupInterface> gripper)
  {
    move_group_gripper_ = gripper;
  }

private:
  void set_gripper_callback(
    const std::shared_ptr<er_crane_x7_srvs::srv::SetGripper::Request> request,
    std::shared_ptr<er_crane_x7_srvs::srv::SetGripper::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received a request to set gripper to angle: %f", request->angle);
    
    auto gripper_joint_values = move_group_gripper_->getCurrentJointValues();
    if (gripper_joint_values.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get current joint values. Check if joint_states are being published.");
      response->success = false;
      return;
    }

    gripper_joint_values[0] = angles::from_degrees(request->angle);
    move_group_gripper_->setJointValueTarget(gripper_joint_values);
    bool success = (move_group_gripper_->move() == moveit::core::MoveItErrorCode::SUCCESS);

    response->success = success;
    if (success) {
      RCLCPP_INFO(this->get_logger(), "Successfully set gripper to angle: %f", request->angle);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to set gripper to angle: %f", request->angle);
    }
  }

  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
  rclcpp::Service<er_crane_x7_srvs::srv::SetGripper>::SharedPtr gripper_service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  // ノードオプションを設定
  auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
  auto node = std::make_shared<GripperControlServer>(options);

  // MoveGroupInterface の初期化
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", options);
  auto move_group_gripper = std::make_shared<MoveGroupInterface>(move_group_gripper_node, "gripper");

  move_group_gripper->setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_gripper->setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0

  node->set_move_group_interface(move_group_gripper);

  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_gripper_node);
  std::thread([&executor]() {executor.spin();}).detach();

  // メインスレッドを適切にブロックするための無限ループ
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
