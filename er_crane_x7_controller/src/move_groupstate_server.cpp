#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "er_crane_x7_srvs/srv/set_goalstate.hpp"
#include <memory>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using SetGoalstate = er_crane_x7_srvs::srv::SetGoalstate;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pose_groupstate");

class MoveGroupStateServer : public rclcpp::Node
{
public:
  MoveGroupStateServer(const rclcpp::NodeOptions & options)
  : Node("move_groupstate_server", options)
  {
    // サービスの作成
    service_ = this->create_service<SetGoalstate>(
      "/move_groupstate",
      std::bind(&MoveGroupStateServer::handle_set_goalstate, this, std::placeholders::_1, std::placeholders::_2)
    );
  }

  void set_move_group_interface(std::shared_ptr<MoveGroupInterface> move_group)
  {
    move_group_arm = move_group;
  }

private:
  void handle_set_goalstate(
    const std::shared_ptr<SetGoalstate::Request> request,
    std::shared_ptr<SetGoalstate::Response> response)
  {
    RCLCPP_INFO(LOGGER, "Received request to move to '%s'", request->goal_state.c_str());

    move_group_arm->setMaxVelocityScalingFactor(request->max_vel);  // Set 0.0 ~ 1.0
    move_group_arm->setMaxAccelerationScalingFactor(request->max_vel);  // Set 0.0 ~ 1.0

    // Named target が存在するかをチェック
    if (!move_group_arm->setNamedTarget(request->goal_state)) {
      RCLCPP_ERROR(LOGGER, "The requested named target '%s' does not exist", request->goal_state.c_str());
      response->result = false;
      return;
    }

    bool success = (move_group_arm->move() == moveit::core::MoveItErrorCode::SUCCESS);

    response->result = success;
    if (success) {
      RCLCPP_INFO(LOGGER, "Successfully moved to '%s'", request->goal_state.c_str());
    } else {
      RCLCPP_ERROR(LOGGER, "Failed to move to '%s'", request->goal_state.c_str());
    }
  }

  rclcpp::Service<SetGoalstate>::SharedPtr service_;
  std::shared_ptr<MoveGroupInterface> move_group_arm;  // shared_ptrとしてメンバー変数に保存
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  // ノードオプションを設定
  auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
  auto node = std::make_shared<MoveGroupStateServer>(options);

  // MoveGroupInterface の初期化
  auto move_group_arm = std::make_shared<MoveGroupInterface>(node, "arm");
  node->set_move_group_interface(move_group_arm);

  // メインスレッドを適切にブロックするための無限ループ
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
