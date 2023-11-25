#ifndef ROBOTARM_H
#define ROBOTARM_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

class RobotArm : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;


    explicit RobotArm(std::string name);

private:
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

    // 声明话题发布者
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void execute_move(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle);

    // 定时发布关节状态
    void publish_joint_states();
    // 假装真正的关节状态
    std::vector<double> mJointStates;
};

#endif // MYACTIONROBOT_H

