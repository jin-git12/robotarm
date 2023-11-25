#include "RobotArm.h"



namespace moveit_robotarm
{
  RobotArm::RobotArm(std::string name) : Node(name)
  {
      //    control_msgs::FollowJointTrajectoryAction

      using namespace std::placeholders;  // NOLINT

      this->action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
                  this, "/my_group_controller/follow_joint_trajectory",
                  std::bind(&RobotArm::handle_goal, this, _1, _2),
                  std::bind(&RobotArm::handle_cancel, this, _1),
                  std::bind(&RobotArm::handle_accepted, this, _1));

      std::cout  << "the action server:" << this->action_server_.get();

      rclcpp::QoS qos(10);
  //    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  //    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
      joint_states_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);

      mJointStates = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
      std::thread{std::bind(&RobotArm::publish_joint_states, this)}
      .detach();
  }

  rclcpp_action::GoalResponse RobotArm::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
      std::cout << "---handle goal:" << goal->trajectory.joint_names.size();
      std::cout << goal->trajectory.header.frame_id.c_str() << goal->trajectory.header.stamp.sec << goal->trajectory.header.stamp.nanosec;

      int pointSize = goal->trajectory.points.size();
      if(pointSize > 0)
      {
          for(int i = 0; i < pointSize; i++)
          {
              auto point = goal->trajectory.points.at(i);
              std::cout << point.positions
                      << "[sec:" << point.time_from_start.sec << ", nanosec:" << point.time_from_start.nanosec << "]";
          }
      }

    (void)uuid;

      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse RobotArm::handle_cancel(const std::shared_ptr<GoalHandleFJT> goal_handle)
  {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

      return rclcpp_action::CancelResponse::ACCEPT;
  }

  void RobotArm::execute_move(const std::shared_ptr<GoalHandleFJT> goal_handle)
  {
      const auto goal = goal_handle->get_goal();

      RCLCPP_INFO(this->get_logger(), "开始执行移动 %f 。。。", goal->trajectory.points.size());

      auto result = std::make_shared<FollowJointTrajectory::Result>();

      auto trjPointsSize = goal->trajectory.points.size();
      int trjIdx = 0;

  //    rclcpp::Rate rate = rclcpp::Rate(1); // 定时进行操作
      while (rclcpp::ok() && trjIdx < trjPointsSize) {
          auto point =  goal->trajectory.points.at(trjIdx);//第n个轨迹

          // 按照轨迹指定的时间，进行等待
          if(trjIdx > 0)
          {
              auto last_time = goal->trajectory.points.at(trjIdx - 1).time_from_start;
              auto current_time = goal->trajectory.points.at(trjIdx).time_from_start;

              rclcpp::Time time1(last_time.sec, last_time.nanosec);
              rclcpp::Time time2(current_time.sec, current_time.nanosec);
              auto time_to_sleep = time2 - time1;

              int64_t duration = ((int64_t)time_to_sleep.seconds()) * 1e9 + time_to_sleep.nanoseconds();
              std::chrono::nanoseconds ns(duration);

              std::cout  << "sleep for:" << ns.count();

              rclcpp::sleep_for(ns);
          }

          auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
          feedback->header.frame_id = goal->trajectory.header.frame_id;
          feedback->header.stamp = goal->trajectory.header.stamp;
          feedback->joint_names = goal->trajectory.joint_names;
  //        feedback->actual = goal->trajectory.joint_names;
  //        feedback->desired = ;
  //        feedback->error = "";

          for(int i = 0; i < point.positions.size(); i++)
          {
              mJointStates[i] = point.positions.at(i);
          }

          goal_handle->publish_feedback(feedback);
          /*检测任务是否被取消*/
          if (goal_handle->is_canceling()) {
              result->error_code = -1;
              result->error_string = "has cancel";
              goal_handle->canceled(result);
              RCLCPP_INFO(this->get_logger(), "Goal Canceled");
              return;
          }
          RCLCPP_INFO(this->get_logger(), "Publish Feedback"); /*Publish feedback*/

          trjIdx++;

          //        rate.sleep();
      }

  //    result->pose = robot.get_current_pose();
      result->error_code = 0;
      result->error_string = "";

      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
  }

  void RobotArm::handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle) {
      using std::placeholders::_1;

      std::thread{std::bind(&RobotArm::execute_move, this, _1), goal_handle}
      .detach();
  }

  void RobotArm::publish_joint_states()
  {
      rclcpp::Rate rate = rclcpp::Rate(15); // 定时进行操作
      while (rclcpp::ok()) {
          sensor_msgs::msg::JointState jointStates;
          jointStates.header.frame_id = "";
  // 这样子不对
  //        jointStates.header.stamp.sec = this->now().seconds();
  //        jointStates.header.stamp.nanosec = this->now().nanoseconds();

          double timeSec = this->now().seconds();
          int32 sec = timeSec;
          jointStates.header.stamp.sec = sec;
          jointStates.header.stamp.nanosec = (timeSec - sec) * 1e9;
          
          jointStates.name = {"Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "joint6"};
          jointStates.position = mJointStates;

          joint_states_publisher->publish(jointStates);

  //        RCLCPP_INFO(this->get_logger(), "Publish joint states"); /*Publish feedback*/
  //        qDebug() << "joint states:" << mJointStates;
          rate.sleep();
      }
  }

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<moveit_robotarm::RobotArm>("RobotArm");
    // 打印一句自我介绍
    RCLCPP_INFO(node->get_logger(), "node节点已经启动.");
    /* 运行节点，并检测退出信号 Ctrl+C*/
    rclcpp::spin(node);
    /* 停止运行 */
    rclcpp::shutdown();
}