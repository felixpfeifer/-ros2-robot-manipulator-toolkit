#include "../include/robot_teleoperation/demo_movment.hpp"
#include <chrono>

using namespace std::chrono_literals;

double deg2rad(double deg)
{
  return deg * M_PI / 180;
}

void use_gripper(bool open)
{
  if (open)
  {
    RCLCPP_INFO(logger, "Closing the gripper");
  }
  else
  {
    RCLCPP_INFO(logger, "Opening the gripper");
  }

  auto tool_request = std::make_shared<robot_teleoperation_interface::srv::Tool::Request>();
  tool_request->close = true;
  auto tool_result = tool_client->async_send_request(tool_request);
  rclcpp::spin_until_future_complete(node, tool_result);
  if (tool_result.share().get()->success)
  {
    if (open)
    {
      RCLCPP_INFO(logger, "Closing the gripper was successful");
    }
    else
    {
      RCLCPP_INFO(logger, "Closing the gripper was successful");
    }
  }
  else
  {
    if (open)
    {
      RCLCPP_ERROR(logger, "Closing the gripper failed");
    }
    else
    {
      RCLCPP_ERROR(logger, "Opening the gripper failed");
    }
  }
}

void dynamicSleep(int timeout_ms)
{
  // Convert integer milliseconds to std::chrono::nanoseconds
  auto duration = std::chrono::milliseconds(timeout_ms);

  // Use rclcpp::sleep_for with the computed duration
  rclcpp::sleep_for(duration);
}

/**
 * @brief Executes a grasp routine for a robotic manipulator.
 *
 * This function controls the grasping mechanism of a robotic manipulator.
 * It can either initiate a grasp or release action based on the provided
 * parameters.
 *
 * @param grasp A boolean indicating whether to grasp (true) or release (false).
 * @param distance A double representing the distance in mm to move the gripper.
 * @param timeout A int specifying the timeout duration in s for the operation.
 */
void grasp_routine(bool grasp, double distance, int timeout)
{
  use_gripper(grasp);
  dynamicSleep(timeout);

  // Move the robot down 100mm in the Z axis
  RCLCPP_INFO(logger, "Moving the robot down 100mm in the Z axis");
  auto request = std::make_shared<robot_teleoperation_interface::srv::MoveRobot::Request>();
  request->frame_id = 2;
  request->axis.push_back(2);
  request->data.push_back(0.1);

  auto result = move_robot_client->async_send_request(request);
  rclcpp::spin_until_future_complete(node, result);
  if (result.share().get()->success)
  {
    RCLCPP_INFO(logger, "Moving robot was successful");
  }
  else
  {
    RCLCPP_ERROR(logger, "Moving robot failed");
  }
  dynamicSleep(timeout);
  // Close the gripper
  use_gripper(!grasp);
  dynamicSleep(timeout);
  // Move the robot back up 100mm in the Z axis
  RCLCPP_INFO(logger, "Moving the robot back up 100mm in the Z axis");
  request = std::make_shared<robot_teleoperation_interface::srv::MoveRobot::Request>();
  request->frame_id = 2;
  request->axis.push_back(2);
  request->data.push_back(-0.1);

  result = move_robot_client->async_send_request(request);
  rclcpp::spin_until_future_complete(node, result);
  if (result.share().get()->success)
  {
    RCLCPP_INFO(logger, "Moving robot was successful");
  }
  else
  {
    RCLCPP_ERROR(logger, "Moving robot failed");
  }
}

void controlLoop()
{
  while (rclcpp::ok())
  {
    // Select the TCP_GRIPPER as the tool
    RCLCPP_INFO(logger, "Selecting the TCP_GRIPPER as the tool");
    auto select_tool_request = std::make_shared<robot_teleoperation_interface::srv::SelectTool::Request>();
    select_tool_request->tcp_id = 2;
    auto select_tool_result = select_tool_client->async_send_request(select_tool_request);
    rclcpp::spin_until_future_complete(node, select_tool_result);
    if (select_tool_result.share().get()->success)
    {
      RCLCPP_INFO(logger, "Selecting the TCP_GRIPPER as the tool was successful");
    }
    else
    {
      RCLCPP_ERROR(logger, "Selecting the TCP_GRIPPER as the tool failed");
    }
    rclcpp::sleep_for(1s);

    // Move the Robot to the "home" point
    RCLCPP_INFO(logger, "Moving the robot to the home point");
    auto move_point_request = std::make_shared<robot_teleoperation_interface::srv::MovePoint::Request>();
    move_point_request->point_name = "home";
    auto move_point_result = move_point_client->async_send_request(move_point_request);
    rclcpp::spin_until_future_complete(node, move_point_result);
    if (move_point_result.share().get()->success)
    {
      RCLCPP_INFO(logger, "Moving robot to home point was successful");
    }
    else
    {
      RCLCPP_ERROR(logger, "Moving robot to home point failed");
    }
    rclcpp::sleep_for(1s);
    RCLCPP_INFO(logger, "Moved the robot to the home point");
    // Move the Robot 45 degress in the joint 0
    RCLCPP_INFO(logger, "Moving the robot 45 degrees in the joint 0");
    auto request = std::make_shared<robot_teleoperation_interface::srv::MoveRobot::Request>();
    request->frame_id = 0;
    request->axis.push_back(0);
    request->data.push_back(deg2rad(45));
    // Rotate the tool joint 5 by 45 degrees
    request->axis.push_back(5);
    request->data.push_back(deg2rad(45));

    auto result = move_robot_client->async_send_request(request);
    rclcpp::spin_until_future_complete(node, result);
    if (result.share().get()->success)
    {
      RCLCPP_INFO(logger, "Moving robot was successful");
    }
    else
    {
      RCLCPP_ERROR(logger, "Moving robot failed");
    }
    rclcpp::sleep_for(1s);
    // Open the gripper
    grasp_routine(true, 100, 1000);
    rclcpp::sleep_for(1s);

    // Move the robot -45 degress in the joint 0
    RCLCPP_INFO(logger, "Moving the robot -45 degrees in the joint 0");
    request = std::make_shared<robot_teleoperation_interface::srv::MoveRobot::Request>();
    request->frame_id = 0;
    request->axis.push_back(0);
    request->data.push_back(deg2rad(-45));

    // Rotate the tool joint 5 by -90 degrees
    request->axis.push_back(5);
    request->data.push_back(deg2rad(-45));

    result = move_robot_client->async_send_request(request);
    rclcpp::spin_until_future_complete(node, result);
    if (result.share().get()->success)
    {
      RCLCPP_INFO(logger, "Moving robot was successful");
    }
    else
    {
      RCLCPP_ERROR(logger, "Moving robot failed");
    }
    rclcpp::sleep_for(1s);
    // Move the tool z axis 100mm
    RCLCPP_INFO(logger, "Moving the tool z axis 100mm");
    request = std::make_shared<robot_teleoperation_interface::srv::MoveRobot::Request>();
    request->frame_id = 2;
    request->axis.push_back(2);
    request->data.push_back(0.1);
    result = move_robot_client->async_send_request(request);
    rclcpp::spin_until_future_complete(node, result);
    if (result.share().get()->success)
    {
      RCLCPP_INFO(logger, "Moving robot was successful");
    }
    else
    {
      RCLCPP_ERROR(logger, "Moving robot failed");
    }
    rclcpp::sleep_for(1s);
    grasp_routine(false, 100, 1000);
    rclcpp::sleep_for(1s);
  }
}

void setup()
{
  // Create a ROS2 Node

  // Create Service Clients
  align_tcp_client = node->create_client<robot_teleoperation_interface::srv::AllignTCP>("align_tcp");
  select_tool_client = node->create_client<robot_teleoperation_interface::srv::SelectTool>("select_tool");
  move_robot_client = node->create_client<robot_teleoperation_interface::srv::MoveRobot>("move_robot");
  teach_point_client = node->create_client<robot_teleoperation_interface::srv::TeachPoint>("teach_point");
  tool_client = node->create_client<robot_teleoperation_interface::srv::Tool>("tool");
  move_point_client = node->create_client<robot_teleoperation_interface::srv::MovePoint>("move_point");
  rclcpp::sleep_for(5s);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("demo_movement_node");
  setup();
  controlLoop();
  rclcpp::shutdown();
  return 0;
}
