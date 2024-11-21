/* /////////////////////////////////////////////////
Author: Abdurrahman Yilmaz (AYilmaz@lincoln.ac.uk)
version: v05
Date: 30 Sep 2024
Purpose: Generation of pose commands to Robot arm via Moveit2
Input: PoseArray message
Output: Randomly chosen pose from PoseArray for robot arm
        Capture image service call for image_view node
Project: Agri-OpenCore
/////////////////////////////////////////////////// */

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/string.hpp"

#include "std_srvs/srv/empty.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <tf2/LinearMath/Transform.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <string>
#include <iostream>
#include <cmath>
#include <tuple>
#include <fstream>
#include <filesystem>
#include <cstdio> 

#include <moveit_msgs/msg/constraints.hpp>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <moveit/robot_state/robot_state.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.hpp>

class MoveItCommanderRecorder : public rclcpp::Node
{
public:
  MoveItCommanderRecorder(const std::string& node_name, const rclcpp::NodeOptions& options)
    : Node(node_name, options),
      logger_(rclcpp::get_logger(node_name)),
      move_group_interface_(std::make_shared<rclcpp::Node>(node_name), getMoveGroupName(options)),
      next_node_sender_init_(false)   
  {
    // Initialize the subscriber and publisher
    initializeSubscriber();
    initializePublisher();
    // Initialize services and parameters
    high_priority_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    initializeServicesClients();
    auto pause_service_name = this->get_fully_qualified_name() + std::string("/pause_motion");
    pause_service_ = this->create_service<std_srvs::srv::Empty>(
        pause_service_name, std::bind(&MoveItCommanderRecorder::pauseMotion, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_default, high_priority_cb_group_);
    
    auto resume_service_name = this->get_fully_qualified_name() + std::string("/resume_motion");
    resume_service_ = this->create_service<std_srvs::srv::Empty>(
        resume_service_name, std::bind(&MoveItCommanderRecorder::resumeMotion, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_default, high_priority_cb_group_);
    
    high_priority_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    high_priority_executor_->add_callback_group(high_priority_cb_group_, this->get_node_base_interface());  

    high_priority_thread_ = std::thread([this]() {
        high_priority_executor_->spin();
    });
    initializeParameters();
    // Initialize moveit
    initializeMoveIt();

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  }

private:
  std::string getMoveGroupName(const rclcpp::NodeOptions& options)
  {
    // Retrieve the move_group_name parameter from NodeOptions
    (void)options;
    auto param_value = this->declare_parameter<std::string>("move_group_name", "panda_vision");
    RCLCPP_INFO(logger_, "Move group name: %s", param_value.c_str());

    return param_value;
  }

  void initializeParameters(){
    rcl_interfaces::msg::ParameterDescriptor pose_command_seq_type_descriptor;
    pose_command_seq_type_descriptor.name = "pose_command_seq_type";
    pose_command_seq_type_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    this->declare_parameter("pose_command_seq_type", "random", pose_command_seq_type_descriptor);

    rcl_interfaces::msg::ParameterDescriptor pre_delay_descriptor;
    pre_delay_descriptor.name = "pre_delay_";
    pre_delay_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    this->declare_parameter("pre_delay_", 2.0, pre_delay_descriptor);

    rcl_interfaces::msg::ParameterDescriptor post_delay_descriptor;
    post_delay_descriptor.name = "post_delay_";
    post_delay_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    this->declare_parameter("post_delay_", 0.0, post_delay_descriptor);

    rcl_interfaces::msg::ParameterDescriptor capture_images_descriptor;
    capture_images_descriptor.name = "capture_images_";
    capture_images_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    this->declare_parameter("capture_images_", false, capture_images_descriptor);

    rcl_interfaces::msg::ParameterDescriptor move_group_planning_time_descriptor;
    move_group_planning_time_descriptor.name = "move_group_planning_time_";
    move_group_planning_time_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    this->declare_parameter("move_group_planning_time_", 10.0, move_group_planning_time_descriptor);

    rcl_interfaces::msg::ParameterDescriptor move_group_max_vel_scale_descriptor;
    move_group_max_vel_scale_descriptor.name = "move_group_max_vel_scale_";
    move_group_max_vel_scale_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    this->declare_parameter("move_group_max_vel_scale_", 0.05, move_group_max_vel_scale_descriptor);

    rcl_interfaces::msg::ParameterDescriptor move_group_num_planning_attempts_descriptor;
    move_group_num_planning_attempts_descriptor.name = "move_group_num_planning_attempts_";
    move_group_num_planning_attempts_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    this->declare_parameter("move_group_num_planning_attempts_", 2, move_group_num_planning_attempts_descriptor);
    
    rcl_interfaces::msg::ParameterDescriptor image_info_record_sub_directory_descriptor;
    image_info_record_sub_directory_descriptor.name = "image_info_record_sub_directory_";
    image_info_record_sub_directory_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    this->declare_parameter("image_info_record_sub_directory_", "", image_info_record_sub_directory_descriptor);

    rcl_interfaces::msg::ParameterDescriptor image_info_record_file_name_descriptor;
    image_info_record_file_name_descriptor.name = "image_info_record_file_name_";
    image_info_record_file_name_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    this->declare_parameter("image_info_record_file_name_", "image_info", image_info_record_file_name_descriptor);

    rcl_interfaces::msg::ParameterDescriptor ref_frameID_descriptor;
    ref_frameID_descriptor.name = "ref_frameID_";
    ref_frameID_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    this->declare_parameter("ref_frameID_", "plant", ref_frameID_descriptor);

    rcl_interfaces::msg::ParameterDescriptor camera_frameID_descriptor;
    camera_frameID_descriptor.name = "camera_frameID_";
    camera_frameID_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    this->declare_parameter("camera_frameID_", "camera", camera_frameID_descriptor);

    rcl_interfaces::msg::ParameterDescriptor planner_type_descriptor;
    planner_type_descriptor.name = "planner_type";
    planner_type_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    this->declare_parameter("planner_type", "OMPL", planner_type_descriptor);

    rcl_interfaces::msg::ParameterDescriptor pilz_planner_descriptor;
    pilz_planner_descriptor.name = "pilz_planner";
    pilz_planner_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    this->declare_parameter("pilz_planner", "LIN", pilz_planner_descriptor);

    rcl_interfaces::msg::ParameterDescriptor ompl_planner_descriptor;
    ompl_planner_descriptor.name = "ompl_planner";
    ompl_planner_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    this->declare_parameter("ompl_planner", "RRTConnectkConfigDefault", ompl_planner_descriptor);
    
  }

  std::string getCommandSequenceType()
  {
    std::string pose_command_seq_type = this->get_parameter("pose_command_seq_type").as_string();
    RCLCPP_INFO(logger_, "pose_command_seq_type: %s", pose_command_seq_type.c_str());

    return pose_command_seq_type;
  }

  bool getIfCaptureImages()
  {
    bool capture_images_ = this->get_parameter("capture_images_").as_bool();
    RCLCPP_INFO(logger_, "capture_images_: %d", capture_images_);

    return capture_images_;
  }

  std::pair<double, double> getPrePostDelays()
  {
    double param_pre_delay = this->get_parameter("pre_delay_").as_double();
    RCLCPP_INFO(logger_, "pre_delay_: %f", param_pre_delay);

    double param_post_delay = this->get_parameter("post_delay_").as_double();
    RCLCPP_INFO(logger_, "post_delay_: %f", param_post_delay);

    return std::make_pair(param_pre_delay, param_post_delay);
  }

  std::tuple<double, double, int> getMoveGroupPlanningParams()
  {
    double param_planning_time_ = this->get_parameter("move_group_planning_time_").as_double();
    RCLCPP_INFO(logger_, "move_group_planning_time_: %f", param_planning_time_);

    double param_max_vel_scale_ = this->get_parameter("move_group_max_vel_scale_").as_double();
    RCLCPP_INFO(logger_, "move_group_max_vel_scale_: %f", param_max_vel_scale_);

    int param_num_plan_attempts_ = this->get_parameter("move_group_num_planning_attempts_").as_int();
    RCLCPP_INFO(logger_, "move_group_num_planning_attempts_: %d", param_num_plan_attempts_);

    return std::make_tuple(param_planning_time_, param_max_vel_scale_, param_num_plan_attempts_);
  }

  std::tuple<std::string, std::string> getImageInfoFileParams()
  {
    std::string subdirectory = this->get_parameter("image_info_record_sub_directory_").as_string();
    RCLCPP_INFO(logger_, "image_info_record_sub_directory_: %s", subdirectory.c_str());

    std::string file_name = this->get_parameter("image_info_record_file_name_").as_string();
    // Concatenate ".csv" to the file_name
    file_name += ".csv";
    RCLCPP_INFO(logger_, "image_info_record_file_name_: %s", file_name.c_str());

    return std::make_tuple(subdirectory, file_name);
  }

  std::tuple<std::string, std::string> getTFinfoParams()
  {
    std::string plant_frame = this->get_parameter("ref_frameID_").as_string();
    RCLCPP_INFO(logger_, "ref_frameID_: %s", plant_frame.c_str());

    std::string camera_frame = this->get_parameter("camera_frameID_").as_string();
    RCLCPP_INFO(logger_, "camera_frameID_: %s", camera_frame.c_str());

    return std::make_tuple(plant_frame, camera_frame);
  }

  void initializeSubscriber()
  {
    // Create a subscriber for the "pose" topic
    pose_array_subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/lattice_pose_array", 10, std::bind(&MoveItCommanderRecorder::poseArrayCallback, this, std::placeholders::_1));
  }

  void initializePublisher()
  {
    // Create a publisher for the "message" topic
    message_publisher_ = this->create_publisher<std_msgs::msg::String>("message", 10);
    // Create a publisher for the "arm_next_node_pose" topic
    pose_stamped_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("arm_next_node_pose", 10);
  }

  void initializeServicesClients()
  {
    client = create_client<std_srvs::srv::Empty>("image_saver/save");

    if (!client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Service not available. Exiting.");
      return; 
    }
    

  }

  void initializeMoveIt()
  {
    rcl_interfaces::msg::ParameterDescriptor move_group_descriptor;
    move_group_descriptor.name = "move_group_name_";
    move_group_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    this->declare_parameter("move_group_name_", "panda_vision", move_group_descriptor); //panda_arm_hand, panda_vision, panda_arm
    
    std::string move_group_name_param = this->get_parameter("move_group_name_").as_string();
    RCLCPP_INFO(logger_, "move_group_name_: %s", move_group_name_param.c_str());
    
    // Create the MoveIt MoveGroup Interface
    move_group_interface_ = moveit::planning_interface::MoveGroupInterface(std::make_shared<rclcpp::Node>(this->get_name()), move_group_name_param.c_str());

    // Get the planner type (OMPL or PILZ)
    std::string planner_type = this->get_parameter("planner_type").as_string();

    if (planner_type == "PILZ")
    {
      std::string pilz_planner_id = this->get_parameter("pilz_planner").as_string();
      move_group_interface_.setPlannerId(pilz_planner_id);
      RCLCPP_INFO(logger_, "Pilz planner selected: %s", pilz_planner_id.c_str());
    }
    /*else
    {
      std::string ompl_planner_id = this->get_parameter("ompl_planner").as_string();
      move_group_interface_.setPlannerId(ompl_planner_id);
      RCLCPP_INFO(logger_, "OMPL planner selected: %s", ompl_planner_id.c_str());
    }*/

  }

  void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr pose_array_msg)
  {
    // Process the received pose array message
    std_msgs::msg::String message;
    message.data = "Pose Array message stored!";
    message_publisher_->publish(message);

    if (pose_array_msg->poses.size() != previous_pose_array_length_) {
        current_pose_index_ = 0;
        RCLCPP_INFO(this->get_logger(), "Viewpoints array changed from %ld to %ld. Current index reset to 0.", previous_pose_array_length_, pose_array_msg->poses.size());
    }

    pose_array_msg_current = pose_array_msg;

    if(!next_node_sender_init_){
      // Timer to periodically publish next node message to moveit for robot arm
      timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MoveItCommanderRecorder::publishNextNode, this));
      next_node_sender_init_ = true;
    }

    previous_pose_array_length_ = pose_array_msg->poses.size();
    
  }

  void pauseMotion(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                  std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
      (void)request;
      (void)response;
      is_paused_ = true;
      RCLCPP_INFO(this->get_logger(), "Viewpoint tracking motion paused");
  }

  void resumeMotion(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                    std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
      (void)request;
      (void)response;
      is_paused_ = false;
      RCLCPP_INFO(this->get_logger(), "Viewpoint tracking motion resumed");
  }

  // Function to get tf information between plant frame and camera frame
  std::tuple<double, double, double, double, double, double> 
  getTfInfo(const std::string& target_frame, const std::string& source_frame) {
    try {
        // Get the transformation from plant frame to camera frame
        geometry_msgs::msg::TransformStamped transform_stamped =
            tf_buffer_->lookupTransform(target_frame, source_frame, rclcpp::Time(0));

        // Extract translation and rotation information
        double x = transform_stamped.transform.translation.x;
        double y = transform_stamped.transform.translation.y;
        double z = transform_stamped.transform.translation.z;

        double roll, pitch, yaw;
        tf2::Quaternion q(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // Print the tf information, if needed
        /*RCLCPP_INFO(logger_, "X: %f, Y: %f, Z: %f, Roll: %f, Pitch: %f, Yaw: %f",
                    x, y, z, roll, pitch, yaw);*/

        // Return the extracted information
        return std::make_tuple(x, y, z, roll, pitch, yaw);
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(logger_, "Failed to get TF information: %s", ex.what());
        // Return a default value or handle the error case as needed
        return std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
  }

  void save_image(){
    
    auto delays = getPrePostDelays();
    double pre_delay = delays.first;
    double post_delay = delays.second;
    // pre_delay_ seconds delay for eliminating stopping flactuation effects of arm
    RCLCPP_INFO(logger_, "Waiting for %f seconds to make sure arm is at the steady-state", pre_delay);
        
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(round(pre_delay*1000)))); 

    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    
    auto future = client->async_send_request(request);

    auto [subdirectory, csvFilename] = getImageInfoFileParams();

    auto [plant_frame, camera_frame] = getTFinfoParams();

    auto tf_data = getTfInfo(plant_frame, camera_frame);

    // Access the extracted information, if needed
    /*double x, y, z, roll, pitch, yaw;
    std::tie(x, y, z, roll, pitch, yaw) = tf_data;

    RCLCPP_INFO(logger_, "x: %f, y: %f, z: %f, R: %f, P: %f, Y: %f", x, y, z, roll, pitch, yaw);*/

    // Format the image name
    char image_name[20];  // Adjust the array size as needed
    std::sprintf(image_name, "img%04d.png", image_counter_);

    image_counter_++; // for next image name assign

    saveImageInfoToCSV(subdirectory, csvFilename, std::string(image_name), tf_data);

    RCLCPP_INFO(logger_, "Waiting for %f seconds to make sure image captured before arm moving to next pose ", post_delay);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(round(post_delay*1000)))); 

    // Handle the response here (for Empty service, there is no response)

  }

  void saveImageInfoToCSV(const std::string& subdirectory, const std::string& filename, const std::string& imageFileName, 
                          const std::tuple<double, double, double, double, double, double>& tfData)
  {
    // Create the full path by concatenating the current directory with the subdirectory and filename
    std::string fullFilePath = std::filesystem::current_path() / subdirectory / filename;

    // Open the CSV file for appending
    std::ofstream csvFile;
    csvFile.open(fullFilePath, std::ios::app);

    if (!csvFile.is_open()) {
        std::cerr << "Error: Unable to open CSV file: " << fullFilePath << std::endl;
        return;
    }

    // Write XYZ and RPY data to the CSV file on a new line, with comma separators
    csvFile << imageFileName << "," << std::get<0>(tfData) << "," << std::get<1>(tfData) << ","
            << std::get<2>(tfData) << "," << std::get<3>(tfData) << "," << std::get<4>(tfData) << ","
            << std::get<5>(tfData) << "\n";

    // Close the file
    csvFile.close();
  }

  void publishNextNode()
  {
    if (is_paused_) {
      RCLCPP_INFO(logger_, "Motion execution stopped by user! Current pose index: %d. Call resume_motion service to start viewpoint tracker.", current_pose_index_);  
      return;
    }
    if (!pose_array_msg_current->poses.empty())
    {
      auto planning_params = getMoveGroupPlanningParams();
      double planning_time = std::get<0>(planning_params);
      double max_vel_scale = std::get<1>(planning_params);
      int num_plan_attempts = std::get<2>(planning_params);

      move_group_interface_.setMaxVelocityScalingFactor(max_vel_scale);
      move_group_interface_.setNumPlanningAttempts(num_plan_attempts);            
      std::string ref_frameID_ = this->get_parameter("ref_frameID_").as_string();
      move_group_interface_.setPoseReferenceFrame(ref_frameID_);

      std::string source_frame = pose_array_msg_current->header.frame_id;
      auto [x, y, z, roll, pitch, yaw] = getTfInfo(ref_frameID_, source_frame);

      tf2::Transform transform;
      transform.setOrigin(tf2::Vector3(x, y, z));

      tf2::Quaternion q;
      q.setRPY(roll, pitch, yaw); 
      transform.setRotation(q);

      geometry_msgs::msg::PoseArray transformed_pose_array;
      transformed_pose_array.header.frame_id = ref_frameID_;

      RCLCPP_INFO(logger_, "Size of pose_array_msg_current->poses: %ld", pose_array_msg_current->poses.size());

      for (const auto &pose_current : pose_array_msg_current->poses) {
          // Convert geometry_msgs::Pose to tf2::Transform for transformation
          tf2::Transform pose_in_current_frame;
          tf2::fromMsg(pose_current, pose_in_current_frame);

          // Transform the pose into the target frame
          tf2::Transform pose_in_target_frame = transform * pose_in_current_frame;

          // Convert back to geometry_msgs::Pose
          geometry_msgs::msg::Pose transformed_pose;
          transformed_pose.position.x = pose_in_target_frame.getOrigin().x();
          transformed_pose.position.y = pose_in_target_frame.getOrigin().y();
          transformed_pose.position.z = pose_in_target_frame.getOrigin().z();

          tf2::Quaternion quat = pose_in_target_frame.getRotation();
          transformed_pose.orientation.x = quat.x();
          transformed_pose.orientation.y = quat.y();
          transformed_pose.orientation.z = quat.z();
          transformed_pose.orientation.w = quat.w();

          // Add the transformed pose to the new PoseArray
          transformed_pose_array.poses.push_back(transformed_pose);
      }
      RCLCPP_INFO(logger_, "Size of transformed_pose_array.poses: %ld", transformed_pose_array.poses.size());
      //move_group_interface_.setPoseReferenceFrame(pose_array_msg_current->header.frame_id.c_str());
      RCLCPP_INFO(logger_, "Motion will be planned and executed according to frameID: %s", transformed_pose_array.header.frame_id.c_str());

      
      std::string planner_type = this->get_parameter("planner_type").as_string();

      if (planner_type == "PILZ")
      {
        std::string pilz_planner_id = this->get_parameter("pilz_planner").as_string();
        move_group_interface_.setPlannerId(pilz_planner_id);
        RCLCPP_INFO(logger_, "Pilz planner selected: %s", pilz_planner_id.c_str());
      }
      /*else
      {
        std::string ompl_planner_id = this->get_parameter("ompl_planner").as_string();
        move_group_interface_.setPlannerId(ompl_planner_id);
        RCLCPP_INFO(logger_, "OMPL planner selected: %s", ompl_planner_id.c_str());
      }*/

      double minX = 0.0, minY = -0.9, minZ = -0.05;  // Define minimum bounds
      double maxX = 1.5, maxY = 0.9, maxZ = 1.5;   // Define maximum bounds

      moveit_msgs::msg::PositionConstraint box_constraint;
      box_constraint.header.frame_id = move_group_interface_.getPoseReferenceFrame();
      box_constraint.link_name = move_group_interface_.getEndEffectorLink();
      RCLCPP_INFO(logger_, "Constraint base frameID: %s", box_constraint.header.frame_id.c_str());
      RCLCPP_INFO(logger_, "Constraint target frameID: %s", box_constraint.link_name.c_str());
      shape_msgs::msg::SolidPrimitive box;
      box.type = shape_msgs::msg::SolidPrimitive::BOX;
      box.dimensions = { maxX - minX, maxY - minY, maxZ - minZ };
      box_constraint.constraint_region.primitives.emplace_back(box);

      geometry_msgs::msg::Pose box_pose;
      box_pose.position.x = (maxX + minX) / 2.0;
      box_pose.position.y = (maxY + minY) / 2.0;
      box_pose.position.z = (maxZ + minZ) / 2.0;
      box_pose.orientation.w = 1.0;
      box_constraint.constraint_region.primitive_poses.emplace_back(box_pose);
      box_constraint.weight = 1.0;

      moveit_msgs::msg::Constraints box_constraints;
      box_constraints.position_constraints.emplace_back(box_constraint);

      move_group_interface_.setPathConstraints(box_constraints);
      move_group_interface_.setPlanningTime(planning_time);
      
      if (getCommandSequenceType() == "random"){
        // Randomly select one pose from the PoseArray
        geometry_msgs::msg::PoseStamped selected_pose;
        selected_pose.pose = transformed_pose_array.poses[rand() % transformed_pose_array.poses.size()];

        selected_pose.header.frame_id = transformed_pose_array.header.frame_id.c_str();
        selected_pose.header.stamp = rclcpp::Clock().now();
        
        move_group_interface_.setPoseTarget(selected_pose);
        pose_stamped_publisher_->publish(selected_pose);

        // Generate a plan to that target pose
        auto const [success, plan] = [this]() -> std::pair<bool, moveit::planning_interface::MoveGroupInterface::Plan> {
          moveit::planning_interface::MoveGroupInterface::Plan msg;
          auto const ok = static_cast<bool>(move_group_interface_.plan(msg));
          return std::make_pair(ok, msg);
        }();
        // Execute the plan
        if (success)
        {
          double trajectory_length = plan.trajectory_.joint_trajectory.points.back().time_from_start.sec;
          
          RCLCPP_INFO(logger_, "Trajectory length: %f", trajectory_length);

          RCLCPP_INFO(logger_, "Checking motion plan feasibility now!");

          const moveit::core::RobotModelConstPtr& robot_model = move_group_interface_.getRobotModel();
          const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(move_group_interface_.getName());

          for (const auto& point : plan.trajectory_.joint_trajectory.points)
          {
              // Create a RobotState object
              moveit::core::RobotState robot_state(robot_model);

              // Set the joint positions in the RobotState object
              robot_state.setJointGroupPositions(joint_model_group, point.positions);

              // Update the state to compute forward kinematics
              robot_state.update();

              // Get the end-effector link transform
              const Eigen::Isometry3d& end_effector_transform = robot_state.getGlobalLinkTransform(move_group_interface_.getEndEffectorLink());

              // Extract the position from the end-effector pose
              const Eigen::Vector3d& position = end_effector_transform.translation(); 

              // Optional: Check if the position is within constraints
              if (position.x() < minX || position.x() > maxX || position.y() < minY || position.y() > maxY || position.z() < minZ || position.z() > maxZ)
              {
                  RCLCPP_WARN(logger_, "Unfeasible path waypoint: [x: %f, y: %f, z: %f]", position.x(), position.y(), position.z());
              }
          }

          RCLCPP_INFO(logger_, "Motion plan is executed now!");
          moveit::core::MoveItErrorCode execution_result = move_group_interface_.execute(plan);
          
          // Check the result of the execution
          if (execution_result == moveit::core::MoveItErrorCode::SUCCESS)
          {
            RCLCPP_INFO(logger_, "Motion execution successful!");
            // Take capture at current pose via camera
            if(getIfCaptureImages()){
              save_image();
            }
          }
          else
          {
            RCLCPP_ERROR(logger_, "Motion execution failed with error code: %d", execution_result.val);
          }
        }
        else
        {
          RCLCPP_ERROR(logger_, "Planning failed!");
        }
        move_group_interface_.clearPathConstraints();
      }
      else if(getCommandSequenceType() == "all"){
        // Send all poses in a order PoseArray
        geometry_msgs::msg::PoseStamped selected_pose;
        for (std::size_t cnt = current_pose_index_; cnt < transformed_pose_array.poses.size(); cnt++) {
          current_pose_index_ = cnt;
          RCLCPP_INFO(logger_, "cnt: %ld", cnt);  
            
          if (is_paused_) {
            RCLCPP_INFO(logger_, "Motion execution stopped by user! Current pose index: %d. Call resume_motion service to start viewpoint tracker.", current_pose_index_);  
            return;
          }
          selected_pose.pose = transformed_pose_array.poses[cnt];

          selected_pose.header.frame_id = transformed_pose_array.header.frame_id.c_str();
          selected_pose.header.stamp = rclcpp::Clock().now();

          move_group_interface_.setPoseTarget(selected_pose);
          move_group_interface_.setPathConstraints(box_constraints);
          pose_stamped_publisher_->publish(selected_pose);

          // Generate a plan to that target pose
          auto const [success, plan] = [this]() -> std::pair<bool, moveit::planning_interface::MoveGroupInterface::Plan> {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_interface_.plan(msg));
            return std::make_pair(ok, msg);
          }();
          // Execute the plan
          if (success)
          {
            double trajectory_length = plan.trajectory_.joint_trajectory.points.back().time_from_start.sec;
            
            RCLCPP_INFO(logger_, "Trajectory length: %f", trajectory_length);

            RCLCPP_INFO(logger_, "Checking motion plan feasibility now!");

            const moveit::core::RobotModelConstPtr& robot_model = move_group_interface_.getRobotModel();
            const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(move_group_interface_.getName());

            for (const auto& point : plan.trajectory_.joint_trajectory.points)
            {
                // Create a RobotState object
                moveit::core::RobotState robot_state(robot_model);

                // Set the joint positions in the RobotState object
                robot_state.setJointGroupPositions(joint_model_group, point.positions);

                // Update the state to compute forward kinematics
                robot_state.update();

                // Get the end-effector link transform
                const Eigen::Isometry3d& end_effector_transform = robot_state.getGlobalLinkTransform(move_group_interface_.getEndEffectorLink());

                // Extract the position from the end-effector pose
                const Eigen::Vector3d& position = end_effector_transform.translation(); 

                // Optional: Check if the position is within constraints
                if (position.x() < minX || position.x() > maxX || position.y() < minY || position.y() > maxY || position.z() < minZ || position.z() > maxZ)
                {
                    RCLCPP_WARN(logger_, "Unfeasible path waypoint: [x: %f, y: %f, z: %f]", position.x(), position.y(), position.z());
                }
            }

            RCLCPP_INFO(logger_, "Motion plan is executed now!");
            moveit::core::MoveItErrorCode execution_result = move_group_interface_.execute(plan);
            
            // Check the result of the execution
            if (execution_result == moveit::core::MoveItErrorCode::SUCCESS)
            {
              RCLCPP_INFO(logger_, "Motion execution successful!");
              // Take capture at current pose via camera
              if(getIfCaptureImages()){
                save_image();
              }
            }
            else
            {
              RCLCPP_ERROR(logger_, "Motion execution failed with error code: %d", execution_result.val);
            }
          }
          else
          {
            RCLCPP_ERROR(logger_, "Planning failed!");
          }
          move_group_interface_.clearPathConstraints();
        }
        if (!is_paused_) {
          RCLCPP_INFO(logger_, "All lattices scanned, returning back to first one!");
          current_pose_index_ = 0;
        }
      }
    }
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr message_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Logger logger_;
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  //moveit_visual_tools::MoveItVisualTools moveit_visual_tools_;
  //std::string move_group_name_;
  bool next_node_sender_init_;
  geometry_msgs::msg::PoseArray::SharedPtr pose_array_msg_current;
  rcl_interfaces::msg::ParameterDescriptor pose_command_seq_type_descriptor;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client;
  int image_counter_ = 0;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  int current_pose_index_ = 0;   // To track the current pose index for "all" option
  bool is_paused_ = true;
  size_t previous_pose_array_length_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr pause_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr resume_service_;

  rclcpp::CallbackGroup::SharedPtr high_priority_cb_group_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> high_priority_executor_;
  std::thread high_priority_thread_;

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Specify the node name
  std::string node_name = "moveit_commander_node";  // Change this to your desired node name

  // Create node options with parameters YAML file
  std::vector<rclcpp::Parameter> parameters;
  parameters.push_back(rclcpp::Parameter("move_group_name", "panda_vision")); // panda_arm_hand, panda_vision, panda_arm
  // Add more parameters as needed

  // Create an instance of the RandomMoveItCommandNode class with the specified node name and parameters
  auto node_options = rclcpp::NodeOptions()
    .arguments({"--ros-args", "-p", "move_group_name:=custom_move_group_name"})  // Override parameter if needed
    .parameter_overrides(parameters);
  auto node = std::make_shared<MoveItCommanderRecorder>(node_name, node_options);

  // Spin the node
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
