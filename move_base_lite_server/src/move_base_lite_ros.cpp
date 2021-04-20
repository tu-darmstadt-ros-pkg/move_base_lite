//=================================================================================================
// Copyright (c) 2017, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <move_base_lite_server/move_base_lite_ros.h>

#include <hector_move_base_msgs/MoveBaseActionPath.h>

#include <grid_map_ros/GridMapRosConverter.hpp>

#include <nav_msgs/Path.h>



namespace move_base_lite{


MoveBaseLiteRos::MoveBaseLiteRos(ros::NodeHandle& nh_, ros::NodeHandle& pnh_)
{
  p_source_frame_name_ = "base_link";
  p_target_frame_name_ = "world";

  pose_source_.header.frame_id = p_source_frame_name_;
  pose_source_.pose.orientation.w = 1.0;

  move_base_action_server_.reset(new actionlib::SimpleActionServer<move_base_lite_msgs::MoveBaseAction>(nh_, "/move_base", false));

  explore_action_server_.reset(new actionlib::SimpleActionServer<move_base_lite_msgs::ExploreAction>(nh_, "/explore", false));

  tfl_ = boost::make_shared<tf::TransformListener>();
  grid_map_planner_ = boost::make_shared<grid_map_planner::GridMapPlanner>();




  map_sub_ = nh_.subscribe("/map", 1, &MoveBaseLiteRos::mapCallback, this);

  debug_map_pub_ = pnh_.advertise<grid_map_msgs::GridMap>("debug_planning_map", 1 );


  drivepath_pub_ = pnh_.advertise<nav_msgs::Path>("path", 1, true);
  //controller_result_sub_ = controller_nh.subscribe<hector_move_base_msgs::MoveBaseActionResult>("result", 1, boost::bind(&MoveBaseLiteRos::controllerResultCB, this, _1));

  simple_goal_sub_ = pnh_.subscribe<geometry_msgs::PoseStamped>("/move_base/simple_goal", 1, boost::bind(&MoveBaseLiteRos::simple_goalCB, this, _1));
  //simple_goal_sub_ = pnh_.subscribe<geometry_msgs::PoseStamped>("/goal", 1, boost::bind(&MoveBaseLiteRos::goalCB, this, _1));



  move_base_action_server_->registerGoalCallback(boost::bind(&MoveBaseLiteRos::moveBaseGoalCB, this));
  move_base_action_server_->registerPreemptCallback(boost::bind(&MoveBaseLiteRos::moveBaseCancelCB, this));
  move_base_action_server_->start();

  explore_action_server_->registerGoalCallback(boost::bind(&MoveBaseLiteRos::exploreGoalCB, this));
  explore_action_server_->registerPreemptCallback(boost::bind(&MoveBaseLiteRos::exploreCancelCB, this));
  explore_action_server_->start();

  follow_path_client_.reset(new actionlib::SimpleActionClient<move_base_lite_msgs::FollowPathAction>("/controller/follow_path", false));

  dyn_rec_server_.reset(new ReconfigureServer(config_mutex_, pnh_));
  dyn_rec_server_->setCallback(boost::bind(&MoveBaseLiteRos::reconfigureCallback, this, _1, _2));    
  
}

void MoveBaseLiteRos::reconfigureCallback(move_base_lite_server::MoveBaseLiteConfig &config, uint32_t level) {

    ROS_INFO("move_base_lite_server received dynamic reconfigure, lethal dist: %f, penalty dist: %f penalty_weight: %f",
             config.lethal_dist,
             config.penalty_dist,
             config.penalty_weight
            );
    
    grid_map_planner_->setDistanceThresholds(config.lethal_dist, config.penalty_dist, config.penalty_weight);
    
}


void MoveBaseLiteRos::moveBaseGoalCB() {
  ROS_DEBUG("[move_base_lite] In ActionServer goal callback");
  if (explore_action_server_->isActive()){
    exploreCancelCB();
  }
  
  move_base_action_goal_ = move_base_action_server_->acceptNewGoal();

  current_goal_ = move_base_action_goal_->target_pose;

  move_base_lite_msgs::FollowPathGoal follow_path_goal;
  follow_path_goal.follow_path_options = move_base_action_goal_->follow_path_options;

  if (move_base_action_goal_->plan_path_options.planning_approach == move_base_lite_msgs::PlanPathOptions::DEFAULT_COLLISION_FREE){
    if (generatePlanToGoal(current_goal_, follow_path_goal)){
      sendActionToController(follow_path_goal);
    }
  }else if(move_base_action_goal_->plan_path_options.planning_approach == move_base_lite_msgs::PlanPathOptions::NO_PLANNNING_FORWARD_GOAL){
    // Push original target pose into path to follow
    follow_path_goal.target_path.poses.push_back(current_goal_);
    sendActionToController(follow_path_goal);
  }

}



void MoveBaseLiteRos::moveBaseCancelCB() {
  if (move_base_action_server_->isActive()){
    move_base_lite_msgs::MoveBaseResult result;
    result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
    move_base_action_server_->setPreempted(result, "preempt from incoming message to server");
    if (follow_path_client_->isServerConnected()){
        follow_path_client_->cancelAllGoals();
    }
  }else{
    ROS_WARN("[move_base_lite] Cancel request although server ist not active!");
  }
}

void MoveBaseLiteRos::followPathDoneCb(const actionlib::SimpleClientGoalState& state,
            const move_base_lite_msgs::FollowPathResultConstPtr& result_in)
{
 if (result_in->result.val == move_base_lite_msgs::ErrorCodes::SUCCESS){
    if (move_base_action_server_->isActive()){
      move_base_lite_msgs::MoveBaseResult result;
      result.result.val = move_base_lite_msgs::ErrorCodes::SUCCESS;
      move_base_action_server_->setSucceeded(result, "reached goal");
     }else if (explore_action_server_->isActive()){
      move_base_lite_msgs::ExploreResult result;
      result.result.val = move_base_lite_msgs::ErrorCodes::SUCCESS;
      explore_action_server_->setSucceeded(result, "reached goal");
    }

  }else if (result_in->result.val == move_base_lite_msgs::ErrorCodes::CONTROL_FAILED){
    // If control fails (meaning carrot is more than threshold away from robot), we try replanning
    move_base_lite_msgs::FollowPathGoal follow_path_goal;

    if (move_base_action_server_->isActive()){
      follow_path_goal.follow_path_options = move_base_action_goal_->follow_path_options;

      if (move_base_action_goal_->plan_path_options.planning_approach == move_base_lite_msgs::PlanPathOptions::NO_PLANNNING_FORWARD_GOAL){
        follow_path_goal.target_path.poses.push_back(current_goal_);
        sendActionToController(follow_path_goal);
      }else{
      if (generatePlanToGoal(current_goal_, follow_path_goal)){
        sendActionToController(follow_path_goal);
      }else{
        move_base_lite_msgs::MoveBaseResult result;
        result.result.val = move_base_lite_msgs::ErrorCodes::PLANNING_FAILED;
        move_base_action_server_->setAborted(result, "Planning failed when trying to replan after control failure.");
      }
      }
    }

    if (explore_action_server_->isActive()){

          move_base_lite_msgs::ExploreResult result;
          result.result.val = move_base_lite_msgs::ErrorCodes::PLANNING_FAILED;
          explore_action_server_->setAborted(result, "Control failed while following exploration plan.");
	}

  }else{
    if (move_base_action_server_->isActive()){
      move_base_lite_msgs::MoveBaseResult result;
      result.result.val = result_in->result.val;
      move_base_action_server_->setAborted(result, "Controller failed with message: " + state.getText());
    }else if (explore_action_server_->isActive()){
      move_base_lite_msgs::ExploreResult result;
      result.result.val = result_in->result.val;
      explore_action_server_->setAborted(result, "Controller failed with message: " + state.getText());
    }
  }

}

void MoveBaseLiteRos::followPathActiveCb()
{

}

void MoveBaseLiteRos::followPathFeedbackCb(const move_base_lite_msgs::FollowPathFeedbackConstPtr& feedback)
{

}

void MoveBaseLiteRos::simple_goalCB(const geometry_msgs::PoseStampedConstPtr &simpleGoal)
{
  current_goal_ = *simpleGoal;

  if (move_base_action_server_->isActive()){
    move_base_lite_msgs::MoveBaseResult result;
    result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
    move_base_action_server_->setPreempted(result, "move_base_lite goal action preempt via simple goal callback");
  }

  if (explore_action_server_->isActive()){
    move_base_lite_msgs::ExploreResult result;
    result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
    explore_action_server_->setAborted(result, "move_base_lite exploration action preempt via simple goal callback ");
  }

  move_base_lite_msgs::FollowPathGoal follow_path_goal;

  if (generatePlanToGoal(current_goal_, follow_path_goal)){
    sendActionToController(follow_path_goal);
  }else{
    ROS_WARN("Planning to simple goal pose failed!");
  }

  /*
  geometry_msgs::PoseStamped current_pose;

  if (!getPose (current_pose)){
    ROS_ERROR("[move_base_lite] Could not retrieve robot pose, aborting planning.");
    return;
  }

  goal.target_path.header.frame_id = "world";

  if (!grid_map_planner_->makePlan(current_pose.pose, simpleGoal->pose, goal.target_path.poses))
  {
    ROS_ERROR("[move_base_lite] Planning to goal pose failed, aborting planning.");
    return;
  }


  //goal.goal.target_path

  follow_path_client_->sendGoal(goal,
                                boost::bind(&MoveBaseLiteRos::followPathDoneCb, this, _1, _2),
                                actionlib::SimpleActionClient<move_base_lite_msgs::FollowPathAction>::SimpleActiveCallback(),
                                actionlib::SimpleActionClient<move_base_lite_msgs::FollowPathAction>::SimpleFeedbackCallback());
*/
  //path.goal.target_path

  /*
  static int goal_id = 0;
  ++goal_id;

  path.goal_id.stamp = current_pose.header.stamp;
  path.goal_id.id = std::to_string(goal_id);
  path.goal.speed = 0.2;//goal.speed;
  path.goal.target_path.header.frame_id = current_pose.header.frame_id;
  //path.reverse_allowed = goal.reverse_allowed;

  //geometry_msgs::PoseStamped targetPose;
  //targetPose.header = goal.target_pose.header;
  //targetPose.pose = goal.target_pose.pose;
  path.goal.target_path.poses = plan;

  drivepath_pub_.publish(path);
  */
}

void MoveBaseLiteRos::exploreGoalCB() {
  ROS_DEBUG("[move_base_lite] In ActionServer explore callback");
  if (move_base_action_server_->isActive()){
    moveBaseCancelCB();
  }

  explore_action_goal_ = explore_action_server_->acceptNewGoal();

  move_base_lite_msgs::FollowPathGoal follow_path_goal;
  if (!makeExplorationPlan(follow_path_goal)){
    //ROS_ERROR("Failed to generate exploration plan, aborting!");
    std::string error_desc = "Failed to generate exploration plan, aborting!";
    ROS_WARN_STREAM(error_desc);
    move_base_lite_msgs::ExploreResult result;
    result.result.val = move_base_lite_msgs::ErrorCodes::PLANNING_FAILED;
    explore_action_server_->setAborted(result, error_desc);
    return;
  }

  sendActionToController(follow_path_goal);
}

void MoveBaseLiteRos::exploreCancelCB() {
  if (explore_action_server_->isActive()){
    move_base_lite_msgs::ExploreResult result;
    result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
    explore_action_server_->setPreempted(result, "preempt from incoming message to server");

    follow_path_client_->cancelGoal();
    ROS_INFO("Exploration goal cancelled!");
  }else{
    ROS_WARN("[move_base_lite] Cancel request although exploration server ist not active!");
  }

}

bool MoveBaseLiteRos::makePlan(const geometry_msgs::Pose &start,
              const geometry_msgs::Pose &original_goal,
              std::vector<geometry_msgs::PoseStamped> &plan)
{
  bool success = grid_map_planner_->makePlan(start, original_goal, plan);

  if (debug_map_pub_.getNumSubscribers() > 0){
    grid_map_msgs::GridMap grid_map_msg;
    grid_map::GridMapRosConverter::toMessage(grid_map_planner_->getPlanningMap(), grid_map_msg);
    debug_map_pub_.publish(grid_map_msg);
  }


  return success;
}

bool MoveBaseLiteRos::makeExplorationPlan(move_base_lite_msgs::FollowPathGoal& goal)
{
  geometry_msgs::PoseStamped current_pose;
  if (!getPose(current_pose)){
    ROS_ERROR_STREAM("Unable to retrieve robot pose in move_base_lite, aborting!");
    return false;
  }

  nav_msgs::Path path;
  bool success = grid_map_planner_->makeExplorationPlan(current_pose.pose, path.poses);

  if (debug_map_pub_.getNumSubscribers() > 0){
    grid_map_msgs::GridMap grid_map_msg;
    grid_map::GridMapRosConverter::toMessage(grid_map_planner_->getPlanningMap(), grid_map_msg);
    debug_map_pub_.publish(grid_map_msg);
  }

  path.header.frame_id = "world";
  goal.target_path = path;

  goal.follow_path_options.desired_speed = explore_action_goal_->desired_speed;
  goal.follow_path_options.reset_stuck_history = explore_action_goal_->reset_stuck_history;
  goal.follow_path_options.reverse_allowed = true;

  return success;
}


bool MoveBaseLiteRos::generatePlanToGoal(geometry_msgs::PoseStamped& goal_pose, move_base_lite_msgs::FollowPathGoal& goal)
{
  geometry_msgs::PoseStamped current_pose;

  if (!getPose (current_pose)){
    ROS_ERROR("[move_base_lite] Could not retrieve robot pose, aborting planning.");
    return false;
  }

  goal.target_path.header.frame_id = "world";

  if (!this->makePlan(current_pose.pose, goal_pose.pose, goal.target_path.poses))
  {
    ROS_ERROR("[move_base_lite] Planning to goal pose failed, aborting planning.");
    return false;
  }
  return true;
}

void MoveBaseLiteRos::sendActionToController(const move_base_lite_msgs::FollowPathGoal& goal)
{
  drivepath_pub_.publish(goal.target_path);
  follow_path_client_->sendGoal(goal,
                                boost::bind(&MoveBaseLiteRos::followPathDoneCb, this, _1, _2),
                                actionlib::SimpleActionClient<move_base_lite_msgs::FollowPathAction>::SimpleActiveCallback(),
                                actionlib::SimpleActionClient<move_base_lite_msgs::FollowPathAction>::SimpleFeedbackCallback());
}


void MoveBaseLiteRos::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  ROS_DEBUG("[move_base_lite] Received map.");
  latest_occ_grid_map_ = msg;
  grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, std::string("occupancy"), grid_map_planner_->getPlanningMap());
  if (move_base_action_server_->isActive() || explore_action_server_->isActive()) {
    ROS_DEBUG("[move_base_lite] Planning new path");

    move_base_lite_msgs::FollowPathGoal follow_path_goal;
    bool success = false;
    if (move_base_action_server_->isActive() && move_base_action_goal_->plan_path_options.planning_approach == move_base_lite_msgs::PlanPathOptions::DEFAULT_COLLISION_FREE) {
      follow_path_goal.follow_path_options = move_base_action_goal_->follow_path_options;

      if (generatePlanToGoal(current_goal_, follow_path_goal)){
        success = true;
      }
    } else {
      if (explore_action_server_->isActive()) {
        if (makeExplorationPlan(follow_path_goal)){
          success = true;
        }
      }
    }

    if (success) {
      sendActionToController(follow_path_goal);
    }
  }
}

bool MoveBaseLiteRos::getPose(geometry_msgs::PoseStamped& pose_out)
{

  //pose_source_.header.frame_id = p_source_frame_name_;
  //pose_source_.pose.orientation.w = 1.0;
  // Always get latest transform
  pose_source_.header.stamp = ros::Time(0);

  try {
    tfl_->transformPose(p_target_frame_name_, pose_source_, pose_out);
    ROS_DEBUG("[move_base_lite] source pose (%s): %f, %f, %f ---> target pose (%s): %f, %f, %f",
             pose_source_.header.frame_id.c_str(), pose_source_.pose.position.x, pose_source_.pose.position.y, pose_source_.pose.position.z,
             pose_out.header.frame_id.c_str(),pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z );
    return true;

      //pose_pub_.publish(pose_out);
  }
  catch (tf::TransformException ex){
    ROS_WARN_THROTTLE(5.0, "[move_base_lite] tf lookup failed when trying to retrieve robot pose in move_base_lite:  %s. This message is throttled.",ex.what());
    return false;
  }
}

}
