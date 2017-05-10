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

#include <move_base_exploration_server/move_base_exploration_ros.h>

#include <hector_move_base_msgs/MoveBaseActionPath.h>

#include <grid_map_ros/GridMapRosConverter.hpp>

namespace move_base_exploration{


MoveBaseLiteRos::MoveBaseExplorationRos(ros::NodeHandle& nh_, ros::NodeHandle& pnh_)
{
  p_source_frame_name_ = "base_link";
  p_target_frame_name_ = "world";

  pose_source_.header.frame_id = p_source_frame_name_;
  pose_source_.pose.orientation.w = 1.0;

  move_base_exploration_server_.reset(new actionlib::SimpleActionServer<move_base_lite_msgs::ExploreAction>(nh_, "/start_explore", false));

  tfl_ = boost::make_shared<tf::TransformListener>();
  grid_map_planner_ = boost::make_shared<grid_map_planner::GridMapPlanner>();



  map_sub_ = nh_.subscribe("/map", 1, &MoveBaseLiteRos::mapCallback, this);



  ros::NodeHandle controller_nh("/controller");
  drivepath_pub_ = controller_nh.advertise<hector_move_base_msgs::MoveBaseActionPath>("path", 0 );
  //controller_result_sub_ = controller_nh.subscribe<hector_move_base_msgs::MoveBaseActionResult>("result", 1, boost::bind(&MoveBaseLiteRos::controllerResultCB, this, _1));

//  simple_goal_sub_ = pnh_.subscribe<geometry_msgs::PoseStamped>("/move_base/simple_goal", 1, boost::bind(&MoveBaseLiteRos::simple_goalCB, this, _1));
  //simple_goal_sub_ = pnh_.subscribe<geometry_msgs::PoseStamped>("/goal", 1, boost::bind(&MoveBaseLiteRos::goalCB, this, _1));



  move_base_exploration_server_->registerGoalCallback(boost::bind(&MoveBaseLiteRos::moveExplorationGoalCB, this));
  move_base_exploration_server_->registerPreemptCallback(boost::bind(&MoveBaseLiteRos::moveBaseCancelCB, this));
  move_base_exploration_server_->start();

  follow_path_client_.reset(new actionlib::SimpleActionClient<move_base_lite_msgs::FollowPathAction>("/controller/follow_path"));

}



void MoveBaseLiteRos::moveExplorationGoalCB() {
  ROS_DEBUG("[move_base_exploration] In ActionServer goal callback");
//  move_base_action_goal_ = move_base_action_server_->acceptNewGoal();

//  current_goal_ = move_base_action_goal_->target_pose;

//  move_base_lite_msgs::FollowPathGoal follow_path_goal;
//  follow_path_goal.follow_path_options = move_base_action_goal_->follow_path_options;

//  if (generatePlanToGoal(current_goal_, follow_path_goal)){
//    sendActionToController(follow_path_goal);
//  }

}



void MoveBaseLiteRos::moveBaseCancelCB() {
  if (move_base_action_server_->isActive()){
    move_base_lite_msgs::MoveBaseResult result;
    result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
    move_base_action_server_->setPreempted(result, "preempt from incoming message to server");
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
    }

  }else if (result_in->result.val == move_base_lite_msgs::ErrorCodes::CONTROL_FAILED){
    // If control fails (meaning carrot is more than threshold away from robot), we try replanning
    move_base_lite_msgs::FollowPathGoal follow_path_goal;

    if (move_base_action_server_->isActive()){
      follow_path_goal.follow_path_options = move_base_action_goal_->follow_path_options;
    }

    if (generatePlanToGoal(current_goal_, follow_path_goal)){
      sendActionToController(follow_path_goal);
    }else{
      if (move_base_action_server_->isActive()){
        move_base_lite_msgs::MoveBaseResult result;
        result.result.val = move_base_lite_msgs::ErrorCodes::PLANNING_FAILED;
        move_base_action_server_->setAborted(result, "Planning failed when trying to replan after control failure.");
      }
    }
  }else{
    if (move_base_action_server_->isActive()){
      move_base_lite_msgs::MoveBaseResult result;
      result.result.val = result_in->result.val;
      move_base_action_server_->setAborted(result, "Controller failed with message: " + state.getText());
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
    move_base_action_server_->setPreempted(result, "preempt via simple goal callback");
  }

  move_base_lite_msgs::FollowPathGoal follow_path_goal;

  if (generatePlanToGoal(current_goal_, follow_path_goal)){
    sendActionToController(follow_path_goal);
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

void MoveBaseLiteRos::exploreCB(const geometry_msgs::PoseStampedConstPtr &exploreGoal) {
  ROS_DEBUG("[move_base_lite] In ActionServer explore callback");
 
//  if (move_base_action_server_->isActive()){
//    move_base_lite_msgs::MoveBaseResult result;
//    result.result.val = move_base_lite_msgs::ErrorCodes::PREEMPTED;
//    move_base_action_server_->setPreempted(result, "preempt via explore goal callback");
//  }

}

bool MoveBaseLiteRos::generatePlanToGoal(geometry_msgs::PoseStamped& goal_pose, move_base_lite_msgs::FollowPathGoal& goal)
{
  geometry_msgs::PoseStamped current_pose;

  if (!getPose (current_pose)){
    ROS_ERROR("[move_base_lite] Could not retrieve robot pose, aborting planning.");
    return false;
  }

  goal.target_path.header.frame_id = "world";

  if (!grid_map_planner_->makePlan(current_pose.pose, goal_pose.pose, goal.target_path.poses))
  {
    ROS_ERROR("[move_base_lite] Planning to goal pose failed, aborting planning.");
    return false;
  }
  return true;
}

void MoveBaseLiteRos::sendActionToController(const move_base_lite_msgs::FollowPathGoal& goal)
{
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

  //static_map_fused_ = static_map_retrieved_;


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
