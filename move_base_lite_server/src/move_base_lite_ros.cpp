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

namespace move_base_lite{


MoveBaseLiteRos::MoveBaseLiteRos(ros::NodeHandle& nh_, ros::NodeHandle& pnh_)
{
  p_source_frame_name_ = "world";
  p_target_frame_name_ = "base_link";

  pose_source_.header.frame_id = p_source_frame_name_;
  pose_source_.pose.orientation.w = 1.0;

  action_server_.reset(new actionlib::SimpleActionServer<hector_move_base_msgs::MoveBaseAction>(nh_, "move_base", false));

  tfl_ = boost::make_shared<tf::TransformListener>();
  grid_map_planner_ = boost::make_shared<grid_map_planner::GridMapPlanner>();



  //map_sub_ = nh_.subscribe("/dynamic_map", 1, &MoveBaseLiteRos::mapCallback, this);



  ros::NodeHandle controller_nh("/controller");
  drivepath_pub_ = controller_nh.advertise<hector_move_base_msgs::MoveBaseActionPath>("path", 0 );
  controller_result_sub_ = controller_nh.subscribe<hector_move_base_msgs::MoveBaseActionResult>("result", 1, boost::bind(&MoveBaseLiteRos::controllerResultCB, this, _1));

  simple_goal_sub_ = pnh_.subscribe<geometry_msgs::PoseStamped>("simple_goal", 1, boost::bind(&MoveBaseLiteRos::simple_goalCB, this, _1));
  //simple_goal_sub_ = pnh_.subscribe<geometry_msgs::PoseStamped>("/goal", 1, boost::bind(&MoveBaseLiteRos::goalCB, this, _1));



  action_server_->registerGoalCallback(boost::bind(&MoveBaseLiteRos::asGoalCB, this));
  action_server_->registerPreemptCallback(boost::bind(&MoveBaseLiteRos::asCancelCB, this));
  action_server_->start();

}



void MoveBaseLiteRos::asGoalCB() {
  ROS_INFO("[hector_move_base]: In ActionServer goal callback");
  const hector_move_base_msgs::MoveBaseGoalConstPtr goal = action_server_->acceptNewGoal();
/*
  clearGoal();

  handlerActionGoal newGoal = handlerActionGoal();
  newGoal.goal_id.id = goal_id_counter_++;
  newGoal.goal_id.stamp = ros::Time::now();
  newGoal.speed = goal->speed;
  newGoal.reverse_allowed = goal->reverse_allowed;
  if (goal->exploration == true) {
      newGoal.do_exploration = true;
  }
  else {
      newGoal.do_exploration = false;
  }


  //make sure goal could be transformed to costmap frame
  if (goal->exploration == false) {
    newGoal.target_pose = goalToGlobalFrame(goal->target_pose);
    if (newGoal.target_pose.header.frame_id != costmap_->getGlobalFrameID()) {
        ROS_ERROR("[hector_move_base]: tf transformation into global frame failed. goal will be canceled");
        //new Goal has to be set in order to publish goal aborted result
        pushCurrentGoal(newGoal);
        abortedGoal();
        return;
    }
  }
  pushCurrentGoal(newGoal);
  setNextState(planningState_);
  return;
  */
}

void MoveBaseLiteRos::asCancelCB() {
  if (action_server_->isActive()){
    action_server_->setPreempted();
  }
  //ROS_INFO("[hector_move_base]: In ActionServer Preempt callback");
  //abortedGoal();
  //setNextState(idleState_);
}

void MoveBaseLiteRos::simple_goalCB(const geometry_msgs::PoseStampedConstPtr &simpleGoal)
{
  if (action_server_->isActive()){
    action_server_->setPreempted();
  }

  geometry_msgs::PoseStamped current_pose;

  if (!getPose (current_pose)){
    ROS_ERROR("Could not retrieve robot pose in move_base_lite, aborting planning.");
    return;
  }

  std::vector<geometry_msgs::PoseStamped> plan;

  if (!grid_map_planner_->makePlan(current_pose.pose, simpleGoal->pose, plan))
  {
    ROS_ERROR("Planning to goal pose failed in move_base_lite, aborting planning.");
  }

  hector_move_base_msgs::MoveBaseActionPath path;

  //path.goal.target_path

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
}

void MoveBaseLiteRos::mapCallback(const nav_msgs::OccupancyGrid& msg)
{
  //grid_map::GridMapRosConverter::fromOccupancyGrid(msg, "occupancy", static_map_retrieved_);

  //static_map_fused_ = static_map_retrieved_;


}

void MoveBaseLiteRos::controllerResultCB(const hector_move_base_msgs::MoveBaseActionResultConstPtr &result)
{
  if (action_server_->isActive()){
      if (result->result.result != hector_move_base_msgs::MoveBaseResult::SUCCESS){
        action_server_->setAborted();
      }else{
        action_server_->setSucceeded();
      }
  }

  if (result->result.result != hector_move_base_msgs::MoveBaseResult::SUCCESS){
    ROS_INFO("Controller reports failure");
  }else{
    ROS_INFO("Controller reports success");
  }
}

bool MoveBaseLiteRos::getPose(geometry_msgs::PoseStamped& pose_out)
{

  pose_source_.header.frame_id = p_source_frame_name_;
  pose_source_.pose.orientation.w = 1.0;
  // Always get latest transform
  pose_source_.header.stamp = ros::Time(0);

  try {
      tfl_->transformPose(p_target_frame_name_, pose_source_, pose_out);
      return true;
      //ROS_INFO(" source pose (%s): %f, %f, %f ---> target pose (%s): %f, %f, %f",
      //         pose_source_.header.frame_id.c_str(), pose_source_.pose.position.x, pose_source_.pose.position.y, pose_source_.pose.position.z,
      //         pose_out.header.frame_id.c_str(),pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z );
      //pose_pub_.publish(pose_out);
  }
  catch (tf::TransformException ex){
       ROS_WARN_THROTTLE(5.0, "tf lookup failed when trying to retrieve robot pose in move_base_lite:  %s. This message is throttled.",ex.what());
       return false;
  }
}

}
