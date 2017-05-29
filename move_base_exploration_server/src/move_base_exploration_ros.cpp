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

#include <pluginlib/class_loader.h>

#include <hector_exploration_planner/hector_exploration_planner.h>

namespace move_base_exploration{


MoveBaseExplorationRos::MoveBaseExplorationRos(ros::NodeHandle& nh_, ros::NodeHandle& pnh_)
{
  p_source_frame_name_ = "base_link";
  p_target_frame_name_ = "world";

  pose_source_.header.frame_id = p_source_frame_name_;
  pose_source_.pose.orientation.w = 1.0;

  move_base_exploration_server_.reset(new actionlib::SimpleActionServer<move_base_lite_msgs::ExploreAction>(nh_, "/start_explore", false));

  tfl_ = boost::make_shared<tf::TransformListener>();
  grid_map_planner_ = boost::make_shared<grid_map_planner::GridMapPlanner>();

  pluginlib::ClassLoader<hector_exploration_planner::HectorExplorationPlanner> expl_loader_;
  boost::shared_ptr<hector_exploration_planner::HectorExplorationPlanner> exploration_planner_;

  std::string exploration_planner_name = "hector_exploration_base_global_planner_plugin/HectorExplorationBaseGlobalPlannerPlugin";

  exploration_planner_ = expl_loader_.createInstance(exploration_planner_name);
  exploration_planner_->initialize(expl_loader_.getName(exploration_planner_name), costmap_);

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

  std::vector<geometry_msgs::PoseStamped> plan;
  geometry_msgs::PoseStamped& current_pose;
  if (getPose(current_pose)){
    exploration_planner_->doExploration(current_pose, plan);
  }

  move_base_lite_msgs::FollowPathGoal follow_path_goal;
  follow_path_goal.follow_path_options.rotate_front_to_goal_pose_orientation = false;

  follow_path_goal.target_path = plan;
  sendActionToController(follow_path_goal);
  

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
