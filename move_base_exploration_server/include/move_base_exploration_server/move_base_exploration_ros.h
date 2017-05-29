//=================================================================================================
// Copyright (c) 2017, Gabriel Hüttenberger, TU Darmstadt
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

#ifndef MOVE_BASE_EXPLORATION_ROS__H_
#define MOVE_BASE_EXPLORATION_ROS__H_

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <grid_map_planner_lib/grid_map_planner.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <hector_move_base_msgs/MoveBaseAction.h>

#include <move_base_lite_msgs/FollowPathAction.h>
#include <move_base_lite_msgs/ExploreAction.h>

#include <actionlib/client/simple_action_client.h>

#include <nav_msgs/OccupancyGrid.h>

namespace move_base_exploration{

  typedef actionlib::SimpleActionServer<move_base_lite_msgs::ExploreAction> MoveBaseExplorationActionServer;

class MoveBaseExplorationRos
{

  enum{
    ACTION_ACTIVE,
    GOAL_ACTIVE,
    IDLE
  };

public:
  MoveBaseExplorationRos(ros::NodeHandle& nh_, ros::NodeHandle& pnh_);

protected:
  bool getPose(geometry_msgs::PoseStamped& pose_out);
//  void simpleGoalCallback(const geometry_msgs::PointStampedConstPtr goal);

  void moveBaseExplorationCB();
  void moveBaseCancelCB();

//  void followPathDoneCb(const actionlib::SimpleClientGoalState& state,
//              const move_base_lite_msgs::FollowPathResultConstPtr& result_in);
//  void followPathActiveCb();
//  void followPathFeedbackCb(const move_base_lite_msgs::FollowPathFeedbackConstPtr& feedback);


//  void simple_goalCB(const geometry_msgs::PoseStampedConstPtr &simpleGoal);
  //void cmd_velCB(const ros::MessageEvent<geometry_msgs::Twist> &event);
  //void controllerResultCB(const hector_move_base_msgs::MoveBaseActionResultConstPtr &result);

//  bool generatePlanToGoal(geometry_msgs::PoseStamped& goal_pose, move_base_lite_msgs::FollowPathGoal& goal);
//  void sendActionToController(const move_base_lite_msgs::FollowPathGoal& goal);

//  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);


//  ros::Subscriber simple_goal_sub_;
  ros::Subscriber map_sub_;

  ros::Publisher path_pub_;

  ros::Publisher drivepath_pub_;
  ros::Subscriber controller_result_sub_;


  boost::shared_ptr<MoveBaseExplorationActionServer> as_;

  boost::shared_ptr<tf::TransformListener> tfl_;
  boost::shared_ptr<grid_map_planner::GridMapPlanner> grid_map_planner_;

  boost::shared_ptr<actionlib::SimpleActionServer<move_base_lite_msgs::ExploreAction> > move_base_exploration_server_;
  actionlib::SimpleActionServer<move_base_lite_msgs::ExploreAction>::GoalConstPtr move_base_exploration_goal_;

  boost::shared_ptr<actionlib::SimpleActionClient<move_base_lite_msgs::FollowPathAction> > follow_path_client_;

  // Always set this on receiving  plan
  geometry_msgs::PoseStamped current_goal_;


  geometry_msgs::PoseStamped pose_source_;

  nav_msgs::OccupancyGridConstPtr latest_occ_grid_map_;

  std::string p_source_frame_name_;
  std::string p_target_frame_name_;
};

}

#endif