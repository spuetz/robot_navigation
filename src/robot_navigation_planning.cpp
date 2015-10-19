/*
 *  Software License Agreement (BSD License)
 *
 *  Robot Operating System code by the University of Osnabrück
 *  Copyright (c) 2015, University of Osnabrück
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSoITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *
 *  robot_navigation_planning.cpp
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */

#include "robot_navigation/robot_navigation_planning.h"

namespace robot_navigation{

  RobotNavigationPlanning::RobotNavigationPlanning(boost::condition_variable& condition)
    : condition_(condition),
      class_loader_global_planner_("robot_navigation", "robot_navigation::BaseGlobalPlanner"),
      state_(robot_navigation_state::planning::STOPPED)

  {
    std::string global_planner_plugin_name;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("global_planner", global_planner_plugin_name, std::string(""));
    private_nh.param("robot_base_frame", robot_frame_, std::string("base_link"));
    private_nh.param("global_frame", global_frame_, std::string("map"));

    // try to load and init global planner
    try{
      global_planner_ = class_loader_global_planner_.createInstance(global_planner_plugin_name);
      global_planner_-> initialize(class_loader_global_planner_.getName(global_planner_plugin_name));
    }catch(const pluginlib::PluginlibException& ex){
      ROS_FATAL("Failed to create the %s global planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner_plugin_name.c_str(), ex.what());
      exit(1);
    }

    // set the calling duration by the planning frequency
    //planning_timeout_ = boost::chrono::microseconds( (int) (1e6 / planning_freque) );

  }
  
  RobotNavigationPlanning::~RobotNavigationPlanning(){
    
  }

  void RobotNavigationPlanning::setState(robot_navigation_state::planning::Input state){
    boost::lock_guard<boost::mutex> guard(state_mtx_);
    state_ = state;
  }

  robot_navigation_state::planning::Input RobotNavigationPlanning::getState(){
    boost::lock_guard<boost::mutex> guard(state_mtx_);
    return state_;
  }

  void RobotNavigationPlanning::setNewPlan(const std::vector<geometry_msgs::PoseStamped>& plan, const double& cost){
    boost::lock_guard<boost::mutex> guard(plan_mtx_);
    plan_ = plan;
    cost_ = cost;
  }

  void RobotNavigationPlanning::getNewPlan(std::vector<geometry_msgs::PoseStamped>& plan, double& cost){
    if(getState() != robot_navigation_state::planning::FOUND_PLAN ){
      ROS_ERROR("No plan found!");
      return;
    }
    boost::lock_guard<boost::mutex> guard(plan_mtx_);
    plan = plan_;
    cost = cost_;
  }
  
  void RobotNavigationPlanning::startPlanning(
    const geometry_msgs::PoseStamped& start, 
    const geometry_msgs::PoseStamped& goal,
    const double& tolerance
  ){
    start_ = start;
    goal_ = goal;
    tolerance_ = tolerance;
    
    thread_ = boost::thread(&RobotNavigationPlanning::run, this);
    setState(robot_navigation_state::planning::PLANNING);
  }
  
  void RobotNavigationPlanning::stopPlanning(){
    // only useful if there are any interruption points in the global planner
    thread_.interrupt();
    setState(robot_navigation_state::planning::STOPPED);
  }

  void RobotNavigationPlanning::run(){
    
    std::vector<geometry_msgs::PoseStamped> plan;
    double cost;
    try{
      if(global_planner_->makePlan(start_, goal_, tolerance_, plan, cost)){
        setNewPlan(plan, cost);
        setState(robot_navigation_state::planning::FOUND_PLAN);
        condition_.notify_one();
      }else{
        setState(robot_navigation_state::planning::NO_PLAN_FOUND);
        condition_.notify_one();
      }
    }catch(boost::thread_interrupted& ex){
      setState(robot_navigation_state::planning::STOPPED);
      condition_.notify_one();
    }
  }


} /* namespace robot_navigation */
