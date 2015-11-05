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
 *  robot_navigation_moving.cpp
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */

#include "robot_navigation/robot_navigation_moving.h"

namespace robot_navigation{

  RobotNavigationMoving::RobotNavigationMoving(boost::condition_variable& condition)
    : condition_(condition),
      class_loader_local_planner_("robot_navigation", "robot_navigation::BaseLocalPlanner"),
      state_(robot_navigation_state::moving::STOPPED)

  {
    std::string local_planner_plugin_name;
  int moving_frequency;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("local_planner", local_planner_plugin_name, std::string(""));
    private_nh.param("robot_base_frame", robot_frame_, std::string("base_link"));
    private_nh.param("global_frame", global_frame_, std::string("map"));
    private_nh.param("moving_frequency", moving_frequency, 100);
    private_nh.param("tf_timeout", tf_timeout_, 1.0);

  if(moving_frequency < 1){
    ROS_WARN("The moving frequency must be greater or equal one!");
    exit(0);
  }

    // try to load and init local planner
    ROS_INFO("Load local planner plugin.");
    try{
      local_planner_ = class_loader_local_planner_.createInstance(local_planner_plugin_name);
      local_planner_-> initialize(class_loader_local_planner_.getName(local_planner_plugin_name));
    }catch(const pluginlib::PluginlibException& ex){
      ROS_FATAL("Failed to create the %s local planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner_plugin_name.c_str(), ex.what());
      exit(1);
    }
    ROS_INFO("Local planner plugin loaded.");


    // init cmd_vel publisher for the robot velocity command
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // set the calling duration by the moving frequency
    calling_duration_ = boost::chrono::microseconds( (int) (1e6 / moving_frequency) );


  }
  
  RobotNavigationMoving::~RobotNavigationMoving(){
    
  }

  void RobotNavigationMoving::startMoving(){
    thread_ = boost::thread(&RobotNavigationMoving::run, this);
  }

  void RobotNavigationMoving::stopMoving(){
    thread_.interrupt();
    setState(robot_navigation_state::moving::STOPPED);
  }

  void RobotNavigationMoving::setState(robot_navigation_state::moving::Input state){
    boost::lock_guard<boost::mutex> guard(state_mtx_);
    state_ = state;
  }

  robot_navigation_state::moving::Input RobotNavigationMoving::getState(){
    boost::lock_guard<boost::mutex> guard(state_mtx_);
    return state_;
  }

  void RobotNavigationMoving::setNewPlan(const std::vector<geometry_msgs::PoseStamped>& plan){
    if(getState() == robot_navigation_state::moving::GOT_LOCAL_CMD ){
      ROS_WARN("Setting new path while moving!");
    }
    boost::lock_guard<boost::mutex> guard(plan_mtx_);
    new_plan_ = true;
    plan_ = plan;
  }

  bool RobotNavigationMoving::hasNewPlan(){
    boost::lock_guard<boost::mutex> guard(plan_mtx_);
    return new_plan_;
  }

  void RobotNavigationMoving::getNewPlan(std::vector<geometry_msgs::PoseStamped>& plan){
    boost::lock_guard<boost::mutex> guard(plan_mtx_);
    new_plan_ = false;
    plan = plan_;
  }

  void RobotNavigationMoving::run(){
    bool moving = true;

    // init plan
    std::vector<geometry_msgs::PoseStamped> plan;
    if(!hasNewPlan()){
      moving = false;
      ROS_ERROR("RobotNaviagtionMoving has no plan!");
    }

    try{
      while(moving){
        boost::chrono::thread_clock::time_point start_time = boost::chrono::thread_clock::now();

        // update plan
        if(hasNewPlan()){
          getNewPlan(plan);
          local_planner_->setPlan(plan);
        }

        // ask planner if the goal is reached
        if(local_planner_->isGoalReached()){
          setState(robot_navigation_state::moving::ARRIVED_GOAL);
          // goal reached, tell it the controller
          condition_.notify_one();
          moving = false;
        // if not, keep moving
        }else{
          geometry_msgs::Twist cmd_vel;
          geometry_msgs::PoseStamped robot_pose;
          robot_navigation_state::getRobotPose(tf_listener, robot_frame_, global_frame_, ros::Duration(tf_timeout_), robot_pose);
          if(local_planner_->computeVelocityCommands(robot_pose, cmd_vel)){
            setState(robot_navigation_state::moving::GOT_LOCAL_CMD);
            vel_pub_.publish(cmd_vel);

          }else{
            setState(robot_navigation_state::moving::NO_LOCAL_CMD);
            // if the planner can not go on, tell it the controller.
            condition_.notify_one();
            publishZeroVelocity(); // command the robot to stop
            moving = false;
          }
        }

        boost::chrono::thread_clock::time_point end_time = boost::chrono::thread_clock::now();
        boost::chrono::microseconds execution_duration =
          boost::chrono::duration_cast<boost::chrono::microseconds>(end_time - start_time);
        boost::chrono::microseconds sleep_time = calling_duration_ - execution_duration;
        if( sleep_time > boost::chrono::microseconds(0)){
          boost::this_thread::sleep_for(sleep_time);
        }else{
          ROS_WARN_THROTTLE(100, "Calculation needs to much time to stay in the moving frequency!");
        }
      }
    }catch(boost::thread_interrupted& ex){
      publishZeroVelocity();
      setState(robot_navigation_state::moving::STOPPED);
      condition_.notify_one();
    }
  }

  void RobotNavigationMoving::publishZeroVelocity(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }



} /* namespace robot_navigation */
