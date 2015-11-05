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
 *  robot_navigation_controller.cpp
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */

#include "robot_navigation/robot_navigation_controller.h"
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>

namespace robot_navigation{

  RobotNavigationController::RobotNavigationController(const boost::shared_ptr<tf::TransformListener>& tf_listener_ptr)
    :  tf_listener_ptr_(tf_listener_ptr), planning_(condition_, tf_listener_ptr), moving_(condition_), controller_is_running_(false)
  {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("robot_frame", robot_frame_, std::string("base_link"));
    private_nh.param("global_frame", global_frame_, std::string("map"));
    private_nh.param("goal_tolerance", goal_tolerance_, 0.2);
    private_nh.param("tf_timeout", tf_timeout_, 1.0);

    marker_pub_ = nh.advertise<visualization_msgs::Marker>("global_path_marker", 1);
    path_pub_ = nh.advertise<nav_msgs::Path>("global_path", 1);
    pose_array_pub_ = nh.advertise<geometry_msgs::PoseArray>("global_path_poses", 1);
  }

  RobotNavigationController::~RobotNavigationController(){
    stopController();
  }

  bool RobotNavigationController::isControllerRunning(){
    return controller_is_running_;
  }

  void RobotNavigationController::setNewGoal(const geometry_msgs::PoseStamped& goal){
    {
      boost::lock_guard<boost::mutex> guard(goal_mtx_);
      new_goal_ = true;
      goal_ = goal;
    }
    condition_.notify_one();
  }
  
  void RobotNavigationController::getNewGoal(geometry_msgs::PoseStamped& goal){
    boost::lock_guard<boost::mutex> guard(goal_mtx_);
    new_goal_ = false;
    goal = goal_;
  }
  
  bool RobotNavigationController::hasNewGoal(){
    boost::lock_guard<boost::mutex> guard(goal_mtx_);
    return new_goal_;
  }

  void RobotNavigationController::startController(){
    controller_is_running_ = true;
    thread_ = boost::thread(&RobotNavigationController::run, this);
  }
  
  void RobotNavigationController::stopController(){
    moving_.stopMoving();
    planning_.stopPlanning();
    
    // only useful if there are any 
    // interruption points in the controller
    thread_.interrupt();
    thread_.join();
  }
  
  void RobotNavigationController::publishPath(std::vector<geometry_msgs::PoseStamped>& plan){
        
        if(plan.empty()){
          return;
        }
        
        geometry_msgs::PoseArray pose_array;
        pose_array.header.stamp = plan.front().header.stamp;
        pose_array.header.frame_id = plan.front().header.frame_id;
        
        std::vector<geometry_msgs::PoseStamped>::iterator iter;
        for(iter = plan.begin(); iter != plan.end(); ++iter){
            pose_array.poses.push_back(iter->pose);
            visualization_msgs::Marker marker;
            marker.header.frame_id = plan.front().header.frame_id;
            marker.header.stamp = plan.front().header.stamp;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = iter->pose;
            marker.scale.x = 1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker_pub_.publish(marker);
            ROS_INFO("Go to point: (%f %f %f)", 
                iter->pose.position.x,
                iter->pose.position.y,
                iter->pose.position.z);
        }
        
        nav_msgs::Path path;
        path.poses = plan;
        path.header.frame_id = plan.front().header.frame_id;
        path.header.stamp = plan.front().header.stamp;
        path_pub_.publish(path);
        pose_array_pub_.publish(pose_array);
        
        ROS_INFO("Published path.");
    }

  void RobotNavigationController::run(){
    try{
      
      geometry_msgs::PoseStamped start, goal;
      std::vector<geometry_msgs::PoseStamped> plan, global_plan;
      double costs;
      
      bool run_planning = true;
      bool run_moving = false;
      bool has_new_plan = false;
      bool tf_succress = false;

      while( run_planning || run_moving ){
        
        state_planning_input_ = planning_.getState();
        state_moving_input_ = moving_.getState();
        
        // new goal received stop the planner and the robot
        if(hasNewGoal()){
          moving_.stopMoving();
          planning_.stopPlanning();
          run_planning = true;
        }
        
        if(run_planning){
          // global planning state
          switch(planning_.getState()){
            
            
            // planning stopped
            case robot_navigation_state::planning::STOPPED:
              ROS_INFO("robot navigation state: stoppped");
              if(hasNewGoal()){
                ROS_INFO("Has new goal");
                // get the robot pose as start pose
                tf_succress = robot_navigation_state::getRobotPose(
                  *tf_listener_ptr_,
                  robot_frame_,
                  global_frame_,
                  ros::Duration(tf_timeout_),
                  start
                );
                if(!tf_succress){
                    ROS_INFO("Could not get the robot pose! Abort!");
                    run_planning = false;
                    break;
                }
                
                ROS_INFO("Transform the goal to the global frame.");
                getNewGoal(goal);
                tf_succress = robot_navigation_state::transformPose(
                  *tf_listener_ptr_,
                  global_frame_,
                  goal.header.stamp,
                  ros::Duration(tf_timeout_),
                  goal,
                  global_frame_,
                  goal
                );

                if(!tf_succress){
                  ROS_ERROR("Can not transform the goal into the global frame!");
                  run_planning = false;
                  break;
                }
                
                // start planning
                planning_.startPlanning(start, goal, goal_tolerance_);
              }
              else{
                run_planning = false;
              }
              break;
              
              
            // in progress
            case robot_navigation_state::planning::PLANNING:
              ROS_INFO("robot navigation state: planning");
              break;
              
              
            // found a new plan
            case robot_navigation_state::planning::FOUND_PLAN:
              ROS_INFO("robot navigation state: found plan");

              // start moving 
              planning_.getNewPlan(plan, costs);
              planning_.stopPlanning();

              tf_succress = transformPlanToGlobalFrame(plan, global_plan);
              if(!tf_succress){
                ROS_ERROR("Can not transform the plan into the global frame!");
                break;
              }
              publishPath(global_plan);
              has_new_plan = true;
              run_moving = true;
              break;
              
              
            // no plan found
            case robot_navigation_state::planning::NO_PLAN_FOUND:
              ROS_INFO("robot navigation state: no plan found");
              run_planning = false;
              
            break;
          }
        }
        if(run_moving){
          if(has_new_plan){
            moving_.setNewPlan(global_plan);
          }
          
          switch(moving_.getState()){
            // stopped
            case robot_navigation_state::moving::STOPPED:
              if(has_new_plan){
                moving_.startMoving();
              }else{
                run_moving = false;
              }
              break;
            
            
            // no velocity command received
            case robot_navigation_state::moving::NO_LOCAL_CMD:
              ROS_INFO("No velocity command -> no local plan available");
              moving_.stopMoving();
              break;
            
            
            // got a velocity command
            case robot_navigation_state::moving::GOT_LOCAL_CMD:
              ROS_INFO_THROTTLE(10, "GOT velocity command -> moving...");
              break;
              
            
            // arrived the goal
            case robot_navigation_state::moving::ARRIVED_GOAL:
              ROS_INFO("Goal arrived!");
              moving_.stopMoving();
              break;
          }
        }
        
                // try to sleep a bit
                // normally the thread should be woken up from the planning oder
                // the moving unit in the one second the controller is sleeping 
                boost::mutex mutex;
                boost::unique_lock<boost::mutex> lock(mutex);
                condition_.wait_for(lock, boost::chrono::seconds(1));
        
        // for the next cycle there is no new plan
        has_new_plan = false;
        
                
      }
      ROS_INFO("Nothing to do, robot navigation controller is stopping!");
    
    }catch(boost::thread_interrupted& ex){
      planning_.stopPlanning();
      moving_.stopMoving();
    }
    controller_is_running_ = false;
    ROS_INFO("Controller has been shut down!");
  }
  
  bool RobotNavigationController::transformPlanToGlobalFrame(std::vector<geometry_msgs::PoseStamped>& plan, std::vector<geometry_msgs::PoseStamped>& global_plan){
    std::vector<geometry_msgs::PoseStamped>::iterator iter;
    bool tf_success = false;
    for(iter = plan.begin(); iter!=plan.end(); ++iter){
      geometry_msgs::PoseStamped global_pose;
      tf_success = robot_navigation_state::transformPose(*tf_listener_ptr_, global_frame_, iter->header.stamp, ros::Duration(tf_timeout_), *iter, global_frame_, global_pose);
      if(!tf_success){
        ROS_ERROR("Can not transform pose from %s frame into %s frame !", iter->header.frame_id.c_str(), global_frame_.c_str());
        return false;
      }
      global_plan.push_back(global_pose);
    }
    return true;
  }

} /* namespace robot_navigation */
