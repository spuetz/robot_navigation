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

namespace robot_navigation{

  RobotNavigationController::RobotNavigationController()
    :  planning_(condition_), moving_(condition_), controller_is_running_(false)
  {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("robot_base_frame", robot_frame_, std::string("base_link"));
    private_nh.param("global_frame", global_frame_, std::string("map"));
    private_nh.param("goal_tolerance", goal_tolerance_, 0.2);

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
    }
    condition_.notify_one();
  }
  
  void RobotNavigationController::getNewGoal(geometry_msgs::PoseStamped& goal){
    boost::lock_guard<boost::mutex> guard(goal_mtx_);
    new_goal_ = false;
    goal_ = goal;
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
    controller_is_running_ = false;
  }

  void RobotNavigationController::run(){
    try{
      
      geometry_msgs::PoseStamped start, goal;
      std::vector<geometry_msgs::PoseStamped> plan;
      double costs;
      
      bool run_planning = true;
      bool run_moving = true;
      bool has_new_plan = false;

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
            
            
            // planning is stopped
            case robot_navigation_state::planning::STOPPED:
              if(hasNewGoal()){
                // get the robot pose as start pose
                robot_navigation_state::getRobotPose(tf_listener_, robot_frame_, global_frame_, start);
                // get the goal pose
                getNewGoal(goal);
                planning_.startPlanning(start, goal, goal_tolerance_);
              }else{
                run_planning = false;
              }
              break;
              
              
            // in progress
            case robot_navigation_state::planning::PLANNING:
              ROS_INFO("planning...");
              break;
              
              
            // found a new plan
            case robot_navigation_state::planning::FOUND_PLAN:
              // start moving 
              planning_.getNewPlan(plan, costs);
              planning_.stopPlanning();
              has_new_plan = true;
              run_moving = true;
              break;
            // no plan found
            case robot_navigation_state::planning::NO_PLAN_FOUND:
              ROS_WARN("No plan found for the given goal!");
              
            break;
          }
        }
        if(run_moving){
          if(has_new_plan){
            moving_.setNewPlan(plan);
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
        if(!run_planning && !run_moving){
            ROS_INFO("Nothing to do stopping the robot navigation controller!");
            stopController();
        }
        
        // for the next cycle there is no new plan
        has_new_plan = false;
        
      }
      
      // try to sleep a bit
      // normally the thread should be woken up from the planning oder
      // the moving unit in the one second the controller is sleeping 
      boost::mutex mutex;
      boost::unique_lock<boost::mutex> lock(mutex);
      condition_.wait_for(lock, boost::chrono::seconds(1));
      
    }catch(boost::thread_interrupted& ex){
      planning_.stopPlanning();
      moving_.stopMoving();
    }
  }

} /* namespace robot_navigation */
