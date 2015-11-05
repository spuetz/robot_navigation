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
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *
 *  robot_navigation_moving.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */

#ifndef ROBOT_NAVIGATION__ROBOT_NAVIAGTION_MOVING_H_
#define ROBOT_NAVIGATION__ROBOT_NAVIGATION_MOVING_H_


#include <pluginlib/class_loader.h>
#include <boost/chrono/thread_clock.hpp>
#include <boost/chrono/duration.hpp>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include "robot_navigation/base_local_planner.h"
#include "robot_navigation_state/robot_navigation_state.h"

namespace robot_navigation{
  
  class RobotNavigationMoving{
    public:
      
      RobotNavigationMoving(boost::condition_variable& condition);
      
      ~RobotNavigationMoving();
      
      void startMoving();
      
      void stopMoving();
      
      void setNewPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
      
      robot_navigation_state::moving::Input getState();
        
    protected:
    
    private:
      // main thread function -> moving the robot
      void run();

      // stops the robot
      void publishZeroVelocity();
      
      boost::mutex state_mtx_;
      void setState(robot_navigation_state::moving::Input state);
		

      boost::mutex plan_mtx_;
      bool new_plan_;
      bool hasNewPlan();
      void getNewPlan(std::vector<geometry_msgs::PoseStamped>& plan);

      // current global path
      std::vector<geometry_msgs::PoseStamped> plan_;
      
      // class loader, to load the local planner plugin
      pluginlib::ClassLoader<robot_navigation::BaseLocalPlanner> class_loader_local_planner_;
      
      // the local planer to calculate the velocity command
      BaseLocalPlanner::Ptr local_planner_;

      // condition to wake up controll thread
      boost::condition_variable& condition_;

      // thread for moving
      boost::thread thread_;

      // timing of the moving thread
      boost::chrono::microseconds calling_duration_;

      // frames to get the current global robot pose
      std::string robot_frame_;
      std::string global_frame_;

      // subscribers and publishers
      ros::Publisher vel_pub_;
      
      tf::TransformListener tf_listener;
      
      robot_navigation_state::moving::Input state_;
      
      double tf_timeout_;

};

} /* namespace robot_navigation */

#endif /* robot_navigation_moving.h */
