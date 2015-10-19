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
 *  robot_navigation_planning.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */

#ifndef ROBOT_NAVIGATION__ROBOT_NAVIGATION_PLANNING_H_
#define ROBOT_NAVIGATION__ROBOT_NAVIGATION_PLANNING_H_


#include <pluginlib/class_loader.h>
#include <boost/chrono/thread_clock.hpp>
#include <boost/chrono/duration.hpp>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include "robot_navigation/base_global_planner.h"
#include "robot_navigation_state/robot_navigation_state.h"

namespace robot_navigation{
  
  class RobotNavigationPlanning{
    public:
      
      RobotNavigationPlanning(boost::condition_variable& condition);
      
      ~RobotNavigationPlanning();
      
      void getNewPlan(std::vector<geometry_msgs::PoseStamped>& plan, double& cost);
      
      robot_navigation_state::planning::Input getState();
      
      void startPlanning(
        const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal,
        const double& tolerance
      );
      
      void stopPlanning();
        
    protected:
    
    private:
      void run();
      
      boost::mutex state_mtx_;
      void setState(robot_navigation_state::planning::Input state);

      boost::mutex plan_mtx_;
      void setNewPlan(const std::vector<geometry_msgs::PoseStamped>& plan, const double& cost);

      // current global plan
      std::vector<geometry_msgs::PoseStamped> plan_;
      
      // current global plan cost
      double cost_;
      
      geometry_msgs::PoseStamped start_, goal_;
      double tolerance_;
      
      // class loader, to load the global planner plugin
      pluginlib::ClassLoader<robot_navigation::BaseGlobalPlanner> class_loader_global_planner_;
      
      // the global planer to plan a global path
      BaseGlobalPlanner::Ptr global_planner_;

      // condition to wake up controll thread
      boost::condition_variable& condition_;

      // thread for planning
      boost::thread thread_;

      // timing of the planning thread
      boost::chrono::microseconds planning_duration_;

      // frames to get the current global robot pose
      std::string robot_frame_;
      std::string global_frame_;

      tf::TransformListener tf_listener;
      
      robot_navigation_state::planning::Input state_;

};

} /* namespace robot_navigation */

#endif /* robot_navigation_planning.h */
