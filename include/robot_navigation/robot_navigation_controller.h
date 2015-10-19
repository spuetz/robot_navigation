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
 *  robot_navigation_controller.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */

#ifndef ROBOT_NAVIGATION__ROBOT_NAVIGATION_CONTROLLER_H_
#define ROBOT_NAVIGATION__ROBOT_NAVIGATION_CONTROLLER_H_


#include <boost/chrono/thread_clock.hpp>
#include <boost/chrono/duration.hpp>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

#include "robot_navigation_state/robot_navigation_state.h"
#include "robot_navigation/robot_navigation_planning.h"
#include "robot_navigation/robot_navigation_moving.h"

namespace robot_navigation{
  
  class RobotNavigationController{
    public:
      
      RobotNavigationController();
      
      ~RobotNavigationController();
      
      void getNewPlan(std::vector<geometry_msgs::PoseStamped>& plan, double& cost);
            
      void startController();
      
      void setNewGoal(const geometry_msgs::PoseStamped& goal);
      
      void stopController();
      
      bool isControllerRunning();
        
    protected:
    
    private:
    
      void run();
      
      void getNewGoal(geometry_msgs::PoseStamped& goal);
      bool hasNewGoal();
      bool new_goal_;
      geometry_msgs::PoseStamped goal_;
      boost::mutex goal_mtx_;

      RobotNavigationPlanning planning_;
      RobotNavigationMoving moving_;

      robot_navigation_state::planning::Input state_planning_input_;
      robot_navigation_state::moving::Input state_moving_input_;

      // current global plan
      std::vector<geometry_msgs::PoseStamped> plan_;
      
      // condition to wake up controll thread
      boost::condition_variable condition_;

      // thread for controller
      boost::thread thread_;

      // timing of the controller thread
      boost::chrono::microseconds controller_duration_;

      // frames to get the current global robot pose
      std::string robot_frame_;
      std::string global_frame_;

      tf::TransformListener tf_listener_;
      
      double goal_tolerance_;
      
      bool controller_is_running_;
};

} /* namespace robot_navigation */

#endif /* robot_navigation_controller.h */
