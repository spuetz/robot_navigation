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
 *  robot_navigation.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */

#ifndef ROBOT_NAVIGATION__ROBOT_NAVIGATION_H_
#define ROBOT_NAVIGATION__ROBOT_NAVIGATION_H_

#include <vector>
#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>

#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include "robot_navigation/RobotNavigationConfig.h"

#include "robot_navigation/base_local_planner.h"
#include "robot_navigation/base_global_planner.h"
#include "robot_navigation/robot_navigation_controller.h"

#include <robot_navigation_state/robot_navigation_state.h>

namespace robot_navigation{


class RobotNavigation{
  public:
    /**
     * @brief Default-Constructor for the RobotNavigation
     */
    //RobotNavigation();
    
    /**
     * @brief Constructor for the RobotNavigation
     */
    RobotNavigation();

    /**
     * @brief Destructor - Cleans up
     */
    ~RobotNavigation();
  protected:

  private:

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& pose);

    void reconfigureCallback(robot_navigation::RobotNavigationConfig& config, uint32_t level);

    ros::Publisher current_goal_pub_;
    ros::Subscriber goal_sub_;

    // Dynamic Reconfigure
    dynamic_reconfigure::Server<robot_navigation::RobotNavigationConfig>*
      dynamic_reconfigure_server_;
      
    boost::shared_ptr<RobotNavigationController> controller_ptr_;
    boost::shared_ptr<tf::TransformListener> tf_listener_ptr_;
};
}

#endif /* robot_navigation.h */
