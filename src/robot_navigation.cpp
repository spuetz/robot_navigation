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
 *  robot_navigation.cpp
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */


#include "robot_navigation/robot_navigation.h"
#include <tf/transform_listener.h>

namespace robot_navigation{

RobotNavigation::RobotNavigation()
{
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
 
  // init subscribers and publishers
  current_goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0);
  goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
    "goal",
    1,
    &robot_navigation::RobotNavigation::goalCallback,
    this
  );

  tf_listener_ptr_ = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener(nh, ros::Duration(30), true));
  controller_ptr_ = boost::shared_ptr<RobotNavigationController>(new RobotNavigationController(tf_listener_ptr_));

  // set up dynamic_reconfigure server
  dynamic_reconfigure_server_ = 
    new dynamic_reconfigure::Server<robot_navigation::RobotNavigationConfig>(
      ros::NodeHandle("~")
    );
  dynamic_reconfigure::Server<robot_navigation::RobotNavigationConfig>::CallbackType cb_type = boost::bind(&robot_navigation::RobotNavigation::reconfigureCallback, this, _1, _2);
  dynamic_reconfigure_server_->setCallback(cb_type);

}

void RobotNavigation::reconfigureCallback(robot_navigation::RobotNavigationConfig& config, uint32_t level){
  // TODO
}

void RobotNavigation::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal){
  controller_ptr_->setNewGoal(*goal);
  if(!controller_ptr_->isControllerRunning()){
	controller_ptr_->startController();
  }
}

RobotNavigation::~RobotNavigation(){
  // clean up pointer created instances;
  delete dynamic_reconfigure_server_;
  
}


} /* end namespace robot_navigation */
