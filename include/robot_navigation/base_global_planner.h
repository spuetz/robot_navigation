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
 *  base_global_planner.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */

#ifndef ROBOT_NAVIGATION__BASE_GLOBAL_PLANNER_H_
#define ROBOT_NAVIGATION__BASE_GLOBAL_PLANNER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/shared_ptr.hpp>
#include <tf/transform_listener.h>

namespace robot_navigation
{

  class BaseGlobalPlanner{

    public:
      typedef boost::shared_ptr< ::robot_navigation::BaseGlobalPlanner > Ptr;

      /**
       * @brief  Destructor
       */
      virtual ~BaseGlobalPlanner(){};

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose
       * @param tolerance The tolerance to the goal pose
       * @param plan The plan... filled by the planner
       * @param cost The cost for the the plan
       * @return True if a valid plan was found, false otherwise
       */
      virtual bool makePlan(
        const geometry_msgs::PoseStamped& start, 
        const geometry_msgs::PoseStamped& goal,
        const double& tolerance, 
        std::vector<geometry_msgs::PoseStamped>& plan,
        double& cost) = 0;


      /**
       * @brief  Initialization function for the BaseGlobalPlanner
       * @param  name The name of this planner
       * @param  global_frame The global frame name
       */
      virtual void initialize(const std::string& name, const boost::shared_ptr<tf::TransformListener>& tf_listener_ptr, const std::string& global_frame) = 0;

    protected:
      /**
       * @brief  Constructor
       */
      BaseGlobalPlanner(){};
  };
} /* namespace robot_navigation */

#endif /* base_global_planner.h */
