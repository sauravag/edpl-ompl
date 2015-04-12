/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Texas A&M University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Texas A&M University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Authors: Saurav Agarwal */

#ifndef ROS_SPACE_INFORMATION_
#define ROS_SPACE_INFORMATION_

#include "SpaceInformation.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "aruco_msgs/Marker.h"
#include "aruco_msgs/MarkerArray.h"
#include "geometry_msgs/Twist.h"

/**
The ROSSpaceInformation information class is a derivative of the firm::spaceinformation
that enables us to overwrite the get observation and apply control functions.
We basically use this space information to get actual sensor measurements and apply control using ROS.
We subscribe to the aruco marker publisher and advertise to the geometry twist.
*/

namespace firm
{
    class ROSSpaceInformation : public firm::SpaceInformation
    {

        public:
            /** \brief Constructer */
            ROSSpaceInformation(const ompl::base::StateSpacePtr &stateSpace, const ompl::control::ControlSpacePtr &controlSpace) :
            firm::SpaceInformation(stateSpace, controlSpace), nHandle_("~"), loopRate_(30)
            {
                // Subscribe to the Aruco Marker Publisher
                arucoSubscriber_  = nHandle_.subscribe("/aruco_marker_publisher/markers", 1, &ROSSpaceInformation::arucoListenerCallback, this);

                // Publish to vel topic to move robot
                controlPublisher_ = nHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
            }

            /** \brief Destructor */
            virtual ~ROSSpaceInformation(void)
            {

            }

            /** \brief This function gets called everytime the node hears a message on the aruco marker publisher topic */
            void arucoListenerCallback(const aruco_msgs::MarkerArray &msg);

            /** \brief Calculates the range and bearing to a marker */
            void calculateRangeBearingToMarker(double x, double y, double z, double &range, double &bearing);

            /** \brief Applies the control to the robot by publishing geometry twist messages */
            virtual void applyControl(const ompl::control::Control *control, bool withNoise = true);

            /** \brief Outputs the real sensor observation */
            virtual ObservationType getObservation() ;

        private:

            /** \brief A handle for the node */
            ros::NodeHandle nHandle_;

            /** \brief A container for the observations */
            ObservationModelMethod::ObservationType cameraObservation_;

            /** \brief A publisher for robot control commands */
            ros::Publisher  controlPublisher_;

            /** \brief A subscriber for the aruco topic */
            ros::Subscriber arucoSubscriber_;

            ros::Rate loopRate_;

    };
}
#endif
