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

#include "../../include/SpaceInformation/ROSSpaceInformation.h"
#include "../../include/ObservationModels/CamAruco2DObservationModel.h"

void firm::ROSSpaceInformation::arucoListenerCallback(const aruco_msgs::MarkerArray &msg)
{
    int numMarkers = msg.markers.size();

    ObservationModelMethod::ObservationType z;

    // From the aruco observation model, get the dimension of a single observation
    int singleObservationDim = CamAruco2DObservationModel::singleObservationDim;

    for(int i=0; i < numMarkers; i++)
    {
        z.resize((i+1)*singleObservationDim ,  1);

        aruco_msgs::Marker marker_i = msg.markers.at(i);

        double range, bearing;

        calculateRangeBearingToMarker(marker_i.pose.pose.position.x, marker_i.pose.pose.position.y, marker_i.pose.pose.position.z, range, bearing);

        z[singleObservationDim*i] = marker_i.id ; // id of the landmark
        z[singleObservationDim*i + 1] = range; // distance to landmark
        z[singleObservationDim*i + 2] = bearing; // bearing
        z[singleObservationDim*i + 3] = 0; // in simulation we used this to add the relative angle of observation, not needed now

    }

    cameraObservation_ = z;

}

void firm::ROSSpaceInformation::calculateRangeBearingToMarker(double x, double y, double z, double &range, double &bearing)
{
    range = sqrt(x*x+y*y+z*z); // distance to landmark

    double markerX_wrt_robot = z; // The distance to marker along x axis of robot
    double markerY_wrt_robot = x; // the distamce to marker along y axis of robot

    bearing = -atan2(markerY_wrt_robot, markerX_wrt_robot) ;
}

void firm::ROSSpaceInformation::applyControl(const ompl::control::Control *control, bool withNoise)
{

    const double *conVals = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

    // Create a geometry twist message
    geometry_msgs::Twist cmd_vel;

    // put the input control value into the message
    cmd_vel.linear.x =  conVals[0];
    cmd_vel.linear.y =  0.0;
    cmd_vel.linear.z =  0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = conVals[1];

    controlPublisher_.publish(cmd_vel);

    ros::spinOnce();

}

ObservationModelMethod::ObservationType firm::ROSSpaceInformation::getObservation()
{
    return cameraObservation_;
}

