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

/* Authors: Saurav Agarwal, Ali-akbar Agha-mohammadi */

#ifndef EXTENDED_KF_
#define EXTENDED_KF_

#include "KalmanFilterMethod.h"

/** \brief  Implementation of the Extended Kalman Filter. */
class ExtendedKF : public  KalmanFilterMethod
{


	public:

        typedef typename MotionModelMethod::ControlType   ControlType;
        typedef MotionModelMethod::SpaceType SpaceType;
        typedef MotionModelMethod::StateType StateType;
        typedef typename ObservationModelMethod::ObservationType ObservationType;
        typedef typename MotionModelMethod::MotionModelPointer MotionModelPointer;
        typedef typename ObservationModelMethod::ObservationModelPointer ObservationModelPointer;

        /** \brief  Constructor */
        ExtendedKF() { }

        /** \brief  Constructor */
        ExtendedKF(const firm::SpaceInformation::SpaceInformationPtr si): KalmanFilterMethod(si) {}

        /** \brief  Gets as input belief and control, returns predicted belief if control
                    were to be applied to the robot. Also called the Prior. */
        void Predict(const ompl::base::State *belief,
                                    const ompl::control::Control* control,
                                    const LinearSystem& ls,
                                    ompl::base::State *predictedState) ;

        /** \brief  Gets as input belief and observation, returns the updated state of the robot. Also called the Posterior.*/
        void Update(const ompl::base::State *belief,
                    const ObservationType& obs,
                    const	LinearSystem& ls,
                    ompl::base::State *updatedState) ;

        /** \brief  Evolves the robot's belief on the input control, previous state and new observation. It first calls predict
                    and then update.*/
//         void Evolve(const ompl::base::State *belief,
        void Evolve(ompl::base::State *belief,
                    const ompl::control::Control* control,
                    const ObservationType& obs,
                    const LinearSystem& lsPred,
                    const LinearSystem& lsUpdate,
                    ompl::base::State *evolvedState) ;

        /** \brief  Compute the covariance for a given linear system. A linear system describes a robot's state at a point in
                    an open loop trajectory. Helps to understand the expected uncertainty at a point in the trajectory.*/
        arma::mat computeStationaryCovariance (const LinearSystem& ls) {return arma::zeros(3,3);}

    private:

        /** \brief This factor grows the covariance by a factor when there is no update i.e no information/innovation */
        static const int covGrowthFactor_;

};


#endif

