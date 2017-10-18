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

#ifndef KALMAN_FILTER_METHOD_
#define KALMAN_FILTER_METHOD_

#include "MotionModels/MotionModelMethod.h"
#include "ObservationModels/ObservationModelMethod.h"
#include "LinearSystem/LinearSystem.h"
#include "dare.h"
#include "SpaceInformation/SpaceInformation.h"


/** \brief  The base class for Kalman filter implementations.*/
class KalmanFilterMethod
{
  typedef arma::colvec ObservationType;
  typedef arma::colvec ControlType;
  typedef arma::colvec NoiseType;
  typedef MotionModelMethod::SpaceType SpaceType;
  typedef MotionModelMethod::StateType StateType;
  typedef ObservationModelMethod::ObservationModelPointer ObservationModelPointer;
  typedef MotionModelMethod::MotionModelPointer MotionModelPointer;

	public:

    /** \brief  Constructor.*/
  	KalmanFilterMethod() {}

    /** \brief  Constructor. */
  	KalmanFilterMethod (const firm::SpaceInformation::SpaceInformationPtr si):
  	si_(si), observationModel_(si->getObservationModel()), motionModel_(si->getMotionModel()){}

  	//virtual ~KalmanFilterMethod() {}

    /** \brief  Return the pointer to the motion model used by the filter.*/
  	MotionModelPointer getMotionModelPointer(){return motionModel_;}

    /** \brief  Set the Motion Model pointer.*/
  	void setMotionModelPointeer(const MotionModelPointer& mm) { motionModel_ = mm;}

    /** \brief  Return the observation model poointer. */
  	ObservationModelPointer getObservationModelPointer(){return observationModel_;}

    /** \brief  Set the observation model pointer. */
  	void setObservationModelPointer(const ObservationModelPointer& om) { observationModel_ = om;}

    /** \brief  Gets as input belief and control, returns predicted belief if control
            were to be applied to the robot. Also called the Prior. */
  	virtual void Predict(const ompl::base::State *belief,
                                const ompl::control::Control* control,
                                const LinearSystem& ls,
                                ompl::base::State *predictedState)  = 0;

    /** \brief  Gets as input belief and observation, returns the updated state of the robot. Also called the Posterior.*/
  	virtual void Update(const ompl::base::State *belief,
                                const ObservationType& obs,
                                const	LinearSystem& ls,
                                ompl::base::State *updatedState) = 0;

    /** \brief  Evolves the robot's belief on the input control, previous state and new observation. It first calls predict
            and then update.*/
    virtual void Evolve(ompl::base::State *belief,
                                const ompl::control::Control* control,
                                const ObservationType& obs,
                                const LinearSystem& lsPred,
                                const LinearSystem& lsUpdate,
                                ompl::base::State *evolvedState) = 0;

    /** \brief  Compute the covariance for a given linear system. A linear system describes a robot's state at a point in
            an open loop trajectory. Helps to understand the expected uncertainty at a point in the trajectory.*/
  	virtual arma::mat computeStationaryCovariance(const LinearSystem& ls) = 0;


	protected:

        /** \brief Pointer to the space information.*/
        firm::SpaceInformation::SpaceInformationPtr si_;

        /** \brief Pointer to the observation model.*/
        ObservationModelPointer observationModel_;

        /** \brief Pointer to the motion model. */
        MotionModelPointer motionModel_;

};


#endif

