/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Texas A&M University
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
*   * Neither the name of the Rice University nor the names of its
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

#include "../MotionModels/MotionModelMethod.h"
#include "../ObservationModels/ObservationModelMethod.h"
#include "../LinearSystem/LinearSystem.h"
#include "dare.h"
#include "../SpaceInformation/SpaceInformation.h"
//#include "HState.h"


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

  	KalmanFilterMethod() {}

  	KalmanFilterMethod (firm::SpaceInformation::SpaceInformationPtr si):
  	si_(si), observationModel_(si->getObservationModel()), motionModel_(si->getMotionModel()){}

  	//virtual ~KalmanFilterMethod() {}

  	//gets a belief and control, returns predicted belief if control
  	//were to be applied

  	MotionModelPointer getMotionModelPointer(){return motionModel_;}

  	void setMotionModelPointeer(const MotionModelPointer& mm) { motionModel_ = mm;}

  	ObservationModelPointer getObservationModelPointer(){return observationModel_;}

  	void setObservationModelPointer(const ObservationModelPointer& om) { observationModel_ = om;}

  	virtual void Predict(const ompl::base::State *belief,
                                const ompl::control::Control* control,
                                const LinearSystem& ls,
                                ompl::base::State *predictedState,
                                const bool isConstruction=false)  = 0;

  	//gets a belief and observation, returns
  	virtual
  	void Update(const ompl::base::State *belief,
                                const ObservationType& obs,
                                const	LinearSystem& ls,
                                ompl::base::State *updatedState,
                                const bool isConstruction=false) = 0;

    virtual
  	void Evolve(const ompl::base::State *belief,
                                const ompl::control::Control* control,
                                const ObservationType& obs,
                                const LinearSystem& lsPred,
                                const LinearSystem& lsUpdate,
                                ompl::base::State *evolvedState,
                                const bool isConstruction=false) = 0;

  	virtual
  	arma::mat computeStationaryCovariance(const LinearSystem& ls) = 0;


	protected:
        firm::SpaceInformation::SpaceInformationPtr si_;
        MotionModelPointer motionModel_;
        ObservationModelPointer observationModel_;

};


#endif

