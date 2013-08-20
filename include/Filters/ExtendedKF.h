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

#ifndef EXTENDED_KF_
#define EXTENDED_KF_

#include "KalmanFilterMethod.h"


class ExtendedKF : public  KalmanFilterMethod 
{


	public:
	
  	typedef typename MotionModelMethod::ControlType   ControlType;
  	typedef typename MPTraits::CfgType CfgType;
    //typedef typename MotionModelMethod::CfgType           CfgType;
  	typedef MotionModelMethod::SpaceType SpaceType;
    typedef MotionModelMethod::StateType StateType;
  	typedef typename ObservationModelMethod::ObservationType ObservationType;
  	typedef typename MotionModelMethod::MotionModelPointer MotionModelPointer;
  	typedef typename ObservationModelMethod::ObservationModelPointer ObservationModelPointer;	
          
  	ExtendedKF() { }

  	ExtendedKF(MotionModelPointer motionModel,	ObservationModelPointer observationModel);
  	
  	//gets a belief and control, returns predicted belief if control
  	//were to be applied
  	ompl::base::State* Predict(const ompl::base::State *belief,
  	const ControlType& control,
  	const LinearSystem& ls, const bool isConstruction=false);

  	//gets a belief and observation, returns 
  	ompl::base::State* Update(const ompl::base::State *belief,
  	const ObservationType& obs,
    const	LinearSystem& ls, const bool isConstruction=false);

  	ompl::base::State* Evolve(const ompl::base::State *belief,
  	const ControlType& control,
  	const ObservationType& obs,
  	const LinearSystem& lsPred, 
  	const LinearSystem& lsUpdate,
  	const bool isConstruction=false);
  	
  	
  	arma::mat computeStationaryCovariance (const LinearSystem& ls){}

};


#endif

