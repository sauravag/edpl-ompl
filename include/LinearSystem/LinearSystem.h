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
#ifndef LINEARSYSTEM_
#define LINEARSYSTEM_

/* Authors: Saurav Agarwal, Ali-akbar Agha-mohammadi */

#include "../MotionModels/MotionModelMethod.h"
#include "../ObservationModels/ObservationModelMethod.h"
#include <ompl/base/SpaceInformation.h>
#include "armadillo"
#include "../SpaceInformation/SpaceInformation.h"

/**
    @par Description of the Linear System Class
    A Linear System is a construct which is used to store information about the system at a given state in the
    open loop trajectory. It is used to compute and retrieve the Jacobians at the particular state.
    and
 */
/** \brief The linear system class. */
class LinearSystem
{

  public:
    typedef MotionModelMethod::SpaceType SpaceType;
    typedef MotionModelMethod::StateType StateType;
  	typedef MotionModelMethod::MotionModelPointer MotionModelPointer;
  	typedef ObservationModelMethod::ObservationModelPointer ObservationModelPointer;
  	typedef arma::mat ControlType;
    typedef arma::mat MotionNoiseType;
    typedef arma::mat ObservationType;
    typedef arma::mat ObsNoiseType;
    typedef arma::mat MotionJacobianType;
    typedef arma::mat ObsJacobianType;

    /** \brief  Constructor.*/
  	LinearSystem() {}

    /** \brief  Constructor.*/
    LinearSystem (const ompl::base::SpaceInformationPtr si, const ompl::base::State *state, const ompl::control::Control* control,
                  MotionModelPointer motionModel, ObservationModelPointer observationModel):
                  si_(si),u_(control), motionModel_(motionModel), observationModel_(observationModel)
    {

      using namespace arma;

      x_ = si_->cloneState(state);

      w_ = motionModel_->getZeroNoise();

      colvec junknoise = arma::zeros<colvec>(1); // this is just useless junk that we are creating just so that we don't need to change interface to observationmodel functions

      v_ = junknoise;

    }

    /** \brief  Constructor.*/
    LinearSystem (const ompl::base::SpaceInformationPtr si, const ompl::base::State *state, const ompl::control::Control* control, const ObservationType& obs,
      MotionModelPointer motionModel,
      ObservationModelPointer observationModel):
      si_(si),u_(control), motionModel_(motionModel),
      observationModel_(observationModel)
    {

      using namespace arma;

      x_ = si_->cloneState(state);

      ObservationType observation = obs;

      z_ = observation; // using the ids from outside

      w_ = motionModel_->getZeroNoise();

      colvec junknoise = arma::zeros<colvec>(1); // this is just junk that we are creating just so that we don't need to change interface to observationmodel functions

      v_ = junknoise;

    }

    /** \brief  Return the state at which this system was constructed. */
    ompl::base::State* getX() {return x_; }

    /** \brief  Get the state transition jacobian. */
    arma::mat getA() const { return motionModel_->getStateJacobian(x_, u_, w_); }

    /** \brief  Get the control jacobian for the state transition. */
    arma::mat getB() const { return motionModel_->getControlJacobian(x_, u_, w_); }

    /** \brief  Get the noise jacobian in the state transition. */
    arma::mat getG() const { return motionModel_->getNoiseJacobian(x_, u_, w_); }

    /** \brief  Get the process noise covariance for the state transition. */
    arma::mat getQ() const { return motionModel_->processNoiseCovariance(x_, u_); }

    /** \brief  Get the jacobian for the observation. */
    arma::mat getH() const { return observationModel_->getObservationJacobian(x_, v_, z_);}

    /** \brief  Get the observation noise jacobian. */
    arma::mat getM() const { return observationModel_->getNoiseJacobian(x_, v_, z_); }

    /** \brief  Get the observation noise covariance. */
    arma::mat getR() const { return observationModel_->getObservationNoiseCovariance(x_, z_); }

  private:

    /** \brief Pointer to space information. */
    //firm::SpaceInformation::SpaceInformationPtr si_;
    ompl::base::SpaceInformationPtr si_;

    /** \brief  The state at which the linear system is constructed.*/
    ompl::base::State *x_;

    /** \brief  The control applied at the internal state. */
    const ompl::control::Control* u_;

    /** \brief  The motion noise. */
    MotionNoiseType w_;

    /** \brief  The observation noise. */
    ObsNoiseType v_;

    /** \brief  Observation at the state. */
    ObservationType z_;

    /** \brief  Pointer to the motion model that models the state transition. */
    MotionModelPointer motionModel_;

    /** \brief  Pointer to the observation model that models the sensor measurement. */
    ObservationModelPointer observationModel_;
};

#endif
