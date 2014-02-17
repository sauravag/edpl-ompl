#ifndef LINEARSYSTEM_
#define LINEARSYSTEM_

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

#include "../MotionModels/MotionModelMethod.h"
#include "../ObservationModels/ObservationModelMethod.h"
#include <ompl/base/SpaceInformation.h>
#include "armadillo"

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

    /*
    LinearSystem (CfgType& _x, shared_ptr<MotionModelMethod> _motionModel,
                  shared_ptr<ObservationModelMethod> _observationModel,
                  ControlType& _u,
                  MotionNoiseType& _w, ObsNoiseType& _v) :
                  motionModel_(_motionModel),
                  observationModel_(_observationModel),
                  x_(_x), u_(_u), w_(_w), v_(_v) { }
    */

  	LinearSystem() {}

    LinearSystem (const ompl::base::State *state, const ompl::control::Control* control,
                  MotionModelPointer motionModel, ObservationModelPointer observationModel):
                  u_(control), motionModel_(motionModel), observationModel_(observationModel)
      {

      using namespace arma;

      assert(observationModel_);
      ompl::base::StateSpacePtr space(new SpaceType());

      //ompl::base::State *tempState
      ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
      x_ = si->cloneState(state);

       //= tempState;

      //TODO: zero noise??
      w_ = motionModel_->getZeroNoise();
      //v_ = observationModel_->GetZeroNoise();
      colvec junknoise = arma::zeros<colvec>(1); // this is just useless junk that we are creating just so that we don't need to change interface to observationmodel functions
      v_ = junknoise;
      //cout << "GetA: " << endl << GetA() << endl;
      //cout << "GetB: " << endl << GetB() << endl;
      //cout << "GetG: " << endl << GetG() << endl;
      //cout << "GetQ: " << endl << GetQ() << endl;

  		//cout << "GetR: " << endl << GetR() << endl;
      //
  		//cout << "GetH: " << GetH() << endl;
      //cout << "GetM: " << GetM() << endl;
      //cout << "GetR: " << GetR() << endl;
    }

    LinearSystem (const ompl::base::State *state, const ompl::control::Control* control, const ObservationType& obs,
      MotionModelPointer motionModel,
      ObservationModelPointer observationModel):
      u_(control), motionModel_(motionModel),
      observationModel_(observationModel)
      {

      using namespace arma;
      assert(observationModel_);

      ompl::base::StateSpacePtr space(new SpaceType());
      ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
      x_ = si->cloneState(state);

      ObservationType observation = obs;

      z_ = observation; // using the ids from outside

      //TODO: zero noise??
      w_ = motionModel_->getZeroNoise();
      //v_ = observationModel_->GetZeroNoise();
      colvec junknoise = arma::zeros<colvec>(1); // this is just useless junk that we are creating just so that we don't need to change interface to observationmodel functions
      v_ = junknoise;
      //cout << "GetA: " << endl << GetA() << endl;
      //cout << "GetB: " << endl << GetB() << endl;
      //cout << "GetG: " << endl << GetG() << endl;
      //cout << "GetQ: " << endl << GetQ() << endl;

  		//cout << "GetR: " << endl << GetR() << endl;
      //
  		//cout << "GetH: " << GetH() << endl;
      //cout << "GetM: " << GetM() << endl;
      //cout << "GetR: " << GetR() << endl;
    }

    ompl::base::State* getX() {return x_; }

    arma::mat getA() const { return motionModel_->getStateJacobian(x_, u_, w_); }
    arma::mat getB() const { return motionModel_->getControlJacobian(x_, u_, w_); }
    arma::mat getG() const { return motionModel_->getNoiseJacobian(x_, u_, w_); }
    arma::mat getQ() const { return motionModel_->processNoiseCovariance(x_, u_); }

    arma::mat getH() const { return observationModel_->getObservationJacobian(x_, v_, z_);}

    arma::mat getM() const { return observationModel_->getNoiseJacobian(x_, v_, z_); }

    arma::mat getR() const { return observationModel_->getObservationNoiseCovariance(x_, z_); }

  private:

    ompl::base::State *x_;
    const ompl::control::Control* u_;
    MotionNoiseType w_;
    ObsNoiseType v_;
    ObservationType z_;

    MotionModelPointer motionModel_;
    ObservationModelPointer observationModel_;
};

#endif
