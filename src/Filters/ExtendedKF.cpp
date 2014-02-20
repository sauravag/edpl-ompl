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

#include "../../include/Filters/ExtendedKF.h"
/*
ExtendedKF::ExtendedKF(MotionModelPointer motionModel,
ObservationModelPointer observationModel) :
KalmanFilterMethod(motionModel, observationModel) {

}
*/

void ExtendedKF::Predict(const ompl::base::State *belief,
  const ompl::control::Control* control,
  const LinearSystem& ls, ompl::base::State *predictedState)
{

  using namespace arma;

  this->motionModel_->Evolve(belief, control,this->motionModel_->getZeroNoise(), predictedState);

  mat covPred = ls.getA() * belief->as<StateType>()->getCovariance() * trans(ls.getA()) +
    ls.getG() * ls.getQ() * trans(ls.getG());

  predictedState->as<StateType>()->setCovariance(covPred);

}

void ExtendedKF::Update(const ompl::base::State *belief, const typename ObservationModelMethod::ObservationType& obs,
const LinearSystem& ls, ompl::base::State *updatedState)
{

  using namespace arma;

  colvec innov = this->observationModel_->computeInnovation(belief, obs);

 // cout<<"innovation calculated"<<endl;
  if(!innov.n_rows || !innov.n_cols)
  {
    si_->copyState(updatedState, belief);
    return; // return the prediction if you don't have any innovation
  }

  assert(innov.n_rows);
  assert(innov.n_cols);

  mat covPred = belief->as<StateType>()->getCovariance();

  mat rightMatrix = ls.getH() * covPred * trans(ls.getH()) + ls.getR();
  mat leftMatrix = covPred * trans(ls.getH());
  mat KalmanGain = solve(trans(rightMatrix), trans(leftMatrix));
  KalmanGain = KalmanGain.t();

  //cout<<"Kalman gain calculated"<<endl;

  colvec xPredVec = belief->as<StateType>()->getArmaData();

  colvec xEstVec = xPredVec + KalmanGain*innov;

  //cout<<"The innovation (in EKF update) :"<<endl<<innov<<endl;
  //cout<<"The Kalman Gain (in EKF update) :"<<endl<<KalmanGain<<endl;

  //cout<<"New State estimated"<<endl;

  updatedState->as<StateType>()->setXYYaw(xEstVec[0], xEstVec[1], xEstVec[2]);

  mat covEst = covPred - KalmanGain* ls.getH() * covPred;

  updatedState->as<StateType>()->setCovariance(covEst);

}

void ExtendedKF::Evolve(const ompl::base::State *belief,
    const ompl::control::Control* control,
    const ObservationType& obs,
    const LinearSystem& lsPred,
    const LinearSystem& lsUpdate,
    ompl::base::State *evolvedState)
{

  // In the EKF we do not use the linear systems passed to the filter, instead we generate the linear systems on the fly

  using namespace arma;

  LinearSystem lsPredicted(belief, control, this->motionModel_, this->observationModel_) ;

  ompl::base::State *bPred = si_->allocState();

  Predict(belief, control, lsPredicted, bPred);

  if(!obs.n_rows || !obs.n_cols)
  {
    si_->copyState(evolvedState, bPred);
    return;
  }

  LinearSystem lsUpdated(bPred, control, obs, this->motionModel_, this->observationModel_) ;

  Update(bPred, obs, lsUpdated, evolvedState);

}
