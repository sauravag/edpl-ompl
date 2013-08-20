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

#include "../include/Filters/ExtendedKF.h"

ExtendedKF::ExtendedKF(MotionModelPointer motionModel,
ObservationModelPointer observationModel) :
KalmanFilterMethod(motionModel, observationModel) {

}


ompl::base::State*
ExtendedKF::Predict(const ompl::base::State *belief,
  const ControlType& control, const LinearSystem& ls, const bool isConstruction)
{

  using namespace arma;
  SpaceType *space;
  space =  new SpaceType();
  ompl::base::State *predState = space->allocState();
  predState = this->motionModel_->Evolve(belief, control,this->motionModel_->getZeroNoise());

  mat covPred = ls.getA() * belief->as<StateType>()->getCovariance() * trans(ls.getA()) +
    ls.getG() * ls.getQ() * trans(ls.getG());
  /*
  if(!isConstruction)
    {
    std::ofstream myfile;
    myfile.open("DataOut/PredictStepOutput.txt", std::fstream::app);
    assert(myfile.is_open());
    myfile <<"<--ThisStepOutput---->\n";
    myfile <<"<-----Control-------->\n";
    myfile <<_control<<endl;
    myfile <<"<----Predicted Mean------>\n";
    myfile <<predState->as<StateType>()->getArmaData()<<endl;
    myfile <<"<----Predicted Covariance---->\n";
    myfile <<covPred<<endl;
    //cout<<"Writing"<<endl;
    myfile.close();
  }
  */
  predState->as<StateType>()->setCovariance(covPred);

  return predState;

}

ompl::base::State*
ExtendedKF::Update(const ompl::base::State *belief, const typename ObservationModelMethod::ObservationType& obs,
const LinearSystem& ls, const bool isConstruction) {

  using namespace arma;

  colvec innov = this->observationModel_->computeInnovation(belief, obs);

  ompl::base::StateSpacePtr space(new SpaceType());
  ompl::base::State  *estimatedState = space->allocState();
 // cout<<"innovation calculated"<<endl;
  if(!innov.n_rows || !innov.n_cols)
  {
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
    estimatedState = si->cloneState(belief);
    return estimatedState; // return the prediction if you don't have any innovation
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

  estimatedState->as<StateType>()->setXYYaw(xEstVec[0], xEstVec[1], xEstVec[2]);

  mat covEst = covPred - KalmanGain* ls.getH() * covPred;

  estimatedState->as<StateType>()->setCovariance(covEst);

  /*
  //------Printing Data to text file-----
  if(!isConstruction)
  {
    ofstream myfile;
    myfile.open("DataOut/UpdateStepOutput.txt", std::fstream::app);
    assert(myfile.is_open());

    myfile <<"<--ThisStepOutput---->\n";
    myfile <<"<---Observation------>\n";
    myfile <<_obs<<endl;
    myfile <<"<----Innovation------>\n";
    myfile <<innov<<endl;
    myfile <<"<-----Estimated Mean----->\n";
    myfile << xEst <<endl;
    myfile <<"<--- Estimated Covariance---->\n";
    myfile <<covEst<<endl;
    //cout<<"Writing"<<endl;
    myfile.close();
  }
  */

  return estimatedState;

}

ompl::base::State*
ExtendedKF::Evolve(const ompl::base::State *belief,
    const ControlType& control,
    const ObservationType& obs,
    const LinearSystem& lsPred,
    const LinearSystem& lsUpdate,
    const bool isConstruction)
{

  // In the EKF we do not use the linear systems passed to the filter, instead we generate the linear systems on the fly

  using namespace arma;

  LinearSystem lsPredicted(belief, control, this->motionModel_, this->observationModel_) ;

  ompl::base::State *bPred = Predict(belief, control, lsPredicted, isConstruction);


  if(!obs.n_rows || !obs.n_cols) {
    return bPred;
  }

  LinearSystem lsUpdated(bPred, control, obs, this->motionModel_, this->observationModel_) ;

  ompl::base::State *bEst = Update(bPred, obs, lsUpdated, isConstruction);

  //cout<<"Returning updated estimate form EKF"<<endl;

  return bEst;

}
