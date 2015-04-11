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

#include "../../include/Filters/LinearizedKF.h"

void LinearizedKF::Predict(const ompl::base::State *belief,
  const ompl::control::Control* control, const LinearSystem& ls, ompl::base::State *predictedState)
{
    using namespace arma;

    this->motionModel_->Evolve(belief, control,this->motionModel_->getZeroNoise(), predictedState);

    mat covPred = ls.getA() * belief->as<StateType>()->getCovariance() * trans(ls.getA()) +
    ls.getG() * ls.getQ() * trans(ls.getG());

    predictedState->as<StateType>()->setCovariance(covPred);

}


void LinearizedKF::Update(const ompl::base::State *belief, const typename ObservationModelMethod::ObservationType& obs,
const LinearSystem& ls, ompl::base::State *updatedState)
{

    using namespace arma;

    colvec innov = this->observationModel_->computeInnovation(belief, obs);

    if(!innov.n_rows || !innov.n_cols)
    {
        updatedState = si_->cloneState(belief);
        return; // return the prediction if you don't have any innovation
    }

    assert(innov.n_rows);
    assert(innov.n_cols);

    mat covPred = belief->as<StateType>()->getCovariance();

    mat rightMatrix = ls.getH() * covPred * trans(ls.getH()) + ls.getR();
    mat leftMatrix = covPred * trans(ls.getH());
    mat KalmanGain = solve(trans(rightMatrix), trans(leftMatrix));
    KalmanGain = KalmanGain.t();

    colvec xPredVec = belief->as<StateType>()->getArmaData();

    colvec xEstVec = xPredVec + KalmanGain*innov;

    updatedState->as<StateType>()->setXYYaw(xEstVec[0], xEstVec[1], xEstVec[2]);

    mat covEst = covPred - KalmanGain* ls.getH() * covPred;

    updatedState->as<StateType>()->setCovariance(covEst);

 }


void LinearizedKF::Evolve(const ompl::base::State *belief,
    const ompl::control::Control* control,
    const ObservationType& obs,
    const LinearSystem& lsPred,
    const LinearSystem& lsUpdate,
    ompl::base::State *evolvedState)
{

    using namespace arma;

    ompl::base::State *bPred = si_->allocState();

    Predict(belief, control, lsPred, bPred);

    if(!obs.n_rows || !obs.n_cols)
    {
        return;
    }

    Update(bPred, obs, lsUpdate, evolvedState);

    si_->freeState(bPred);
}



arma::mat LinearizedKF::computeStationaryCovariance (const LinearSystem& ls)
{
    using namespace arma;

    mat H = ls.getH();
    mat M = ls.getM();
    mat R = ls.getR();

    mat Pprd;
    bool dareSolvable = dare (trans(ls.getA()),trans(ls.getH()),ls.getG() * ls.getQ() * trans(ls.getG()),
    ls.getM() * ls.getR() * trans(ls.getM()), Pprd );

    if(!dareSolvable)
    {
        OMPL_ERROR("Dare Unsolvable: The given system state is not stable/observable. Try a different state.");
        exit(1);
    }

    //makes this symmetric
    Pprd = (Pprd + trans(Pprd)) / 2;

    mat Pest = Pprd - ( Pprd * H.t()) * inv( H*Pprd*H.t() + M * R * M.t(), true) * trans(Pprd * H.t()) ;

    //makes this symmetric
    Pest = (Pest + trans(Pest)) / 2;

    return Pest;

}
