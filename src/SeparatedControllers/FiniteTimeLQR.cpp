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
#include "SeparatedControllers/FiniteTimeLQR.h"

FiniteTimeLQR::FiniteTimeLQR(ompl::base::State *goal,
        const std::vector<ompl::base::State*> &nominalXs,
        const std::vector<ompl::control::Control*> &nominalUs,
        const std::vector<LinearSystem>& linearSystems,  // Linear systems are not used in this class but it is here to unify the interface
        const MotionModelPointer mm,
        const firm::SpaceInformation::SpaceInformationPtr si) :
        SeparatedControllerMethod(goal, nominalXs, nominalUs, linearSystems, mm, si)
{

    // set the weighting matrices
    Wxf_ = mm->getTerminalStateCost();

    Wx_ = mm->getStateCost();

    Wu_ = mm->getStateCost();

    // store nominal values
    nominalUs_ = nominalUs;

    nominalXs_ = nominalXs;

    linearSystems_ = linearSystems;

    numT_ = linearSystems_.size();

    arma::mat ZZ(Wu_.n_rows, Wx_.n_cols); ZZ.zeros(); // 3x3 zero matrix

    feedbackGains_.resize(numT_, ZZ);

    assert(numT_ > 0 && "Cannot pass in empty linear system vector to FinitetimeLQR");

    this->generateFeedbackGains();

    //============TEMP HACK==================
    std::vector<ompl::base::State*> txs; txs.push_back(goal);

    std::vector<ompl::control::Control*> tus; tus.push_back(mm->getZeroControl());

    std::vector<LinearSystem>  tls; tls.push_back(linearSystems[linearSystems.size()-1]);

    StationaryLQR stblr =  StationaryLQR(goal, txs, tus, tls, mm, si);

    stabilizer_  = stblr;
    //=================================

}

ompl::control::Control* FiniteTimeLQR::generateFeedbackControl(const ompl::base::State *state, const size_t& Ts)
{

    using namespace arma;

    ompl::base::State *relativeState = si_->allocState();

    ompl::control::Control* newcontrol;

    size_t TT = Ts;

    if(TT <= numT_ - 1)
    {

        dynamic_cast<SpaceType*>(si_.get())->getRelativeState(nominalXs_[TT], state, relativeState);

        colvec relativeCfg =  relativeState->as<StateType>()->getArmaData();

        // nominal control vec
        colvec nomU = motionModel_->OMPL2ARMA(nominalUs_[TT]);

        colvec dU = -1.0*feedbackGains_[TT]*relativeCfg;

        newcontrol  = motionModel_->ARMA2OMPL(nomU + dU);// control is nomU + dU
    }
    else
    {

        newcontrol  = stabilizer_.generateFeedbackControl(state);

    }

    // free the memory
    si_->freeState(relativeState);

    return newcontrol;
}

void FiniteTimeLQR::generateFeedbackGains()
{
  
    using namespace arma;

    mat S = Wxf_;

    int k = numT_ -1;

    // we solve this Riccati BACKWARDS
    while(k >= 0)
    {
        mat A = linearSystems_[k].getA();

        mat B = linearSystems_[k].getB();

        feedbackGains_[k] = arma::solve( B.t()*S*B + Wu_ , B.t()*S*A);

        S = Wx_ + A.t()*S*A - A.t()*S*B*feedbackGains_[k];

        k--;
    }  

}
