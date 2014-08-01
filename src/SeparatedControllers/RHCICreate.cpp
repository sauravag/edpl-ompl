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
#include "../../include/SeparatedControllers/RHCICreate.h"

int RHCICreate::controlQueueSize_ = -1;

double RHCICreate::turnOnlyDistance_ = -1;

ompl::control::Control*
RHCICreate::generateFeedbackControl(const ompl::base::State *state, const size_t& _t)
{

  using namespace arma;

  //if no more controls left, regenerate controls
  if(openLoopControls_.size() == 0) {

    std::vector<ompl::control::Control*> openLoopControls;

    this->motionModel_->generateOpenLoopControls(state , this->goal_, openLoopControls) ;

    openLoopControls_ = std::deque<ompl::control::Control*>(openLoopControls.begin(), openLoopControls.end());
    //if motion model cannot generate valid open loop controls from start to goal, return an empty vector signifying invalid control
    if(openLoopControls_.size() == 0) {
      return this->motionModel_->getZeroControl();
    }

    //if we generate more controls than the length of controlQueueSize (user-defined), we truncate the rest
    if(openLoopControls_.size() > controlQueueSize_)
      openLoopControls_.resize(controlQueueSize_);
    }
    // if there are controls left, apply and remove them one-by-one
    ompl::control::Control* control;
    control = openLoopControls_.front();

    openLoopControls_.pop_front();

    colvec diff =  (this->goal_)->as<StateType>()->getArmaData() - state->as<StateType>()->getArmaData();

    double distance  = norm(diff.subvec(0,1), 2);

    SpaceType *space;
    space =  new SpaceType();

    ompl::base::State *relativeState = space->allocState();

    space->getRelativeState(state, goal_, relativeState);

    colvec relativeCfg =  relativeState->as<StateType>()->getArmaData();

    //cout<<"RHCICreate controller,                        goal_: "<<endl<<this->goal_.GetArmaData()<<endl;
    //cout<<"RHCICreate controller, relativeCfg bearing (degrees): "<<relativeCfg[2]*180/PI<<endl;
    //cout<<"RHCICreate controller, relativeCfg distance (cm )   : "<<distance*100<<endl;
    //if( distance < 0.20 && !this->m_reachedFlag)  this->m_reachedFlag = true;

    if(distance < turnOnlyDistance_){

      ompl::control::Control* newcontrol  = this->motionModel_->getZeroControl();
      //cout<<"Applying Only Turn Control !"<<endl;
      if (abs(relativeCfg[2]) > 1e-6)
      {
        //cout<<"control val:  "<<0.2 * relativeCfg[2]/abs(relativeCfg[2])<<endl;
        //SO std::cin.get();
        newcontrol->as<ompl::control::RealVectorControlSpace::ControlType>()->values[1] = 0.2 * relativeCfg[2]/abs(relativeCfg[2]);
      }

      return newcontrol;
    }

    return control;
}
