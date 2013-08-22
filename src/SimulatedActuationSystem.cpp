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
#include ".../include/ActuationSystems/SimulatedActuationSystem.h"

void SimulatedActuationSystem::applyControl(ControlType& u) 
{

  typename MotionModelMethod::NoiseType noise = motionModel_->generateNoise(trueState_, u);

  trueState_ = motionModel_->Evolve(trueState_, u, noise);
  
  //OGLDisplay<MPTraits>::UpdateTrueState(m_trueState);
  //cout<<" The True State is :"<<endl<<m_trueState.GetArmaData()<<endl;

}

typename SimulatedActuationSystem::ObservationType
SimulatedActuationSystem::getObservation() 
{
  return observationModel_->getObservation(m_trueState, true);
}

bool SimulatedActuationSystem::checkCollision() 
{
  /*
  typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
  // following variables are used in collision checking procedure
  ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
  string callee = this->GetName();
  CDInfo cdInfo;
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  
   if(!m_trueState.InBoundary(this->m_environment) || 
        !vc->IsValid(m_trueState, this->m_environment,  *(this->GetMPProblem()->GetStatClass()), cdInfo, &callee)) {
     
      return true;
    }
  */
    return false;
  
}
