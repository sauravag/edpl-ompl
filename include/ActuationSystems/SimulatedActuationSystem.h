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

#ifndef SIMULATED_ACTUATION_SYSTEM_
#define SIMULATED_ACTUATION_SYSTEM_

#include "ActuationSystemMethod.h"
//<Utilities/OGLDisplay.h>
#include "armadillo"

class SimulatedActuationSystem : public ActuationSystemMethod 
{

  public:

    typedef typename MotionModelMethod::MotionModelPointer MotionModelPointer;
    typedef typename ObservationModelMethod::ObservationModelPointer ObservationModelPointer;
    
    SimulatedActuationSystem(MotionModelPointer mm,  ObservationModelPointer om) 
    : motionModel_(mm), observationModel_(om) 
    {
            
      //this->m_environment = _problem->GetEnvironment();
      
      //m_vcLabel = _node.stringXMLParameter("vcLabel", true, "", "Validity Test Method");
      
      //OGLDisplay<MPTraits>::Initialize(_problem);
    }

    virtual ~SimulatedActuationSystem() {}

    virtual void applyControl(ControlType& u);

    virtual ObservationType getObservation();

    virtual ompl::base::State* getTrueState() 
    {
      return trueState_; 
    }

    virtual void setTrueState(const ompl::base::State *state) 
    {
      ompl::base::StateSpacePtr space(new SpaceType());
      ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
      si->copyState(trueState_, state)
      trueState_->as<StateType>()->SetCovariance(arma::mat(0,0));
      //OGLDisplay<MPTraits>::UpdateTrueState(trueState_); 
    }
    
    bool checkCollision();

    void setBelief(const ompl::base::State *state) 
    {
      //cout << "SimulatedActuationSystem::SetBelief" << endl;;
      //cout << "_belief cov: " << endl << _belief.m_covariance << endl;
      ompl::base::StateSpacePtr space(new SpaceType());
      ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
      si->copyState(belief_, state)
      //cout << "belief_ cov: " << endl << belief_.m_covariance << endl; 
      //OGLDisplay<MPTraits>::UpdateCurrentBelief(_belief);
    }

  protected:
    ompl::base::State *trueState_;
    ompl::base::State *belief_;
    MotionModelPointer motionModel_;
    ObservationModelPointer observationModel_;

};


#endif
