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
/* Author: Saurav Agarwal, Ali-akbar Agha-mohammadi */

#include "../../include/MotionModels/UnicycleStatePropagator.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/util/Exception.h"
using namespace ompl;

UnicycleStatePropagator::UnicycleStatePropagator(const control::SpaceInformationPtr &si) : StatePropagator(si)
{
    // The path to this setup file must not be hardcopied, need a better way to do this
    MotionModelMethod::MotionModelPointer mm(new UnicycleMotionModel("/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml"));

    motionModel_ = mm;
}

void UnicycleStatePropagator::propagate(const base::State *state, const control::Control* control, const double duration, base::State *result) const
{

    // convert control into vector of doubles
    arma::colvec controlVec(2);

    const double *conVals = control->as<control::RealVectorControlSpace::ControlType>()->values;

    for (unsigned int i = 0; i < motionModel_->controlDim(); i++)
    {
        controlVec[i] = conVals[i];
    }

    // set the time step
    motionModel_->setTimeStep(duration);

    typedef SE2BeliefSpace::StateType StateType;

    ompl::base::StateSpacePtr space(new SE2BeliefSpace());

    ompl::base::State *to = space->allocState();

    // use the motionmodel to apply the controls
    motionModel_->Evolve(state, controlVec, motionModel_->getZeroNoise(), result);

}

bool UnicycleStatePropagator::canPropagateBackward(void) const
{
    return false;
}

