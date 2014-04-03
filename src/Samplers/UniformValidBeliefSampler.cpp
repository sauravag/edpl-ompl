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

#include "../../include/Samplers/UniformValidBeliefSampler.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/tools/config/MagicConstants.h"


UniformValidBeliefSampler::UniformValidBeliefSampler(const ompl::base::SpaceInformation *si) :
    ValidStateSampler(si), sampler_(si->allocStateSampler())
{
    name_ = "uniformBelief";
}

bool UniformValidBeliefSampler::sample(State *state)
{
    unsigned int attempts = 0;
    bool valid = false;
    do
    {
        sampler_->sampleUniform(state);
        valid = si_->isValid(state) && isObservable(state);

        ++attempts;
    } while (!valid && attempts < attempts_);
    return valid;
}

bool UniformValidBeliefSampler::sampleNear(State *state, const State *near, const double distance)
{
    unsigned int attempts = 0;
    bool valid = false;
    do
    {
        sampler_->sampleUniformNear(state, near, distance);
        valid = si_->isValid(state) && isObservable(state) ;
        ++attempts;
    } while (!valid && attempts < attempts_);
    return valid;
}

bool UniformValidBeliefSampler::isObservable(ompl::base::State *state)
{
    //if(observationModel_->isStateObservable(state)) std::cout<<"Observable"<<std::endl;
    //else std::cout<<"Not Observable"<<std::endl;
    //return observationModel_->isStateObservable(state);
    return true;//si_->getObservationModel()->isStateObservable(state);
}
