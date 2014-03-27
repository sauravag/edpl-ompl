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

#ifndef UNIFORM_VALID_BELIEF_SAMPLER_
#define UNIFORM_VALID_BELIEF_SAMPLER_

#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/StateSampler.h"
#include "../ActuationSystems/ActuationSystemMethod.h"
#include "../SpaceInformation/SpaceInformation.h"

/*
Samples states in the belief space uniformly.
While sampling, the function should check collision and observability.
*/

/** \brief Generate valid samples using the Uniform sampling strategy */
class UniformValidBeliefSampler : public ompl::base::ValidStateSampler
{
  public:
    //typedef ObservationModelMethod::ObservationModelPointer ObservationModelPointer;
    //typedef ActuationSystemMethod::ActuationSystemPointer ActuationSystemPointer;

    /** \brief Constructor */
    UniformValidBeliefSampler(const firm::SpaceInformation *si);

    virtual ~UniformValidBeliefSampler(void)
    {
    }

    virtual bool sample(State *state);
    virtual bool sampleNear(State *state, const State *near, const double distance);

    /**
    void setObservationModel(ObservationModelPointer om)
    {
        observationModel_ = om;
    }
    */

  protected:

    /** \brief The sampler to build upon */
    StateSamplerPtr sampler_;

    /** brief Checks if the sample is observable
        i.e. If it can observe sufficient landmarks
        Ideally, observability check should be performed in the filter instead of obs model
    */
    bool isObservable(ompl::base::State *state);

    //ActuationSystemPointer actuationSystem_;
    //ObservationModelPointer observationModel_;
};

#endif
