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

#ifndef OBSERVATION_MODEL_METHOD_
#define OBSERVATION_MODEL_METHOD_

//#include "Utilities/MPUtils.h"
#include "armadillo"

/*
Base class for all observation models
Provides a uniform interface
*/
class ObservationModelMethod
{
  public:
    //dimensionality of motion and observation noise
    //maybe these should go into the specific observation model?
    //static const int observationDim;
    //static const int obsNoiseDim;

    typedef arma::colvec ObservationType;
    typedef arma::colvec NoiseType;
    //typedef arma::mat JacobianType;
    typedef arma::mat ObsToStateJacobianType;
    typedef arma::mat ObsToNoiseJacobianType;
    typedef boost::shared_ptr<ObservationModelMethod> ObservationModelPointer;


    //TODO: ensure all children pass up the noise dimension
    //ObservationModelMethod() : noiseDim_(0) {}

    ObservationModelMethod(int nDim=0) : noiseDim_(nDim), zeroNoise_(nDim) {}

    // z = h(x,v)
    //get the observation for a given configuration,
    //corrupted by noise from a given distribution
    virtual
      ObservationType getObservation(const ompl::base::State *state, bool isSimulation) = 0;

    virtual
      ObservationType getObservationPrediction(const ompl::base::State *state, const ObservationType& Zg) = 0;

    virtual
      ObservationType removeSpuriousObservations(const ObservationType& Zg) = 0;

    // Jx = dh/dx
    virtual
      ObsToStateJacobianType getObservationJacobian(const ompl::base::State *state, const NoiseType& v, const ObservationType& z) = 0;

    // Jv = dh/dv
    virtual
      ObsToNoiseJacobianType getNoiseJacobian(const ompl::base::State *state, const NoiseType& v, const ObservationType& z) = 0;

    //virtual
    //  NoiseType GenerateObservationNoise(const CfgType& _x) = 0;

    virtual
      ObservationType computeInnovation(const ompl::base::State *predictedState, const ObservationType& Zg) = 0;

    virtual
      arma::mat getObservationNoiseCovariance(const ompl::base::State *state, const ObservationType& z) = 0;

    virtual const
      NoiseType getZeroNoise() {return zeroNoise_; }

    arma::colvec etaPhi_;
    arma::colvec etaD_;
    arma::colvec sigma_;
  private:

    //noise vector dimension is specific to each observation model subclass
    const int noiseDim_;
    const NoiseType zeroNoise_;
};

#endif