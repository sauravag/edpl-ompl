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

#ifndef OBSERVATION_MODEL_METHOD_
#define OBSERVATION_MODEL_METHOD_

#include "armadillo"
#include <ompl/control/SpaceInformation.h>

/**
  @par Short Description
  Base class for all observation models. An observation model contains a description
  of the sensor and the landmarks. It generates observations and associated observation jacobians/noises.

  \brief Base class for observation models.
*/
class ObservationModelMethod
{
    public:

        typedef arma::colvec ObservationType;
        typedef arma::colvec NoiseType;
        typedef arma::mat ObsToStateJacobianType;
        typedef arma::mat ObsToNoiseJacobianType;
        typedef boost::shared_ptr<ObservationModelMethod> ObservationModelPointer;

        /** \brief Constructor. */
        ObservationModelMethod(ompl::control::SpaceInformationPtr si,int nDim=0) : si_(si), noiseDim_(nDim), zeroNoise_(nDim) {}

        /** \brief z = h(x,v). Get the observation for a given configuration.
            \param The state based on which the observation should be generated.
            \param if true, observation corrupted by noise from a given distribution, otherwise noise free observation.
        */
        virtual ObservationType getObservation(const ompl::base::State *state, bool isSimulation) = 0;

        /** \brief Get the predicted observation based on predicted state.

            @para
            Returns the predicted observation based on the previous state estimate for each landmark that is seen by the sensor.
            For example, if the (real world) sensor detects landmark 'A', this function will estimate the predicted observation of
            landmark 'A' based on the predicted state.

            \para The predicted State
            \para The true observation.
        */
        virtual ObservationType getObservationPrediction(const ompl::base::State *state, const ObservationType& Zg) = 0;

        /** \brief Find the observation based on the given state and landmark to a correspongind landmark.
            eg. if ground robot sees landmark 1, then what is the predicted observation to this landmark
        */
        virtual ObservationType getObservationToCorrespondingLandmark(const ompl::base::State *state, const arma::colvec &observedLandmark) = 0;

        /** \brief Calculate the observation Jacobian i.e. Jx = dh/dx, where h is the observation model and x is the state. */
        virtual ObsToStateJacobianType getObservationJacobian(const ompl::base::State *state, const NoiseType& v, const ObservationType& z) = 0;

        /** \brief Calculates the observation noise jacobian i.e. Jv = dh/dv, where h is the observation model and v is the noise */
        virtual ObsToNoiseJacobianType getNoiseJacobian(const ompl::base::State *state, const NoiseType& v, const ObservationType& z) = 0;

        /** \brief Computes the innovation between observations that are predicted for the given state and the true observation. */
        virtual ObservationType computeInnovation(const ompl::base::State *predictedState, const ObservationType& Zg) = 0;

        /** \brief Calculates the observation noise covariance.*/
        virtual arma::mat getObservationNoiseCovariance(const ompl::base::State *state, const ObservationType& z) = 0;

        /** \brief Checks if a state is observable. */
        virtual bool isStateObservable(const ompl::base::State *state) = 0;

        /** \brief Returns the zero observation noise.*/
        virtual const NoiseType getZeroNoise() {return zeroNoise_; }

        /** \brief */
        arma::colvec etaPhi_;

        /** \brief */
        arma::colvec etaD_;

        /** \brief */
        arma::colvec sigma_;

    protected:

        /** \brief A pointer to the space information. */
	    ompl::control::SpaceInformationPtr si_;

    private:

        /** \brief Noise vector dimension, it is specific to each observation model subclass. */
        const int noiseDim_;

        /** \brief Stores the value of zero noise. */
        const NoiseType zeroNoise_;


};

#endif
