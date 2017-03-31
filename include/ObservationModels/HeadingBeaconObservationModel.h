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

/* Authors: Saurav Agarwal */

#ifndef HEADING_BEACON_H_
#define HEADING_BEACON_H_

#include "ObservationModelMethod.h"
#include <boost/math/constants/constants.hpp>
/**
  @par Short Description
  This is an Observation model method based on beacons to which we can measure we can signal strength.
  Signal Strength falls quadratically with distance. Additionally this model provides a direct measurement of heading.

  \brief observations are signal strength of beacons with unique ids.
*/
class HeadingBeaconObservationModel : public ObservationModelMethod
{

  static const int stateDim = 3;
  //static const int singleObservationDim = 3;
  static const int landmarkInfoDim = 2; /*[X, Y]*/
  static const int numLandmarksForObservability = 1;
  static const int obsNoiseDim = 1;

  public:

    static const int singleObservationDim = 1; /*[ signal strength ]*/

    typedef ObservationModelMethod::ObservationType ObservationType;
    typedef ObservationModelMethod::NoiseType ObsNoiseType;
    typedef arma::mat JacobianType;

    /** \brief Constructor */
    HeadingBeaconObservationModel(ompl::control::SpaceInformationPtr si, const char *pathToSetupFile) : ObservationModelMethod(si)
    {

        // initialize etaPhi_, etaD_, sigma_;
        this->loadLandmarks(pathToSetupFile);
        this->loadParameters(pathToSetupFile);
    }

    /** \brief z = h(x,v) get the observation for a given configuration, corrupted by noise from a given distribution */
    ObservationType getObservation(const ompl::base::State *state, bool isSimulation);

    ObservationType getObservationPrediction(const ompl::base::State *state, const ObservationType& Zg);

     /** \brief Find the observation based on the given state and landmark to a corresponding landmark.
        eg. if ground robot sees landmark 1, then what is the predicted observation to this landmark
        This function is a dummy for this observation model.
    */
    ObservationType getObservationToCorrespondingLandmark(const ompl::base::State *state, const arma::colvec &observedLandmark)
    {

        arma::colvec candidate;

        return candidate;
    }

    /** \brief Jx = dh/dx */
    JacobianType getObservationJacobian(const ompl::base::State *state, const ObsNoiseType& v, const ObservationType& z);
    
    /** \brief Jv = dh/dv */
    JacobianType getNoiseJacobian(const ompl::base::State *state, const ObsNoiseType& v, const ObservationType& z);

    /** \brief Compute innovation between actual observation and predicted observation */
    ObservationType computeInnovation(const ompl::base::State *predictedState, const ObservationType& Zg);

    /** \brief The sensor noise covariance */
    arma::mat getObservationNoiseCovariance(const ompl::base::State *state, const ObservationType& z);

    bool isStateObservable(const ompl::base::State *state);

  private:

    /** \brief Estimates the range and bearing from given state to landmark */
    void calculateRangeToLandmark(const ompl::base::State *state, const arma::colvec& landmark, double& range);

    std::vector<arma::colvec> landmarks_;

    //Function to load landmarks from XML file into the object
    void loadLandmarks(const char *pathToSetupFile);

    void loadParameters(const char *pathToSetupFile);

    arma::colvec sigmaHeading_;

};

#endif
