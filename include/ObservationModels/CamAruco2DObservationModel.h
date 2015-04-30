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

#ifndef LANDMARK_RANGEBEARING_H_
#define LANDMARK_RANGEBEARING_H_

#include "ObservationModelMethod.h"
#include <boost/math/constants/constants.hpp>
/**
  @par Short Description
  This is an Observation model method based on the combination of a monocular camera,
  and visual markers with unique ids.
  How it works:
  1. The locations of visual landmarks is known a-priori.
  2. Using location of the robot and pre-known location of landmarks, it generates a noisy observation
     of the relative location of the landmark w.r.t the robot.

  \brief A monocular vision based sensor model where the observations are beacons with unique ids.
*/
class CamAruco2DObservationModel : public ObservationModelMethod
{

  static const int stateDim = 3;
  //static const int singleObservationDim = 3;
  static const int landmarkInfoDim = 2; /*[ X, Y]*/
  static const int numLandmarksForObservability = 2;
  //static const int obsNoiseDim = 3;

  public:

    static const int singleObservationDim = 4; /*[ ID, Range, Bearing, Orientation of Landmark in Environment ]*/

    typedef ObservationModelMethod::ObservationType ObservationType;
    typedef ObservationModelMethod::NoiseType ObsNoiseType;
    typedef arma::mat JacobianType;

    // z = h(x,v)
    // get the observation for a given configuration,
    // corrupted by noise from a given distribution
    /** \brief Constructor */
    CamAruco2DObservationModel(ompl::control::SpaceInformationPtr si, const char *pathToSetupFile) : ObservationModelMethod(si)
    {

        // initialize etaPhi_, etaD_, sigma_;
        this->loadLandmarks(pathToSetupFile);
        this->loadParameters(pathToSetupFile);
    }

    ObservationType getObservation(const ompl::base::State *state, bool isSimulation);

    ObservationType getObservationPrediction(const ompl::base::State *state, const ObservationType& Zg);

     /** \brief Find the observation based on the given state and landmark to a correspongind landmark.
        eg. if ground robot sees landmark 1, then what is the predicted observation to this landmark
    */
    ObservationType getObservationToCorrespondingLandmark(const ompl::base::State *state, const arma::colvec &observedLandmark)
    {
        arma::colvec candidate;

        this->findCorrespondingLandmark(state, observedLandmark, candidate);

        return candidate;
    }

    // Jx = dh/dx
    JacobianType getObservationJacobian(const ompl::base::State *state, const ObsNoiseType& v, const ObservationType& z);
    // Jv = dh/dv
    JacobianType getNoiseJacobian(const ompl::base::State *state, const ObsNoiseType& v, const ObservationType& z);

    ObservationType computeInnovation(const ompl::base::State *predictedState, const ObservationType& Zg);

    arma::mat getObservationNoiseCovariance(const ompl::base::State *state, const ObservationType& z);

    /** \brief Checks if there is a clear line of sight from the robot to the landmark */
    bool hasClearLineOfSight(const ompl::base::State *state, const arma::colvec& landmark);

    bool isLandmarkVisible(const ompl::base::State *state, const arma::colvec& landmark, double& range, double& bearing, double& viewingAngle);

    //void WriteLandmarks();

    bool isStateObservable(const ompl::base::State *state);

    // TODO: write this function
    /** \brief Check if this landmark is actually in the map, reject extraneous landmarks*/
    //bool isLandmarkInMap(const arma::colvec landmark);

  private:

    ObservationType removeSpuriousObservations(const ObservationType& Zg);

    /** \brief Estimates the range and bearing from given state to landmark */
    void calculateRangeBearingToLandmark(const ompl::base::State *state, const arma::colvec& landmark, double& range, double& bearing);

    /** \brief Calculates the likelihood of an observation prediction */
    double getDataAssociationLikelihood(const arma::colvec trueObs, const arma::colvec predictedObs);

    /** \brief Given a landmark that the robot observes (id, range, bearing..) Find the corresponding landmark,returns the position in the landmark list  */
    int findCorrespondingLandmark(const ompl::base::State *state, const arma::colvec &observedLandmark, arma::colvec &candidateObservation);

    std::vector<arma::colvec> landmarks_;

    //Function to load landmarks from XML file into the object
    void loadLandmarks(const char *pathToSetupFile);

    void loadParameters(const char *pathToSetupFile);

    double cameraRange_;
    double cameraHalfFov_;
};

#endif
