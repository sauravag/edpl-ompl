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
#include "../../include/Spaces/SE2BeliefSpace.h"
#include "../../include/ObservationModels/CamAruco2DObservationModel.h"
#include <tinyxml.h>
#include "../../include/Visualization/Visualizer.h"
#include "../../include/Utils/FIRMUtils.h"

namespace ompl
{
    namespace magic
    {
        static const double ONE_STEP_DISTANCE_FOR_VISIBILITY = 0.09 ; // meters, 0.5 for create (Use radius of robot)
    }
}

/*
  For each landmark, produces an observation that is the range and bearing
  of the given State from that landmark. Result is the concatenation of all
  such observations Obs dim is 2x the size of the landmarks
*/
CamAruco2DObservationModel::ObservationType CamAruco2DObservationModel::getObservation(const ompl::base::State *state, bool isSimulation)
{
    using namespace arma;

    //colvec xVec = state->as<SE2BeliefSpace::StateType>()->getArmaData();

    ObservationType z;

    int counter = 0;

    //generate observation from state, and corrupt with the given noise
    for(unsigned int i = 0; i < landmarks_.size(); i++)
    {

        double landmarkRange =0, landmarkBearing = 0, relativeAngle = 0;

        if(isLandmarkVisible(state, landmarks_[i], landmarkRange , landmarkBearing, relativeAngle))
        {

            //cout<<"Trying to resize"<<endl;
            z.resize((counter+1)*singleObservationDim ,  1);

            colvec noise = zeros<colvec>(landmarkInfoDim);

            if(isSimulation)
            {

                // generate gaussian noise
                //get standard deviations of noise (sqrt of covariance matrix)
                //extract state from Cfg and normalize
                //generate noise scaling/shifting factor

                colvec noise_std = this->etaD_*landmarkRange + this->etaPhi_*relativeAngle + this->sigma_;

                //generate raw noise
                colvec randNoiseVec = randn<colvec>(2);

                //generate noise from a distribution scaled and shifted from
                //normal distribution N(0,1) to N(0,eta*range + sigma)
                //(shifting was done in GetNoiseCovariance)
                noise = noise_std%randNoiseVec;
            }

            z[singleObservationDim*counter] = landmarks_[i](0) ; // id of the landmark
            assert(landmarkRange <= cameraRange_);
            z[singleObservationDim*counter + 1 ] = landmarkRange + noise[0]; // distance to landmark
            z[singleObservationDim*counter+2] = landmarkBearing + noise[1];
            z[singleObservationDim*counter+3] = landmarks_[i](3);

            assert(abs( z[singleObservationDim*counter+2]) <= boost::math::constants::pi<double>());

            counter++;
        }
    }

  return z;

}

bool isUnique(const arma::colvec z)
{

    int singleObservationDim = 4;
    //Need to check if observations are not repeated in Zg.
    for(unsigned int k = 0; k< z.n_rows / singleObservationDim ;k++)
    {
        for(unsigned int p = 0; p< z.n_rows / singleObservationDim ;p++)
        {
            if(k!=p && z[singleObservationDim*k]==z[singleObservationDim*p])
            {
                return false;
            }
        }

    }
    return true;
}

typename CamAruco2DObservationModel::ObservationType
CamAruco2DObservationModel::getObservationPrediction(const ompl::base::State *state, const ObservationType& Zg)
{

    using namespace arma;

    colvec xVec =  state->as<SE2BeliefSpace::StateType>()->getArmaData();

    ObservationType z;

    for(unsigned int k = 0; k< Zg.n_rows / singleObservationDim ;k++)
    {
        // The candidate landmark is the closest landmark in the list with the same ID as that of what real robot sees
        colvec candidate;

        int candidateIndx = this->findCorrespondingLandmark(state, Zg.subvec(singleObservationDim*k,singleObservationDim*k+3), candidate);

        z.resize((k+1)*singleObservationDim ,  1);
        z[singleObservationDim*k]     = candidate(0)  ; // id of the landmark
        z[singleObservationDim*k + 1] = candidate(1) ;  // distance to landmark
        z[singleObservationDim*k+2]   = candidate(2)  ; // bearing
        z[singleObservationDim*k + 3] = candidate(3)  ; // orientation of the landmark

        assert(abs(z[singleObservationDim*k+2]) <= boost::math::constants::pi<double>());

    }

  return z;

}

void CamAruco2DObservationModel::calculateRangeBearingToLandmark(const ompl::base::State *state, const arma::colvec& landmark, double& range, double& bearing)
{
    using namespace arma;

    colvec xVec =  state->as<SE2BeliefSpace::StateType>()->getArmaData();

    colvec diff =  landmark.subvec(1,2) - xVec.subvec(0,1);

    //norm is the 2nd Holder norm, i.e. the Euclidean norm
    double distance = norm(diff,2);

    double robot_landmark_ray =  atan2(diff[1],diff[0]) ;

    double delta_theta = robot_landmark_ray - xVec[2];

    //if(distance < 0.1)
    //{
    //    OMPL_INFORM("Aruco: Range: %f  bearing: %f", distance, delta_theta);
    //}

    FIRMUtils::normalizeAngleToPiRange(delta_theta);

    range = distance;

    bearing = delta_theta;
}

double CamAruco2DObservationModel::getDataAssociationLikelihood(const arma::colvec trueObs, const arma::colvec predictedObs)
{
    // Find the most likely landmark to predict associated with the true observation
    double weight = 0.0;

    arma::colvec noise = this->sigma_;

    arma::mat covariance = arma::diagmat(arma::pow(noise,2));

    arma::colvec innov = trueObs-predictedObs;

    FIRMUtils::normalizeAngleToPiRange(innov(1));

    arma::mat t = -0.5*trans(innov)*covariance.i()*innov;

    weight = std::exp(t(0,0));

    assert(weight >= 0 && "Weight cannot be less than 0!");

    return weight;
}


int CamAruco2DObservationModel::findCorrespondingLandmark(const ompl::base::State *state, const arma::colvec &observedLandmark, arma::colvec &candidateObservation)
{
    using namespace arma;

    int landmarkID = observedLandmark[0];

    double maxLikelihood = -1.0;

    double candidatelandmarkRange =0, candidatelandmarkBearing = 0;

    int candidateIndx = -1;

    for(unsigned int i = 0; i < landmarks_.size() ; i++)
    {
        if(landmarks_[i](0) == landmarkID)
        {
            double landmarkRange =0, landmarkBearing = 0;

            // get range and bearing to landmark
            this->calculateRangeBearingToLandmark(state, landmarks_[i], landmarkRange,landmarkBearing);

            arma::colvec prediction;

            prediction<<landmarkRange<<landmarkBearing<<endr;

            // calculate the likelihood
            double lkhd = getDataAssociationLikelihood(observedLandmark.subvec(1,2), prediction);

            if(lkhd > maxLikelihood)
            {
                candidateIndx = i;
                maxLikelihood = lkhd;
                candidatelandmarkRange = landmarkRange;
                candidatelandmarkBearing = landmarkBearing;
            }
        }

    }

    assert(candidateIndx >= 0 && "Candidate index cannot be negative, maybe robot saw landmark that is not in map");

    // The observation is id, range, bearing, orientation of the landmark
    candidateObservation<<landmarkID<<candidatelandmarkRange<<candidatelandmarkBearing<<landmarks_[candidateIndx][3]<<endr;

    return candidateIndx;

}


bool CamAruco2DObservationModel::hasClearLineOfSight(const ompl::base::State *state, const arma::colvec& landmark )
{

    using namespace arma;

    colvec xVec = state->as<SE2BeliefSpace::StateType>()->getArmaData();

    colvec robot_to_landmark_ray =  landmark.subvec(1,2) - xVec.subvec(0,1);

    double distance = norm(robot_to_landmark_ray,2);

    int steps = std::floor(distance/ompl::magic::ONE_STEP_DISTANCE_FOR_VISIBILITY) - 1 ; // subtract 1 one step because you dont need robot center to be at landmark

    if(steps < 0) steps = 0;

    ompl::base::State *tempState = this->si_->allocState();

    for(int i=1 ; i < steps; i++)
    {
        double newX = xVec(0) + i*robot_to_landmark_ray(0)/steps;

        double newY = xVec(1) + i*robot_to_landmark_ray(1)/steps;

        tempState->as<SE2BeliefSpace::StateType>()->setXYYaw(newX, newY,0);

        if(!this->si_->isValid(tempState))
        {
            return false;
        }

    }

    si_->freeState(tempState);

    return true;
}


bool CamAruco2DObservationModel::isLandmarkVisible(const ompl::base::State *state, const arma::colvec& landmark,
                                                              double& range, double& bearing, double& viewingAngle)
{
    using namespace arma;

    colvec xVec = state->as<SE2BeliefSpace::StateType>()->getArmaData();

    double fov = cameraHalfFov_*boost::math::constants::pi<double>()/180; // radians

    double maxRange = cameraRange_; // meters // NOTE: if you change this value, you must make a corresponding change in the "draw" function of the Cfg.cpp file.

    this->calculateRangeBearingToLandmark(state, landmark, range , bearing);

    double thetaLandmark =  landmark[3]*boost::math::constants::pi<double>() ;

    viewingAngle = abs(acos(cos(thetaLandmark )*cos(xVec[2]) + sin(thetaLandmark )*sin(xVec[2])));

    if( viewingAngle > boost::math::constants::pi<double>()/2 )
        viewingAngle = abs(viewingAngle-boost::math::constants::pi<double>() );

    // This is to prevent cholesky decomposition from collapsing
    // if range less than 1 cm, limit it to 1 cm as smallest value
    if(range < 1e-2)
    {
        //range = 1e-2;
        return false; // if too close or on top of landmark, dont see it
    }

    if( abs(bearing) <= fov && range <= maxRange )
    {
        if(hasClearLineOfSight(state, landmark))
        {
            assert(abs(viewingAngle) <= boost::math::constants::pi<double>() / 2 );
            return true;
        }
    }

    return false;

}

typename CamAruco2DObservationModel::JacobianType
CamAruco2DObservationModel::getObservationJacobian(const ompl::base::State *state, const ObsNoiseType& v,
  const ObservationType& z)
{
    using namespace arma;

    unsigned int number_of_landmarks = z.n_rows / singleObservationDim ;

    colvec xVec = state->as<SE2BeliefSpace::StateType>()->getArmaData();

    mat H( (landmarkInfoDim)* number_of_landmarks, stateDim); // Since we are passing the common id list

    for(unsigned int i = 0; i < number_of_landmarks ; ++i)
    {
        colvec candidate;

        int Indx = this->findCorrespondingLandmark(state, z.subvec(i*singleObservationDim,i*singleObservationDim+3), candidate);

        colvec diff =  landmarks_[Indx].subvec(1,2) - xVec.subvec(0,1);

        double phi = atan2(diff[1], diff[0]);

        double r = norm(diff,2);

        mat H_i((landmarkInfoDim),stateDim);

        H_i <<  -cos(phi)    <<  -sin(phi)    <<   0 << endr
            <<   sin(phi)/r  <<  -cos(phi)/r  <<  -1 << endr;

        H.submat((landmarkInfoDim)*i, 0, (landmarkInfoDim)*i+1, 2) = H_i;

    }

    return H;
}


typename CamAruco2DObservationModel::JacobianType
CamAruco2DObservationModel::getNoiseJacobian(const ompl::base::State *state, const ObsNoiseType& _v, const ObservationType& z)
{
  using namespace arma;

  //noise jacobian is just an identity matrix
  int number_of_landmarks = z.n_rows / singleObservationDim ;

  mat M = eye(number_of_landmarks*2, number_of_landmarks*2);

  return M;

}



arma::mat CamAruco2DObservationModel::getObservationNoiseCovariance(const ompl::base::State *state, const ObservationType& z)
{
    using namespace arma;

    //extract state from Cfg and normalize
    colvec xVec = state->as<SE2BeliefSpace::StateType>()->getArmaData();

    unsigned int number_of_landmarks = z.n_rows/singleObservationDim ;

    //generate noise scaling/shifting factors
    colvec noise( number_of_landmarks*(landmarkInfoDim));

    for(unsigned int i =0; i< number_of_landmarks ; i++)
    {
        colvec candidate;

        int indx = this->findCorrespondingLandmark(state, z.subvec(i*singleObservationDim,i*singleObservationDim+3), candidate);

        double range = candidate(1);//norm( landmarks_[indx].subvec(1,2) - xVec.subvec(0,1) , 2);

        double thetaLandmark =  candidate(3)*boost::math::constants::pi<double>() ;

        double viewingAngle = abs(acos(cos(thetaLandmark )*cos(xVec[2]) + sin(thetaLandmark )*sin(xVec[2])));

        if( viewingAngle > boost::math::constants::pi<double>()/2 ) viewingAngle = abs(viewingAngle-boost::math::constants::pi<double>() );

        noise.subvec(2*i, 2*i+1) = this->etaD_*range + this->etaPhi_*viewingAngle + this->sigma_;

    }

    //square the factors to get the covariances
    noise = pow(noise,2);

    //return covariance matrix generated from vector of covariances
    return diagmat(noise);

}


typename CamAruco2DObservationModel::ObservationType
CamAruco2DObservationModel::computeInnovation(const ompl::base::State *predictedState, const ObservationType& Zg)
{
    using namespace arma;

    colvec xPrd = predictedState->as<SE2BeliefSpace::StateType>()->getArmaData();

    //return the discrepancy between the expected observation
    //for a predicted state and the actual observation generated

    ObservationType Zprd = getObservationPrediction(predictedState, Zg);

    if(Zprd.n_rows == 0)
    {
        ObservationType innov;

        return innov;
    }

    ObservationType innov( (landmarkInfoDim)* Zg.n_rows /singleObservationDim ) ;

    assert( Zg.n_rows == Zprd.n_rows);

    for(unsigned int i =0; i< Zg.n_rows/singleObservationDim ; i++)
    {

        assert(Zg(i*singleObservationDim) == Zprd(i*singleObservationDim)) ;

        innov( i*(landmarkInfoDim) ) = Zg(i*singleObservationDim + 1) - Zprd(i*singleObservationDim + 1) ;

        double delta_theta = Zg(i*singleObservationDim + 2) - Zprd(i*singleObservationDim + 2) ;

        FIRMUtils::normalizeAngleToPiRange(delta_theta);

        innov( i*(landmarkInfoDim) + 1 ) =  delta_theta;

        assert(abs(delta_theta) <= boost::math::constants::pi<double>() );

    }

    return innov;

}

typename CamAruco2DObservationModel::ObservationType CamAruco2DObservationModel::removeSpuriousObservations(const ObservationType& Zg)
{


    ObservationType Zcorrected;

    int counter  = 0;
    for(unsigned int i=0; i < Zg.n_rows / singleObservationDim ; i++)
    {

        for(unsigned int j=0; j < landmarks_.size() ; j++)
        {

            if(Zg(i*singleObservationDim) == landmarks_[j](0))
            {

                Zcorrected.resize(singleObservationDim*(counter +1));

                Zcorrected(singleObservationDim*counter) =    Zg(i*singleObservationDim);

                Zcorrected(singleObservationDim*counter+1) =  Zg(i*singleObservationDim+1);

                Zcorrected(singleObservationDim*counter+2) =  Zg(i*singleObservationDim+2);

                counter++;
                break;
            }

        }

    }

    return Zcorrected;

}


void CamAruco2DObservationModel::loadLandmarks(const char *pathToSetupFile)
{
  using namespace arma;
  // Load XML containing landmarks
  TiXmlDocument doc(pathToSetupFile);
  bool loadOkay = doc.LoadFile();

  if ( !loadOkay )
  {
    printf( "Could not load Landmark list . Error='%s'. Exiting.\n", doc.ErrorDesc() );

    exit( 1 );
  }

  TiXmlNode* node = 0;
  TiXmlElement* landmarkElement = 0;
  TiXmlElement* itemElement = 0;

  // Get the landmarklist node
  node = doc.FirstChild( "LandmarkList" );
  assert( node );
  landmarkElement = node->ToElement(); //convert node to element
  assert( landmarkElement  );

  TiXmlNode* child = 0;

  //Iterate through all the landmarks and put them into the "landmarks_" list
  while( (child = landmarkElement ->IterateChildren(child)))
  {
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );

    ObservationType landmark(singleObservationDim);
    landmark.zeros();
    double attributeVal;
    itemElement->QueryDoubleAttribute("id", &attributeVal) ;
    landmark[0] = attributeVal;
    itemElement->QueryDoubleAttribute("x", &attributeVal) ;
    landmark[1] = attributeVal;
    itemElement->QueryDoubleAttribute("y", &attributeVal) ;
    landmark[2] = attributeVal;
    itemElement->QueryDoubleAttribute("theta", &attributeVal) ;
    landmark[3] = attributeVal;

    this->landmarks_.push_back(landmark);

  }

    OMPL_INFORM("CamArucoObservationModel: Total number of landmarks loaded successfully : %u", landmarks_.size() );

    Visualizer::addLandmarks(landmarks_);
}

void CamAruco2DObservationModel::loadParameters(const char *pathToSetupFile)
{
    using namespace arma;
    // Load XML containing landmarks
    TiXmlDocument doc(pathToSetupFile);
    bool loadOkay = doc.LoadFile();

    if ( !loadOkay )
    {
        printf( "Could not load setup file . Error='%s'. Exiting.\n", doc.ErrorDesc() );

        exit( 1 );
    }

    TiXmlNode* node = 0;

    TiXmlElement* itemElement = 0;

    // Get the landmarklist node
    node = doc.FirstChild( "ObservationModels" );
    assert( node );


    TiXmlNode* child = 0;

    child = node->FirstChild("CamAruco2DObservationModel");
    //Iterate through all the landmarks and put them into the "landmarks_" list
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );

    double cameraRange = 0;
    double cameraHalfFov = 0;
    double sigmaRange=0;
    double sigmaAngle=0;
    double etaRD=0;
    double etaRPhi=0;
    double etaThetaD=0;
    double etaThetaPhi=0;

    itemElement->QueryDoubleAttribute("camera_range", &cameraRange) ;
    itemElement->QueryDoubleAttribute("camera_half_fov", &cameraHalfFov) ;
    itemElement->QueryDoubleAttribute("sigma_range", &sigmaRange) ;
    itemElement->QueryDoubleAttribute("sigma_angle", &sigmaAngle) ;
    itemElement->QueryDoubleAttribute("eta_rd", &etaRD) ;
    itemElement->QueryDoubleAttribute("eta_rphi", &etaRPhi) ;
    itemElement->QueryDoubleAttribute("eta_thetad", &etaThetaD) ;
    itemElement->QueryDoubleAttribute("eta_thetaphi", &etaThetaPhi) ;

    this->sigma_ << sigmaRange << sigmaAngle * boost::math::constants::pi<double>() / 180.0 << endr;
    this->etaD_  << etaRD << etaThetaD <<endr;
    this->etaPhi_<< etaRPhi << etaThetaPhi << endr;

    cameraRange_ = cameraRange;
    cameraHalfFov_ = cameraHalfFov;

    OMPL_INFORM("CamArucoObservationModel: sigmaRange = %f", sigmaRange );

    OMPL_INFORM("CamArucoObservationModel: sigma_ = ");
    std::cout<<sigma_<<std::endl;

    OMPL_INFORM("CamArucoObservationModel: etaD_ = ");
    std::cout<<etaD_<<std::endl;

    OMPL_INFORM("CamArucoObservationModel: etaPhi = ");
    std::cout<<etaPhi_<<std::endl;

}

bool CamAruco2DObservationModel::isStateObservable(const ompl::base::State *state)
{
  // TODO: Calculate observability matrix and check rank.

  using namespace arma;

  colvec obs = this->getObservation(state, false);

  if(obs.n_rows >= numLandmarksForObservability*singleObservationDim)
      return true;

  return false;

}
