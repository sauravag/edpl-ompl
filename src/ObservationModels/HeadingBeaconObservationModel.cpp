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

/* Authors: Saurav Agarwal  */

#include "Spaces/SE2BeliefSpace.h"
#include "ObservationModels/HeadingBeaconObservationModel.h"
#include <tinyxml.h>
#include "Visualization/Visualizer.h"
#include "Utils/FIRMUtils.h"

typename HeadingBeaconObservationModel::ObservationType 
HeadingBeaconObservationModel::getObservation(const ompl::base::State *state, bool isSimulation)
{
	using namespace arma;

    ObservationType z(1);

    z[0] = state->as<SE2BeliefSpace::StateType>()->getYaw();

    if(isSimulation)
    {
        colvec headingNoiseVec =  randn<colvec>(1);

        colvec headingNoise = sigmaHeading_%headingNoiseVec;

        z[0] = z[0] + headingNoise[0];

    }

    //generate observation from state, and corrupt with the given noise
    for(unsigned int i = 0; i < landmarks_.size(); i++)
    {

        double range =0;

        calculateRangeToLandmark(state, landmarks_[i], range);
        
        z.resize(1+(i+1)*singleObservationDim ,  1);

        colvec noise = zeros<colvec>(obsNoiseDim);

        if(isSimulation)
        {

            // generate gaussian noise
            //get standard deviations of noise (sqrt of covariance matrix)
            //extract state from Cfg and normalize
            //generate noise scaling/shifting factor
            //generate raw noise
            colvec randNoiseVec = randn<colvec>(obsNoiseDim);

            //generate noise from a distribution scaled and shifted from
            //normal distribution N(0,1) to N(0,eta*range + sigma)
            //(shifting was done in GetNoiseCovariance)
            noise = sigma_%randNoiseVec;
        }

        z[1+singleObservationDim*i] = 1.0/(pow(range,2)+1) + noise[0];        
    }

 	return z;
}

typename HeadingBeaconObservationModel::ObservationType 
HeadingBeaconObservationModel::getObservationPrediction(const ompl::base::State *state, const ObservationType& Zg)
{
	using namespace arma;

    ObservationType z(1);

    z[0] = state->as<SE2BeliefSpace::StateType>()->getYaw();

    //generate observation from predicted state
    for(unsigned int i = 0; i < landmarks_.size(); i++)
    {

        double range =0;

        calculateRangeToLandmark(state, landmarks_[i], range);
        
        z.resize(1+(i+1)*singleObservationDim ,  1);

        z[1+singleObservationDim*i] = 1/(pow(range,2)+1);        
    }

 	return z;
}

typename HeadingBeaconObservationModel::JacobianType HeadingBeaconObservationModel::getObservationJacobian(const ompl::base::State *state, const ObsNoiseType& v, const ObservationType& z)
{

	using namespace arma;

    unsigned int number_of_landmarks = landmarks_.size();

    colvec xVec = state->as<SE2BeliefSpace::StateType>()->getArmaData();

    mat H(1+ number_of_landmarks, 3); // Since we are passing the common id list

    // Jacobian of heading measurement w.r.t state
    H(0,0) = 0; H(0,1) = 0; H(0,2) = 1;

    for(unsigned int i = 0; i < number_of_landmarks ; i++)
    {

        colvec lmk = landmarks_[i];

        double dx =  xVec[0] - lmk[1];

        double dy = xVec[1] - lmk[2];

        double r = sqrt(dx*dx + dy*dy);

        mat H_i(1,3);

        H_i << (-2/pow(r*r+1,2)) * dx << (-2/pow(r*r+1,2)) * dy << 0 << endr;

        H.submat(1 + i, 0, 1 + i, 2) = H_i;

    }

    return H;
}

typename HeadingBeaconObservationModel::JacobianType HeadingBeaconObservationModel::getNoiseJacobian(const ompl::base::State *state, const ObsNoiseType& v, const ObservationType& z)
{

	using namespace arma;

    unsigned int number_of_landmarks = landmarks_.size() ;

    mat M(1+number_of_landmarks,1+number_of_landmarks);

    M.eye();

    return M;

}

typename HeadingBeaconObservationModel::ObservationType HeadingBeaconObservationModel::computeInnovation(const ompl::base::State *predictedState, const ObservationType& Zg)
{

	using namespace arma;

	colvec Zprd = getObservationPrediction(predictedState, Zg);

	colvec innov = Zg - Zprd;

	return innov; 

}

arma::mat HeadingBeaconObservationModel::getObservationNoiseCovariance(const ompl::base::State *state, const ObservationType& z)
{
	using namespace arma;

    unsigned int number_of_landmarks = landmarks_.size();

    mat R(1+number_of_landmarks,1+number_of_landmarks);

    R.eye();

    R = R*pow(this->sigma_(0),2);

    // heading obs error covariance needs to be set separately
    R(0,0) = pow(sigmaHeading_[0],2);

    return R;
}

void HeadingBeaconObservationModel::calculateRangeToLandmark(const ompl::base::State *state, const arma::colvec& landmark, double& range)
{

	using namespace arma;

	colvec xVec = state->as<SE2BeliefSpace::StateType>()->getArmaData();

	colvec diff = xVec.subvec(0,1) - landmark.subvec(1,2);

	range = norm(diff,2);
}

bool HeadingBeaconObservationModel::isStateObservable(const ompl::base::State *state)
{

  using namespace arma;

  colvec obs = this->getObservation(state, false);

  if(obs.n_rows > 2)
      return true;

  return false;

}

void HeadingBeaconObservationModel::loadLandmarks(const char *pathToSetupFile)
{

	using namespace arma;
	// Load XML containing landmarks
	TiXmlDocument doc(pathToSetupFile);
	bool loadOkay = doc.LoadFile();

	if (!loadOkay)
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

		ObservationType landmark(3);
		landmark.zeros();
		double attributeVal;
		itemElement->QueryDoubleAttribute("id", &attributeVal) ;
		landmark[0] = attributeVal;
		itemElement->QueryDoubleAttribute("x", &attributeVal) ;
		landmark[1] = attributeVal;
		itemElement->QueryDoubleAttribute("y", &attributeVal) ;
		landmark[2] = attributeVal;

		this->landmarks_.push_back(landmark);
	}

	OMPL_INFORM("HeadingBeaconObservationModel: Total number of landmarks loaded successfully : %u", landmarks_.size() );

	Visualizer::addLandmarks(landmarks_);
}

void HeadingBeaconObservationModel::loadParameters(const char *pathToSetupFile)
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

    child = node->FirstChild("HeadingBeaconObservationModel");
    
    //Iterate through all the landmarks and put them into the "landmarks_" list
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );

    double sigma_ss = 0, sigma_heading = 0;

    itemElement->QueryDoubleAttribute("sigma_ss", &sigma_ss) ;
    itemElement->QueryDoubleAttribute("sigma_heading", &sigma_heading) ;

    this->sigma_ << sigma_ss << endr;

    sigmaHeading_<<sigma_heading<<endr;


    OMPL_INFORM("HeadingBeaconObservationModel: sigma_ss = ");
    std::cout<<sigma_ss<<std::endl;

    OMPL_INFORM("HeadingBeaconObservationModel: sigma_heading = ");
    std::cout<<sigma_heading<<std::endl;

}