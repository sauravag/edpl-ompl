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

#include "Spaces/R2BeliefSpace.h"
#include "ObservationModels/TwoDBeaconObservationModel.h"
#include <tinyxml.h>
#include "Visualization/Visualizer.h"
#include "Utils/FIRMUtils.h"

typename TwoDBeaconObservationModel::ObservationType 
TwoDBeaconObservationModel::getObservation(const ompl::base::State *state, bool isSimulation)
{
	using namespace arma;

    ObservationType z;

    //generate observation from state, and corrupt with the given noise
    for(unsigned int i = 0; i < landmarks_.size(); i++)
    {

        double range =0;

        calculateRangeToLandmark(state, landmarks_[i], range);
        
        z.resize((i+1)*singleObservationDim ,  1);

        colvec noise = zeros<colvec>(obsNoiseDim);

        if(isSimulation)
        {

            // generate gaussian noise
            //get standard deviations of noise (sqrt of covariance matrix)
            //extract state from Cfg and normalize
            //generate noise scaling/shifting factor
            colvec noise_std = this->sigma_;

            //generate raw noise
            colvec randNoiseVec = randn<colvec>(obsNoiseDim);

            //generate noise from a distribution scaled and shifted from
            //normal distribution N(0,1) to N(0,eta*range + sigma)
            //(shifting was done in GetNoiseCovariance)
            noise = noise_std%randNoiseVec;
        }

        z[singleObservationDim*i] = 1/(pow(range,2)+1) + noise;        
    }

 	return z;
}

typename TwoDBeaconObservationModel::ObservationType 
TwoDBeaconObservationModel::getObservationPrediction(const ompl::base::State *state, const ObservationType& Zg)
{
	using namespace arma;

    ObservationType z;

    //generate observation from predicted state
    for(unsigned int i = 0; i < landmarks_.size(); i++)
    {

        double range =0;

        calculateRangeToLandmark(state, landmarks_[i], range);
        
        z.resize((i+1)*singleObservationDim ,  1);

        z[singleObservationDim*i] = 1/(pow(range,2)+1);        
    }

 	return z;
}

typename TwoDBeaconObservationModel::JacobianType TwoDBeaconObservationModel::getObservationJacobian(const ompl::base::State *state, const ObsNoiseType& v, const ObservationType& z)
{

	using namespace arma;

    unsigned int number_of_landmarks = z.n_rows / singleObservationDim ;

    colvec xVec = state->as<R2BeliefSpace::StateType>()->getArmaData();

    mat H( (singleObservationDim)* number_of_landmarks, stateDim); // Since we are passing the common id list

    for(unsigned int i = 0; i < number_of_landmarks ; ++i)
    {
        colvec candidate;

        colvec diff =  xVec.subvec(0,1) - landmarks_[Indx].subvec(1,2);

        double r = norm(diff,2);

        mat H_i(singleObservationDim,stateDim);

        H_i << (-2/(r(i)^2+1)^2) * diff(0) << (-2/(r(i)^2+1)^2) * diff(1) << endr;

        H.submat(singleObservationDim*i, 0, singleObservationDim*i, 1) = H_i;

    }

    return H;
}

typename TwoDBeaconObservationModel::JacobianType TwoDBeaconObservationModel::getNoiseJacobian(const ompl::base::State *state, const ObsNoiseType& v, const ObservationType& z)
{

	using namespace arma;

    unsigned int number_of_landmarks = z.n_rows / singleObservationDim ;

    mat M(number_of_landmarks,number_of_landmarks);

    M.eye();

    return M;

}

typename TwoDBeaconObservationModel::ObservationType TwoDBeaconObservationModel::computeInnovation(const ompl::base::State *predictedState, const ObservationType& Zg)
{

	using namespace arma;

	colvec zPrd = getObservationPrediction(predictedState, Zg);

	colvec innov = Zg - Zprd;

	return innov; 

}

arma::mat TwoDBeaconObservationModel::getObservationNoiseCovariance(const ompl::base::State *state, const ObservationType& z)
{
	using namespace arma;

    unsigned int number_of_landmarks = z.n_rows / singleObservationDim ;

    mat R(number_of_landmarks,number_of_landmarks);

    R.eye();

    return R*pow(this->sigma_(0),2);
}

void TwoDBeaconObservationModel::calculateRangeToLandmark(const ompl::base::State *state, const arma::colvec& landmark, double& range)
{

	using namespace arma;

	colvec xVec = state->as<R2BeliefSpace::StateType>()->getArmaData();

	colvec diff = xVec - landmark.subvec(1,2);

	range = norm(diff,2);
}

void TwoDBeaconObservationModel::loadLandmarks(const char *pathToSetupFile)
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

	OMPL_INFORM("TwoDBeaconObservationModel: Total number of landmarks loaded successfully : %u", landmarks_.size() );

	Visualizer::addLandmarks(landmarks_);
}

void TwoDBeaconObservationModel::loadParameters(const char *pathToSetupFile)
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

    child = node->FirstChild("TwoDBeaconObservationModel");
    
    //Iterate through all the landmarks and put them into the "landmarks_" list
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );

    double sigma = 0;

    itemElement->QueryDoubleAttribute("sigma", &sigma) ;

    this->sigma_ << sigma << endr;

    OMPL_INFORM("TwoDBeaconObservationModel: sigma_ = ");
    std::cout<<sigma_<<std::endl;

}