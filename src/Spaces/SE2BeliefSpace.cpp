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

#include "Spaces/SE2BeliefSpace.h"

double SE2BeliefSpace::StateType::meanNormWeight_  = -1;
double SE2BeliefSpace::StateType::covNormWeight_   = -1;
double SE2BeliefSpace::StateType::reachDist_   = -1;
double SE2BeliefSpace::StateType::reachDistPos_   = -1;
double SE2BeliefSpace::StateType::reachDistOri_   = -1;
double SE2BeliefSpace::StateType::reachDistCov_   = -1;
arma::colvec SE2BeliefSpace::StateType::normWeights_ = arma::zeros<arma::colvec>(3);

// bool SE2BeliefSpace::StateType::isReached(ompl::base::State *state, bool relaxedConstraint) const
// {
//     // subtract the two beliefs and get the norm
//     arma::colvec stateDiff = state->as<SE2BeliefSpace::StateType>()->getArmaData() - this->getArmaData();
//
//     if(stateDiff[2] > boost::math::constants::pi<double>() )
//     {
//
//         stateDiff[2] =  (stateDiff[2] - 2*boost::math::constants::pi<double>()) ;
//     }
//     if( stateDiff[2] < -boost::math::constants::pi<double>() ){
//
//         stateDiff[2] =  stateDiff[2] + 2*boost::math::constants::pi<double>() ;
//     }
//
//     arma::mat covDiff = state->as<SE2BeliefSpace::StateType>()->getCovariance() - this->getCovariance();
//
//     arma::colvec covDiffDiag = covDiff.diag();
//
//     // NOTE if the given state's covariance is already smaller than this goal state, set the difference to zero
//     for (int i=0; i<covDiffDiag.size(); i++)
//         if(covDiffDiag[i] < 0.0)
//             covDiffDiag[i] = 0.0;
//
//
//     // Need weighted supNorm of difference in means
//     double meanNorm = arma::norm(stateDiff % normWeights_, "inf");
//
//     double covDiagNorm = arma::norm(sqrt(abs(covDiffDiag)) % normWeights_, "inf");
//
//     double norm2 =  std::max(meanNorm*meanNormWeight_, covDiagNorm*covNormWeight_) ;
//
//     double reachConstraint  = reachDist_;
//
//     // HACK why only for rollout execution?
//     if(relaxedConstraint)
//         reachConstraint *= 4;
//
//     if(norm2 <= reachConstraint)
//     {
//         return true;
//     }
//
//     return false;
//
// }

bool SE2BeliefSpace::StateType::isReached(ompl::base::State *state, bool relaxedConstraint) const
{
    // check if position and orientation errors are less than thresholds
    if(!this->isReachedPose(state))
        return false;

    // check if covariance error is less than a threshold
    if(!this->isReachedCov(state))
        return false;

    // for debug
    //std::cout << "isReached()!" << std::endl;

    // otherwise, the given state is considered to have reached this state
    return true;
}

bool SE2BeliefSpace::StateType::isReachedPose(const ompl::base::State *state) const
{
    // subtract the two beliefs and get the norm
    arma::colvec stateDiff = state->as<SE2BeliefSpace::StateType>()->getArmaData() - this->getArmaData();

    if(stateDiff[2] > boost::math::constants::pi<double>())
    {
        stateDiff[2] = (stateDiff[2] - 2*boost::math::constants::pi<double>());
    }
    if(stateDiff[2] < -boost::math::constants::pi<double>())
    {
        stateDiff[2] = stateDiff[2] + 2*boost::math::constants::pi<double>();
    }

    // compute position and orientation errors
    double pos_distance_to_goal = arma::norm(stateDiff.subvec(0,1), 2);
    double ori_distance_to_goal = std::abs(stateDiff[2]);

    // check for position and orientation thresholds
    if(pos_distance_to_goal > reachDistPos_)
    {
        // for debug
        //std::cout << "pos_distance_to_goal: " << pos_distance_to_goal << "  (threshold: " << reachDistPos_ << ")" << std::endl;

        return false;
    }
    if(ori_distance_to_goal > reachDistOri_)
    {
        // for debug
        //std::cout << "ori_distance_to_goal: " << ori_distance_to_goal << "  (threshold: " << reachDistOri_ << ")" << std::endl;

        return false;
    }
    return true;
}

bool SE2BeliefSpace::StateType::isReachedCov(const ompl::base::State *state) const
{
    // subtract the two covariances
    arma::mat covDiff = state->as<SE2BeliefSpace::StateType>()->getCovariance() - this->getCovariance();

    arma::colvec covDiffDiag = covDiff.diag();

    // NOTE if the given state's covariance is already smaller than this goal state, set the difference to zero
    for (int i=0; i<covDiffDiag.size(); i++)
        if(covDiffDiag[i] < 0.0)
            covDiffDiag[i] = 0.0;


    // compute covariance error
    //double cov_distance_to_goal = arma::norm(sqrt(abs(covDiffDiag)) % normWeights_, "inf");    // deprecated
    double cov_distance_to_goal = arma::norm(abs(covDiffDiag) % normWeights_, 2);

    // check for position and orientation thresholds
    if(cov_distance_to_goal > reachDistCov_)
    {
        // for debug
        //std::cout << "cov_distance_to_goal: " << cov_distance_to_goal << "  (threshold: " << reachDistCov_ << ")" << std::endl;

        return false;
    }
    return true;

}

bool SE2BeliefSpace::StateType::sampleBorderBeliefState(ompl::base::State* borderBelief) const
{
    // get the mean of the center belief state
    arma::colvec center_mean = this->getArmaData();

    // compute the offset for from epsilon-relaxation parameters for isReached() condition
    // 1) consider mean offset
//     arma::colvec pos_rand(2, arma::fill::randu);  // range: [ 0.0, 1.0]
//     pos_rand -= 0.5;                              // range: [-0.5, 0.5]
//     double ori_rand_dir = (std::rand()%2>0) ? +1.0 : -1.0;
//     arma::colvec pos_offset = reachDistPos_ * pos_rand / arma::norm(pos_rand, 2);
//     arma::colvec ori_offset = reachDistOri_ * arma::colvec({ori_rand_dir});
    // 2) ignore mean offset
    arma::colvec pos_offset = {0, 0};
    arma::colvec ori_offset = {0};

    // set the new state property
    borderBelief->as<StateType>()->setX(center_mean[0] + pos_offset[0]);
    borderBelief->as<StateType>()->setY(center_mean[1] + pos_offset[1]);
    borderBelief->as<StateType>()->setYaw(center_mean[2] + ori_offset[0]);


    // get the covariance of the center belief state
    arma::mat center_cov = this->getCovariance();

    // compute the offset for from epsilon-relaxation parameters for isReached() condition
    // 1) random relaxation
    // NOTE this can lead to high variance in edge cost computation with not-too-tight reachDeisCov_ value and a limited number of particles
//     arma::colvec cov_rand(3, arma::fill::randu);  // range: [ 0.0, 1.0]    // NOTE consider border belief's covariance larger, but not less, than the center belief's covariance only
    // 2) uniform relaxation
    arma::colvec cov_rand = {1, 1, 1};   // same proportional relaxation for each coordinate; will be weighted according to normWeights_

    arma::colvec cov_offset = reachDistCov_ * cov_rand / arma::norm(cov_rand % normWeights_, 2);

    // for debug
    //std::cout << "cov_rand_norm: " << arma::norm(abs(cov_rand / arma::norm(cov_rand % normWeights_, 2)) % normWeights_, 2) << std::endl;
    //std::cout << "cov_offset_norm: " << arma::norm(abs(cov_offset) % normWeights_, 2) << std::endl;

    // set the new state property
    arma::mat border_cov = center_cov;
    for (int i=0; i<cov_offset.size(); i++)
        border_cov(i,i) += cov_offset[i];
    borderBelief->as<StateType>()->setCovariance(border_cov);

    // for debug
    //bool isReachedCov = this->isReachedCov(borderBelief);
    //std::cout << "isReachedCov(borderBelief): " << isReachedCov << std::endl;

    return true;
}

bool SE2BeliefSpace::StateType::sampleTrueStateFromBelief(ompl::base::State* sampState) const
{
    // Cholesky decomposition such that covariance_ = transform * transform.t()
    arma::mat transform;
    if(!arma::chol(transform, covariance_, "lower"))
    {
        OMPL_ERROR("Failed to decompose the covariance matrix for random sampling!");
        return false;
    }

    // draw a random sample from standard normal distribution
    arma::colvec randvec(3, arma::fill::randn);

    // transform this random sample for this Gaussian distribution
    arma::colvec mean = getArmaData();
    arma::colvec randvec_transformed = mean + transform * randvec;

    // set the new state property
    sampState->as<StateType>()->setX(randvec_transformed[0]);
    sampState->as<StateType>()->setY(randvec_transformed[1]);
    sampState->as<StateType>()->setYaw(randvec_transformed[2]);
    sampState->as<StateType>()->setCovariance(covariance_);    // REVIEW set the covariance as the same with this state

    return true;
}

ompl::base::State* SE2BeliefSpace::allocState(void) const
{
    StateType *state = new StateType();
    
    allocStateComponents(state);
    
    state->setYaw(0.0);
    
    return state;
}

void SE2BeliefSpace::copyState(State *destination, const State *source) const
{
    destination->as<StateType>()->setX(source->as<StateType>()->getX());
    destination->as<StateType>()->setY(source->as<StateType>()->getY());
    destination->as<StateType>()->setYaw(source->as<StateType>()->getYaw());
    destination->as<StateType>()->setCovariance(source->as<StateType>()->getCovariance());
}

void SE2BeliefSpace::freeState(State *state) const
{
    CompoundStateSpace::freeState(state);
}

double SE2BeliefSpace::distance(const State* state1, const State *state2)
{
    double dx = state1->as<StateType>()->getX() - state2->as<StateType>()->getX();
    double dy = state1->as<StateType>()->getY() - state2->as<StateType>()->getY();

    return pow(dx*dx+dy*dy, 0.5);
}
void SE2BeliefSpace::getRelativeState(const State *from, const State *to, State *state)
{
	state->as<StateType>()->setX(to->as<StateType>()->getX() - from->as<StateType>()->getX());
	state->as<StateType>()->setY(to->as<StateType>()->getY() - from->as<StateType>()->getY());

	/*
    	Calculating relative angle is a bit tricky.
    	Refer to "interpolate" function of SO2StateSpace at line 122 of SO2StateSpace.h  in OMPL lib
        to see the original implementation in OMPL
	*/

	double diff = to->as<StateType>()->getYaw() - from->as<StateType>()->getYaw();
	if (fabs(diff) <= boost::math::constants::pi<double>())
        state->as<StateType>()->setYaw(diff);
    else
    {
        double v;
        if (diff > 0.0)
            diff = 2.0 * boost::math::constants::pi<double>() - diff;
        else
            diff = -2.0 * boost::math::constants::pi<double>() - diff;

        v = - diff ;
        // input states are within bounds, so the following check is sufficient
        if (v > boost::math::constants::pi<double>())
            v -= 2.0 * boost::math::constants::pi<double>();
        else
            if (v < -boost::math::constants::pi<double>())
                v += 2.0 * boost::math::constants::pi<double>();
    	state->as<StateType>()->setYaw(v);
    }

    arma::mat fcov = from->as<StateType>()->getCovariance();
    arma::mat tocov = to->as<StateType>()->getCovariance();

    if (fcov.n_rows != 0 && fcov.n_cols != 0 && tocov.n_rows != 0 && tocov.n_cols != 0 )
    {
   		state->as<StateType>()->setCovariance(tocov - fcov);
    }
}

void SE2BeliefSpace::printBeliefState(const State *state)
{
    std::cout<<"----Printing BeliefState----"<<std::endl;
    std::cout<<"State [X, Y, Yaw]: ";
    std::cout<<"["<<state->as<SE2BeliefSpace::StateType>()->getX()<<", "<<state->as<SE2BeliefSpace::StateType>()->getY()
        <<", "<<state->as<SE2BeliefSpace::StateType>()->getYaw()<<"]"<<std::endl;
    std::cout<<"Covariance  is" <<std::endl;
    std::cout<<state->as<SE2BeliefSpace::StateType>()->getCovariance()<<std::endl;
    std::cout<<"------End BeliefState-------"<<std::endl;
}
