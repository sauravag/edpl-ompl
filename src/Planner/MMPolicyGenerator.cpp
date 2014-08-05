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

#include "../../include/Planner/MMPolicyGenerator.h"
#include <boost/thread.hpp>
#include "../../include/Visualization/Visualizer.h"

#define FAILURE_COST 1e6 // cost of colliding
#define RRT_PLAN_MAX_TIME 5.0 // maximum time allowed for RRT to plan
#define RRT_FINAL_PROXIMITY_THRESHOLD 1.0 // maximum distance for RRT to succeed

void MMPolicyGenerator::generatePolicy(std::vector<ompl::control::Control*> &policy)
{
    this->updateWeights();

    this->printWeights();

    // Make sure that each beliefstate has a target state
    assert(currentBeliefStates_.size() == targetStates_.size());

    //container to store the sequence of controls for each mode/target pair
    std::vector<std::vector<ompl::control::Control*> > openLoopPolicies;

    // Iterate over the mode/target pairs and generate open loop controls
    for(int i = 0; i < currentBeliefStates_.size(); i++)
    {

        //std::cout<<"Belief Number "<<i<<std::endl;
        //std::cout<<"The belief is :"<<std::endl;
        //si_->printState(currentBeliefStates_[i]);
        //std::cout<<"The target is :"<<std::endl;
        //si_->printState(targetStates_[i]);
        //Generate a path for the mode/target pair
        if(si_->isValid(currentBeliefStates_[i]) /*&& si_->distance(currentBeliefStates_[i], targetStates_[i])>0.50*/)
        {

            ompl::base::PlannerPtr planner(new ompl::geometric::RRT(si_));

            ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si_));

            pdef->setStartAndGoalStates(currentBeliefStates_[i], targetStates_[i], RRT_FINAL_PROXIMITY_THRESHOLD);

            planner->setProblemDefinition(pdef);

            planner->setup();

            ompl::base::PlannerStatus solved = planner->solve(RRT_PLAN_MAX_TIME);


            if(solved)
            {
                const ompl::base::PathPtr &path = pdef->getSolutionPath();

                ompl::geometric::PathGeometric gpath = static_cast<ompl::geometric::PathGeometric&>(*path);

                std::vector<ompl::control::Control*> olc;

                si_->getMotionModel()->generateOpenLoopControlsForPath(gpath, olc);

                openLoopPolicies.push_back(olc);

                // If there actually exists a policy, then only add a corresponding weight.
                //weights.push_back(unNormalizedWeight/totalWeight);

            }

            planner->clear();
        }
    }

    // Now that you have the open loop controls, need to execute them on all modes to see which is best
    std::vector<ompl::base::Cost> policyInfGains;

    double maxGain = -1e10;

    int maxGainPolicyIndx=-1;

    for(int i = 0; i < openLoopPolicies.size(); i++)
    {
        ompl::base::Cost pGain;

        pGain.v = 0;

        for(int j = 0; j < currentBeliefStates_.size(); j++)
        {

            ompl::base::Cost c = executeOpenLoopPolicyOnMode(openLoopPolicies[i],currentBeliefStates_[j]);

            pGain.v += c.v;

            //std::cout<<"Policy Number #"<<i<<" Info Gain value :"<<c.v<<std::endl;

        }

        pGain.v = weights_[i]*pGain.v;

        policyInfGains.push_back(pGain);

        if(pGain.v >= maxGain)
        {
            maxGain = pGain.v;

            maxGainPolicyIndx = i;
        }
    }

    if( !areSimilarWeights() && maxGainPolicyIndx >= 0)
    {
        //std::cout<<"The max gain policy is :"<< maxGainPolicyIndx<<std::endl;
        OMPL_INFORM("A minimum cost policy was found");
        policy = openLoopPolicies[maxGainPolicyIndx];
        previousPolicy_ = policy;
    }
    else
    {
        if(openLoopPolicies.size()>0)
        {
            OMPL_INFORM("Weights were similar");
            std::random_device rd; // obtain a random number from hardware
            std::mt19937 eng(rd()); // seed the generator
            std::uniform_int_distribution<> distr(0, openLoopPolicies.size()-1); // define the range
            int rndp = distr(eng);
            policy = openLoopPolicies[rndp];
            //std::cout<<"Chose policy :"<<rndp<<std::endl;
            //std::cin.get();
        }
        else
        {
            policy = previousPolicy_;
            //std::cin.get();
        }

    }

}

ompl::base::Cost MMPolicyGenerator::executeOpenLoopPolicyOnMode(std::vector<ompl::control::Control*> controls,
                                                                const ompl::base::State* state)
{

    // Execute the open loop policy and get the information gain.
    // if the mode collides then add a cost of collision.
    ExtendedKF kf(si_);

    LinearSystem dummy;

    ompl::base::State *tempState = si_->allocState();

    ompl::base::State *nextState = si_->allocState();

    si_->copyState(tempState, state);

    si_->copyState(nextState, state);

    ompl::base::Cost olpInfGain;

    olpInfGain.v = 0;


    for(unsigned int i=0; i< controls.size() ; i++)
    {
        ompl::base::State *kfEstimate = si_->allocState();

        si_->copyState(kfEstimate,tempState);

        arma::colvec motionNoise = si_->getMotionModel()->getZeroNoise();//->generateNoise(from, openLoopControls[i]);

        si_->getMotionModel()->Evolve(tempState, controls[i], motionNoise, nextState);

        si_->copyState(tempState, nextState);

        arma::colvec obs = si_->getObservationModel()->getObservation(nextState, false);

        double I_1 = arma::trace(kfEstimate->as<SE2BeliefSpace::StateType>()->getCovariance());

        kf.Evolve(kfEstimate, controls[i], obs, dummy, dummy, kfEstimate);

        double I_2 = arma::trace(kfEstimate->as<SE2BeliefSpace::StateType>()->getCovariance());

        olpInfGain.v += 1/I_2-1/I_1;

        if(!si_->isValid(nextState))
        {

            olpInfGain.v -= FAILURE_COST/(i+1); // add a high cost for collision, the sooner the robot collides, more the cost
            break;//return olpCost;
        }

    }

    return olpInfGain;

}


void MMPolicyGenerator::propagateBeliefs(const ompl::control::Control *control)
{
    // To propagate beliefs we need to apply the control to the true state, get observations and update the beliefs.
    ExtendedKF kf(si_);

    LinearSystem dummy;

    arma::colvec obs = si_->getObservation();

    for(int i = 0; i < currentBeliefStates_.size(); i++)
    {
        ompl::base::State *kfEstimate = si_->allocState();

        si_->copyState(kfEstimate, currentBeliefStates_[i]);

        kf.Evolve(kfEstimate, control, obs, dummy, dummy, kfEstimate);

        si_->copyState(currentBeliefStates_[i], kfEstimate);

        si_->freeState(kfEstimate);

    }

    Visualizer::clearStates();

    for(int i = 0; i < currentBeliefStates_.size(); i++)
    {
        Visualizer::addState(currentBeliefStates_[i]);
    }

}

bool MMPolicyGenerator::areSimilarWeights()
{

    arma::colvec w(weights_.size(),1);
    for(int i = 0; i < weights_.size(); i++)
    {
        w[i] = weights_[i];
        //std::cout<<"Weight is:"<<w[i]<<std::endl;
    }
    double sd = arma::stddev(w);

    if(sd < 1e-6)
    {
        return true;
    }

    return false;
}

void MMPolicyGenerator::updateWeights()
{
    // true obs
    arma::colvec trueObs = si_->getObservation();

    arma::colvec sigma = si_->getObservationModel()->sigma_ ;

    arma::colvec variance = arma::pow(sigma,2);

    float totalWeight = 0.0;

    for(int i = 0; i < currentBeliefStates_.size(); i++)
    {
        // compute the innovation
        arma::colvec beliefObservation =  si_->getObservationModel()->getObservation(currentBeliefStates_[i], false);
        arma::colvec innov = this->computeInnovation(beliefObservation, trueObs);

        ompl::base::State* ts = si_->allocState();
        si_->getTrueState(ts);

        arma::mat covariance = arma::diagmat(arma::repmat(arma::diagmat(variance), innov.n_rows/2, innov.n_rows/2)) ;//innov.n_rows, innov.n_rows);

        arma::mat t = -0.5*trans(innov)*covariance.i()*innov;

        float w = std::pow(2.71828, t(0,0));//std::exp(t(0,0));

        totalWeight += weights_[i]*w;

        weights_[i]  = weights_[i]*w;

    }

    // if totalWeight becomes 0, reset to uniform distribution
    if(totalWeight==0)
    {
        for(int i=0; i< weights_.size(); i++)
        {
            weights_[i] = 1.0/weights_.size();
        }
        return;
    }

    // Normalize the weights
     for(int i = 0; i < weights_.size(); i++)
    {
        if(totalWeight!=0)
            weights_[i] =  weights_[i]/totalWeight;
    }
      for(int i = 0; i < weights_.size(); i++)
    {
        //if(weights_[i] < 1e-6 )
            //this->removeBelief(i);
    }

}

arma::colvec MMPolicyGenerator::computeInnovation(const arma::colvec Zprd, const arma::colvec Zg)
{
    const int singleObservationDim = 4;//CamAruco2DObservationModel::singleObservationDim;

    const int landmarkInfoDim = 2;//CamAruco2DObservationModel::landmarkInfoDim;

    int greaterRows = Zg.n_rows >= Zprd.n_rows ? Zg.n_rows : Zprd.n_rows ;

    arma::colvec innov( (landmarkInfoDim)* greaterRows /singleObservationDim ) ;

    for(unsigned int i =0; i< greaterRows/singleObservationDim ; i++)
    {

        if(Zg.n_rows >= (i+1)*singleObservationDim && Zprd.n_rows >= (i+1)*singleObservationDim)
        {

            innov( i*(landmarkInfoDim) ) = Zg(i*singleObservationDim + 1) - Zprd(i*singleObservationDim + 1) ;

            double delta_theta = Zg(i*singleObservationDim + 2) - Zprd(i*singleObservationDim + 2) ;

            if(delta_theta > boost::math::constants::pi<double>() ) {
              delta_theta = delta_theta - 2*boost::math::constants::pi<double>() ;
            }

            if( delta_theta < -boost::math::constants::pi<double>() ){
              delta_theta =  delta_theta + 2*boost::math::constants::pi<double>() ;
            }

            innov( i*(landmarkInfoDim) + 1 ) =  delta_theta;
        }
        else
        {
            if(Zg.n_rows >= (i+1)*singleObservationDim)
            {
                 innov( i*(landmarkInfoDim) ) = Zg(i*singleObservationDim + 1);
                 innov( i*(landmarkInfoDim) + 1 ) =  Zg(i*singleObservationDim + 2);
            }
            else
            {
                innov( i*(landmarkInfoDim) ) = -Zprd(i*singleObservationDim + 1);
                innov( i*(landmarkInfoDim) + 1 ) =  -Zprd(i*singleObservationDim + 2);
            }
        }

    }
    if(innov.n_rows==0)
    {
        innov = arma::zeros<arma::colvec>(2);

    }

    return innov;
}

void MMPolicyGenerator::removeBelief(const int Indx)
{
    currentBeliefStates_.erase(currentBeliefStates_.begin(),currentBeliefStates_.begin()+Indx);
    weights_.erase(weights_.begin(),weights_.begin()+Indx);
    targetStates_.erase(targetStates_.begin(),targetStates_.begin()+Indx);
}
