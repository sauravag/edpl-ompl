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

/* Author: Saurav Agarwal */

#include "../../include/Planner/MMPolicyGenerator.h"
#include <boost/thread.hpp>
#include "../../include/Visualization/Visualizer.h"
#include "../../include/Utils/FIRMUtils.h"
#include <boost/foreach.hpp>
#include "../../include/ObservationModels/CamAruco2DObservationModel.h"
#include "../../include/LinearSystem/LinearSystem.h"
#include <numeric>

#define foreach BOOST_FOREACH
#define foreach_reverse BOOST_REVERSE_FOREACH


namespace ompl
{
    namespace magic
    {
        static const double COLISSION_FAILURE_COST  = 1e6;// cost of colliding

        static const double RRT_PLAN_MAX_TIME = 1.0; // maximum time allowed for RRT to plan

        static const double RRT_FINAL_PROXIMITY_THRESHOLD = 1.0; // maximum distance for RRT to succeed

        static const double NEIGHBORHOOD_RANGE = 15.0 ; // range within which to find neighbors

        static const float MIN_ROBOT_CLEARANCE = 0.30;

        static const double SIGMA_RANGE = 0.5;// meters

        static const double SIGMA_THETA = 0.2; // radians

        static const double MODE_DELETION_THRESHOLD = 1e-4; // the percentage of weight a mode should hold, below which it gets deleted

        static const double ZERO_CONTROL_UPDATE_TIME = 1.0 ;

        static const double SAMPLING_ROTATION_SPACING = 5.0; // degrees

        static const double SAMPLING_GRID_SIZE = 0.50 ; // meters
    }
}

void MMPolicyGenerator::sampleNewBeliefStates()
{
    /**
    Logic:
    1. Grid the environment
    2. At each point in grid, put a robot with periodic orientations (10 degree spacing)
    */
    //Make sure the current states and weights are cleared out
    currentBeliefStates_.clear();
    weights_.clear();

    ompl::base::StateSpacePtr sp = si_->getStateSpace();
    ompl::base::RealVectorBounds bounds = sp->as<SE2BeliefSpace>()->getBounds();

    //Get the environment boundaries
    double X_1 = bounds.low[0];
    double X_2 = bounds.high[0];
    double Y_1 = bounds.low[1];
    double Y_2 = bounds.high[1];

    double spacing = ompl::magic::SAMPLING_GRID_SIZE;

    // grid size
    int gridSizeX = std::ceil( (X_2-X_1) / spacing);
    int gridSizeY = std::ceil( (Y_2-Y_1) / spacing);

    double rotationSpacing = FIRMUtils::degree2Radian(ompl::magic::SAMPLING_ROTATION_SPACING);// radians

    int numHeadings = std::floor(2*boost::math::constants::pi<double>()/rotationSpacing);

    arma::mat cov = arma::eye(3,3);
    cov(0,0) = 0.25;
    cov(1,1) = 0.25;
    cov(2,2) = 0.01;

    OMPL_INFORM("MMPolicyGenerator: Sampling start beliefs");


    for(int i = 0; i <= gridSizeX; i++)
    {
        for(int j=0; j <= gridSizeY; j++)
        {
            double newX = X_1 + i*spacing;
            double newY = Y_1 + j*spacing;

            for(int k =0; k < numHeadings; k++ )
            {
                double newYaw = -boost::math::constants::pi<double>() + k*rotationSpacing;

                ompl::base::State *newState = si_->allocState();

                newState->as<SE2BeliefSpace::StateType>()->setXYYaw(newX, newY, newYaw);
                newState->as<SE2BeliefSpace::StateType>()->setCovariance(cov);

                if(si_->isValid(newState))
                {
                    currentBeliefStates_.push_back(si_->cloneState(newState));
                    weights_.push_back(0.0); // push zero weight
                    timeSinceDivergence_.push_back(0.0);
                }
                si_->freeState(newState);
            }
        }
    }

    OMPL_INFORM("MMPolicyGenerator: Sampling beliefs Completed");

    assignUniformWeight();

    OMPL_INFORM("MMPolicyGenerator: Sample Belief Weights Added");

    arma::colvec obs = si_->getObservation();

    OMPL_INFORM("MMPolicyGenerator: Updating the weights before proceeding");

    unsigned int numModes = currentBeliefStates_.size();
    unsigned int updatedNumModes = 0;

    // We will update weights till we converge to most likely modes
    while( numModes != updatedNumModes )
    {
        numModes = currentBeliefStates_.size();

        this->updateWeights(obs);

        updatedNumModes = currentBeliefStates_.size();

    }

    // show the beliefs
    this->drawBeliefs();

    this->printWeights();

    // After converging to most likely, assign uniform weights
    assignUniformWeight();

    double t = 0;

    // Run filter for a few seconds to update initial samples, zero control
    while( t < ompl::magic::ZERO_CONTROL_UPDATE_TIME )
    {

        this->propagateBeliefs(si_->getMotionModel()->getZeroControl());

        t += si_->getMotionModel()->getTimestepSize();

    }

    // Before beginning the strategy, assign all modes the same weight
    assignUniformWeight();

    this->printWeights();

    // Print the states
    OMPL_INFORM("MMPolicyGenerator: The final sampled beliefs are :");
    for(int i=0; i< currentBeliefStates_.size(); i++)
    {
        si_->printState(currentBeliefStates_[i]);
    }

}



void MMPolicyGenerator::generatePolicy(std::vector<ompl::control::Control*> &policy)
{

    this->printWeights();

    //container to store the sequence of controls for each mode/target pair
    std::vector<std::vector<ompl::control::Control*> > openLoopPolicies;

    Visualizer::ClearFeedbackEdges();

    // Iterate over the mode/target pairs and generate open loop controls
    for(unsigned int i = 0; i < currentBeliefStates_.size(); i++)
    {

        //Generate a path for the mode/target pair
        if( si_->isValid(currentBeliefStates_[i]) )
        {

            ompl::base::PlannerPtr planner(new ompl::geometric::RRT(si_));

            ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si_));

            Vertex targetVertex = findTarget(i);

            //------ To check if target makes sense
            Visualizer::addFeedbackEdge(currentBeliefStates_[i],stateProperty_[targetVertex], 1.0);
            //------

            pdef->setStartAndGoalStates(currentBeliefStates_[i], stateProperty_[targetVertex], ompl::magic::RRT_FINAL_PROXIMITY_THRESHOLD);

            planner->as<ompl::geometric::RRT>()->setRange(1.0);

            planner->setProblemDefinition(pdef);

            planner->setup();


            ompl::base::PlannerStatus solved = planner->solve(ompl::magic::RRT_PLAN_MAX_TIME);


            if(solved)
            {
                const ompl::base::PathPtr &path = pdef->getSolutionPath();

                ompl::geometric::PathGeometric gpath = static_cast<ompl::geometric::PathGeometric&>(*path);

                std::vector<ompl::control::Control*> olc;

                si_->getMotionModel()->generateOpenLoopControlsForPath(gpath, olc);

                openLoopPolicies.push_back(olc);

            }

            planner->clear();
        }
    }

    // Now that you have the open loop controls, need to execute them on all modes to see which is best
    std::vector<ompl::base::Cost> policyInfGains;

    double maxGain = -1e10;

    int maxGainPolicyIndx=-1;

    OMPL_INFORM("Evaluating the Open Loop policies on all the modes");

    for(unsigned int i = 0; i < openLoopPolicies.size(); i++)
    {
        ompl::base::Cost pGain;

        pGain.v = 0;

        for(unsigned int j = 0; j < currentBeliefStates_.size(); j++)
        {

            ompl::base::Cost c = executeOpenLoopPolicyOnMode(openLoopPolicies[i],currentBeliefStates_[j]);

            pGain.v += c.v;

            OMPL_INFORM("MMPolicyGenerator: Evaluating Policy Number #%u  on Mode Number #%u",i,j);
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
        OMPL_INFORM("MMPolicyGenerator: A minimum cost policy was found");

        policy = openLoopPolicies[maxGainPolicyIndx];

        previousPolicy_ = policy;
    }
    else
    {
        if(openLoopPolicies.size()>0)
        {
            int rndp = FIRMUtils::generateRandomIntegerInRange(0, openLoopPolicies.size()-1);

            policy = openLoopPolicies[rndp];

        }
        else
        {
            policy = previousPolicy_;
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
 ;
    ompl::base::State *nextState = si_->allocState();

    si_->copyState(tempState, state);

    si_->copyState(nextState, state);

    ompl::base::Cost olpInfGain;

    olpInfGain.v = 0;


    for(unsigned int i=0; i< controls.size() ; i++)
    {
        ompl::base::State *kfEstimate = si_->allocState();

        si_->copyState(kfEstimate,tempState);

        arma::colvec motionNoise = si_->getMotionModel()->getZeroNoise();

        si_->getMotionModel()->Evolve(tempState, controls[i], motionNoise, nextState);

        si_->copyState(tempState, nextState);

        arma::colvec obs = si_->getObservationModel()->getObservation(nextState, false);

        double I_1 = arma::trace(kfEstimate->as<SE2BeliefSpace::StateType>()->getCovariance());

        kf.Evolve(kfEstimate, controls[i], obs, dummy, dummy, kfEstimate);

        double I_2 = arma::trace(kfEstimate->as<SE2BeliefSpace::StateType>()->getCovariance());

        olpInfGain.v += 1/I_2-1/I_1;

        if(!si_->isValid(nextState))
        {

            olpInfGain.v -= ompl::magic::COLISSION_FAILURE_COST/(i+1); // add a high cost for collision, the sooner the robot collides, more the cost
            break;
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

    for(unsigned int i = 0; i < currentBeliefStates_.size(); i++)
    {
        ompl::base::State *kfEstimate = si_->allocState();
        ompl::base::State *kfEstimateUpdated = si_->allocState();

        //si_->printState(currentBeliefStates_[i]);

        si_->copyState(kfEstimate, currentBeliefStates_[i]);

        kf.Evolve(kfEstimate, control, obs, dummy, dummy, kfEstimateUpdated);

        si_->copyState(currentBeliefStates_[i], kfEstimateUpdated);

        si_->freeState(kfEstimate);

        si_->freeState(kfEstimateUpdated);

    }

    OMPL_INFORM("MMPolicyGenerator: BEFORE updating weights in propogate: ");
    this->printWeights();

    this->updateWeights(obs);

    OMPL_INFORM("MMPolicyGenerator: AFTER updating weights in propogate: ");
    this->printWeights();

    this->removeDuplicateModes();

    this->drawBeliefs();

}

bool MMPolicyGenerator::areSimilarWeights()
{

    arma::colvec w(weights_.size(),1);

    for(unsigned int i = 0; i < weights_.size(); i++)
    {
        w[i] = weights_[i];
    }

    double sd = arma::stddev(w);

    if(sd < 1e-6)
    {
        return true;
    }

    return false;
}

void MMPolicyGenerator::updateWeights(const arma::colvec trueObservation)
{

    arma::colvec sigma(2);

    sigma(0) = ompl::magic::SIGMA_RANGE;
    sigma(1) = ompl::magic::SIGMA_THETA;

    arma::colvec variance = arma::pow(sigma,2);

    float totalWeight = 0.0;

    for(unsigned int i = 0; i < currentBeliefStates_.size(); i++)
    {

        double weightFactor= 1.0;

        arma::colvec innov = this->computeInnovation(i,trueObservation,weightFactor);

        float w;

        // robot and mode see something in common
        if(innov.n_rows != 0)
        {
            arma::mat covariance = arma::diagmat(arma::repmat(arma::diagmat(variance), innov.n_rows/2, innov.n_rows/2)) ;

            arma::mat t = -0.5*trans(innov)*covariance.i()*innov;

            w = std::exp(t(0,0)) * weightFactor;

        }
        // if there is no innov it means there is no common observation
        else
        {
            // case 1: robot also doesnt see anything
            if(trueObservation.n_rows ==0)
            {
                // mode could be seeing something
                w = weightFactor;
            }
            // robot is seeing something and mode sees nothing in common
            else
            {
                w = 0;
            }
        }

        weights_[i]  = weights_[i]*w;

        totalWeight += weights_[i];

    }

    // if totalWeight becomes 0, reset to uniform distribution
    if(totalWeight==0)
    {
        for(unsigned int i=0; i< weights_.size(); i++)
        {
            weights_[i] = 1.0/weights_.size();
        }
        return;
    }
    else
    {
        normalizeWeights();

    }

    std::vector<int> beliefsToRemove;

    for(unsigned int i = 0; i < weights_.size(); i++)
    {
        // if the weight of the mode is less than threshold, delete it
        if(weights_[i]/totalWeight < ompl::magic::MODE_DELETION_THRESHOLD || weights_[i] == 0.0 )
        {
            beliefsToRemove.push_back(i);
        }
    }

    removeBeliefs(beliefsToRemove);

}

/*
// Alternate implementation of calculating weight update
float MMPolicyGenerator::computeWeightForMode(const int currentBeliefIndx,const arma::colvec trueObservation)
{
    const int singleObservationDim = 4;//CamAruco2DObservationModel::singleObservationDim;

    const int landmarkInfoDim = 2;//CamAruco2DObservationModel::landmarkInfoDim;

    // the true observation
    arma::colvec Zg = trueObservation;

    int landmarksActuallySeen = Zg.n_rows / singleObservationDim;

    // the belief's predicted observation for what the robot sees
    arma::colvec Zprd =  si_->getObservationModel()->getObservationPrediction(currentBeliefStates_[currentBeliefIndx], Zg);

    // get observation
    arma::colvec innov = si_->getObservationModel()->computeInnovation(currentBeliefStates_[currentBeliefIndx], obs);

    float weightUpdate = 1.0;

    // robot and mode see something in common
    if(innov.n_rows != 0)
    {
        // construct a linear system at where the mode is
        LinearSystem ls(si_,currentBeliefStates_[currentBeliefIndx],si_->getMotionModel()->getZeroControl(), Zprd, si_->getMotionModel(), si_->getObservationModel());

        arma::mat P = currentBeliefStates_[currentBeliefIndx]->as<SE2BeliefSpace::StateType>()->getCovariance();

        mat obsLikelihood = ls.getH() * P * trans(ls.getH()) + ls.getR();

        arma::mat t = -0.5*trans(innov)*obsLikelihood.i()*innov;

        weightUpdate = t(0,0);

    }
    // if there is no innov it means there is no common observation
    else
    {
        // case 1: robot also doesnt see anything
        if(Zg.n_rows ==0)
        {
            // mode could be seeing something
            weightUpdate = 1.0;
        }
        // robot is seeing something and mode sees nothing in common
        else
        {
            weightUpdate = 0;
        }
    }



     // the beliefs predicted observation for what is should see based on where it is
    arma::colvec Zmode =  si_->getObservationModel()->getObservation(currentBeliefStates_[currentBeliefIndx], false);

    int numIntersection=0;

    int predictedLandmarksSeen = Zmode.n_rows / singleObservationDim ;


    for(int j = 0 ; j < landmarksActuallySeen ; j++)
    {
        // match ids
        for(int k = 0 ; k < predictedLandmarksSeen ; k++)
        {
            if(Zprd(k*singleObservationDim) == Zg(j*singleObservationDim))
            {

                numIntersection++;

                break;
            }
        }

    }

    float weightReductionFactor=1.0;

    // there is a mismatch in what the robot sees and predicted
    if(numIntersection != landmarksActuallySeen || numIntersection != predictedLandmarksSeen )
    {

        //weightFactor = std::min(1.0 / abs(1 + landmarksActuallySeen - numIntersection) , 1.0 / abs(1 + predictedLandmarksSeen - numIntersection));
        float heuristicVal = std::max(abs(1 + landmarksActuallySeen - numIntersection) , abs(1 + predictedLandmarksSeen - numIntersection));

        weightReductionFactor = std::exp(-heuristicVal*timeSinceDivergence_[currentBeliefIndx]);

        timeSinceDivergence_[currentBeliefIndx] = timeSinceDivergence_[currentBeliefIndx] + std::pow(si_->getMotionModel()->getTimestepSize(),2);

    }
    else
    {
         timeSinceDivergence_[currentBeliefIndx] = timeSinceDivergence_[currentBeliefIndx] - 1.0;

         if(timeSinceDivergence_[currentBeliefIndx] < 0)
         {
            timeSinceDivergence_[currentBeliefIndx] = 0;
         }
    }

    return weightUpdate*weightReductionFactor;

}
*/


arma::colvec MMPolicyGenerator::computeInnovation(const int currentBeliefIndx,const arma::colvec trueObservation, double &weightFactor)
{
    const int singleObservationDim = 4;//CamAruco2DObservationModel::singleObservationDim;

    const int landmarkInfoDim = 2;//CamAruco2DObservationModel::landmarkInfoDim;

    // the true observation
    arma::colvec Zg = trueObservation;

    int landmarksActuallySeen = Zg.n_rows / singleObservationDim;

    // the beliefs predicted observation
    arma::colvec Zprd =  si_->getObservationModel()->getObservation(currentBeliefStates_[currentBeliefIndx], false);

    int predictedLandmarksSeen = Zprd.n_rows / singleObservationDim ;

    arma::colvec innov;

    int numIntersection=0;

    //First we see if a landmark that is observed by robot is seen by mode:
    //1. If yes, then we compute the innov
    //2. If no, then we predict where the mode "would" see that particular landmark and then calculate the innov
    for(int j = 0 ; j < landmarksActuallySeen ; j++)
    {
        // match ids
        for(int k = 0 ; k < predictedLandmarksSeen ; k++)
        {
            if(Zprd(k*singleObservationDim) == Zg(j*singleObservationDim))
            {

                arma::colvec innov_for_landmark(2);

                innov_for_landmark(0) = Zg(j*singleObservationDim + 1) - Zprd(k*singleObservationDim + 1) ;

                double delta_theta = Zg(j*singleObservationDim + 2) - Zprd(k*singleObservationDim + 2) ;

                FIRMUtils::normalizeAngleToPiRange(delta_theta);

                assert(abs(delta_theta) <= 2*boost::math::constants::pi<double>()) ;

                innov_for_landmark(1) =  delta_theta;

                innov.insert_rows(numIntersection*landmarkInfoDim, innov_for_landmark);

                numIntersection++;

                break;
            }
        }

    }

    // there is a mismatch in what the robot sees and predicted
    if(numIntersection != landmarksActuallySeen || numIntersection != predictedLandmarksSeen )
    {

        //weightFactor = std::min(1.0 / abs(1 + landmarksActuallySeen - numIntersection) , 1.0 / abs(1 + predictedLandmarksSeen - numIntersection));
        float heuristicVal = std::max(abs(1 + landmarksActuallySeen - numIntersection) , abs(1 + predictedLandmarksSeen - numIntersection));

        weightFactor = std::exp(-heuristicVal*timeSinceDivergence_[currentBeliefIndx]);

        timeSinceDivergence_[currentBeliefIndx] = timeSinceDivergence_[currentBeliefIndx] + std::pow(si_->getMotionModel()->getTimestepSize(),2);

    }
    else
    {
         timeSinceDivergence_[currentBeliefIndx] = timeSinceDivergence_[currentBeliefIndx] - 1.0;

         if(timeSinceDivergence_[currentBeliefIndx] < 0)
         {
            timeSinceDivergence_[currentBeliefIndx] = 0;
         }
    }


    return innov;
}

void MMPolicyGenerator::removeBeliefs(const std::vector<int> Indxs)
{
    // Make a local copy
    std::vector<ompl::base::State*> currentBeliefStatesCopy = currentBeliefStates_;
    std::vector<float> weightsCopy = weights_;
    std::vector<double>  timeSinceDivergenceCopy = timeSinceDivergence_;

    weights_.clear();
    currentBeliefStates_.clear();
    timeSinceDivergence_.clear();

    for(int i = 0 ; i < currentBeliefStatesCopy.size(); i++)
    {
        // check if this index is in the list marked to be deleted. If not, then we keep it
        std::vector<int>::const_iterator it = std::find(Indxs.begin(), Indxs.end(), i) ;

        if(it == Indxs.end())
        {
            currentBeliefStates_.push_back(currentBeliefStatesCopy[i]);
            weights_.push_back(weightsCopy[i]);
            timeSinceDivergence_.push_back(timeSinceDivergenceCopy[i]);
        }
        // if it was marked to be deleted, then we free memory for that state
        else
        {
            si_->freeState(currentBeliefStatesCopy[i]);
        }
    }

    normalizeWeights();

}


bool MMPolicyGenerator::isConverged()
{

    if(currentBeliefStates_.size()==1)
    {
        return true;
    }

    return false;
}


 bool MMPolicyGenerator::areCurrentBeliefsValid()
{
    if(currentBeliefStates_.size()==1)
        return true;

    for(int i =0 ; i< currentBeliefStates_.size(); i++)
    {
        if(si_->getStateValidityChecker()->clearance(currentBeliefStates_[i]) < ompl::magic::MIN_ROBOT_CLEARANCE)
            return false;

    }

    return true;
}

void MMPolicyGenerator::removeDuplicateModes()
{
    std::vector<int> toDelete;

    for(int i = 0; i < currentBeliefStates_.size(); i++)
    {
        for(int j = i+1; j < currentBeliefStates_.size(); j++ )
        {
            if(i!=j)
            {
                arma::colvec xi = currentBeliefStates_[i]->as<SE2BeliefSpace::StateType>()->getArmaData();
                arma::colvec xj = currentBeliefStates_[j]->as<SE2BeliefSpace::StateType>()->getArmaData();

                double xd = xi(0) - xj(0);
                double yd = xi(1) - xj(1);
                double thetad = xi(2) - xj(2);

                if(std::abs(xd) < 0.01 && std::abs(yd) < 0.01 &&  std::abs(thetad) < FIRMUtils::degree2Radian(1.0) )
                {

                    if(weights_[i] >= weights_[j] )
                    {
                        toDelete.push_back(j);
                        weights_[i] = weights_[i] + weights_[j]; // transfer the weight to the more likely mode
                    }
                    else
                    {
                        toDelete.push_back(i);
                        weights_[j] = weights_[i] + weights_[j]; // transfer the weight to the more likely mode

                    }

                }
            }

        }
    }

    this->removeBeliefs(toDelete);

}


void MMPolicyGenerator::drawBeliefs()
{

    Visualizer::clearStates();

    for(unsigned int i = 0; i < currentBeliefStates_.size(); i++)
    {
        Visualizer::addState(currentBeliefStates_[i]);
    }

}

void MMPolicyGenerator::getStateWithMaxWeight(ompl::base::State *state, float &weight)
{

    std::vector<float>::iterator biggest = std::max_element(weights_.begin(), weights_.end());

    weight = *biggest;

    int index = std::distance(weights_.begin(),biggest);

    si_->copyState(state, currentBeliefStates_[index]);

}


void MMPolicyGenerator::normalizeWeights()
{
    float totalWeight = std::accumulate(weights_.begin(), weights_.end(), 0.0);

    // Normalize the weights
     for(unsigned int i = 0; i < weights_.size(); i++)
    {
        weights_[i] =  weights_[i]/totalWeight;

    }
}

void MMPolicyGenerator::assignUniformWeight()
{
    int Nm = weights_.size();

    float Wu = 1.0 / Nm;

    for(unsigned int i = 0; i < Nm ; i++)
    {
        weights_[i] =  Wu;

    }

}

void MMPolicyGenerator::addStateToObservationGraph(ompl::base::State *state)
{
    //boost::mutex::scoped_lock _(graphMutex_);

    // Add state to graph
    Vertex m = boost::add_vertex(g_);

    stateProperty_[m] = state;

    // Iterate over existing nodes and add edges
    foreach (Vertex n, boost::vertices(g_))
    {
        if ( m!=n && stateProperty_[m]!=stateProperty_[n])
        {
            addEdgeToObservationGraph(m,n);
        }
    }
}


void MMPolicyGenerator::addEdgeToObservationGraph(const Vertex a, const Vertex b)
{
    //boost::mutex::scoped_lock _(graphMutex_);

    // See if there is an overlap in observation between the two vertices
    unsigned int weight = 0;

    if(getObservationOverlap(a, b, weight))
    {
        // Add edge if overlap true, with weight = number of overlaps
        const unsigned int id = maxEdgeID_++;

        const Graph::edge_property_type properties(weight, id);

        // create an edge with the edge weight property
        std::pair<Edge, bool> newEdge = boost::add_edge(a, b, properties, g_);

    }

}

void MMPolicyGenerator::evaluateObservationListForVertex(const Vertex v)
{
    // get Zero Noise Observation
    arma::colvec obs = si_->getObservationModel()->getObservation(stateProperty_[v], false);

    unsigned int singleObsSize = CamAruco2DObservationModel::singleObservationDim;

    std::vector<unsigned int> obsList;

    for(int i = 0; i < obs.n_rows/singleObsSize; i++)
    {
        obsList.push_back(obs[singleObsSize*i]);
    }

    stateObservationProperty_[v] = obsList;
}

bool MMPolicyGenerator::getObservationOverlap(Vertex a, Vertex b, unsigned int &weight)
{

    // get the list of observations for both vertices
    evaluateObservationListForVertex(a);

    evaluateObservationListForVertex(b);


    bool isOverlapping = false;

    weight = 0;

    // Check if there is an overlap
    for(int i = 0; i < stateObservationProperty_[a].size(); i++)
    {
        for(int j= 0; j < stateObservationProperty_[b].size(); j++)
        {
            if(stateObservationProperty_[a][i] == stateObservationProperty_[b][j])
            {
                // for every overlap, increase weight
                weight++;

                isOverlapping = true;

            }
        }
    }

    return isOverlapping;
}

std::vector<MMPolicyGenerator::Vertex> MMPolicyGenerator::getNeighbors(const ompl::base::State *state)
{
    std::vector<Vertex> nn;

    foreach(Vertex v, boost::vertices(g_))
    {
        if(si_->distance(state, stateProperty_[v]) <  ompl::magic::NEIGHBORHOOD_RANGE)
        {
            nn.push_back(v);
        }
    }

    return nn;
}

MMPolicyGenerator::Vertex MMPolicyGenerator::findTarget(const unsigned int beliefStateIndx)
{
    // mapping from index of belief state to its neighbors
    std::map<unsigned int, std::vector<Vertex> > setOfAllNeighbors;

    // find the neighbors of all the belief States
    for(int i=0; i < currentBeliefStates_.size(); i++)
    {
        std::vector<Vertex> nn;

        nn = this->getNeighbors(currentBeliefStates_[i]);

        setOfAllNeighbors[i] = nn; // insert in mapping
    }

    std::vector<Vertex> neighborsOfBelief = setOfAllNeighbors[beliefStateIndx];

     // find the node in the required belief's neighbors that is least similar to nodes in other neighboorhoods
    int targetNodeIndx = -1;
    int minWeight = 1e6;

    // iterate over the nodes in the required belief's neighborhood;
    for(int i = 0; i < neighborsOfBelief.size(); i++)
    {
        int w = 0;

        // iterate over all other neighborhoods
        for(int j = 0; j < setOfAllNeighbors.size(); j++)
        {

            if(j != beliefStateIndx)
            {
                w += calculateIntersectionWithNeighbor(neighborsOfBelief[i],  setOfAllNeighbors[j]);

            }
        }

        //TODO: If 2 targets have same weight, then choose the closer one
        if(w < minWeight)
        {
            minWeight = w;
            targetNodeIndx = i;
        }

    }

    assert(targetNodeIndx >= 0);

    return neighborsOfBelief[targetNodeIndx];
}

int MMPolicyGenerator::calculateIntersectionWithNeighbor(const Vertex v, std::vector<Vertex> neighbors)
{
    int w = 0;

    foreach(Edge e, boost::out_edges(v, g_))
    {
        for(int i = 0; i < neighbors.size(); i++)
        {
            if(neighbors[i] == boost::target(e, g_))
            {
                // then get the edge weight
                int edgeWeight =  boost::get(boost::edge_weight, g_, e);

                w += edgeWeight;
            }
        }
    }

    return w;
}
