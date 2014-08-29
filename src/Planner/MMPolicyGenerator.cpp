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
#include "../../include/Utils/FIRMUtils.h"
#include <boost/foreach.hpp>
#include "../../include/ObservationModels/CamAruco2DObservationModel.h"

#define foreach BOOST_FOREACH
#define foreach_reverse BOOST_REVERSE_FOREACH

#define WEIGHT_UPDATE_COVARIANCE_MAGNIFIER 10.0
namespace ompl
{
    namespace magic
    {
        static const double COLISSION_FAILURE_COST  = 1e6;// cost of colliding

        static const double RRT_PLAN_MAX_TIME = 2.0; // maximum time allowed for RRT to plan

        static const double RRT_FINAL_PROXIMITY_THRESHOLD = 1.0; // maximum distance for RRT to succeed

        static const double NEIGHBORHOOD_RANGE = 10.0 ; // range within which to find neighbors
    }
}

void MMPolicyGenerator::generatePolicy(std::vector<ompl::control::Control*> &policy)
{
    this->updateWeights();

    this->printWeights();

    // Make sure that each beliefstate has a target state
    //assert(currentBeliefStates_.size() == targetStates_.size());

    //container to store the sequence of controls for each mode/target pair
    std::vector<std::vector<ompl::control::Control*> > openLoopPolicies;

    // Iterate over the mode/target pairs and generate open loop controls
    for(unsigned int i = 0; i < currentBeliefStates_.size(); i++)
    {

        //Generate a path for the mode/target pair
        if(si_->isValid(currentBeliefStates_[i]) /*&& si_->distance(currentBeliefStates_[i], targetStates_[i])>0.50*/)
        {

            ompl::base::PlannerPtr planner(new ompl::geometric::RRT(si_));

            ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si_));

            std::cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<std::endl;
            std::cout<<"For belief :"<<std::endl;
            si_->printState(currentBeliefStates_[i]);

            Vertex targetVertex = findTarget(i);

            std::cout<<" Target :"<<std::endl;
            si_->printState(stateProperty_[targetVertex]);
            std::cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<std::endl;

            pdef->setStartAndGoalStates(currentBeliefStates_[i], stateProperty_[targetVertex]/*targetStates_[i]*/, ompl::magic::RRT_FINAL_PROXIMITY_THRESHOLD);

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

    for(unsigned int i = 0; i < openLoopPolicies.size(); i++)
    {
        ompl::base::Cost pGain;

        pGain.v = 0;

        for(unsigned int j = 0; j < currentBeliefStates_.size(); j++)
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
        OMPL_INFORM("A minimum cost policy was found");

        policy = openLoopPolicies[maxGainPolicyIndx];

        previousPolicy_ = policy;
    }
    else
    {
        if(openLoopPolicies.size()>0)
        {
            OMPL_INFORM("Weights were similar");

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

        si_->copyState(kfEstimate, currentBeliefStates_[i]);

        kf.Evolve(kfEstimate, control, obs, dummy, dummy, kfEstimate);

        si_->copyState(currentBeliefStates_[i], kfEstimate);

        si_->freeState(kfEstimate);

    }

    this->updateWeights();

    Visualizer::clearStates();

    for(unsigned int i = 0; i < currentBeliefStates_.size(); i++)
    {
        Visualizer::addState(currentBeliefStates_[i]);
    }


}

bool MMPolicyGenerator::areSimilarWeights()
{

    arma::colvec w(weights_.size(),1);

    for(unsigned int i = 0; i < weights_.size(); i++)
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

    arma::colvec sigma = WEIGHT_UPDATE_COVARIANCE_MAGNIFIER*si_->getObservationModel()->sigma_ ;

    arma::colvec variance = arma::pow(sigma,2);

    float totalWeight = 0.0;

    for(unsigned int i = 0; i < currentBeliefStates_.size(); i++)
    {
        std::cout<<"Index #"<<i<<std::endl;
        si_->printState(currentBeliefStates_[i]);
        // compute the innovation
        arma::colvec beliefObservation =  si_->getObservationModel()->getObservation(currentBeliefStates_[i], false);

        arma::colvec innov = this->computeInnovation(beliefObservation, trueObs);

        arma::mat covariance = arma::diagmat(arma::repmat(arma::diagmat(variance), innov.n_rows/2, innov.n_rows/2)) ;

        arma::mat t = -0.5*trans(innov)*covariance.i()*innov;

        //std::cout<<"innov at index #"<<i<<"   = "<<innov<<std::endl;
        //std::cout<<"t at index #"<<i<<"   = "<<t<<std::endl;

        float w = std::pow(2.71828, t(0,0));

        std::cout<<"The weight update multiplier at index #"<<i<<"   = "<<w<<std::endl;

        weights_[i]  = weights_[i]*w;

        totalWeight += weights_[i];

        //std::cout<<"(innov update) Weight at index #"<<i<<"   = "<<weights_[i]<<std::endl;

    }

    // if totalWeight becomes 0, reset to uniform distribution
    if(totalWeight==0)
    {
        for(unsigned int i=0; i< weights_.size(); i++)
        {
            //std::cout<<"(totalweight=0) Weight at index #"<<i<<"   = "<<weights_[i]<<std::endl;

            weights_[i] = 1.0/weights_.size();
        }
        return;
    }
    else
    {
        // Normalize the weights
         for(unsigned int i = 0; i < weights_.size(); i++)
        {
            weights_[i] =  weights_[i]/totalWeight;

            std::cout<<"(After Norm) Weight at index #"<<i<<"   = "<<weights_[i]<<std::endl;
        }

    }

    if(weights_[0]/totalWeight < 0.01 || weights_[1]/totalWeight < 0.01 )
    {
        OMPL_INFORM("Problem detected,mode 1 weight should not go to zero");
        //assert(weights_[0]!=0);
        std::cin.get();
    }

    for(unsigned int i = 0; i < weights_.size(); i++)
    {
        // if the weight of the mode is less than 1%, delete it
        if(weights_[i]/totalWeight < 0.01 )
            this->removeBelief(i);
    }

}

arma::colvec MMPolicyGenerator::computeInnovation(const arma::colvec Zprd, const arma::colvec Zg)
{
    const int singleObservationDim = 4;//CamAruco2DObservationModel::singleObservationDim;

    const int landmarkInfoDim = 2;//CamAruco2DObservationModel::landmarkInfoDim;

    int greaterRows = Zg.n_rows >= Zprd.n_rows ? Zg.n_rows : Zprd.n_rows ;

    arma::colvec innov;

    std::cout<<"Ground Obs:"<<Zg<<std::endl;
    std::cout<<"Predicted obs :"<<Zprd<<std::endl;

    //std::cin.get();
    //std::cout<<"Greater Rows :"<<greaterRows<<std::endl;

    innov  = arma::zeros<arma::colvec>( 2 + (landmarkInfoDim)* greaterRows /singleObservationDim ) ;

    for(unsigned int i =0; i< greaterRows/singleObservationDim ; i++)
    {

        if(Zg.n_rows >= (i+1)*singleObservationDim && Zprd.n_rows >= (i+1)*singleObservationDim)
        {

            innov( i*(landmarkInfoDim) ) = Zg(i*singleObservationDim + 1) - Zprd(i*singleObservationDim + 1) ;

            double delta_theta = Zg(i*singleObservationDim + 2) - Zprd(i*singleObservationDim + 2) ;

            FIRMUtils::normalizeAngleToPiRange(delta_theta);

            assert(abs(delta_theta) <= 2*boost::math::constants::pi<double>()) ;

            innov( i*(landmarkInfoDim) + 1 ) =  delta_theta;
        }
        else
        {
            if(Zg.n_rows >= (i+1)*singleObservationDim)
            {
                 innov( i*(landmarkInfoDim) ) = Zg(i*singleObservationDim + 1);
                 innov( i*(landmarkInfoDim) + 1 ) =  Zg(i*singleObservationDim + 2);
                 assert(abs(innov( i*(landmarkInfoDim) + 1 ) ) <= 2*boost::math::constants::pi<double>()) ;
            }

            else
            {
                innov( i*(landmarkInfoDim) ) = -Zprd(i*singleObservationDim + 1);
                innov( i*(landmarkInfoDim) + 1 ) =  -Zprd(i*singleObservationDim + 2);
            }


        }

    }

    std::cout<<"Innovation:\n" <<innov;
    //std::cin.get();
    return innov;
}

void MMPolicyGenerator::removeBelief(const int Indx)
{
    currentBeliefStates_.erase(currentBeliefStates_.begin()+Indx);

    weights_.erase(weights_.begin()+Indx);

    targetStates_.erase(targetStates_.begin()+Indx);
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
        //std::cout<<"Observations overlap"<<std::endl;
        //std::cin.get();
        // Add edge if overlap true, with weight = number of overlaps
        const unsigned int id = maxEdgeID_++;

        const Graph::edge_property_type properties(weight, id);

        // create an edge with the edge weight property
        std::pair<Edge, bool> newEdge = boost::add_edge(a, b, properties, g_);

        //std::cout<<"Added edge to ob graph"<<std::endl;

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
    //std::cout<<"Calculating observation overlap for: "<<std::endl;
    si_->printState(stateProperty_[a]);
    si_->printState(stateProperty_[b]);

    // get the list of observations for both vertices
    evaluateObservationListForVertex(a);

    evaluateObservationListForVertex(b);

    /*
    std::cout<<"Observations for a"<<std::endl;

    for(int i=0; i < stateObservationProperty_[a].size(); i++)
    {
        std::cout<<stateObservationProperty_[a][i]<<std::endl;
    }

    std::cout<<"Observations for b"<<std::endl;
    for(int i=0; i < stateObservationProperty_[b].size(); i++)
    {
        std::cout<<stateObservationProperty_[b][i]<<std::endl;
    }

    //std::cin.get();
    */

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

                //std::cout<<"Observation overlap"<<std::endl;
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

        //std::cout<<"My neighbor : "<<i<<std::endl;

        // iterate over all other neighborhoods
        for(int j = 0; j < setOfAllNeighbors.size(); j++)
        {

            if(j != beliefStateIndx)
            {
                w += calculateIntersectionWithNeighbor(neighborsOfBelief[i],  setOfAllNeighbors[j]);

            }
        }

        //std::cout<<"ob w :"<<w<<std::endl;
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

                //std::cout<<"Edge weight in og:"<<edgeWeight<<std::endl;
                w += edgeWeight;
            }
        }
    }

    return w;
}
