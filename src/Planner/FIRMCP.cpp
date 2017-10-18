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

/* Authors: Sung Kyun Kim, Ali-akbar Agha-mohammadi, Saurav Agarwal */

#include "Planner/FIRMCP.h"
#include "Visualization/Visualizer.h"
#include <boost/circular_buffer.hpp>
#include <tinyxml.h>

#define foreach BOOST_FOREACH
#define foreach_reverse BOOST_REVERSE_FOREACH

FIRMCP::FIRMCP(const firm::SpaceInformation::SpaceInformationPtr &si, bool debugMode)
    : FIRM(si, debugMode)
{
}

FIRMCP::~FIRMCP(void)
{
}

void FIRMCP::loadParametersFromFile(const std::string &pathToFile)
{
    // load parameters for FIRM
    FIRM::loadParametersFromFile(pathToFile);


    // load parameters for FIRMCP
    TiXmlDocument doc(pathToFile);
    bool loadOkay = doc.LoadFile();
    if( !loadOkay )
    {
        printf( "FIRMCP: Could not load setup file. Error='%s'. Exiting.\n", doc.ErrorDesc() );
        exit( 1 );
    }

    TiXmlNode* node = 0;
    TiXmlNode* child = 0;
    TiXmlElement* itemElement = 0;

    node = doc.FirstChild( "FIRMCP" );
    assert( node );


    child = node->FirstChild("numPOMCPParticles");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryIntAttribute("numPOMCPParticles", &numPOMCPParticles_);
    itemElement = 0;

    child = node->FirstChild("maxPOMCPDepth");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryIntAttribute("maxPOMCPDepth", &maxPOMCPDepth_);
    itemElement = 0;

    child = node->FirstChild("maxFIRMReachDepth");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryIntAttribute("maxFIRMReachDepth", &maxFIRMReachDepth_);
    itemElement = 0;

    child = node->FirstChild("nSigmaForPOMCPParticle");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("nSigmaForPOMCPParticle", &nSigmaForPOMCPParticle_);
    itemElement = 0;

    child = node->FirstChild("cExplorationForSimulate");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("cExplorationForSimulate", &cExplorationForSimulate_);
    itemElement = 0;

    child = node->FirstChild("cExploitationForRolloutOutOfReach");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("cExploitationForRolloutOutOfReach", &cExploitationForRolloutOutOfReach_);
    itemElement = 0;

    child = node->FirstChild("cExploitationForRolloutWithinReach");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("cExploitationForRolloutWithinReach", &cExploitationForRolloutWithinReach_);
    itemElement = 0;

    child = node->FirstChild("costToGoRegulatorOutOfReach");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("costToGoRegulatorOutOfReach", &costToGoRegulatorOutOfReach_);
    itemElement = 0;

    child = node->FirstChild("costToGoRegulatorWithinReach");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("costToGoRegulatorWithinReach", &costToGoRegulatorWithinReach_);
    itemElement = 0;

    child = node->FirstChild("nEpsilonForRolloutIsReached");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("nEpsilonForRolloutIsReached", &nEpsilonForRolloutIsReached_);
    itemElement = 0;

    child = node->FirstChild("heurPosStepSize");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("heurPosStepSize", &heurPosStepSize_);
    itemElement = 0;

    child = node->FirstChild("heurOriStepSize");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("heurOriStepSize", &heurOriStepSize_);
    itemElement = 0;

    child = node->FirstChild("heurCovStepSize");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("heurCovStepSize", &heurCovStepSize_);
    itemElement = 0;

    child = node->FirstChild("covConvergenceRate");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("covConvergenceRate", &covConvergenceRate_);
    itemElement = 0;

    child = node->FirstChild("scaleStabNumSteps");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryIntAttribute("scaleStabNumSteps", &scaleStabNumSteps_);
    itemElement = 0;

    child = node->FirstChild("nEpsilonForQVnodeMerging");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("nEpsilonForQVnodeMerging", &nEpsilonForQVnodeMerging_);
    itemElement = 0;

    child = node->FirstChild("inflationForApproxStabCost");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryIntAttribute("inflationForApproxStabCost", &inflationForApproxStabCost_);
    itemElement = 0;
}

void FIRMCP::executeFeedbackWithPOMCP(void)
{
    bool edgeControllerStatus;
    bool nodeControllerStatus;


    const Vertex start = startM_[0];
    const Vertex goal = goalM_[0];
    Vertex currentVertex = start;
    Vertex targetNode;

    ompl::base::State *cstartState = siF_->allocState();
    ompl::base::State *cendState = siF_->allocState();
    ompl::base::State *goalState = stateProperty_[goal];
    ompl::base::State *tempTrueStateCopy = siF_->allocState();

    //===== SET A Custom Init Covariance=====================
    // using namespace arma;
    // mat tempCC(3,3);
    // tempCC<< 0.1 << 0.0 << 0.0 << endr
    //         << 0.0 << 0.1 << 0.0 << endr
    //         << 0.0 << 0.0 << 0.0000001<<endr;
    // stateProperty_[start]->as<StateType>()->setCovariance(tempCC);
    // Visualizer::updateCurrentBelief(stateProperty_[start]);
    //================================

    siF_->copyState(cstartState, stateProperty_[start]);
    siF_->setTrueState(stateProperty_[start]);
    siF_->setBelief(stateProperty_[start]);


    // Open a file for writing rollout computation time
    std::ofstream outfile;
    if(doSaveLogs_)
    {
        outfile.open(logFilePath_+"RolloutComputationTime.csv");
        outfile<<"RolloutNum, RadiusNN, NumNN, MCParticles, avgTimePerNeighbor, totalTimeSecs" <<std::endl;
    }

    Visualizer::setMode(Visualizer::VZRDrawingMode::RolloutMode);
    Visualizer::clearRobotPath();
    sendMostLikelyPathToViz(start, goal);

    Visualizer::doSaveVideo(doSaveVideo_);
    siF_->doVelocityLogging(true);
    nodeReachedHistory_.push_back(std::make_pair(currentTimeStep_, numberofNodesReached_) );


    //double averageTimeForRolloutComputation = 0;
    int numberOfRollouts = 0;

    // local variables for robust connection to a desirable (but far) FIRM nodes during rollout
    Edge e = feedback_.at(currentVertex);
    targetNode = boost::target(e, g_);

    // counter of the number of executions of the same edge controller in a row
    int kStepOfEdgeController = 0;
    Edge e_prev;  // do not set this to e from the beginning

    // 4) forcefully include the next FIRM node of the previously reached FIRM node in the candidate (nearest neighbor) list for rollout policy
    //Vertex nextFIRMVertex = boost::target(e, g_);

    // HACK WORKAROUNDS FOR INDEFINITE STABILIZATION DURING ROLLOUT: {1} CONNECTION TO FUTURE FIRM NODES
    // 5) forcefully include future feedback nodes of several previous target nodes in the candidate (nearest neighbor) list
    // initialize a container of FIRM nodes on feedback path
    boost::circular_buffer<Vertex> futureFIRMNodes(numberOfTargetsInHistory_ * numberOfFeedbackLookAhead_);  // it is like a size-limited queue or buffer


    OMPL_INFORM("FIRMCP: Running POMCP on top of FIRM");

    // While the robot state hasn't reached the goal state, keep running
    // HACK setting relaxedConstraint argument to false means isReached() condition is exceptionally relaxed for termination
    while(!goalState->as<FIRM::StateType>()->isReached(cstartState, true))
    //while(!goalState->as<FIRM::StateType>()->isReached(cstartState, false))
    {

        // [0] POMCP
        {
            Visualizer::doSaveVideo(false);
            siF_->doVelocityLogging(false);

            // start profiling time to compute rollout
            auto start_time = std::chrono::high_resolution_clock::now();


            // retrieve the execution result from previous iteration
            ompl::base::State* currentBelief = stateProperty_[currentVertex];  // latest start state that is already executed
            ompl::base::State* evolvedBelief = cstartState;                    // current start state to be executed
            Vertex selectedChildQnode = targetNode;  // target node of last execution

            // update the evolved node belief after adding it if it is new to the POMCP tree
            Vertex evolvedVertex;
            bool reset = true;  // NOTE to clean up noisy beliefs from previous simulations (possibly with a larger nSigmaForPOMCPParticle_)
            if (!updateQVnodeBeliefOnPOMCPTree(currentVertex, selectedChildQnode, evolvedBelief, evolvedVertex, reset))
            {
                OMPL_ERROR("Failed to updateQVnodeBeliefOnPOMCPTree()!");
                return;
            }

            // for debug
            OMPL_INFORM("FIRMCP: Moved from Vertex %u (%2.3f, %2.3f, %2.3f, %2.6f) to %u (%2.3f, %2.3f, %2.3f, %2.6f)", currentVertex, evolvedVertex, 
                    stateProperty_[currentVertex]->as<FIRM::StateType>()->getX(),
                    stateProperty_[currentVertex]->as<FIRM::StateType>()->getY(),
                    stateProperty_[currentVertex]->as<FIRM::StateType>()->getYaw(),
                    arma::trace(stateProperty_[currentVertex]->as<FIRM::StateType>()->getCovariance()),
                    stateProperty_[evolvedVertex]->as<FIRM::StateType>()->getX(),
                    stateProperty_[evolvedVertex]->as<FIRM::StateType>()->getY(),
                    stateProperty_[evolvedVertex]->as<FIRM::StateType>()->getYaw(),
                    arma::trace(stateProperty_[evolvedVertex]->as<FIRM::StateType>()->getCovariance()));


            // prune the old tree to free the memory
            const std::vector<Vertex>& childQnodes = currentBelief->as<FIRM::StateType>()->getChildQnodes();
            for (const auto& childQnode : childQnodes)
            {
                // const Vertex childQVnode = currentBelief->as<FIRM::StateType>()->getChildQVnode(childQnode);
                // if (childQVnode != ompl::magic::INVALID_VERTEX_ID)
                // {
                //     if (childQVnode != evolvedVertex)
                //     {
                //         prunePOMCPTreeFrom(childQVnode);
                //     }
                // }
                const std::vector<Vertex>& childQVnodes = currentBelief->as<FIRM::StateType>()->getChildQVnodes(childQnode);
                for (const auto& childQVnode : childQVnodes)
                {
                    if (childQVnode != evolvedVertex)
                    {
                        prunePOMCPTreeFrom(childQVnode);
                    }
                }
            }


            // update currentVertex for next iteration
            Vertex previousVertex = currentVertex;  // backup for POMCP tree pruning
            currentVertex = evolvedVertex;

            // if want/do not want to show monte carlo sim
            siF_->showRobotVisualization(ompl::magic::SHOW_MONTE_CARLO);


            // SELECT THE BEST ACTION
            e = generatePOMCPPolicy(currentVertex, goal);
            targetNode = boost::target(e, g_);

            // if the edge controller of the last execution is being used again now, apply the kStep'th open-loop control of the edge controller
            if (e == e_prev)
            {
                kStepOfEdgeController++;
            }
            else
            {
                kStepOfEdgeController = 0;
                prunePOMCPNode(previousVertex);  // since the edge controllers of this node will no longer be used
            }
            e_prev = e;

            // for debug
            OMPL_INFORM("FIRMCP: Moving from Vertex %u (%2.3f, %2.3f, %2.3f, %2.6f) to [%u] (%2.3f, %2.3f, %2.3f, %2.6f)", currentVertex, targetNode, 
                    stateProperty_[currentVertex]->as<FIRM::StateType>()->getX(),
                    stateProperty_[currentVertex]->as<FIRM::StateType>()->getY(),
                    stateProperty_[currentVertex]->as<FIRM::StateType>()->getYaw(),
                    arma::trace(stateProperty_[currentVertex]->as<FIRM::StateType>()->getCovariance()),
                    stateProperty_[targetNode]->as<FIRM::StateType>()->getX(),
                    stateProperty_[targetNode]->as<FIRM::StateType>()->getY(),
                    stateProperty_[targetNode]->as<FIRM::StateType>()->getYaw(),
                    arma::trace(stateProperty_[targetNode]->as<FIRM::StateType>()->getCovariance()));


            // end profiling time to compute rollout
            auto end_time = std::chrono::high_resolution_clock::now();

            numberOfRollouts++;
            double timeToDoRollout = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            if(doSaveLogs_)
            {
                outfile<<numberOfRollouts<<","<<NNRadius_<<","<<numMCParticles_<<","<<timeToDoRollout/1000<<std::endl;
            }

            // enable robot visualization again
            siF_->showRobotVisualization(true);

            Visualizer::doSaveVideo(doSaveVideo_);
            siF_->doVelocityLogging(true);

            // NOTE commented to prevent redundant call of nearest neighbor search just for visualization! moved to FIRM::addStateToGraph()
            //showRolloutConnections(currentVertex);

            // for rollout edge visualization
            //boost::this_thread::sleep(boost::posix_time::milliseconds(50));  // doesn't seem to be necessary

            // clear the rollout candidate connection drawings and show the selected edge
            Visualizer::clearRolloutConnections();
            Visualizer::setChosenRolloutConnection(stateProperty_[currentVertex], stateProperty_[targetNode]);

        } // [0] POMCP


        ompl::base::Cost costCov;
        int stepsExecuted = 0;
        int stepsToStop = 0;

        // NOTE NodeController will be invoked after executing EdgeController for the given rolloutSteps_ steps

        // [1] EdgeController
        // NOTE instead of generating edge controllers immediately after adding a node, do that lazily at the time the controller is actually being used!
        //EdgeControllerType& edgeController = edgeControllers_.at(e);
        EdgeControllerType& edgeController = getEdgeControllerOnPOMCPTree(e);
        edgeController.setSpaceInformation(policyExecutionSI_);
        if(!edgeController.isTerminated(cstartState, 0))  // check if cstartState is near to the target FIRM node (by x,y position); this is the termination condition B) for EdgeController::Execute()
        {
            // NOTE do not execute edge controller to prevent jiggling motion around the target node

            // NOTE let the edge controller know which step the current state is at, especially when following the same edge controller of the previous iteration
            edgeControllerStatus = edgeController.executeFromUpto(kStepOfEdgeController, rolloutSteps_, cstartState, cendState, costCov, stepsExecuted, false);

            // NOTE how to penalize uncertainty (covariance) and path length (time steps) in the cost
            //*1) cost = wc * sum(trace(cov_k))  + wt * K  (for k=1,...,K)
            // 2) cost = wc * trace(cov_f)       + wt * K
            // 3) cost = wc * mean(trace(cov_k)) + wt * K
            // 4) cost = wc * sum(trace(cov_k))

            currentTimeStep_ += stepsExecuted;

            executionCostCov_ += costCov.value() - ompl::magic::EDGE_COST_BIAS;    // 1,2,3,4) costCov is actual execution cost but only for covariance penalty (even without weight multiplication)

            executionCost_ = informationCostWeight_*executionCostCov_ + timeCostWeight_*currentTimeStep_;    // 1)
            //executionCost_ = informationCostWeight_*executionCostCov_/(currentTimeStep_==0 ? 1e-10 : currentTimeStep_) + timeCostWeight_*currentTimeStep_;    // 3)
            //executionCost_ = informationCostWeight_*executionCostCov_;    // 4)

            costHistory_.push_back(std::make_tuple(currentTimeStep_, executionCostCov_, executionCost_));


            // this is a secondary (redundant) collision check for the true state
            siF_->getTrueState(tempTrueStateCopy);
            if(!siF_->isValid(tempTrueStateCopy))
            {
                OMPL_INFORM("Robot Collided :(");
                return;
            }

            // update cstartState for next iteration
            siF_->copyState(cstartState, cendState);

        } // [1] EdgeController

        // [2] NodeController
        else
        {
            NodeControllerType& nodeController = nodeControllers_.at(targetNode);
            nodeController.setSpaceInformation(policyExecutionSI_);

            nodeControllerStatus = nodeController.StabilizeUpto(rolloutSteps_, cstartState, cendState, costCov, stepsExecuted, false);


            // NOTE how to penalize uncertainty (covariance) and path length (time steps) in the cost
            //*1) cost = wc * sum(trace(cov_k))  + wt * K  (for k=1,...,K)
            // 2) cost = wc * trace(cov_f)       + wt * K
            // 3) cost = wc * mean(trace(cov_k)) + wt * K
            // 4) cost = wc * sum(trace(cov_k))

            currentTimeStep_ += stepsExecuted;

            executionCostCov_ += costCov.value() - ompl::magic::EDGE_COST_BIAS;    // 1,2,3,4) costCov is actual execution cost but only for covariance penalty (even without weight multiplication)

            executionCost_ = informationCostWeight_*executionCostCov_ + timeCostWeight_*currentTimeStep_;    // 1)
            //executionCost_ = informationCostWeight_*executionCostCov_/(currentTimeStep_==0 ? 1e-10 : currentTimeStep_) + timeCostWeight_*currentTimeStep_;    // 3)
            //executionCost_ = informationCostWeight_*executionCostCov_;    // 4)

            costHistory_.push_back(std::make_tuple(currentTimeStep_, executionCostCov_, executionCost_));


            // this is a secondary (redundant) collision check for the true state
            siF_->getTrueState(tempTrueStateCopy);
            if(!siF_->isValid(tempTrueStateCopy))
            {
                OMPL_INFORM("Robot Collided :(");
                return;
            }

            // update the cstartState for next iteration
            siF_->copyState(cstartState, cendState);

        } // [2] NodeController


        // check if cendState after execution has reached targetNode, just for logging
        if(stateProperty_[targetNode]->as<FIRM::StateType>()->isReached(cendState))
        {
            OMPL_INFORM("FIRMCP: Reached FIRM Node: %u", targetNode);
            numberofNodesReached_++;
            nodeReachedHistory_.push_back(std::make_pair(currentTimeStep_, numberofNodesReached_) );
        }

    } // while()


    // for analysis

    // this data is also saved in run-(TIMESTAMP)/FIRMCPCostHistory.csv
    std::cout << std::endl;
    std::cout << "Execution time steps: " << currentTimeStep_ << std::endl;
    std::cout << "Execution covariance cost: " << executionCostCov_ << std::endl;
    std::cout << "Execution cost: " << executionCost_ << "  ( = " << informationCostWeight_ << "*" << executionCostCov_ << " + " << timeCostWeight_ << "*" << currentTimeStep_ << " )" << std::endl;     // 1)
    //std::cout << "Execution cost: " << executionCost_ << "  ( = " << informationCostWeight_ << "*" << executionCostCov_ << "/" << currentTimeStep_ << " + " << timeCostWeight_ << "*" << currentTimeStep_ << " )" << std::endl;     // 3)
    //std::cout << "Execution cost: " << executionCost_ << "  ( = " << informationCostWeight_ << "*" << executionCostCov_ << " )" << std::endl;     // 4)
    std::cout << std::endl;

    // this data is also saved in run-(TIMESTAMP)/FIRMCPCostHistory.csv
    //std::cout << "Number of nodes with stationary penalty: " << numberOfStationaryPenalizedNodes_ << std::endl;
    //std::cout << "Sum of stationary penalties: " << sumOfStationaryPenalties_ << std::endl;

    nodeReachedHistory_.push_back(std::make_pair(currentTimeStep_, numberofNodesReached_) );
    //OMPL_INFORM("FIRM: Number of nodes reached with Rollout: %u", numberofNodesReached_);
    //averageTimeForRolloutComputation = averageTimeForRolloutComputation / (1000*numberOfRollouts);
    //std::cout<<"Nearest Neighbor Radius: "<<NNRadius_<<", Monte Carlo Particles: "<<numMCParticles_<<", Avg Time/neighbor (seconds): "<<averageTimeForRolloutComputation<<std::endl;    

    if(doSaveLogs_)
    {
        outfile.close();
        writeTimeSeriesDataToFile("FIRMCPCostHistory.csv", "costToGo");
        writeTimeSeriesDataToFile("FIRMCPSuccessProbabilityHistory.csv", "successProbability");
        writeTimeSeriesDataToFile("FIRMCPNodesReachedHistory.csv","nodesReached");
        writeTimeSeriesDataToFile("FIRMCPStationaryPenaltyHistory.csv","stationaryPenalty");
        std::vector<std::pair<double, double>> velLog;
        siF_->getVelocityLog(velLog);
        for(int i=0; i < velLog.size(); i++)
        {
            velocityHistory_.push_back(std::make_pair(i, sqrt( pow(velLog[i].first,2) + pow(velLog[i].second,2) ))); // omni
            //velocityHistory_.push_back(std::make_pair(i, velLog[i].first)); // unicycle
        }
        writeTimeSeriesDataToFile("FIRMCPVelocityHistory.csv", "velocity");
    }
    Visualizer::doSaveVideo(true);
    sleep(0.33);

    // free the memory
    siF_->freeState(cstartState);
    siF_->freeState(cendState);
    siF_->freeState(tempTrueStateCopy);
}

FIRM::Edge FIRMCP::generatePOMCPPolicy(const FIRM::Vertex currentVertex, const FIRM::Vertex goal)
{
    // declare local variables
    ompl::base::State* tempTrueStateCopy = siF_->allocState();
    ompl::base::State* sampState = siF_->allocState();

    // save the current true state
    siF_->getTrueState(tempTrueStateCopy);
    Visualizer::setMode(Visualizer::VZRDrawingMode::FIRMCPMode);


    // call pomcpSimulate() for N particles
    for (unsigned int i=0; i<numPOMCPParticles_; i++)
    {
        // sample a particle from the root (current) belief state
        // NOTE random sampling of a true state from the current belief state for Monte Carlo simulation
        // HACK a larger nSigmaForPOMCPParticle_ to increase the chance of detecting collision with a few number of particles
        if(!stateProperty_[currentVertex]->as<FIRM::StateType>()->sampleTrueStateFromBelief(sampState, nSigmaForPOMCPParticle_))
        {
            OMPL_WARN("Could not sample a true state from the current belief state!");
            continue;
        }
        siF_->setTrueState(sampState);  // true state is only used for collision check by checkTrueStateValidity()
        // for debug
        std::cout << currentVertex;

        // run Monte Carlo simulation for one particle and update cost-to-go and number of visits
        int currentDepth = 0;
        int collisionDepth = (int)ompl::magic::DEFAULT_INF_COST_TO_GO;  // initialize as if collision never happens
        Edge selectedEdgeDummy;  // just for syntax

        double totalCostToGo = pomcpSimulate(currentVertex, currentDepth, selectedEdgeDummy, collisionDepth);

        // for debug
        std::cout << "thisQVmincosttogo: " << totalCostToGo << std::endl;
    }


    // SELECT THE BEST ACTION
    const std::vector<Vertex>& childQnodes = stateProperty_[currentVertex]->as<FIRM::StateType>()->getChildQnodes();
    double minQcosttogo = infiniteCostToGo_;
    std::vector<Vertex> minQcosttogoNodes;
    Vertex childQnode, selectedChildQnode;
    double childQcosttogo;
    // for debug
    std::cout << "childQcosttogoes: ";
    for (int j=0; j<childQnodes.size(); j++)
    {
        childQnode = childQnodes[j];
        childQcosttogo = stateProperty_[currentVertex]->as<FIRM::StateType>()->getChildQcosttogo(childQnode);
        // for debug
        std::cout << "[" << childQnode << "]" << childQcosttogo << " ";

        if (minQcosttogo >= childQcosttogo)
        {
            if (minQcosttogo > childQcosttogo)
            {
                minQcosttogoNodes.clear();
            }
            minQcosttogo = childQcosttogo;
            minQcosttogoNodes.push_back(childQnode);
        }
    }
    if (minQcosttogoNodes.size()==1)
    {
        selectedChildQnode = minQcosttogoNodes[0];
    }
    else
    {
        assert(minQcosttogoNodes.size()!=0);
        //OMPL_WARN("More than one childQnodes are with the minQcosttogo!");
        int random = rand() % minQcosttogoNodes.size();
        selectedChildQnode = minQcosttogoNodes[random];  // to break the tie
    }
    // get the selected edge
    Edge selectedEdge = boost::edge(currentVertex, selectedChildQnode, g_).first;

    // for debug
    std::cout << std::endl;
    std::cout << "minQcosttogo: " << "[" << selectedChildQnode << "]" << minQcosttogo << std::endl;
    std::cout << "executionCost: " << executionCost_ << std::endl;
    std::cout << "expTotalCost: " << minQcosttogo + executionCost_ << std::endl;


    // restore the current true state
    siF_->setTrueState(tempTrueStateCopy);
    Visualizer::setMode(Visualizer::VZRDrawingMode::RolloutMode);

    // free the memory
    siF_->freeState(tempTrueStateCopy);
    siF_->freeState(sampState);

    return selectedEdge;
}

double FIRMCP::pomcpSimulate(const Vertex currentVertex, const int currentDepth, const Edge& selectedEdgePrev, int& collisionDepth)
{
    // declare local variables
    ompl::base::State* currentBelief = stateProperty_[currentVertex];
    Edge selectedEdge;
    Vertex selectedChildQnode;
    double executionCost;
    bool isNewNodeExpanded = false;


    // call pomcpRollout() if the current belief has never been expanded yet
    if (!currentBelief->as<FIRM::StateType>()->getChildQexpanded())
    {
        isNewNodeExpanded = true;  // set a flag for initialization of this node for all actions

        // total cost-to-go from this node
        double totalCostToGo = pomcpRollout(currentVertex, currentDepth, selectedEdgePrev, collisionDepth, isNewNodeExpanded);

        return totalCostToGo;
    }

    // call pomcpRollout() if the current depth is out of the finite horizon of the POMCP tree
    if (currentDepth >= maxPOMCPDepth_)
    {
        // total cost-to-go from this node
        double totalCostToGo = pomcpRollout(currentVertex, currentDepth, selectedEdgePrev, collisionDepth);

        return totalCostToGo;
    }


    // SELECT AN ACTION BY GREEDY UCB TREE POLICY
    const std::vector<Vertex>& childQnodes = currentBelief->as<FIRM::StateType>()->getChildQnodes();
    double minQcosttogo = infiniteCostToGo_;
    std::vector<Vertex> minQcosttogoNodes;
    Vertex childQnode;
    double childQcosttogo;
    for (int j=0; j<childQnodes.size(); j++)
    {
        // retrieve up-to-date cost-to-go value
        childQnode = childQnodes[j];
        childQcosttogo = currentBelief->as<FIRM::StateType>()->getChildQcosttogo(childQnode);

        // apply exploration bonus!
        double thisQVvisit = currentBelief->as<FIRM::StateType>()->getThisQVvisit();            // N(h)
        double childQvisit = currentBelief->as<FIRM::StateType>()->getChildQvisit(childQnode);  // N(ha)
        // NOTE we minimize, not maximize, the cost-to-go value
        childQcosttogo -= cExplorationForSimulate_ * std::sqrt( std::log(thisQVvisit+1.0) / (childQvisit+1e-10) );
        // NOTE commented this line to allow childQcosttogo with exploration bonus can be a negative value, only for action selection in this function
        //childQcosttogo = (childQcosttogo > 0.0) ? childQcosttogo : 0.0;

        // find the action with the minimum cost-to-go
        if (minQcosttogo >= childQcosttogo)
        {
            if (minQcosttogo > childQcosttogo)
            {
                minQcosttogoNodes.clear();
            }
            minQcosttogo = childQcosttogo;
            minQcosttogoNodes.push_back(childQnode);
        }
    }
    if (minQcosttogoNodes.size()==1)
    {
        selectedChildQnode = minQcosttogoNodes[0];
    }
    else
    {
        assert(minQcosttogoNodes.size()!=0);
        int random = rand() % minQcosttogoNodes.size();
        selectedChildQnode = minQcosttogoNodes[random];  // to break the tie
    }
    // select the action
    selectedEdge = boost::edge(currentVertex, selectedChildQnode, g_).first;


    // SIMULATE ACTION EXECUTION
    ompl::base::State* evolvedBelief = siF_->allocState();
    int kStep = std::max(0, currentDepth - maxPOMCPDepth_ + 1);  // if currentDepth>=maxPOMCPDepth_, the previous edge controller will be used from kStep
    bool executionStatus = executeSimulationFromUpto(kStep, rolloutSteps_, currentBelief, selectedEdge, evolvedBelief, executionCost);
    if (!executionStatus)
    {
        OMPL_ERROR("Failed to executeSimulationFromUpto()!");
        executionCost = obstacleCostToGo_;
    }
    Visualizer::clearRolloutConnections();


    // update the evolved node belief after adding it if it is new to the POMCP tree
    Vertex evolvedVertex;
    if (!updateQVnodeBeliefOnPOMCPTree(currentVertex, selectedChildQnode, evolvedBelief, evolvedVertex))
    {
        OMPL_ERROR("Failed to updateQVnodeBeliefOnPOMCPTree()!");
        return infiniteCostToGo_;
    }
    // for debug
    if (currentDepth < maxPOMCPDepth_)
        std::cout << "-[" << selectedChildQnode << "]-" << evolvedVertex;
    else
        std::cout << ".[" << selectedChildQnode << "]." << evolvedVertex;


    // RECURSIVELY CALL pomcpSimulate()
    double selectedChildQVmincosttogo = 0.0;
    if (executionStatus)  // if not collided during latest execution
    {
        selectedChildQVmincosttogo = pomcpSimulate(evolvedVertex, currentDepth+1, selectedEdge, collisionDepth);
    }
    // free the memory
    siF_->freeState(evolvedBelief);


    // update the number of visits and cost-to-go values
    double thisQVmincosttogoUpdated;
    updateQVnodeValuesOnPOMCPTree(currentVertex, selectedChildQnode, executionStatus, executionCost, selectedChildQVmincosttogo, thisQVmincosttogoUpdated);

    // return the minimum total cost-to-go over all explored trajectories
    return thisQVmincosttogoUpdated;
}

double FIRMCP::pomcpRollout(const Vertex currentVertex, const int currentDepth, const Edge& selectedEdgePrev, int& collisionDepth, const bool isNewNodeExpanded)
{
    // declare local variables
    ompl::base::State* currentBelief = stateProperty_[currentVertex];  // current belief after applying previous control toward the latest target
    Edge selectedEdge;
    Vertex selectedChildQnode;
    double executionCost;


    // if the current depth is out of the finite horizon of the POMCP tree
    if (currentDepth >= maxPOMCPDepth_)
    {

        // if the current depth is out of the maximum depth of the variable horizon for bridging from POMCP tree to FIRM graph
        if (currentDepth >= maxFIRMReachDepth_)
        {
            OMPL_WARN("Could not reach to the target node within %d iterations", maxFIRMReachDepth_);

            double totalCostToGo = obstacleCostToGo_;

            // update the number of visits and cost-to-go
            currentBelief->as<FIRM::StateType>()->addThisQVvisit();                    // N(h) += 1
            currentBelief->as<FIRM::StateType>()->setThisQVmincosttogo(totalCostToGo);

            return totalCostToGo;
        }


        // create a new node if the current belief has never been expanded yet
        if (!currentBelief->as<FIRM::StateType>()->getChildQexpanded())
        {
            // this will compute approximate edge cost, cost-to-go, and heuristic action weight
            // NOTE call this function just once per POMCP tree node
            if (!expandQnodesOnPOMCPTreeWithApproxCostToGo(currentVertex, isNewNodeExpanded))
            {
                OMPL_WARN("Failed to expandQnodesOnPOMCPTreeWithApproxCostToGo()!");

                double totalCostToGo = obstacleCostToGo_;

                // update the number of visits and cost-to-go
                currentBelief->as<FIRM::StateType>()->addThisQVvisit();                    // N(h) += 1
                currentBelief->as<FIRM::StateType>()->setThisQVmincosttogo(totalCostToGo);

                return totalCostToGo;
            }
        }


        // if the current belief already reached the most recent target FIRM node
        Vertex targetVertex = boost::target(selectedEdgePrev, g_);   // latest target before reaching the finite horizon
        if (stateProperty_[targetVertex]->as<FIRM::StateType>()->isReached(currentBelief))
        {
            // clear the rollout candidate connection drawings and show the selected edge
            Visualizer::clearRolloutConnections();
            //Visualizer::setChosenRolloutConnection(stateProperty_[currentVertex], stateProperty_[targetNode]);

            // for debug
            std::cout << std::endl;

            // compute approximate edge cost and cost-to-go
            double approxEdgeCost = computeApproxEdgeCost(currentVertex, targetVertex);
            // NOTE instead of the raw costToGo_ from FIRM, use costToGoWithApproxStabCost_ with stabilization cost compenstation along the feedback paths
            //double approxCostToGo = costToGo_[targetVertex] + approxEdgeCost;
            double approxCostToGo = getCostToGoWithApproxStabCost(targetVertex) + approxEdgeCost;

            // update the number of visits and cost-to-go
            currentBelief->as<FIRM::StateType>()->addThisQVvisit();                    // N(h) += 1
            currentBelief->as<FIRM::StateType>()->setThisQVmincosttogo(approxCostToGo);

            return approxCostToGo;
        }


        // otherwise, continue to move toward the latest target FIRM node and return cost-to-go
        selectedEdge = selectedEdgePrev;
        selectedChildQnode = targetVertex;

        // check if previously selected action is still valid for this node
        const std::vector<Vertex>& childQnodes = currentBelief->as<FIRM::StateType>()->getChildQnodes();
        if (std::find(childQnodes.begin(), childQnodes.end(), selectedChildQnode) == childQnodes.end())
        {
            OMPL_WARN("selectedChildQnode action for %d node to reach a FIRM node %d during pomcpRollout() is not available for this current node!", currentVertex, selectedChildQnode);

            double totalCostToGo = obstacleCostToGo_;

            // update the number of visits and cost-to-go
            currentBelief->as<FIRM::StateType>()->addThisQVvisit();                    // N(h) += 1
            currentBelief->as<FIRM::StateType>()->setThisQVmincosttogo(totalCostToGo);

            return totalCostToGo;
        }

    } // if (currentDepth >= maxPOMCPDepth_)


    // if the current depth is within the finite horizon of the POMCP tree
    else // if (currentDepth < maxPOMCPDepth_)
    {

        // create a new node if the current belief has never been expanded yet
        if (!currentBelief->as<FIRM::StateType>()->getChildQexpanded())
        {
            // this will compute approximate edge cost, cost-to-go, and heuristic action weight
            // NOTE call this function just once per POMCP tree node
            if (!expandQnodesOnPOMCPTreeWithApproxCostToGo(currentVertex, isNewNodeExpanded))
            {
                OMPL_WARN("Failed to expandQnodesOnPOMCPTreeWithApproxCostToGo()!");

                double totalCostToGo = obstacleCostToGo_;

                // update the number of visits and cost-to-go
                currentBelief->as<FIRM::StateType>()->addThisQVvisit();                    // N(h) += 1
                currentBelief->as<FIRM::StateType>()->setThisQVmincosttogo(totalCostToGo);

                return totalCostToGo;
            }
        }


        // SELECT AN ACTION BY ROLLOUT POLICY
        // importance sampling to pick one neighbor as a target based on approximate cost-to-go

        // get the connected neighbor list
        const std::vector<Vertex>& childQnodes = currentBelief->as<FIRM::StateType>()->getChildQnodes();

        // allot a section in the weight bar according to each weight
        std::vector<double> weightSections;  // upper limit of accumulated weight for each action
        double costtogo, weight, weightSum = 0.0;

        // check for isReachedWithinNEpsilon for any of nearest neighbor FIRM nodes
        bool isReachedWithinNEpsilon = false;
        for (const auto& childQnode : childQnodes)
        {
            isReachedWithinNEpsilon = stateProperty_[childQnode]->as<FIRM::StateType>()->isReachedWithinNEpsilon(currentBelief, nEpsilonForRolloutIsReached_);
            break;
        }

        // compute weight for each action
        for (const auto& childQnode : childQnodes)
        {
            // retrieve up-to-date cost-to-go value
            costtogo = currentBelief->as<FIRM::StateType>()->getChildQcosttogo(childQnode);  // unchanged from the initial value until this node is added to the POMCP tree

            // NOTE POMCP-Rollout is to explore the unknown area of the search space!
            // in our problem, the main unknown area is between the approximate stabilization of isReached() and the optimal stabilization
            // if isReachedWithinNEpsilon() is satisfied, increase the randomness of POMCP-Rollout policy to explore this region,
            // otherwise, decrease the randomness to be more exploitative like FIRM-Rollout

            if (isReachedWithinNEpsilon)  // explorative
            {
                weight = 1.0 / (std::pow(costtogo, cExploitationForRolloutWithinReach_) + costToGoRegulatorWithinReach_);
            }
            else  // exploitative
            {
                weight = 1.0 / (std::pow(costtogo, cExploitationForRolloutOutOfReach_) + costToGoRegulatorOutOfReach_);
            }

            weightSum += weight;
            weightSections.push_back(weightSum);
        }
        for (auto& weight : weightSections)
        {
            weight /= weightSum;  // normalize the accumulated weight
        }

        // randomly pick an accumulated weight point
        arma::colvec weightPickedVec = arma::randu<arma::colvec>(1,1);  // range: [0, 1]
        double weightPicked = weightPickedVec[0];

        // enumerate to find the matching weightSection, assuming that childQnodes.size() is not so big
        int jSelected;
        for (int j=0; j<weightSections.size(); j++)
        {
            if (weightPicked < weightSections[j])
            {
                jSelected = j;
                break;
            }
        }

        // select the action
        selectedChildQnode = childQnodes[jSelected];
        selectedEdge = boost::edge(currentVertex, selectedChildQnode, g_).first;

    } // else if (currentDepth < maxPOMCPDepth_)


    // SIMULATE ACTION EXECUTION
    ompl::base::State* evolvedBelief = siF_->allocState();
    int kStep = std::max(0, currentDepth - maxPOMCPDepth_ + 1);  // if currentDepth>=maxPOMCPDepth_, the previous edge controller will be used from kStep
    bool executionStatus = executeSimulationFromUpto(kStep, rolloutSteps_, currentBelief, selectedEdge, evolvedBelief, executionCost);
    if (!executionStatus)
    {
        OMPL_ERROR("Failed to executeSimulationFromUpto()!");
        executionCost = obstacleCostToGo_;
    }
    Visualizer::clearRolloutConnections();


    // update the evolved node belief after adding it if it is new to the POMCP tree
    Vertex evolvedVertex;
    if (!updateQVnodeBeliefOnPOMCPTree(currentVertex, selectedChildQnode, evolvedBelief, evolvedVertex))
    {
        OMPL_ERROR("Failed to updateQVnodeBeliefOnPOMCPTree()!");
        return infiniteCostToGo_;
    }
    // for debug
    if (currentDepth < maxPOMCPDepth_)
        std::cout << "~(" << selectedChildQnode << ")~" << evolvedVertex;
    else
        std::cout << ".(" << selectedChildQnode << ")." << evolvedVertex;


    // RECURSIVELY CALL pomcpRollout()
    double selectedChildQVmincosttogo = 0.0;
    if (executionStatus)  // if not collided during latest execution
    {
        selectedChildQVmincosttogo = pomcpRollout(evolvedVertex, currentDepth+1, selectedEdge, collisionDepth);
    }
    // free the memory
    siF_->freeState(evolvedBelief);


    // do backup only for the first new node
    if (isNewNodeExpanded)
    {
        // update the number of visits and cost-to-go values
        double thisQVmincosttogoUpdated;
        updateQVnodeValuesOnPOMCPTree(currentVertex, selectedChildQnode, executionStatus, executionCost, selectedChildQVmincosttogo, thisQVmincosttogoUpdated, isNewNodeExpanded);

        // return the minimum total cost-to-go over all explored trajectories
        return thisQVmincosttogoUpdated;
    }
    // otherwise, just return the sum of execution costs along the trajectory plus the cost-to-go of the leaf
    else
    {
        // total cost-to-go from this node (only for Rollout)
        double selectedChildQcosttogoUpdated = executionCost;
        if (executionStatus)
            selectedChildQcosttogoUpdated += selectedChildQVmincosttogo;
        else
            selectedChildQcosttogoUpdated += obstacleCostToGo_;

        // return total cost-to-go along this trajectory
        return selectedChildQcosttogoUpdated;
    }
}

FIRM::Vertex FIRMCP::addQVnodeToPOMCPTree(ompl::base::State *state)
{
    // just for compatibility with FIRM::addStateToGraph()
    bool addReverseEdge = false;

    boost::mutex::scoped_lock _(graphMutex_);

    // add the given belief state to graph as FIRM node
    Vertex m;
    m = boost::add_vertex(g_);

    stateProperty_[m] = state;
    //totalConnectionAttemptsProperty_[m] = 1;
    //successfulConnectionAttemptsProperty_[m] = 0;

    return m;
}

bool FIRMCP::expandQnodesOnPOMCPTreeWithApproxCostToGo(const Vertex m, const bool isNewNodeExpanded)
{
    // just for compatibility with FIRM::addStateToGraph()
    bool addReverseEdge = false;

    const Vertex start = startM_[0];

    // add this vertex to the database for nearest neighbor search
    if (m != start)
    {
        nn_->add(m);
    }

    // which milestones will we attempt to connect to?
    // NOTE use longer nearest neighbor to avoid oscillating behavior of rollout policy by allowing connection to farther FIRM nodes
    std::vector<Vertex> neighbors;
    std::vector<Vertex> kneighbors;
    if(addReverseEdge)  // graph construction phase
    {
        neighbors = connectionStrategy_(m, NNRadius_);
        //neighbors = kConnectionStrategy_(m, numNearestNeighbors_);    // NOTE this makes sense only if all the points are sampled first and then connected to each other, but not in the case point sampling and connection are done simultaneously
    }
    else  // rollout phase
    {
        // NOTE robust connection to a desirable (but far) FIRM nodes during rollout

        // 1) allow longer edge length for connection to FIRM nodes during rollout
//         neighbors = connectionStrategy_(m, 1.5*NNRadius_);
//         neighbors = connectionStrategy_(m, 1.2*NNRadius_);
        neighbors = connectionStrategy_(m, 1.0*NNRadius_);

        // 2) forcefully include the current state's k-nearest neighbors of FIRM nodes in the candidate list; this may be considered as a change of the graph connection property
//         kneighbors = kConnectionStrategy_(m, numNearestNeighbors_);
//         // remove duplicate entities from kneighbors
//         std::vector<Vertex>::iterator diff;
//         diff = std::set_difference(kneighbors.begin(), kneighbors.end(), neighbors.begin(), neighbors.end(), kneighbors.begin());
//         kneighbors.erase(diff, kneighbors.end());   // {kneighbors} = {kneighbors} - {neighbors}
//         // concantenate kneighbors to neighbors
//         neighbors.insert(neighbors.end(), kneighbors.begin(), kneighbors.end());

        // 3) instead of bounded nearest neighbors, use k-nearest neighbors during rollout; this may be considered as a change of the graph connection property
//         neighbors = kConnectionStrategy_(m, numNearestNeighbors_);

        // 4) forcefully include the next FIRM node of the previously reached FIRM node in the candidate (nearest neighbor) list
        // implemented in FIRM::executeFeedbackWithRollout() to access nextFIRMVertex information
//         neighbors = connectionStrategy_(m, NNRadius_);

        // 5) forcefully include future feedback nodes of several previous target nodes in the candidate (nearest neighbor) list
        // implemented in FIRM::executeFeedbackWithRollout() to access nextFIRMVertex information
//         neighbors = connectionStrategy_(m, NNRadius_);
    }

    // do not print this during rollout
    if(addReverseEdge)
        OMPL_INFORM("Adding a state: %u nearest neighbors from %u nodes in the graph", neighbors.size(), boost::num_vertices(g_));

    // remove this vertex to the database to exclude POMCP tree nodes from edge connection to FIRM nodes
    if (m != start)
    {
        nn_->remove(m);
    }

    // check for valid neighbors
    if (!addReverseEdge)
    {
        if (neighbors.size()==1)
        {
            if (m==neighbors[0])
            {
                OMPL_ERROR("No neighbor other than itself was found for vertex %d", m);
                exit(0);    // XXX for debug
                return false;
            }
        }
    }


    FIRMWeight approxEdgeCost;
    double approxCostToGo, weight;
    foreach (Vertex n, neighbors)
    {
        if ( m!=n )
        {
            //totalConnectionAttemptsProperty_[m]++;
            //totalConnectionAttemptsProperty_[n]++;

            if (siF_->checkMotion(stateProperty_[m], stateProperty_[n]))
            {
                bool forwardEdgeAdded=false;

                // NOTE in execution mode, i.e., addReverseEdge is false, compute edge cost from the center belief state, not from a sampled border belief state
                //approxEdgeCost = addEdgeToPOMCPTreeWithApproxCost(m, n, forwardEdgeAdded);
                // NOTE instead of generating edge controllers immediately after adding a node, do that lazily at the time the controller is actually being used!
                approxEdgeCost = lazilyAddEdgeToPOMCPTreeWithApproxCost(m, n, forwardEdgeAdded);

                if(forwardEdgeAdded)
                {
                    //successfulConnectionAttemptsProperty_[m]++;

                    // compute the approximate cost-to-go
                    // NOTE instead of the raw costToGo_ from FIRM, use costToGoWithApproxStabCost_ with stabilization cost compenstation along the feedback paths
                    //approxCostToGo = approxEdgeCost.getCost() + costToGo_[n];
                    approxCostToGo = approxEdgeCost.getCost() + getCostToGoWithApproxStabCost(n);

                    // save childQnode and approximate childQcosttogo for next POMCP-Rollout
                    stateProperty_[m]->as<FIRM::StateType>()->addChildQnode(n);
                    stateProperty_[m]->as<FIRM::StateType>()->setChildQcosttogo(n, approxCostToGo);
                }
            }
        }
    }

    // for rollout edge visualization
    if(!addReverseEdge)
    {
        foreach(Vertex n, neighbors)
        {
            if(boost::edge(m, n, g_).second)
            {
                Visualizer::addRolloutConnection(stateProperty_[m], stateProperty_[n]);
            }
        }
        //boost::this_thread::sleep(boost::posix_time::milliseconds(50));  // moved to FIRM::executeFeedbackWithRollout() for more accurate time computatiof
    }

    policyGenerator_->addFIRMNodeToObservationGraph(stateProperty_[m]);

    // mark that this node's childQnodes are expanded now
    stateProperty_[m]->as<FIRM::StateType>()->setChildQexpanded();

    return true;
}

FIRMWeight FIRMCP::lazilyAddEdgeToPOMCPTreeWithApproxCost(const FIRM::Vertex a, const FIRM::Vertex b, bool &edgeAdded)
{
    // NOTE instead of generating edge controllers immediately after adding a node, do that lazily at the time the controller is actually being used!

    // compute approximate edge cost
    double approxEdgeCost = computeApproxEdgeCost(a, b);
    double transitionProbability = 1.0;    // just naively set this to 1.0
    FIRMWeight weight(approxEdgeCost, transitionProbability);

    // create an boost::edge with the edge weight property
    const unsigned int id = maxEdgeID_++;
    const Graph::edge_property_type properties(approxEdgeCost, id);
    std::pair<Edge, bool> newEdge = boost::add_edge(a, b, properties, g_);

    edgeAdded = true;

    return weight;
}

FIRM::EdgeControllerType& FIRMCP::getEdgeControllerOnPOMCPTree(const Edge& edge)
{
    if (edgeControllers_.find(edge) == edgeControllers_.end())
    {
        Vertex a = boost::source(edge, g_);
        Vertex b = boost::target(edge, g_);
        bool forwardEdgeAdded=false;

        // generate an edge controller
        addEdgeToPOMCPTreeWithApproxCost(a, b, forwardEdgeAdded, &edge);
        if (!forwardEdgeAdded)
        {
            OMPL_ERROR("Failed to addEdgeToPOMCPTreeWithApproxCost()!");
            exit(0);  // for debug
        }
    }

    return edgeControllers_.at(edge);
}

FIRMWeight FIRMCP::addEdgeToPOMCPTreeWithApproxCost(const FIRM::Vertex a, const FIRM::Vertex b, bool &edgeAdded, const Edge* pEdge)
{
    // just for compatibility with FIRM::addEdgeToGraph()
    bool addReverseEdge = false;

    EdgeControllerType edgeController;

    // for debug
    if(ompl::magic::PRINT_MC_PARTICLES)
        std::cout << "=================================================" << std::endl;

    // HACK WORKAROUNDS FOR INDEFINITE STABILIZATION DURING ROLLOUT: {3} EDGE COST WITH A BORDER BELIEF STATE
    // apply the edge cost computation option
    bool constructionMode;
    if(borderBeliefSampling_)
    {
        // use the border belief state to compute the edge cost when constructing a graph
        constructionMode = addReverseEdge;
    }
    else
    {
        // use the center belief state to compute the edge cost even when constructing a graph
        constructionMode = false;
    }

    // generate an edge controller and compute edge cost
    //const FIRMWeight weight = generateEdgeControllerWithCost(a, b, edgeController, constructionMode);      // deprecated: EdgeController only
    //const FIRMWeight weight = generateEdgeNodeControllerWithCost(a, b, edgeController, constructionMode);  // EdgeController and NodeController concatenated
    const FIRMWeight weight = generateEdgeNodeControllerWithApproxCost(a, b, edgeController);    // EdgeController and NodeController concatenated; heuristically compute edge cost for POMCP-Rollout without Monte Carlo simulation

    assert(edgeController.getGoal() && "The generated controller has no goal");


    // add the generated controller to the container
    if (pEdge)  // if an boost::edge(a,b) is already added to the graph and its controller is being lazily generated here
    {
        edgeControllers_[*pEdge] = edgeController;
    }
    else
    {
        const unsigned int id = maxEdgeID_++;
        const Graph::edge_property_type properties(weight, id);

        // create an edge with the edge weight property
        std::pair<Edge, bool> newEdge = boost::add_edge(a, b, properties, g_);

        edgeControllers_[newEdge.first] = edgeController;
    }

    edgeAdded = true;

    return weight;
}

FIRMWeight FIRMCP::generateEdgeNodeControllerWithApproxCost(const FIRM::Vertex a, const FIRM::Vertex b, EdgeControllerType &edgeController)
{
    ompl::base::State* startNodeState = siF_->cloneState(stateProperty_[a]);
    ompl::base::State* targetNodeState = stateProperty_[b];

     // Generate the edge controller for given start and end state
    generateEdgeController(startNodeState,targetNodeState,edgeController);

    // compute approximate edge cost
    double approxEdgeCost = computeApproxEdgeCost(a, b);
    double transitionProbability = 1.0;    // just naively set this to 1.0
    FIRMWeight weight(approxEdgeCost, transitionProbability);

    // free the memory
    siF_->freeState(startNodeState);

    return weight;
}

double FIRMCP::computeApproxEdgeCost(const FIRM::Vertex a, const FIRM::Vertex b)
{
    double approxTransCost = computeApproxTransitionCost(a, b);
    double approxStabCost = computeApproxStabilizationCost(a, b);

    double approxEdgeCost = approxTransCost + approxStabCost;
    // for debug
    if(ompl::magic::PRINT_EDGE_COST)
        std::cout << "approxEdgeCost[" << a << "->" << b << "]: " << approxEdgeCost << " = " << approxTransCost << " + " << approxStabCost << std::endl;

    return approxEdgeCost;
}

double FIRMCP::computeApproxTransitionCost(const FIRM::Vertex a, const FIRM::Vertex b)
{
    // declare local variables
    ompl::base::State* startNodeState = stateProperty_[a];
    ompl::base::State* targetNodeState = stateProperty_[b];

    // compute the distance between two states
    double posDistance = startNodeState->as<FIRM::StateType>()->getPosDistanceTo(targetNodeState);
    double oriDistance = startNodeState->as<FIRM::StateType>()->getOriDistanceTo(targetNodeState);

    double startTraceCov = startNodeState->as<FIRM::StateType>()->getTraceCovariance();
    // NOTE deprecated: do not consider covariance stabilization in edge cost (transition only until isReached() is satisfied)
    //double targetTraceCov = targetNodeState->as<FIRM::StateType>()->getTraceCovariance();
    //double covDistance = startTraceCov - targetTraceCov;
    //covDistance = (covDistance > 0.0) ? covDistance : 0.0;

    // compensate the distance considering isReached() tolerance
    // OPTIONAL) enabling these may lead to under-estimation of the actual cost, which then can result in jiggling execution
    posDistance = ((posDistance - StateType::reachDistPos_) > 0.0) ? (posDistance - StateType::reachDistPos_) : 0.0;
    oriDistance = ((oriDistance - StateType::reachDistOri_) > 0.0) ? (oriDistance - StateType::reachDistOri_) : 0.0;
    //covDistance = ((covDistance - StateType::reachDistCov_) > 0.0) ? (covDistance - StateType::reachDistCov_) : 0.0;

    // compute an approximate edge cost (heuristically!)
    double numPosConvergence = posDistance / heurPosStepSize_;
    double numOriConvergence = oriDistance / heurOriStepSize_;
    //double numCovConvergence = covDistance / heurCovStepSize_;   // heurCovStepSize_ is no longer being used!
    double maxNumConvergence = std::max(numPosConvergence, numOriConvergence);
    //double maxNumConvergence = std::max(std::max(numPosConvergence, numOriConvergence), numCovConvergence); // deprecated
    double stepsToStop = maxNumConvergence;
    double filteringCost = (covConvergenceRate_ == 1.0) ? (startTraceCov * stepsToStop) : ( startTraceCov * covConvergenceRate_ * (1 - std::pow(covConvergenceRate_, stepsToStop)) / (1 - covConvergenceRate_) );  // sum_{k=1}^{stepsToStop}(covConvergenceRate^k * startTraceCov)

    // NOTE how to penalize uncertainty (covariance) and path length (time steps) in the cost
    //*1) cost = wc * sum(trace(cov_k))  + wt * K  (for k=1,...,K)
    // 2) cost = wc * trace(cov_f)       + wt * K
    // 3) cost = wc * mean(trace(cov_k)) + wt * K
    // 4) cost = wc * sum(trace(cov_k))

    double approxEdgeCost = informationCostWeight_*filteringCost + timeCostWeight_*stepsToStop;   // 1,2,3)
    //double approxEdgeCost = informationCostWeight_*filteringCost.value();   // 4)

    return approxEdgeCost;
}

double FIRMCP::computeApproxStabilizationCost(const FIRM::Vertex a, const FIRM::Vertex b)
{
    // declare local variables
    ompl::base::State* startNodeState = stateProperty_[a];
    ompl::base::State* targetNodeState = stateProperty_[b];

    // compute the covariance ratio between two states
    // NOTE more rigorously, startTraceCov should be after maxNumConvergence step of computeApproxTransitionCost()
    double startTraceCov = startNodeState->as<FIRM::StateType>()->getTraceCovariance();
    double targetTraceCov = targetNodeState->as<FIRM::StateType>()->getTraceCovariance();
    double covRatio = targetTraceCov / startTraceCov;
    covRatio = (covRatio < 1.0) ? covRatio : 1.0;

    // compute an approximate stabilization cost (heuristically!)
    double numCovConvergence = std::log(covRatio) / std::log(covConvergenceRate_);

    double maxNumConvergence = numCovConvergence;
    double stepsToStop = maxNumConvergence;
    double filteringCost = (covConvergenceRate_ == 1.0) ? (startTraceCov * stepsToStop) : ( startTraceCov * covConvergenceRate_ * (1 - std::pow(covConvergenceRate_, stepsToStop)) / (1 - covConvergenceRate_) );  // sum_{k=1}^{stepsToStop}(covConvergenceRate^k * startTraceCov)

    // NOTE how to penalize uncertainty (covariance) and path length (time steps) in the cost
    //*1) cost = wc * sum(trace(cov_k))  + wt * K  (for k=1,...,K)
    // 2) cost = wc * trace(cov_f)       + wt * K
    // 3) cost = wc * mean(trace(cov_k)) + wt * K
    // 4) cost = wc * sum(trace(cov_k))

    double approxStabCost = informationCostWeight_*filteringCost + timeCostWeight_*stepsToStop;   // 1,2,3)
    //double approxStabCost = informationCostWeight_*filteringCost.value();   // 4)

    // for debug
    if(ompl::magic::PRINT_EDGE_COST)
        std::cout << "approxStabCost[" << a << "->" << b << "] " << approxStabCost << std::endl;

    return approxStabCost;
}

double FIRMCP::getCostToGoWithApproxStabCost(const Vertex vertex)
{
    if (costToGoWithApproxStabCost_.find(vertex) == costToGoWithApproxStabCost_.end())
    {
        if (!updateCostToGoWithApproxStabCost(vertex))
        {
            //OMPL_ERROR("Failed to updateCostToGoWithApproxStabCost()!");
            return infiniteCostToGo_;
        }
    }

    return costToGoWithApproxStabCost_.at(vertex);
}

bool FIRMCP::updateCostToGoWithApproxStabCost(const Vertex current)
{
    const Vertex goal = goalM_[0];

    if (current == goal)
    {
        costToGoWithApproxStabCost_[current] = costToGo_.at(current);  // no need to update
        return true;
    }

    // Check if feedback from current to goal is valid or not
    if(!isFeedbackPolicyValid(current, goal))
    {
        //OMPL_WARN("No feedback path is found for the given node %d!", current);
        costToGoWithApproxStabCost_[current] = infiniteCostToGo_;  // fill infinite value here to avoid repetitive call for this node
        return false;
    }
    // check if feedback_ path exists for this node
    if (feedback_.find(current) == feedback_.end())
    {
        //OMPL_WARN("No feedback path is found for the given node %d!", current);
        costToGoWithApproxStabCost_[current] = infiniteCostToGo_;  // fill infinite value here to avoid repetitive call for this node
        return false;
    }
    if (costToGo_[current] >= infiniteCostToGo_)
    {
        //OMPL_WARN("costToGo_ is already larger than or equal to infiniteCostToGo_!");
        costToGoWithApproxStabCost_[current] = infiniteCostToGo_;  // fill infinite value here to avoid repetitive call for this node
        return false;
    }

    Edge edge = feedback_.at(current);
    Vertex next = boost::target(edge, g_);
    if (next == goal)
    {
        costToGoWithApproxStabCost_[current] = costToGo_.at(current);  // no need to update
        return true;
    }

    // recursively update the cost-to-go with approximate stabilization cost
    // 1) rigorously, approximate stabilization cost should start from a covariance which is larger than that of next by epsilon
//     costToGoWithApproxStabCost_[current] = costToGo_.at(current) + (getCostToGoWithApproxStabCost(next) - costToGo_.at(next)) + computeApproxStabilizationCost(next+epsilon, next);
    // 2) more rigorously, approximate stabilization cost should consider history from the start (computed by forward simulation like FIRM-Offline)
//     costToGoWithApproxStabCost_[current] = costToGo_.at(current) + (getCostToGoWithApproxStabCost(next) - costToGo_.at(next)) + computeApproxStabilizationCost(forward_simulated_next_reached, next_stationary_cov);
    // 3) approximately, mutliply an inflation factor to compensate under-estimation of actual (history dependent) stabilization cost approximately computed with stationary covariances (which is usually critical in the paths near the start point with high covariance)
    costToGoWithApproxStabCost_[current] = costToGo_.at(current) + (getCostToGoWithApproxStabCost(next) - costToGo_.at(next)) + inflationForApproxStabCost_*computeApproxStabilizationCost(current, next);

    return true;
}

bool FIRMCP::executeSimulationFromUpto(const int kStep, const int numSteps, const ompl::base::State *startState, const Edge& selectedEdge, ompl::base::State* endState, double& executionCost)
{
    bool edgeControllerStatus;
    bool nodeControllerStatus;

    ompl::base::Cost costCov;
    int stepsExecuted = 0;
    int stepsToStop = 0;

    int currentTimeStep = 0;
    double executionCostCov = 0.0;
    executionCost = 0.0;

    const Vertex start = startM_[0];
    const Vertex goal = goalM_[0];

    ompl::base::State *cstartState = si_->allocState();
    ompl::base::State *cendState = si_->allocState();
    ompl::base::State *goalState = stateProperty_[goal];
    ompl::base::State *tempTrueStateCopy = si_->allocState();

    siF_->copyState(cstartState, startState);


    Vertex targetNode = boost::target(selectedEdge, g_);

    // [1] EdgeController
    // NOTE instead of generating edge controllers immediately after adding a node, do that lazily at the time the controller is actually being used!
    //EdgeControllerType& edgeController = edgeControllers_.at(selectedEdge);
    EdgeControllerType& edgeController = getEdgeControllerOnPOMCPTree(selectedEdge);
    edgeController.setSpaceInformation(policyExecutionSI_);
    if(!edgeController.isTerminated(cstartState, 0))  // check if cstartState is near to the target FIRM node (by x,y position); this is the termination condition B) for EdgeController::Execute()
    {
        // NOTE do not execute edge controller to prevent jiggling motion around the target node

        // NOTE let the edge controller know which step the current state is at, especially when following the same edge controller of the previous iteration
        //edgeControllerStatus = edgeController.executeUpto(numSteps, cstartState, cendState, costCov, stepsExecuted, false);
        edgeControllerStatus = edgeController.executeFromUpto(kStep, numSteps, cstartState, cendState, costCov, stepsExecuted, false);


        // NOTE how to penalize uncertainty (covariance) and path length (time steps) in the cost
        //*1) cost = wc * sum(trace(cov_k))  + wt * K  (for k=1,...,K)
        // 2) cost = wc * trace(cov_f)       + wt * K
        // 3) cost = wc * mean(trace(cov_k)) + wt * K
        // 4) cost = wc * sum(trace(cov_k))

        currentTimeStep += stepsExecuted;

        executionCostCov += costCov.value() - ompl::magic::EDGE_COST_BIAS;    // 1,2,3,4) costCov is actual execution cost but only for covariance penalty (even without weight multiplication)

        executionCost = informationCostWeight_*executionCostCov + timeCostWeight_*currentTimeStep;    // 1)
        //executionCost = informationCostWeight_*executionCostCov/(currentTimeStep==0 ? 1e-10 : currentTimeStep) + timeCostWeight_*currentTimeStep;    // 3)
        //executionCost = informationCostWeight_*executionCostCov;    // 4)

        costHistory_.push_back(std::make_tuple(currentTimeStep, executionCostCov, executionCost));


        // if edgeControllerStatus is false (usually due to collision or too much of deviation)
        if(!edgeControllerStatus)
        {
            OMPL_INFORM("Edge controller failed :(");

            // return the simulated result state
            siF_->copyState(endState, cendState);

            return false;
        }

        // this is a secondary (redundant) collision check for the true state
        siF_->getTrueState(tempTrueStateCopy);
        if(!siF_->isValid(tempTrueStateCopy))
        {
            OMPL_INFORM("Robot Collided :(");

            // return the simulated result state
            siF_->copyState(endState, cendState);

            return false;
        }

    } // [1] EdgeController

    // [2] NodeController
    else
    {
        NodeControllerType& nodeController = nodeControllers_.at(targetNode);
        nodeController.setSpaceInformation(policyExecutionSI_);

        //nodeControllerStatus = nodeController.StabilizeUpto(numSteps, cstartState, cendState, costCov, stepsExecuted, false);
        // NOTE to reduce the number of mostly identical POMCP tree nodes during stabilization, inflate the number of execution steps
        nodeControllerStatus = nodeController.StabilizeUpto(scaleStabNumSteps_*numSteps, cstartState, cendState, costCov, stepsExecuted, false);


        // NOTE how to penalize uncertainty (covariance) and path length (time steps) in the cost
        //*1) cost = wc * sum(trace(cov_k))  + wt * K  (for k=1,...,K)
        // 2) cost = wc * trace(cov_f)       + wt * K
        // 3) cost = wc * mean(trace(cov_k)) + wt * K
        // 4) cost = wc * sum(trace(cov_k))

        currentTimeStep += stepsExecuted;

        executionCostCov += costCov.value() - ompl::magic::EDGE_COST_BIAS;    // 1,2,3,4) costCov is actual execution cost but only for covariance penalty (even without weight multiplication)

        executionCost = informationCostWeight_*executionCostCov + timeCostWeight_*currentTimeStep;    // 1)
        //executionCost = informationCostWeight_*executionCostCov/(currentTimeStep==0 ? 1e-10 : currentTimeStep) + timeCostWeight_*currentTimeStep;    // 3)
        //executionCost = informationCostWeight_*executionCostCov;    // 4)

        costHistory_.push_back(std::make_tuple(currentTimeStep, executionCostCov, executionCost));


        // if nodeControllerStatus is false (usually due to too many iterations, more than maxTries_)
        if(!nodeControllerStatus)
        {
            OMPL_INFORM("Node controller failed :(");

            // return the simulated result state
            siF_->copyState(endState, cendState);

            return false;
        }

        // this is a secondary (redundant) collision check for the true state
        siF_->getTrueState(tempTrueStateCopy);
        if(!siF_->isValid(tempTrueStateCopy))
        {
            OMPL_INFORM("Robot Collided :(");

            // return the simulated result state
            siF_->copyState(endState, cendState);

            return false;
        }

    } // [2] NodeController


    // return the simulated result state
    siF_->copyState(endState, cendState);

    // free the memory
    si_->freeState(cstartState);
    si_->freeState(cendState);
    si_->freeState(tempTrueStateCopy);

    return true;
}

bool FIRMCP::updateQVnodeBeliefOnPOMCPTree(const Vertex currentVertex, const Vertex selectedChildQnode, const ompl::base::State* evolvedBelief, Vertex& evolvedVertex, const bool reset)
{
    // NOTE if the evolved belief is near to any of existing childQVnodes_[selectedChildQnode] for the same action, merge it to the existing node

    // declare local variables
    ompl::base::State* currentBelief = stateProperty_[currentVertex];  // latest start state that is already executed

    // const Vertex selectedChildQVnode = currentBelief->as<FIRM::StateType>()->getChildQVnode(selectedChildQnode);
    // if (selectedChildQVnode != ompl::magic::INVALID_VERTEX_ID)
    // {
    //     evolvedVertex = selectedChildQVnode;
    //
    //     // update the matching belief state
    //     // NOTE this function should be called before incrementing N(h) by +1
    //     stateProperty_[evolvedVertex]->as<FIRM::StateType>()->mergeBeliefIntoThis(evolvedBelief);
    //
    //     // for debug
    //     std::cout << "selectedChildQVnodes.size(): " << selectedChildQVnodes.size() << std::endl;
    // }
    // else
    // {
    //     //OMPL_INFORM("A new childQVnode after execution!");
    //     evolvedVertex = addQVnodeToPOMCPTree(siF_->cloneState(evolvedBelief));
    //     currentBelief->as<FIRM::StateType>()->addChildQVnode(selectedChildQnode, evolvedVertex);
    // }

    std::map<double, Vertex> reachedChildQVnodesByDistance;  // this is sorted by distance in increasing order
    const std::vector<Vertex>& selectedChildQVnodes = currentBelief->as<FIRM::StateType>()->getChildQVnodes(selectedChildQnode);
    // for debug
    //std::cout << "selectedChildQVnodes.size(): " << selectedChildQVnodes.size() << std::endl;

    // check for coincident nodes on the POMCP tree
    for (const auto& childQVnode : selectedChildQVnodes)
    {
        // NOTE need to check for both directions since there is no from-to relationship between these selectedChildQVnodes
        // HACK larger/smaller nEpsilonForQVnodeMerging_ for looser/tighter threshold for merging
        if (stateProperty_[childQVnode]->as<FIRM::StateType>()->isReachedWithinNEpsilon(evolvedBelief, nEpsilonForQVnodeMerging_))
        {
            if (evolvedBelief->as<FIRM::StateType>()->isReachedWithinNEpsilon(stateProperty_[childQVnode], nEpsilonForQVnodeMerging_))
            {
                //OMPL_WARN("Coinciding with existing childQVnodes!");

                // compute distance (weighted sum of position and orientation distances)
                // NOTE similarity in covariance is not considered in sorting coincident nodes
                double dist = evolvedBelief->as<FIRM::StateType>()->getStateDistanceTo(stateProperty_[childQVnode]);

                // add this to the candidate list
                reachedChildQVnodesByDistance.insert(std::pair<double, Vertex>(dist, childQVnode));
            }
        }
    }

    // if there are some coincident nodes, merge the evolved belief into the closest one
    if (reachedChildQVnodesByDistance.size() > 0)
    {
        //OMPL_WARN("There are %d reachedChildQVnodes!", reachedChildQVnodes.size());

        // pick the closest coincident node
        Vertex closestChildQVnode = reachedChildQVnodesByDistance.begin()->second;
        evolvedVertex = closestChildQVnode;

        if (!reset)
        {
            // update the matching belief state
            // NOTE this function should be called before incrementing N(h) by +1
            stateProperty_[evolvedVertex]->as<FIRM::StateType>()->mergeBeliefIntoThis(evolvedBelief);
        }
        else
        {
            // NOTE to clean up noisy beliefs from previous simulations (possibly with a larger nSigmaForPOMCPParticle_)
            siF_->copyState(stateProperty_[evolvedVertex], evolvedBelief);
        }
    }
    // otherwise, add a new node to the POMCP tree
    else
    {
        //OMPL_INFORM("A new childQVnode after execution!");
        evolvedVertex = addQVnodeToPOMCPTree(siF_->cloneState(evolvedBelief));
        currentBelief->as<FIRM::StateType>()->addChildQVnode(selectedChildQnode, evolvedVertex);
    }

    return true;
}

bool FIRMCP::updateQVnodeValuesOnPOMCPTree(const Vertex currentVertex, const Vertex selectedChildQnode, const bool executionStatus, const double executionCost, const double selectedChildQVmincosttogo, double& thisQVmincosttogoUpdated, const bool isNewNodeExpanded)
{
    // local variables
    ompl::base::State* currentBelief = stateProperty_[currentVertex];  // latest start state that is already executed
    

    // update the number of visits and misses
    currentBelief->as<FIRM::StateType>()->addThisQVvisit();                       // N(h) += 1
    currentBelief->as<FIRM::StateType>()->addChildQvisit(selectedChildQnode);     // N(ha) += 1
    if (!executionStatus)  // if not collided during latest execution
    {
        currentBelief->as<FIRM::StateType>()->addChildQmiss(selectedChildQnode);  // M(ha) += 1
    }


    // update the cost-to-go

    // Q_k(ha) = c(a) + ( J(hao) or J_obs )
    // Q(ha) = c(a) + p * J(hao) + (1 - p) * J_obs
    //       = Q(ha) + (Q_k(ha) - Q(ha)) / N(ha), where
    // J(h) = min_a'(Q(ha')), but not necessarily J(h) = min(J(h), Q(ha)), since Q(ha) changes (possibly increases) over time

    double selectedChildQvisit = currentBelief->as<FIRM::StateType>()->getChildQvisit(selectedChildQnode);        // N(ha)
    double selectedChildQmiss = currentBelief->as<FIRM::StateType>()->getChildQmiss(selectedChildQnode);          // M(ha)  // not in use
    double selectedChildQcosttogo = currentBelief->as<FIRM::StateType>()->getChildQcosttogo(selectedChildQnode);  // Q(ha)
    double thisQVmincosttogo = currentBelief->as<FIRM::StateType>()->getThisQVmincosttogo();                      // J(h)
    double selectedChildQcosttogoUpdated;

    // Q_k(ha): total cost-to-go along this trajectory
    selectedChildQcosttogoUpdated = executionCost;
    if (executionStatus)
        selectedChildQcosttogoUpdated += selectedChildQVmincosttogo;
    else
        selectedChildQcosttogoUpdated += obstacleCostToGo_;

    // Q(ha) = Q(ha) + (Q_k(ha) - Q(ha)) / N(ha)
    if (isNewNodeExpanded)
    {
        selectedChildQcosttogo = 0.0;  // to discard initial Q(ha) of heuristic approxCostToGo value for the first new node during POMCP-Rollout
    }
    selectedChildQcosttogoUpdated = selectedChildQcosttogo + (selectedChildQcosttogoUpdated - selectedChildQcosttogo) / selectedChildQvisit;

    // J(h) = min_a'(Q(ha'))
    if (thisQVmincosttogo > selectedChildQcosttogoUpdated)
    {
        thisQVmincosttogoUpdated = selectedChildQcosttogoUpdated;
    }
    else
    {
        // even in this case, check for J(h) = min_a'(Q(ha')) 
        // since it is possible that Q(ha*) where a* = argmin_a'(Q(ha')) has increased, so that J(h)=Q(ha*) before the update is no longer the minimum value
        thisQVmincosttogoUpdated = selectedChildQcosttogoUpdated;
        const std::vector<Vertex>& childQnodes = currentBelief->as<FIRM::StateType>()->getChildQnodes();
        for (int j=0; j<childQnodes.size(); j++)
        {
            const Vertex childQnode = childQnodes[j];
            const double childQcosttogo = stateProperty_[currentVertex]->as<FIRM::StateType>()->getChildQcosttogo(childQnode);

            if (thisQVmincosttogoUpdated > childQcosttogo)
            {
                thisQVmincosttogoUpdated = childQcosttogo;
            }
        }
    }


    // save the updated cost-to-go values
    currentBelief->as<FIRM::StateType>()->setChildQcosttogo(selectedChildQnode, selectedChildQcosttogoUpdated);
    currentBelief->as<FIRM::StateType>()->setThisQVmincosttogo(thisQVmincosttogoUpdated);

    return true;
}

void FIRMCP::prunePOMCPTreeFrom(const Vertex rootVertex)
{
    ompl::base::State* rootState = stateProperty_[rootVertex];

    // recursively call prunePOMCPTreeFrom() to destruct the descendent nodes starting from the leaves
    if (rootState->as<FIRM::StateType>()->getChildQexpanded())
    {
        const std::vector<Vertex>& childQnodes = rootState->as<FIRM::StateType>()->getChildQnodes();
        for (const auto& childQnode : childQnodes)
        {
            // const Vertex childQVnode = rootState->as<FIRM::StateType>()->getChildQVnode(childQnode);
            // if (childQVnode != ompl::magic::INVALID_VERTEX_ID)
            // {
            //     prunePOMCPTreeFrom(childQVnode);
            // }
            const std::vector<Vertex>& childQVnodes = rootState->as<FIRM::StateType>()->getChildQVnodes(childQnode);
            for (const auto& childQVnode : childQVnodes)
            {
                prunePOMCPTreeFrom(childQVnode);
            }
        }
    }

    // prune this node on POMCP tree
    prunePOMCPNode(rootVertex);
}

void FIRMCP::prunePOMCPNode(const Vertex rootVertex)
{
    //const Vertex start = startM_[0];
    if (rootVertex != startM_[0])
    {
        // free the memory of controller
        // NOTE there is no node controller generated for POMCP tree nodes during rollout execution
        foreach(Edge edge, boost::out_edges(rootVertex, g_))
        {
            if (edgeControllers_.find(edge) != edgeControllers_.end())
            {
                edgeControllers_.at(edge).freeSeparatedController();
                //edgeControllers_.at(edge).freeLinearSystems();
                edgeControllers_.erase(edge);
            }
        }

        // free the memory of state
        siF_->freeState(stateProperty_[rootVertex]);

        // remove the node/edges from POMCP tree
        boost::clear_vertex(rootVertex, g_);     // remove all edges from or to rootVertex
        //boost::remove_vertex(rootVertex, g_);  // remove rootVertex  // NOTE commented this to avoid confusion from vertex IDs
        //stateProperty_.erase(rootVertex);
    }
}

// for FIRM-Rollout
FIRM::Edge FIRMCP::generateRolloutPolicy(const FIRM::Vertex currentVertex, const FIRM::Vertex goal)
{
    /**
        For the given node, find the out edges and see the total cost of taking that edge
        The cost of taking the edge is cost to go from the target of the edge + the cost of the edge itself
    */
    double minCost = std::numeric_limits<double>::max();
    Edge edgeToTake;

    // for debug
    FIRM::Vertex minCostVertCurrent, minCostVertNext, minCostVertNextNext;

    // Iterate over the out edges
    foreach(Edge e, boost::out_edges(currentVertex, g_))
    {

        // Get the target node of the edge
        Vertex targetNode = boost::target(e, g_);  

        // The FIRM edge to take from the target node
        //Edge nextFIRMEdge = feedback_.at(targetNode);

        // The node to which next firm edge goes
        //Vertex targetOfNextFIRMEdge = boost::target(nextFIRMEdge, g_);

        // for debug
        if(ompl::magic::PRINT_FEEDBACK_PATH)
            std::cout << "PATH[" << currentVertex;

        // Check if feedback from target to goal is valid or not
        if(!isFeedbackPolicyValid(targetNode, goal))
        {
            // //OMPL_INFORM("Rollout: Invalid path detected from Vertex %u to %u", targetNode, targetOfNextFIRMEdge);
            //
            // updateEdgeCollisionCost(targetNode, goal);
            //
            // // resolve Dijkstra/DP
            // solveDijkstraSearch(goal);
            // solveDynamicProgram(goal, false);
            //
            // //targetOfNextFIRMEdge = boost::target(feedback_.at(targetNode), g_);
            // //OMPL_INFORM("Rollout: Updated path, next firm edge moving from Vertex %u to %u", targetNode, targetOfNextFIRMEdge);

            updateCostToGoWithApproxStabCost(targetNode);
        }

        // The cost to go from the target node
        // NOTE instead of the raw costToGo_ from FIRM, use costToGoWithApproxStabCost_ with stabilization cost compenstation along the feedback paths
        //double nextNodeCostToGo = costToGo_[targetNode];
        double nextNodeCostToGo = getCostToGoWithApproxStabCost(targetNode);

        // Find the weight of the edge
        FIRMWeight edgeWeight =  boost::get(boost::edge_weight, g_, e);

        // The transition prob of the edge
        double transitionProbability = edgeWeight.getSuccessProbability();        
        
        // compute distance from goal to target
        //arma::colvec targetToGoalVec = stateProperty_[goal]->as<FIRM::StateType>()->getArmaData() - stateProperty_[targetNode]->as<FIRM::StateType>()->getArmaData();
        //double distToGoalFromTarget = arma::norm(targetToGoalVec.subvec(0,1),2); 

        // get the stationary penalty
        // NOTE this is to myopically improve the suboptimal policy based on approximate value function (with inaccurate edge cost induced from isReached() relaxation)
        double stationaryPenalty = 0.0;
        if(stationaryPenalties_.find(targetNode) != stationaryPenalties_.end())
            stationaryPenalty = stationaryPenalties_.at(targetNode);

        // the cost of taking the edge
        //double edgeCostToGo = transitionProbability*nextNodeCostToGo + (1-transitionProbability)*obstacleCostToGo_ + edgeWeight.getCost();
        // NOTE this is to myopically improve the suboptimal policy based on approximate value function (with inaccurate edge cost induced from isReached() relaxation)
        double edgeCostToGo = transitionProbability*nextNodeCostToGo + (1-transitionProbability)*obstacleCostToGo_ + edgeWeight.getCost() + stationaryPenalty;  // HACK only for rollout policy search; actual execution cost will not consider stationaryPenalty


        // for debug
        if(ompl::magic::PRINT_COST_TO_GO)
            //std::cout << "COST[" << currentVertex << "->" << targetNode << "->G] " << edgeCostToGo << " = " << transitionProbability << "*" << nextNodeCostToGo << " + " << "(1-" << transitionProbability << ")*" << obstacleCostToGo_ << " + " << edgeWeight.getCost() << std::endl;
            std::cout << "COST[" << currentVertex << "->" << targetNode << "->G] " << edgeCostToGo << " = " << transitionProbability << "*" << nextNodeCostToGo << " + " << "(1-" << transitionProbability << ")*" << obstacleCostToGo_ << " + " << edgeWeight.getCost() << " + " << stationaryPenalty << std::endl;


        if(edgeCostToGo < minCost)
        {
            minCost  = edgeCostToGo;
            edgeToTake = e;

            // for debug
            minCostVertCurrent = currentVertex;
            minCostVertNext = targetNode;
            //minCostVertNextNext = targetOfNextFIRMEdge;
        }
    }

    // for debug
    if(ompl::magic::PRINT_COST_TO_GO)
        std::cout << "minC[" << minCostVertCurrent << "->" << minCostVertNext << "->G] " << minCost << std::endl;

    return edgeToTake;
}

