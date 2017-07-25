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
#include <boost/circular_buffer.hpp>  // XXX

#define foreach BOOST_FOREACH
#define foreach_reverse BOOST_REVERSE_FOREACH

FIRMCP::FIRMCP(const firm::SpaceInformation::SpaceInformationPtr &si, bool debugMode)
    : FIRM(si, debugMode)
{
}

FIRMCP::~FIRMCP(void)
{
}

void FIRMCP::executeFeedbackWithPOMCP(void)
{
    EdgeControllerType edgeController;
    NodeControllerType nodeController;

    bool edgeControllerStatus;
    bool nodeControllerStatus;


    const Vertex start = startM_[0];
    const Vertex goal = goalM_[0];
    Vertex currentVertex = start;
    Vertex tempVertex = currentVertex;
    Vertex targetNode;

    ompl::base::State *cstartState = si_->allocState();
    ompl::base::State *cendState = si_->allocState();
    ompl::base::State *goalState = si_->cloneState(stateProperty_[goal]);
    ompl::base::State *tempTrueStateCopy = si_->allocState();

    //===== SET A Custom Init Covariance=====================
    // using namespace arma;
    // mat tempCC(3,3);
    // tempCC<< 0.1 << 0.0 << 0.0 << endr
    //         << 0.0 << 0.1 << 0.0 << endr
    //         << 0.0 << 0.0 << 0.0000001<<endr;
    // stateProperty_[start]->as<StateType>()->setCovariance(tempCC);
    // Visualizer::updateCurrentBelief(stateProperty_[start]);
    //================================

    si_->copyState(cstartState, stateProperty_[start]);
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
        targetNode = boost::target(e, g_);

        double succProb = evaluateSuccessProbability(e, tempVertex, goal);
        successProbabilityHistory_.push_back(std::make_pair(currentTimeStep_, succProb ) );

        OMPL_INFORM("FIRMCP: Moving from Vertex %u (%2.3f, %2.3f, %2.3f, %2.6f) to %u (%2.3f, %2.3f, %2.3f, %2.6f) with TP = %f", tempVertex, targetNode, 
                stateProperty_[tempVertex]->as<FIRM::StateType>()->getX(),
                stateProperty_[tempVertex]->as<FIRM::StateType>()->getY(),
                stateProperty_[tempVertex]->as<FIRM::StateType>()->getYaw(),
                arma::trace(stateProperty_[tempVertex]->as<FIRM::StateType>()->getCovariance()),
                stateProperty_[targetNode]->as<FIRM::StateType>()->getX(),
                stateProperty_[targetNode]->as<FIRM::StateType>()->getY(),
                stateProperty_[targetNode]->as<FIRM::StateType>()->getYaw(),
                arma::trace(stateProperty_[targetNode]->as<FIRM::StateType>()->getCovariance()),
                succProb);

        // HACK WORKAROUNDS FOR INDEFINITE STABILIZATION DURING ROLLOUT: {1} CONNECTION TO FUTURE FIRM NODES
        // 5) forcefully include future feedback nodes of several previous target nodes in the candidate (nearest neighbor) list
        // update the future feedback node at every iteration
        if(connectToFutureNodes_)
        {
            Vertex futureVertex = targetNode;
            for(int i=0; i<numberOfFeedbackLookAhead_; i++)
            {
                futureFIRMNodes.push_back(futureVertex);

                if(feedback_.find(futureVertex) != feedback_.end())    // there exists a valid feedback edge for this vertex
                {
                    futureVertex = boost::target(feedback_.at(futureVertex), g_);
                }
                else    // there is no feedback edge coming from this vertex
                {
                    // fill the rest of the buffer with the last valid vertex; redundant vertices will be ignored for rollout check
                    for(int j=i+1; j<numberOfFeedbackLookAhead_; j++)
                    {
                        futureFIRMNodes.push_back(futureVertex);
                    }
                    break;
                }
            }
            // for debug
            // if(ompl::magic::PRINT_FUTURE_NODES)
            // {
            //     std::cout << "futureFIRMNodesDuplicate: { ";
            //     foreach(futureVertex, futureFIRMNodes)
            //         std::cout << futureVertex << " ";
            //     std::cout << "}" << std::endl;
            // }
        }


        /**
          Instead of executing the entire controller, we need to execute N steps, then calculate the cost to go through the neighboring nodes.
          Whichever gives the lowest cost to go, is our new path. Do this at every N steps.
        */
        ompl::base::Cost costCov;
        int stepsExecuted = 0;
        int stepsToStop = 0;


        // NOTE NodeController will be invoked after executing EdgeController for the given rolloutSteps_ steps

        // [1] EdgeController
        edgeController = edgeControllers_.at(e);
        edgeController.setSpaceInformation(policyExecutionSI_);
        if(edgeController.isTerminated(cstartState, 0))  // check if cstartState is near to the target FIRM node (by x,y position); this is the termination condition B) for EdgeController::Execute()
        {
            // NOTE do not execute edge controller to prevent jiggling motion around the target node


            // HACK WORKAROUNDS FOR INDEFINITE STABILIZATION DURING ROLLOUT: {2} ACCUMULATING STATIONARY PENALTY
            if(applyStationaryPenalty_)
            {
                // incrementally penalize a node that is being selected as a target due to the not-yet-converged current covariance even after the robot reached that node's position and orientation
                // NOTE this is to myopically improve the suboptimal policy based on approximate value function (with inaccurate edge cost induced from isReached() relaxation)
                // it will help to break (almost) indefinite stabilization process during rollout, especially when land marks are not very close
                if(stateProperty_[targetNode]->as<FIRM::StateType>()->isReachedPose(cstartState))
                {
                    if(targetNode != goal)
                    {
                        // increase the stationary penalty
                        if(stationaryPenalties_.find(targetNode) == stationaryPenalties_.end())
                        {
                            stationaryPenalties_[targetNode] = statCostIncrement_;
                            // for log
                            numberOfStationaryPenalizedNodes_++;
                        }
                        else
                        {
                            stationaryPenalties_[targetNode] += statCostIncrement_;
                        }
                        // for log
                        sumOfStationaryPenalties_ += statCostIncrement_;
                        stationaryPenaltyHistory_.push_back(std::make_tuple(currentTimeStep_, numberOfStationaryPenalizedNodes_, sumOfStationaryPenalties_));

                        // for debug
                        if(ompl::magic::PRINT_STATIONARY_PENALTY)
                            std::cout << "stationaryPenalty[" << targetNode << "]: " << stationaryPenalties_[targetNode] << std::endl;
                    }
                }
            }

        }
        else
        {
            edgeControllerStatus = edgeController.executeUpto(rolloutSteps_, cstartState, cendState, costCov, stepsExecuted, false);


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
            if(!si_->isValid(tempTrueStateCopy))
            {
                OMPL_INFORM("Robot Collided :(");
                return;
            }

            // update cstartState for next iteration
            si_->copyState(cstartState, cendState);

        } // [1] EdgeController

        // [2] NodeController
        if(edgeController.isTerminated(cstartState, 0))  // check if cstartState is near to the target FIRM node (by x,y position); this is the termination condition B) for EdgeController::Execute()
        {
            //if(stateProperty_[targetNode]->as<FIRM::StateType>()->isReached(cstartState))
            //{
                // NOTE tried applying the stationary penalty if isReached(), instead of isTerminated(), is satisfied, but the resultant policy was more suboptimal
            //}

            // call StabilizeUpto() at every rollout iteration
            {
                nodeController = nodeControllers_.at(targetNode);
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
                if(!si_->isValid(tempTrueStateCopy))
                {
                    OMPL_INFORM("Robot Collided :(");
                    return;
                }

                // update the cstartState for next iteration
                si_->copyState(cstartState, cendState);

            }
        } // [2] NodeController


        // [3] Free the memory for states and controls for this temporary node/edge created from previous iteration
        if(tempVertex != start)
        {
            foreach(Edge edge, boost::out_edges(tempVertex, g_))
            {
                edgeControllers_[edge].freeSeparatedController();
                edgeControllers_[edge].freeLinearSystems();
                edgeControllers_.erase(edge);
            }

            // NOTE there is no node controller generated for this temporary node during rollout execution


            // remove the temporary node/edges after executing one rollout iteration
            // NOTE this is important to keep tempVertex to be the same over each iteration
            boost::clear_vertex(tempVertex, g_);    // remove all edges from or to tempVertex
            boost::remove_vertex(tempVertex, g_);   // remove tempVertex
            //stateProperty_.erase(tempVertex);
            nn_->remove(tempVertex);
        }


        // [4] Rollout
        if(stateProperty_[targetNode]->as<FIRM::StateType>()->isReached(cendState))
        {
            OMPL_INFORM("FIRMCP: Reached FIRM Node: %u", targetNode);
            numberofNodesReached_++;
            nodeReachedHistory_.push_back(std::make_pair(currentTimeStep_, numberofNodesReached_) );

            // NOTE commented this to do rollout even if the robot (almost) reached a FIRM node
            // tempVertex = boost::target(e, g_);
            // // if the reached node is not the goal
            // if(tempVertex != goal)
            // {
            //     e = feedback_.at(tempVertex);    // NOTE this is not guaranteed to be the best
            //     // for debug
            //     nextFIRMVertex = boost::target(e, g_);
            // }
        }
        // NOTE commented this to do rollout even if the robot (almost) reached a FIRM node
        //else
        {
            Visualizer::doSaveVideo(false);
            siF_->doVelocityLogging(false);

            // start profiling time to compute rollout
            auto start_time = std::chrono::high_resolution_clock::now();

            // save the current true state
            siF_->getTrueState(tempTrueStateCopy);

            // add the current belief state to the graph temporarily
            tempVertex = addStateToGraph(cendState, false);

            // restore the current true state
            siF_->setTrueState(tempTrueStateCopy);


            // NOTE robust connection to a desirable (but far) FIRM nodes during rollout

            // 1) allow longer edge length for connection to FIRM nodes during rollout
            // implemented in FIRM::addStateToGraph()

            // 2) forcefully include the current state's k-nearest neighbors of FIRM nodes in the candidate list for rollout policy
            // implemented in FIRM::addStateToGraph() for si_->checkMotion() validation

            // 3) instead of bounded nearest neighbors, use k-nearest neighbors during rollout; this may be considered as a change of the graph connection property
            // implemented in FIRM::addStateToGraph() for si_->checkMotion() validation

            // 4) forcefully include the next FIRM node of the previously reached FIRM node in the candidate (nearest neighbor) list for rollout policy
            // bool forwardEdgeAdded;
            // if(si_->checkMotion(stateProperty_[tempVertex], stateProperty_[nextFIRMVertex]))
            // {
            //     addEdgeToGraph(tempVertex, nextFIRMVertex, forwardEdgeAdded);
            // }

            // HACK WORKAROUNDS FOR INDEFINITE STABILIZATION DURING ROLLOUT: {1} CONNECTION TO FUTURE FIRM NODES
            // 5) forcefully include future feedback nodes of several previous target nodes in the candidate (nearest neighbor) list
            if(connectToFutureNodes_)
            {
                Vertex futureVertex;
                bool forwardEdgeAdded;
                // for debug
                if(ompl::magic::PRINT_FUTURE_NODES)
                    std::cout << "futureFIRMNodes: { ";
                for(int i=0; i<futureFIRMNodes.size(); i++)
                {
                    futureVertex = futureFIRMNodes[i];
                    // check if not duplicate with the previous nodes
                    bool unique = false;
                    if(std::find(futureFIRMNodes.begin(), futureFIRMNodes.begin()+i-0, futureVertex) == futureFIRMNodes.begin()+i-0)
                    {
                        unique = true;
                        foreach(Edge nnedge, boost::out_edges(tempVertex, g_))
                        {
                            if(futureVertex == boost::target(nnedge, g_))
                            {
                                unique = false;
                                break;
                            }
                        }
                    }
                    // check for motion validity and add to the graph
                    if(unique)
                    {
                        if(si_->checkMotion(stateProperty_[tempVertex], stateProperty_[futureVertex]))
                        {
                            addEdgeToGraph(tempVertex, futureVertex, forwardEdgeAdded);
                            Visualizer::addRolloutConnection(stateProperty_[tempVertex], stateProperty_[futureVertex]);
                        }
                        // for debug
                        if(ompl::magic::PRINT_FUTURE_NODES)
                            std::cout << futureVertex << " ";
                    }
                }
                // for debug
                if(ompl::magic::PRINT_FUTURE_NODES)
                    std::cout << "}" << std::endl;
            }


            // set true state back to its correct value after Monte Carlo (happens during adding state to Graph)
            siF_->setTrueState(tempTrueStateCopy);

            // select the best next edge
            e = generateRolloutPolicy(tempVertex, goal);


            // end profiling time to compute rollout
            auto end_time = std::chrono::high_resolution_clock::now();

            numberOfRollouts++;
            double timeToDoRollout = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            //int numNN = numNearestNeighbors_;
            //averageTimeForRolloutComputation += timeToDoRollout / numNN;    // rollout candidates are no longer limited to numNearestNeighbors_
            if(doSaveLogs_)
            {
                //outfile<<numberOfRollouts<<","<<NNRadius_<<","<<numNN<<","<<numMCParticles_<<","<<timeToDoRollout/(1000*numNN)<<","<<timeToDoRollout/1000<<std::endl;
                outfile<<numberOfRollouts<<","<<NNRadius_<<","<<numMCParticles_<<","<<timeToDoRollout/1000<<std::endl;
            }
            //std::cout << "Time to execute rollout : "<<timeToDoRollout << " milli seconds."<<std::endl;

            Visualizer::doSaveVideo(doSaveVideo_);
            siF_->doVelocityLogging(true);


            // NOTE commented to prevent redundant call of nearest neighbor search just for visualization! moved to FIRM::addStateToGraph()
            //showRolloutConnections(tempVertex);

            // for rollout edge visualization
            //boost::this_thread::sleep(boost::posix_time::milliseconds(50));  // doesn't seem to be necessary

            // clear the rollout candidate connection drawings and show the selected edge
            Visualizer::clearRolloutConnections();
            Visualizer::setChosenRolloutConnection(stateProperty_[tempVertex], stateProperty_[targetNode]);

        } // [4] Rollout

    } // while()

    // [3'] Free the memory for states and controls for this temporary node/edge created from previous iteration
    if(tempVertex != start)
    {
        foreach(Edge edge, boost::out_edges(tempVertex, g_))
        {
            edgeControllers_[edge].freeSeparatedController();
            edgeControllers_[edge].freeLinearSystems();
            edgeControllers_.erase(edge);
        }

        // NOTE there is no node controller generated for this temporary node during rollout execution


        // remove the temporary node/edges after executing one rollout iteration
        // NOTE this is important to keep tempVertex to be the same over each iteration
        boost::clear_vertex(tempVertex, g_);    // remove all edges from or to tempVertex
        boost::remove_vertex(tempVertex, g_);   // remove tempVertex
        //stateProperty_.erase(tempVertex);
        nn_->remove(tempVertex);
    }

    nodeReachedHistory_.push_back(std::make_pair(currentTimeStep_, numberofNodesReached_) );

    //OMPL_INFORM("FIRM: Number of nodes reached with Rollout: %u", numberofNodesReached_);
    //averageTimeForRolloutComputation = averageTimeForRolloutComputation / (1000*numberOfRollouts);
    //std::cout<<"Nearest Neighbor Radius: "<<NNRadius_<<", Monte Carlo Particles: "<<numMCParticles_<<", Avg Time/neighbor (seconds): "<<averageTimeForRolloutComputation<<std::endl;    


    // for analysis

    // this data is also saved in run-(TIMESTAMP)/RolloutFIRMCostHistory.csv
    std::cout << std::endl;
    std::cout << "Execution time steps: " << currentTimeStep_ << std::endl;
    std::cout << "Execution covariance cost: " << executionCostCov_ << std::endl;
    std::cout << "Execution cost: " << executionCost_ << "  ( = " << informationCostWeight_ << "*" << executionCostCov_ << " + " << timeCostWeight_ << "*" << currentTimeStep_ << " )" << std::endl;     // 1)
    //std::cout << "Execution cost: " << executionCost_ << "  ( = " << informationCostWeight_ << "*" << executionCostCov_ << "/" << currentTimeStep_ << " + " << timeCostWeight_ << "*" << currentTimeStep_ << " )" << std::endl;     // 3)
    //std::cout << "Execution cost: " << executionCost_ << "  ( = " << informationCostWeight_ << "*" << executionCostCov_ << " )" << std::endl;     // 4)
    std::cout << std::endl;

    // this data is also saved in run-(TIMESTAMP)/RolloutFIRMCostHistory.csv
    std::cout << "Number of nodes with stationary penalty: " << numberOfStationaryPenalizedNodes_ << std::endl;
    std::cout << "Sum of stationary penalties: " << sumOfStationaryPenalties_ << std::endl;


    if(doSaveLogs_)
    {
        outfile.close();
        writeTimeSeriesDataToFile("RolloutFIRMCostHistory.csv", "costToGo");
        writeTimeSeriesDataToFile("RolloutFIRMSuccessProbabilityHistory.csv", "successProbability");
        writeTimeSeriesDataToFile("RolloutFIRMNodesReachedHistory.csv","nodesReached");
        writeTimeSeriesDataToFile("RolloutFIRMStationaryPenaltyHistory.csv","stationaryPenalty");
        std::vector<std::pair<double, double>> velLog;
        siF_->getVelocityLog(velLog);
        for(int i=0; i < velLog.size(); i++)
        {
            velocityHistory_.push_back(std::make_pair(i, sqrt( pow(velLog[i].first,2) + pow(velLog[i].second,2) ))); // omni
            //velocityHistory_.push_back(std::make_pair(i, velLog[i].first)); // unicycle
        }
        writeTimeSeriesDataToFile("RolloutFIRMVelocityHistory.csv", "velocity");
    }
    Visualizer::doSaveVideo(false);

    // free the memory
    si_->freeState(cstartState);
    si_->freeState(cendState);
    si_->freeState(tempTrueStateCopy);
}

