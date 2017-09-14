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
/* Author: Ali-akbar Agha-mohammadi, Saurav Agarwal */

#include "ompl/geometric/planners/prm/PRM.h"
#include "ompl/geometric/planners/prm/ConnectionStrategy.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/datastructures/PDF.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/tools/config/MagicConstants.h"
#include <boost/lambda/bind.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/thread.hpp>
#include <tinyxml.h>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/circular_buffer.hpp>
#include "Visualization/Visualizer.h"
#include "Utils/FIRMUtils.h"
#include "Planner/FIRM.h"

#define foreach BOOST_FOREACH
#define foreach_reverse BOOST_REVERSE_FOREACH

FIRM::FIRM(const firm::SpaceInformation::SpaceInformationPtr &si, bool debugMode) :
    ompl::base::Planner(si, "FIRM"),
    siF_(si),
    stateProperty_(boost::get(vertex_state_t(), g_)),
    totalConnectionAttemptsProperty_(boost::get(vertex_total_connection_attempts_t(), g_)),
    successfulConnectionAttemptsProperty_(boost::get(vertex_successful_connection_attempts_t(), g_)),
    weightProperty_(boost::get(boost::edge_weight, g_)),
    edgeIDProperty_(boost::get(boost::edge_index, g_)),
    disjointSets_(boost::get(boost::vertex_rank, g_),
                  boost::get(boost::vertex_predecessor, g_)),
    maxEdgeID_(0),
    userSetConnectionStrategy_(false),
    addedSolution_(false)
{
    arma::arma_rng::set_seed_random();

    specs_.recognizedGoal = ompl::base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;

    minFIRMNodes_ = 25;
    maxFIRMNodes_ = 100;

    currentTimeStep_ = 0;

    executionCostCov_ = 0;

    executionCost_ = 0;

    numberofNodesReached_ = 0;

    costHistory_.push_back(std::make_tuple(currentTimeStep_, executionCostCov_, executionCost_));

    numberOfStationaryPenalizedNodes_ = 0;

    sumOfStationaryPenalties_ = 0;

    stationaryPenaltyHistory_.push_back(std::make_tuple(currentTimeStep_, numberOfStationaryPenalizedNodes_, sumOfStationaryPenalties_));

    policyGenerator_ = new NBM3P(si);

    policyExecutionSI_ = siF_; // by default policies are executed in the same space that the roadmap is generated

    logFilePath_ = "./";

    loadedRoadmapFromFile_ = false;

    numMCParticles_ = 5;

    doSavePlannerData_ = false;

    doSaveLogs_ = false;

    doSaveVideo_ = false;

    NNRadius_ = ompl::magic::DEFAULT_NEAREST_NEIGHBOUR_RADIUS;

    numNearestNeighbors_ = ompl::magic::DEFAULT_NEAREST_NEIGHBORS;

    rolloutSteps_ = ompl::magic::DEFAULT_STEPS_TO_ROLLOUT ;

    discountFactorDP_  = ompl::magic::DEFAULT_DP_DISCOUNT_FACTOR; 

    distanceCostWeight_ = ompl::magic::DEFAULT_DISTANCE_TO_GOAL_COST_WEIGHT;


    connectToFutureNodes_ = ompl::magic::CONNECT_TO_FUTURE_NODES;

    applyStationaryPenalty_ = ompl::magic::APPLY_STATIONARY_PENALTY;

    borderBeliefSampling_ = ompl::magic::BORDER_BELIEF_SAMPLING;


    informationCostWeight_ = ompl::magic::DEFAULT_INFORMATION_COST_WEIGHT;

    timeCostWeight_ = ompl::magic::DEFAULT_TIME_TO_STOP_COST_WEIGHT;

    statCostIncrement_ = ompl::magic::DEFAULT_STATIONARY_PENALTY_INCREMENT;


    goalCostToGo_ = ompl::magic::DEFAULT_GOAL_COST_TO_GO;

    obstacleCostToGo_ = ompl::magic::DEFAULT_OBSTACLE_COST_TO_GO;

    initialCostToGo_ = ompl::magic::DEFAULT_INIT_COST_TO_GO;

    infiniteCostToGo_ = ompl::magic::DEFAULT_INF_COST_TO_GO;

    maxDPIterations_ = ompl::magic::DEFAULT_DP_MAX_ITERATIONS;
    
    convergenceThresholdDP_ = ompl::magic::DEFAULT_DP_CONVERGENCE_THRESHOLD;

    numberOfTargetsInHistory_ = ompl::magic::DEFAULT_NUM_OF_TARGETS_IN_HISTORY;

    numberOfFeedbackLookAhead_ = ompl::magic::DEFAULT_NUM_OF_FEEDBACK_LOOK_AHEAD;
}

FIRM::~FIRM(void)
{
    if(doSaveLogs_ && addedSolution_)
        Visualizer::printRobotPathToFile(logFilePath_);

    freeMemory();
    delete policyGenerator_;
}

void FIRM::setup(void)
{
    Planner::setup();
    if (!nn_)
    {
        nn_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        nn_->setDistanceFunction(boost::bind(&FIRM::distanceFunction, this, _1, _2));
    }
    if (!connectionStrategy_)
    {
        connectionStrategy_ = VStrategy<Vertex>(NNRadius_, nn_);

        // NOTE if you want to switch to other strategies, you should change the definition of ConnectionStrategy in FIRM.h
        //connectionStrategy_ = ompl::geometric::KStarStrategy<Vertex>(boost::bind(&FIRM::milestoneCount, this), nn_, si_->getStateDimension());
        //connectionStrategy_ = ompl::geometric::KBoundedStrategy<Vertex>(numNearestNeighbors_, NNRadius_, nn_);
        //connectionStrategy_ = FStrategy<Vertex>(NNRadius_, nn_);
    }

    // additional k-nearest neighbor connection strategy for rollout
    if (!kConnectionStrategy_)
    {
        kConnectionStrategy_ = KStrategy<Vertex>(numNearestNeighbors_, nn_);
    }
}

void FIRM::setMaxNearestNeighbors(unsigned int k)
{
    if (!nn_)
    {
        nn_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        nn_->setDistanceFunction(boost::bind(&FIRM::distanceFunction, this, _1, _2));
    }

    //if (!userSetConnectionStrategy_)
        //connectionStrategy_.clear();
    
    if (isSetup())
        setup();

}

void FIRM::setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef)
{
    Planner::setProblemDefinition(pdef);
    clearQuery();
}

void FIRM::clearQuery(void)
{
    startM_.clear();
    goalM_.clear();
    pis_.restart();
}

void FIRM::clear(void)
{
    Planner::clear();
    sampler_.reset();
    simpleSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    clearQuery();
    maxEdgeID_ = 0;
}

void FIRM::freeMemory(void)
{
    foreach (Vertex v, boost::vertices(g_))
    {
        const auto state = stateProperty_[v];
        if (!state)
            si_->freeState(state);
    }
    g_.clear();
}

void FIRM::expandRoadmap(double expandTime)
{
    expandRoadmap(ompl::base::timedPlannerTerminationCondition(expandTime));
}

void FIRM::expandRoadmap(const ompl::base::PlannerTerminationCondition &ptc)
{
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();

    std::vector<ompl::base::State*> states(ompl::magic::MAX_RANDOM_BOUNCE_STEPS);
    si_->allocStates(states);
    expandRoadmap(ptc, states);
    si_->freeStates(states);
}

void FIRM::expandRoadmap(const ompl::base::PlannerTerminationCondition &ptc,
                                         std::vector<ompl::base::State*> &workStates)
{
    // construct a probability distribution over the vertices in the roadmap as indicated in
    // "Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces"
    // Lydia E. Kavraki, Petr Svestka, Jean-Claude Latombe, and Mark H. Overmars

    ompl::PDF<Vertex> pdf;
    foreach (Vertex v, boost::vertices(g_))
    {
        const unsigned int t = totalConnectionAttemptsProperty_[v];
        pdf.add(v, (double)(t - successfulConnectionAttemptsProperty_[v]) /(double)t);
    }

    if (pdf.empty())
        return;

    while (ptc == false)
    {
        Vertex v = pdf.sample(rng_.uniform01());

        unsigned int s = si_->randomBounceMotion(simpleSampler_, stateProperty_[v], workStates.size(), workStates, false);

        if (s > 0)
        {
            s--;

            Vertex last = addStateToGraph(si_->cloneState(workStates[s]));

            graphMutex_.lock();
            for (unsigned int i = 0 ; i < s ; ++i)
            {
                // add the vertex along the bouncing motion
                Vertex m = boost::add_vertex(g_);
                stateProperty_[m] = si_->cloneState(workStates[i]);    // REVIEW cloneState() again? why not save a pointer to this in the above line?
                totalConnectionAttemptsProperty_[m] = 1;
                successfulConnectionAttemptsProperty_[m] = 0;
                disjointSets_.make_set(m);

                // add the edge to the parent vertex
                bool addedEdgeVM, addedEdgeMV;

                addEdgeToGraph(v,m, addedEdgeVM);

                addEdgeToGraph(m,v, addedEdgeMV);

                if(addedEdgeVM && addedEdgeMV)
                {
                    uniteComponents(v, m);
                }

                // add the vertex to the nearest neighbors data structure
                nn_->add(m);

                v  =  m;

                addStateToVisualization(stateProperty_[m]);

            }

            // if there are intermediary states or the milestone has not been connected to the initially sampled vertex,
            // we add an edge
            if (s > 0 || !sameComponent(v, last))
            {
                bool addedEdgeA, addedEdgeB;

                addEdgeToGraph(v,last, addedEdgeA);

                addEdgeToGraph(last,v, addedEdgeB);

                if( addedEdgeA && addedEdgeB)
                {
                    uniteComponents(v, last);
                }
            }

            graphMutex_.unlock();

        }

    }
}


void FIRM::growRoadmap(double growTime)
{
    growRoadmap(ompl::base::timedPlannerTerminationCondition(growTime));
}

void FIRM::growRoadmap(const ompl::base::PlannerTerminationCondition &ptc)
{
    if (!isSetup())
        setup();
    if (!sampler_)
        sampler_ = siF_->allocValidStateSampler();

    ompl::base::State *workState = siF_->allocState();
    growRoadmap (ptc, workState);
    siF_->freeState(workState);
}

void FIRM::growRoadmap(const ompl::base::PlannerTerminationCondition &ptc,
                                       ompl::base::State *workState)
{
    using namespace arma;

    while (ptc == false)
    {
        // search for a valid state
        bool found = false;
        bool stateStable = false;

        while (!found && ptc == false)
        {
            unsigned int attempts = 0;
            do
            {
                found = sampler_->sample(workState);
                stateStable = false;
                if(found)
                {

                    ompl::base::State *lsState = si_->cloneState(workState);

                    LinearSystem ls(siF_, lsState, siF_->getMotionModel()->getZeroControl(),
                                siF_->getObservationModel()->getObservation(lsState, false), siF_->getMotionModel(), siF_->getObservationModel());

                    arma::mat S;

                    try
                    {
                        stateStable = dare (trans(ls.getA()),trans(ls.getH()),ls.getG() * ls.getQ() * trans(ls.getG()),
                                ls.getM() * ls.getR() * trans(ls.getM()), S );

                        //workState->as<FIRM::StateType>()->setCovariance(S);

                        // If the number of nodes is greater than or equal to max required by setup, then stop building roadmap and try to find a solution
                        if(boost::num_vertices(g_) >= maxFIRMNodes_)
                        {
                            break;
                        }

                    }
                    catch(int e)
                    {
                        stateStable = false;
                    }

                    // free the memory
                    si_->freeState(lsState);
                }

                // If the number of nodes is greater than or equal to max required by setup, then stop building roadmap and try to find a solution
                if(boost::num_vertices(g_) >= maxFIRMNodes_)
                {
                    break;
                }

                attempts++;
            } while (attempts < ompl::magic::FIND_VALID_STATE_ATTEMPTS_WITHOUT_TERMINATION_CHECK && !found && !stateStable);
        }
        // add it as a milestone
        if (found && stateStable)
            addStateToGraph(si_->cloneState(workState));
    }
}

void FIRM::checkForSolution(const ompl::base::PlannerTerminationCondition &ptc,
                                            ompl::base::PathPtr &solution)
{
    // Don't do anything for first one second
    boost::this_thread::sleep(boost::posix_time::seconds(1));

    while (!ptc && !addedSolution_)
    {
        // Check for any new goal states
        /*if (goal->maxSampleCount() > goalM_.size())
        {
            const ompl::base::State *st = pis_.nextGoal();
            if (st)
                goalM_.push_back(addStateToGraph(si_->cloneState(st)));
        }
        */
        OMPL_INFORM("FIRM: Checking for Solution.");

        addedSolution_ = existsPolicy(startM_, goalM_, solution);

        if (!addedSolution_)
        {
            OMPL_INFORM("FIRM: No Solution Yet.");
            boost::this_thread::sleep(boost::posix_time::seconds(30));
        }
    }
}

bool FIRM::existsPolicy(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals, ompl::base::PathPtr &solution)
{
    ompl::base::Goal *g = pdef_->getGoal().get();
    ompl::base::Cost sol_cost(0.0);

    OMPL_INFORM("%s: Number of current states = %u", getName().c_str(), boost::num_vertices(g_));

    if(boost::num_vertices(g_) < minFIRMNodes_)
    {
        OMPL_INFORM("FIRM: Do not yet have enough nodes (< %u).", minFIRMNodes_);
        return false;
    } 

    foreach (Vertex start, starts)
    {
        foreach (Vertex goal, goals)
        {

            graphMutex_.lock();

            bool same_component = sameComponent(start, goal);
            
            graphMutex_.unlock();

            if (same_component /*&& g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start])*/)
            {

                boost::mutex::scoped_lock _(graphMutex_);
                

                // solve the POMDP with sampling by Monte Carlo simulation

                // NOTE use Dijkstra search's approximate solution as an initial guess for Dynamic Programming for Value Iteration
                // if all the transitionProbability is equal to 1, Dijkstra search solution is optimal; otherwise, pass the result to Dynamic Programming
                // with Dijkstra search's result for initialization, Dynamic Programming may not need the weird term 'distToGoalFromTarget' for convergence

                solveDijkstraSearch(goal);

                bool reinit = false;
                solveDynamicProgram(goal, reinit);


                if(!constructFeedbackPath(start, goal, solution))
                    return false;
                
                sendFeedbackEdgesToViz();
                
                return true; // return true if solution is found
            }
            else
            {
                OMPL_WARN("FIRM: start and goal nodes not in same connected component.");
                return false;
            }
        }
    }

    OMPL_INFORM("FIRM: DP/Dijkstra could not converge.");
    return false;
}

bool FIRM::addedNewSolution(void) const
{
    return addedSolution_;
}

ompl::base::PlannerStatus FIRM::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    ompl::base::GoalSampleableRegion *goal = dynamic_cast<ompl::base::GoalSampleableRegion*>(pdef_->getGoal().get());

    if (!goal)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return ompl::base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    // If the number of nodes in the loaded roadmap is greater than max required by setup, abort the process
    if(loadedRoadmapFromFile_ && boost::num_vertices(g_) > maxFIRMNodes_)
    {
        OMPL_ERROR("%s: The number of nodes in the loaded roadmap is greater than maxNodes! Increase the value of maxNodes or set useRoadMap to 0...", getName().c_str());
        return ompl::base::PlannerStatus::ABORT;
    }

    OMPL_INFORM("%s: Adding start state to roadmap.", getName().c_str());

    // Add the valid start states as milestones
    while (const ompl::base::State *st = pis_.nextStart())
    {
        // HACK just for density analysis setup
        if (ompl::magic::DENSITY_ANALYSIS)
        {
            if (loadedRoadmapFromFile_)
            {
                Vertex start_loaded = 0;    // HACK
                startM_.push_back(start_loaded);
                break;
            }
        }

        auto start_time = std::chrono::high_resolution_clock::now();
        
        startM_.push_back(addStateToGraph(si_->cloneState(st)));

        auto end_time = std::chrono::high_resolution_clock::now();

        double time2addstartstate = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        if(doSaveLogs_)
        {
            std::ofstream outfile;
            outfile.open(logFilePath_+"StartStateAddTime.txt",std::ios::app);
            outfile<<"Time: "<<time2addstartstate<<" ms"<<std::endl;
            outfile.close();
        }

    }

    if (startM_.size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return ompl::base::PlannerStatus::INVALID_START;
    }


    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return ompl::base::PlannerStatus::INVALID_GOAL;
    }

    // HACK just for density analysis setup
    if (ompl::magic::DENSITY_ANALYSIS)
    {
        if (loadedRoadmapFromFile_)
        {
            OMPL_INFORM("%s: Adding goal state to roadmap.", getName().c_str());
            Vertex goal_loaded = 1;    // HACK
            goalM_.push_back(goal_loaded);
        }
    }

    // Ensure there is at least one valid goal state
    if (goal->maxSampleCount() > goalM_.size() || goalM_.empty())
    {
        const ompl::base::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
        if (st)
        {
            OMPL_INFORM("%s: Adding goal state to roadmap.", getName().c_str());
            goalM_.push_back(addStateToGraph(si_->cloneState(st)));
        }
        if (goalM_.empty())
        {
            OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
            return ompl::base::PlannerStatus::INVALID_GOAL;
        }
    }

    unsigned int nrStartStates = boost::num_vertices(g_);
    OMPL_INFORM("%s: Starting with %u states", getName().c_str(), nrStartStates);

    addedSolution_ = false;

    if (!isSetup())
        setup();
    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();

    ompl::base::PathPtr sol;
    // NOTE disabled periodic solution checking
//     boost::thread slnThread(boost::bind(&FIRM::checkForSolution, this, ptc, boost::ref(sol)));

    ompl::base::PlannerTerminationCondition ptcOrSolutionFound =
        ompl::base::plannerOrTerminationCondition(ptc, ompl::base::PlannerTerminationCondition(boost::bind(&FIRM::addedNewSolution, this)));

    // If no roadmap was loaded or the number of loaded is less than min required by setup, then build roadmap
    // FIXME a roadmap with nodes more than minNodes does not guantee the start and goal states to be in a connected component (there might be no solution yet)!
    if(!loadedRoadmapFromFile_ || boost::num_vertices(g_) < minFIRMNodes_)
    {
        OMPL_INFORM("%s: No roadmap was loaded or the number of nodes in the roadmap is less than minNodes! More samples will be added to the roadmap...", getName().c_str());
        constructRoadmap(ptcOrSolutionFound);
    }

    // NOTE Now manually check for solution
    OMPL_INFORM("FIRM: Checking for Solution.");
    addedSolution_ = existsPolicy(startM_, goalM_, boost::ref(sol));
    if (!addedSolution_)
    {
        OMPL_ERROR("FIRM: No Solution was found within this roadmap. Try to increase the value of maxNodes...");
    }

    if(doSavePlannerData_)
    {
        this->savePlannerData();
    }

    // NOTE disabled periodic solution checking
//     slnThread.join();

    OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_) - nrStartStates);

    if (sol)
    {
        ompl::base::PlannerSolution psol(sol);
        // if the solution was optimized, we mark it as such
        if (addedNewSolution())
            psol.optimized_ = true;
        pdef_->addSolutionPath(psol);
    }

    return sol ? (addedNewSolution() ? ompl::base::PlannerStatus::EXACT_SOLUTION : ompl::base::PlannerStatus::APPROXIMATE_SOLUTION) : ompl::base::PlannerStatus::TIMEOUT;
}

void FIRM::constructRoadmap(const ompl::base::PlannerTerminationCondition &ptc)
{
    std::vector<ompl::base::State*> xstates(ompl::magic::MAX_RANDOM_BOUNCE_STEPS);
    si_->allocStates(xstates);
    bool grow = true;

    while (ptc() == false)
    {
        // In FIRM, we maintain a 2:1 ratio for growth to expansion
        if(grow)
        {
//             growRoadmap(ompl::base::plannerOrTerminationCondition(ptc, ompl::base::timedPlannerTerminationCondition(2*ompl::magic::ROADMAP_BUILD_TIME)), xstates[0]);
            growRoadmap(ompl::base::plannerOrTerminationCondition(ptc, ompl::base::timedPlannerTerminationCondition(0.1)), xstates[0]);     // HACK arbitrarily selected short time step
        }
        else
        {
            // REVIEW is this helpful?
//             expandRoadmap(ompl::base::plannerOrTerminationCondition(ptc, ompl::base::timedPlannerTerminationCondition(ompl::magic::ROADMAP_BUILD_TIME)), xstates);
        }

        // TODO terminate constructRoadmap() if enough number of nodes are generated or [a solution is found]
        // If the number of nodes is greater than or equal to max required by setup, then stop building roadmap and try to find a solution
        if(boost::num_vertices(g_) >= maxFIRMNodes_)
        {
            OMPL_INFORM("FIRM: Now have enough nodes (>= %u).", maxFIRMNodes_);
            break;
        }

        grow = !grow;
    }

    si_->freeStates(xstates);
}

FIRM::Vertex FIRM::addStateToGraph(ompl::base::State *state, bool addReverseEdge)
{

    boost::mutex::scoped_lock _(graphMutex_);

    // add the given belief state to graph as FIRM node
    Vertex m;
    m = boost::add_vertex(g_);

    // NOTE do not generate a node controller for a temporary node during rollout execution as it will not be used at all
    // in the case generateNodeController() is called during execution, the covariance of 'state' should be reverted to its original (actual) state's covariance after that function
    if(addReverseEdge)
    {
        // construct a node stabilizer controller
        NodeControllerType nodeController;
        generateNodeController(state, nodeController); // Generating the node controller at sampled state, this will set stationary covariance at node
        nodeControllers_[m] = nodeController;

        // visualize the FIRM node
        addStateToVisualization(state);
    }

    stateProperty_[m] = state;
    totalConnectionAttemptsProperty_[m] = 1;
    successfulConnectionAttemptsProperty_[m] = 0;

    // Initialize to its own (dis)connected component.
    disjointSets_.make_set(m);


    // add this vertex to the database for nearest neighbor search
    nn_->add(m);

    // Which milestones will we attempt to connect to?
    //std::vector<Vertex> neighbors = connectionStrategy_(m);

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


    // NOTE commented this to allow connection to farther but better FIRM node
/*
    // If reverse edge not required, that means this is a virtual state added in rollout
    // We will sort by cost and use N lowest cost to go neighbors
    if(!addReverseEdge && !neighbors.empty())
    {
        std::vector<std::pair<double,Vertex>> tempItems;

        foreach (Vertex n, neighbors)
        {
            if(n != m)
                tempItems.push_back(std::make_pair(costToGo_[n], n));
        }

        std::sort(tempItems.begin(), tempItems.end());

        neighbors.clear();

        int nti = tempItems.size();

        // Now we add those neighbors with lowest cost and such that the robot can move to them, i.e. path is valid
        int t_countNN = 0;

        for(int i = 0; i < nti; i++)
        {
            if(si_->checkMotion(stateProperty_[m], stateProperty_[tempItems[i].second]))
            {
                neighbors.push_back(tempItems[i].second);
                t_countNN++;
            }

            if(t_countNN >= numNearestNeighbors_)
                break;            
        }
    }
*/

    // check for valid neighbors
    if (!addReverseEdge)
    {
        if (neighbors.size()==1)
        {
            if (m==neighbors[0])
            {
                OMPL_ERROR("No neighbor other than itself was found for vertex %d", m);
                //return -1;  // invalid vertex id
                exit(0);
            }
        }
    }

    foreach (Vertex n, neighbors)
    {
        if ( m!=n )
        {
            totalConnectionAttemptsProperty_[m]++;
            totalConnectionAttemptsProperty_[n]++;

            if (si_->checkMotion(stateProperty_[m], stateProperty_[n]))
            {

                bool forwardEdgeAdded=false;
                bool reverseEdgeAdded=false;

                // NOTE in execution mode, i.e., addReverseEdge is false, compute edge cost from the center belief state, not from a sampled border belief state
                addEdgeToGraph(m, n, forwardEdgeAdded, addReverseEdge);

                if(forwardEdgeAdded)
                {
                    successfulConnectionAttemptsProperty_[m]++;

                    if(addReverseEdge)
                    {
                        addEdgeToGraph(n, m, reverseEdgeAdded);

                        if(reverseEdgeAdded)
                        {
                            successfulConnectionAttemptsProperty_[n]++;

                            uniteComponents(m, n);

                            Visualizer::addGraphEdge(stateProperty_[m], stateProperty_[n]);

                            Visualizer::addGraphEdge(stateProperty_[n], stateProperty_[m]);

                        }
                        else
                        {
                            boost::remove_edge(m,n,g_); // if you cannot add bidirectional edge, then keep no edge between the two nodes
                        }
                    }

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
        //boost::this_thread::sleep(boost::posix_time::milliseconds(50));  // moved to FIRM::executeFeedbackWithRollout() for more accurate time computation
    }

    policyGenerator_->addFIRMNodeToObservationGraph(state);

    return m;
}

void FIRM::uniteComponents(Vertex m1, Vertex m2)
{
    disjointSets_.union_set(m1, m2);
}

bool FIRM::sameComponent(Vertex m1, Vertex m2)
{
    return boost::same_component(m1, m2, disjointSets_);
}

bool FIRM::constructFeedbackPath(const Vertex &start, const Vertex &goal, ompl::base::PathPtr &solution)
{
    sendFeedbackEdgesToViz();

    FeedbackPath<SeparatedControllerType, FilterType> *p = new FeedbackPath<SeparatedControllerType, FilterType>(siF_);

    std::cout<<"The start vertex is: "<<start<<std::endl;
    std::cout<<"The goal vertex is: "<<goal<<std::endl;

    Vertex currentVertex = start;

    int counter  = 0;

    while(currentVertex!=goal)
    {
        if(feedback_.find(currentVertex) == feedback_.end())  // not connected to the goal
            return false;

        Edge edge = feedback_.at(currentVertex); // get the edge

        Vertex target = boost::target(edge, g_); // get the target of this edge

        if(target > boost::num_vertices(g_))
            OMPL_ERROR("Error in constructing feedback path. Tried to access vertex ID not in graph.");

        p->append(stateProperty_[currentVertex],edgeControllers_.at(edge)); // push the state and controller to take

        if(target == goal)
        {
            p->append(stateProperty_[target]); // because from the goal node you don't need to take a controller
        }

        currentVertex =  target;

        counter++; // the maximum number of nodes that robot can pass through is the total number of nodes

        if(counter > boost::num_vertices(g_))
        {
            OMPL_ERROR("There is no feedback to guide robot to goal. Maybe DP did not converge.");
            return false;
        }

    }

    solution = ompl::base::PathPtr(p);

    return true;
}

void FIRM::addEdgeToGraph(const FIRM::Vertex a, const FIRM::Vertex b, bool &edgeAdded, const bool addReverseEdge)
{

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
    const FIRMWeight weight = generateEdgeNodeControllerWithCost(a, b, edgeController, constructionMode);    // EdgeController and NodeController concatenated

    if(weight.getSuccessProbability() == 0)
    {
        edgeAdded = false;
        return; // this edge should not be added as it has no chance of success
    }

    assert(edgeController.getGoal() && "The generated controller has no goal");

    const unsigned int id = maxEdgeID_++;

    const Graph::edge_property_type properties(weight, id);

    // create an edge with the edge weight property
    std::pair<Edge, bool> newEdge = boost::add_edge(a, b, properties, g_);

    edgeControllers_[newEdge.first] = edgeController;

    edgeAdded = true;
}

FIRMWeight FIRM::generateEdgeNodeControllerWithCost(const FIRM::Vertex a, const FIRM::Vertex b, EdgeControllerType &edgeController, const bool constructionMode)
{
    ompl::base::State* startNodeState = siF_->cloneState(stateProperty_[a]);
//     ompl::base::State* targetNodeState = siF_->cloneState(stateProperty_[b]);
    ompl::base::State* targetNodeState = stateProperty_[b];

    ompl::base::State* tempBelief = siF_->allocState();
    ompl::base::State* sampState = siF_->allocState();
    ompl::base::State* endBelief = siF_->allocState(); // allocate the end state of the controller

    // target node controller that will be concatenated after edgeController
    NodeControllerType nodeController = nodeControllers_.at(b);

     // Generate the edge controller for given start and end state
    generateEdgeController(startNodeState,targetNodeState,edgeController);

    double successCount = 0;

    // initialize costs to 0
    ompl::base::Cost edgeCost(0);
    ompl::base::Cost nodeStabilizationCost(0);

    // if want/do not want to show monte carlo sim
    siF_->showRobotVisualization(ompl::magic::SHOW_MONTE_CARLO);

    for(unsigned int i=0; i< numMCParticles_;i++)
    {
        if(constructionMode)
        {
            // NOTE sample a start belief state that marginally satisfies isReached() condition for the center belief state, startNodeState
            // this is to estimate the actual edge cost more  that can happen during execution
            // NOTE edgeController is not separately generated for these sampled border belief state unlike the actual rollout execution case
            if(!startNodeState->as<FIRM::StateType>()->sampleBorderBeliefState(tempBelief))
            {
                OMPL_WARN("Could not sample a border belief state from the current belief state!");
                continue;
            }
            siF_->setBelief(tempBelief);
        }
        else
        {
            // NOTE in execution mode, the current belief state would already be on the border, so there is no need to inflate the covariance again
            siF_->copyState(tempBelief, startNodeState);
            siF_->setBelief(tempBelief);
        }


        // NOTE random sampling of a true state from the sampled border belief state for Monte Carlo simulation
        if(!tempBelief->as<FIRM::StateType>()->sampleTrueStateFromBelief(sampState))
        {
            OMPL_WARN("Could not sample a true state from the current belief state!");
            continue;
        }
        siF_->setTrueState(sampState);

        // for debug
        if(ompl::magic::PRINT_MC_PARTICLES)
        {
            std::cout << "-------------------------------------------" << std::endl;
            ompl::base::State *tmp = si_->allocState();
            siF_->getTrueState(tmp);
            OMPL_INFORM("True State: (%2.3f, %2.3f, %2.3f, %2.3f)",
                    tmp->as<FIRM::StateType>()->getX(),
                    tmp->as<FIRM::StateType>()->getY(),
                    tmp->as<FIRM::StateType>()->getYaw(),
                    arma::trace(tmp->as<FIRM::StateType>()->getCovariance()));
            OMPL_INFORM("Belief State: (%2.3f, %2.3f, %2.3f, %2.3f)",
                    tempBelief->as<FIRM::StateType>()->getX(),
                    tempBelief->as<FIRM::StateType>()->getY(),
                    tempBelief->as<FIRM::StateType>()->getYaw(),
                    arma::trace(tempBelief->as<FIRM::StateType>()->getCovariance()));
            siF_->freeState(tmp);
            std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
        }


        ompl::base::Cost filteringCost(0);
        int stepsExecuted = 0;
        int stepsToStop = 0;

        // [1] EdgeController until the termination condition
        bool edgeControllerStatus = false;
        if(edgeController.isTerminated(tempBelief, 0))  // check if cstartState is near to the target FIRM node (by x,y position); this is the termination condition B) for EdgeController::Execute()
        {
            edgeControllerStatus = true;
        }
        else
        {
            if(edgeController.Execute(tempBelief, endBelief, filteringCost, stepsExecuted, stepsToStop, true))
            {
                edgeControllerStatus = true;

                // for debug
                ompl::base::Cost edgeCostPrev;
                if(ompl::magic::PRINT_EDGE_COST)
                    edgeCostPrev = ompl::base::Cost(edgeCost.value());


                // NOTE how to penalize uncertainty (covariance) and path length (time steps) in the cost
                //*1) cost = wc * sum(trace(cov_k))  + wt * K  (for k=1,...,K)
                // 2) cost = wc * trace(cov_f)       + wt * K
                // 3) cost = wc * mean(trace(cov_k)) + wt * K
                // 4) cost = wc * sum(trace(cov_k))

                // compute the edge cost by the weighted sum of filtering cost and time to stop (we use number of time steps, time would be steps*dt)
                edgeCost = ompl::base::Cost(edgeCost.value() + informationCostWeight_*filteringCost.value() + timeCostWeight_*stepsToStop);   // 1,2,3)
//                 edgeCost = ompl::base::Cost(edgeCost.value() + informationCostWeight_*filteringCost.value());   // 4) cost = sum(trace(cov_k))

                // for debug
                if(ompl::magic::PRINT_EDGE_COST)
                    std::cout << "edgeCost[" << a << "->" << b << "] " << edgeCost.value() << " = " << edgeCostPrev.value() << " + ( " << informationCostWeight_ << "*" << filteringCost.value() << " + " << timeCostWeight_ << "*" << stepsToStop << " )" << std::endl;


                // update tempBelief for node controller
                si_->copyState(tempBelief, endBelief);
            }
        }

        // [2] NodeController for stabilization
        if(edgeControllerStatus)
        {
            if(nodeController.Stabilize(tempBelief, endBelief, filteringCost, stepsExecuted, true))
            {
                successCount++;

                // for debug
                ompl::base::Cost edgeCostPrev;
                if(ompl::magic::PRINT_EDGE_COST)
                    edgeCostPrev = ompl::base::Cost(edgeCost.value());


                // NOTE how to penalize uncertainty (covariance) and path length (time steps) in the cost
                //*1) cost = wc * sum(trace(cov_k))  + wt * K  (for k=1,...,K)
                // 2) cost = wc * trace(cov_f)       + wt * K
                // 3) cost = wc * mean(trace(cov_k)) + wt * K
                // 4) cost = wc * sum(trace(cov_k))

                // compute the edge cost by the weighted sum of filtering cost and time to stop (we use number of time steps, time would be steps*dt)
                edgeCost = ompl::base::Cost(edgeCost.value() + informationCostWeight_*filteringCost.value() + timeCostWeight_*stepsToStop);   // 1,2,3)
                //             edgeCost = ompl::base::Cost(edgeCost.value() + informationCostWeight_*filteringCost.value());   // 4) cost = sum(trace(cov_k))

                // for debug
                if(ompl::magic::PRINT_EDGE_COST)
                    std::cout << "edgeCost[" << a << "->" << b << "] " << edgeCost.value() << " = " << edgeCostPrev.value() << " + ( " << informationCostWeight_ << "*" << filteringCost.value() << " + " << timeCostWeight_ << "*" << stepsToStop << " )" << std::endl;
            }
        }

    }
    // for debug
    ompl::base::Cost edgeCostSum;
    if(ompl::magic::PRINT_EDGE_COST)
        edgeCostSum = ompl::base::Cost(edgeCost.value());

    siF_->showRobotVisualization(true);

    if(successCount!=0)
        edgeCost = ompl::base::Cost(edgeCost.value() / successCount);
    else
        edgeCost = ompl::base::Cost(infiniteCostToGo_);  // this edge will not be added anyway

    // for debug
    if(ompl::magic::PRINT_EDGE_COST)
        std::cout << "edgeCost[" << a << "->" << b << "] " << edgeCost.value() << " = ( " << edgeCostSum.value() << " ) / " << successCount << std::endl;

    double transitionProbability = successCount / numMCParticles_;

    FIRMWeight weight(edgeCost.value(), transitionProbability);

    // free the memory
    siF_->freeState(startNodeState);
    siF_->freeState(tempBelief);
    siF_->freeState(sampState);
    siF_->freeState(endBelief);

    return weight;
}

// deprecated
FIRMWeight FIRM::generateEdgeControllerWithCost(const FIRM::Vertex a, const FIRM::Vertex b, EdgeControllerType &edgeController, const bool constructionMode)
{
    ompl::base::State* startNodeState = siF_->cloneState(stateProperty_[a]);
//     ompl::base::State* targetNodeState = siF_->cloneState(stateProperty_[b]);
    ompl::base::State* targetNodeState = stateProperty_[b];

    ompl::base::State* tempBelief = siF_->allocState();
    ompl::base::State* sampState = siF_->allocState();
    ompl::base::State* endBelief = siF_->allocState(); // allocate the end state of the controller

     // Generate the edge controller for given start and end state
    generateEdgeController(startNodeState,targetNodeState,edgeController);

    double successCount = 0;

    // initialize costs to 0
    ompl::base::Cost edgeCost(0);
    ompl::base::Cost nodeStabilizationCost(0);

    // if want/do not want to show monte carlo sim
    siF_->showRobotVisualization(ompl::magic::SHOW_MONTE_CARLO);

    for(unsigned int i=0; i< numMCParticles_;i++)
    {
        if(constructionMode)
        {
            // NOTE sample a start belief state that marginally satisfies isReached() condition for the center belief state, startNodeState
            // this is to estimate the actual edge cost more  that can happen during execution
            // NOTE edgeController is not separately generated for these sampled border belief state unlike the actual rollout execution case
            if(!startNodeState->as<FIRM::StateType>()->sampleBorderBeliefState(tempBelief))
            {
                OMPL_WARN("Could not sample a border belief state from the current belief state!");
                continue;
            }
            siF_->setBelief(tempBelief);
        }
        else
        {
            // NOTE in execution mode, the current belief state would already be on the border, so there is no need to inflate the covariance again
            siF_->copyState(tempBelief, startNodeState);
            siF_->setBelief(tempBelief);
        }


        // NOTE random sampling of a true state from the current belief state for Monte Carlo simulation
        if(!tempBelief->as<FIRM::StateType>()->sampleTrueStateFromBelief(sampState))
        {
            OMPL_WARN("Could not sample a true state from the current belief state!");
            continue;
        }
        siF_->setTrueState(sampState);

        // for debug
        if(ompl::magic::PRINT_MC_PARTICLES)
        {
            std::cout << "-------------------------------------------" << std::endl;
            ompl::base::State *tmp = si_->allocState();
            siF_->getTrueState(tmp);
            OMPL_INFORM("True State: (%2.3f, %2.3f, %2.3f, %2.3f)",
                    tmp->as<FIRM::StateType>()->getX(),
                    tmp->as<FIRM::StateType>()->getY(),
                    tmp->as<FIRM::StateType>()->getYaw(),
                    arma::trace(tmp->as<FIRM::StateType>()->getCovariance()));
            OMPL_INFORM("Belief State: (%2.3f, %2.3f, %2.3f, %2.3f)",
                    tempBelief->as<FIRM::StateType>()->getX(),
                    tempBelief->as<FIRM::StateType>()->getY(),
                    tempBelief->as<FIRM::StateType>()->getYaw(),
                    arma::trace(tempBelief->as<FIRM::StateType>()->getCovariance()));
            siF_->freeState(tmp);
            std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
        }


        ompl::base::Cost filteringCost(0);

        int stepsExecuted = 0;

        int stepsToStop = 0;

        if(edgeController.Execute(tempBelief, endBelief, filteringCost, stepsExecuted, stepsToStop, true))
        {
            successCount++;

            // for debug
            ompl::base::Cost edgeCostPrev;
            if(ompl::magic::PRINT_EDGE_COST)
                edgeCostPrev = ompl::base::Cost(edgeCost.value());


            // NOTE how to penalize uncertainty (covariance) and path length (time steps) in the cost
            //*1) cost = wc * sum(trace(cov_k))  + wt * K  (for k=1,...,K)
            // 2) cost = wc * trace(cov_f)       + wt * K
            // 3) cost = wc * mean(trace(cov_k)) + wt * K
            // 4) cost = wc * sum(trace(cov_k))

            // compute the edge cost by the weighted sum of filtering cost and time to stop (we use number of time steps, time would be steps*dt)
            edgeCost = ompl::base::Cost(edgeCost.value() + informationCostWeight_*filteringCost.value() + timeCostWeight_*stepsToStop);   // 1,2,3)
//             edgeCost = ompl::base::Cost(edgeCost.value() + informationCostWeight_*filteringCost.value());   // 4) cost = sum(trace(cov_k))

            // for debug
            if(ompl::magic::PRINT_EDGE_COST)
                std::cout << "edgeCost[" << a << "->" << b << "] " << edgeCost.value() << " = " << edgeCostPrev.value() << " + ( " << informationCostWeight_ << "*" << filteringCost.value() << " + " << timeCostWeight_ << "*" << stepsToStop << " )" << std::endl;
        }
    }
    // for debug
    ompl::base::Cost edgeCostSum;
    if(ompl::magic::PRINT_EDGE_COST)
        edgeCostSum = ompl::base::Cost(edgeCost.value());

    siF_->showRobotVisualization(true);

    //edgeCost.v = edgeCost.v / successCount ;
    edgeCost = ompl::base::Cost(edgeCost.value() / successCount);

    // for debug
    if(ompl::magic::PRINT_EDGE_COST)
        std::cout << "edgeCost[" << a << "->" << b << "] " << edgeCost.value() << " = ( " << edgeCostSum.value() << " ) / " << successCount << std::endl;

    double transitionProbability = successCount / numMCParticles_ ;

    FIRMWeight weight(edgeCost.value(), transitionProbability);

    // free the memory
    siF_->freeState(startNodeState);
    siF_->freeState(tempBelief);
    siF_->freeState(sampState);
    siF_->freeState(endBelief);

    return weight;
}

// void FIRM::generateEdgeController(const ompl::base::State *start, const ompl::base::State* target, FIRM::EdgeControllerType &edgeController)
void FIRM::generateEdgeController(const ompl::base::State *start, ompl::base::State* target, FIRM::EdgeControllerType &edgeController)
{
    assert(target && "The target state for generating the edge controller is NULL");
    std::vector<ompl::base::State*> intermediates;

    ompl::base::State *intermediate = si_->allocState();

    si_->copyState(intermediate, start);

    std::vector<ompl::control::Control*> openLoopControls;

    // get the open loop controls for this edge
    siF_->getMotionModel()->generateOpenLoopControls(start, target, openLoopControls);

    // generate the intermediate states using the open loop controls
    for(typename std::vector<ompl::control::Control*>::iterator c=openLoopControls.begin(), e=openLoopControls.end(); c!=e; ++c)
    {
        ompl::base::State *x = si_->allocState();
        siF_->getMotionModel()->Evolve(intermediate,*c,siF_->getMotionModel()->getZeroNoise(), x);
        intermediates.push_back(x);
        si_->copyState(intermediate, x);
    }

    // assign the edge controller
    edgeController = EdgeControllerType(target, intermediates, openLoopControls, siF_);  // 'target' state should not be freed unless this controller will never be used

    // free the memory
    si_->freeState(intermediate);
}

void FIRM::generateNodeController(ompl::base::State *state, FIRM::NodeControllerType &nodeController)
{
    // Create a copy of the node state
//     ompl::base::State *node = si_->allocState();
//     siF_->copyState(node, state);
    ompl::base::State *node = state;

    arma::mat stationaryCovariance; 

   if(siF_->getObservationModel()->isStateObservable(node))
   {
        // Contruct a linear kalman filter
        LinearizedKF linearizedKF(siF_);

        //Construct a linear system
        LinearSystem linearSystem(siF_, node, siF_->getMotionModel()->getZeroControl(),siF_->getObservationModel()->getObservation(state, false), siF_->getMotionModel(), siF_->getObservationModel());

        // Compute the stationary cov at node state using LKF
        stationaryCovariance = linearizedKF.computeStationaryCovariance(linearSystem);
    }
    else
    {
        // set a default stationary cov at node state
        int stateDim = si_->getStateDimension();

        stationaryCovariance = arma::eye(stateDim,stateDim)*ompl::magic::NON_OBSERVABLE_NODE_COVARIANCE;
    }

    // set the stationary covariance (for construction mode)
    node->as<FIRM::StateType>()->setCovariance(stationaryCovariance);
    // NOTE CHECK if this function was called during execution, the covariance of 'state' should be reverted to its original (actual) state's covariance after this function!
    state->as<FIRM::StateType>()->setCovariance(stationaryCovariance);

    // create a node controller with node as the state and zero control as nominal control
    std::vector<ompl::control::Control*> zeroControl; zeroControl.push_back(siF_->getMotionModel()->getZeroControl());

    std::vector<ompl::base::State*> nodeState; nodeState.push_back(node);

    // assign the node controller
    nodeController = NodeControllerType(node, nodeState, zeroControl, siF_);

}

template<typename Map>
arma::colvec MapToColvec(const Map& _m) {
  arma::colvec mapData(_m.size());
  int ix = 0;
  for(typename Map::const_iterator i= _m.begin(), e= _m.end(); i!=e; ++i, ++ix)
    mapData[ix] = i->second;
  return mapData;
}

void FIRM::solveDynamicProgram(const FIRM::Vertex goalVertex, const bool reinit)
{
    OMPL_INFORM("FIRM: Solving DP");

    auto start_time = std::chrono::high_resolution_clock::now();

    Visualizer::clearMostLikelyPath();

    using namespace arma;

    float discountFactor = discountFactorDP_;

    std::map<Vertex, double> newCostToGo;

    if (reinit)    // do this only if costToGo_ and feedback_ are not initialized by Dijkstra search
    {
        costToGo_ = std::map<Vertex, double>();
        feedback_.clear();

        /**
          --NOTES--
          Assign a high cost to go initially for all nodes that are not in the goal connected component.
          For nodes that are in the goal cc, we assign goal cost to go for the goal and init cost to go
          for all other nodes.
          */
        foreach (Vertex v, boost::vertices(g_))
        {
            if(v == goalVertex)
            {
                costToGo_[v] = goalCostToGo_;
                newCostToGo[v] = goalCostToGo_;
            }
            else
            {
                costToGo_[v] = initialCostToGo_;
                newCostToGo[v] = initialCostToGo_;
            }
        }
    }
    else
    {
        // initialize newCostToGo with the approximate solution obtained from Dijkstra search
        foreach (Vertex v, boost::vertices(g_))
        {
            newCostToGo[v] = costToGo_[v];
        }
    }

    bool convergenceCondition = false;

    int nIter=0;
    double diffCostToGo;
    while(!convergenceCondition && nIter < maxDPIterations_)
    {
        nIter++;

        // REVIEW this seems to be pretty inefficient... switch to Dijkstra search?
        foreach(Vertex v, boost::vertices(g_))
        {

            //value for goal node stays the same or if has no out edges then ignore it
            if( v == goalVertex || boost::out_degree(v,g_) == 0 )
            {
                continue;
            }

            // to not update this node if it is not connected to the goal even after solving Dijkstra search
            // NOTE this is to ignore disconnected components during rollout (which may lead to an infinite loop while checkMotion())
            if (!reinit)
            {
                if (newCostToGo[v] >= infiniteCostToGo_)    // infiniteCostToGo_ implicitly means that this node is not connected to the goal
                {
                    continue;
                }
            }


            // Update the costToGo of vertex
            std::pair<Edge,double> candidate = getUpdatedNodeCostToGo(v, goalVertex);

            // for debug
//             if (!reinit)
//             {
//                 if (feedback_[v] != candidate.first)
//                 {
//                     std::cout << "ORGINAL: costToGo[" << v << "]: " << newCostToGo[v] << "\t feedback_[" << v << "]: " << feedback_[v] << std::endl;
//                     std::cout << "UPDATED: costToGo[" << v << "]: " << candidate.second << "\t feedback_[" << v << "]: " << candidate.first << std::endl;
//                 }
//                 if (candidate.second != newCostToGo[v])
//                 {
//                     std::cout << "Change to UPDATED!!!" << std::endl;
//                 }
//             }

            // update costToGo and feedback_ only if costToGo is not the same with the previous value
            // NOTE this is to prevent Dynamic Programming's naive behavior to cut off the connection obtained from Dijkstra search and just locally connect to a duplicate (or nearby) nodes, which may lead to disconnection to the goal
            if (candidate.second != newCostToGo[v])
            {
                // do not create invalid feedback_[goalVertex]
                if (v != goalVertex)
                {
                    feedback_[v] = candidate.first;
                    newCostToGo[v] = candidate.second * discountFactor;
                }
            }
        }

        //assert(costToGo_.size()==newCostToGo.size());
        diffCostToGo = norm(MapToColvec(costToGo_)-MapToColvec(newCostToGo), "inf");
        convergenceCondition = ( diffCostToGo <= convergenceThresholdDP_);

        costToGo_.swap(newCostToGo);   // Equivalent to costToGo_ = newCostToGo

    }
    if (nIter >= maxDPIterations_)
        OMPL_ERROR("Hit the maximum number of iterations before convergence! DP solution may not be valid...");

    auto end_time = std::chrono::high_resolution_clock::now();

    double timeDP = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    OMPL_INFORM("FIRM: DP Solve Time: %2.3f seconds for %d iterations (diff: %2.3f < %2.3f)<----", timeDP/1000.0, nIter, diffCostToGo, convergenceThresholdDP_);

    OMPL_INFORM("FIRM: Solved DP");

    if(doSaveLogs_)
    {
        std::ofstream outfile;
        outfile.open(logFilePath_+"DPSolveTime.txt",std::ios::app);
        outfile<<"Time: "<<timeDP<<" ms"<<std::endl;
        outfile.close();
    }
    
    sendFeedbackEdgesToViz();

    Visualizer::setMode(Visualizer::VZRDrawingMode::FeedbackViewMode);
    sleep(3);   // for visualization
}

struct DoubleValueComp
{
  template<typename KVP1, typename KVP2>
  bool operator()(const KVP1& kvp1, const KVP2& kvp2) const
  {
    return kvp1.second < kvp2.second;
  }
};


std::pair<typename FIRM::Edge,double> FIRM::getUpdatedNodeCostToGo(const FIRM::Vertex node, const FIRM::Vertex goal)
{

    std::map<Edge,double > candidateCostToGo;

    foreach(Edge e, boost::out_edges(node, g_))
    {
        // the target of given edge "e"
        const Vertex targetNode = boost::target(e, g_);

        double nextNodeCostToGo = costToGo_[targetNode];

        const FIRMWeight edgeWeight =  boost::get(boost::edge_weight, g_, e);

        const double transitionProbability  = edgeWeight.getSuccessProbability();

        arma::colvec targetToGoalVec = stateProperty_[goal]->as<FIRM::StateType>()->getArmaData() - stateProperty_[targetNode]->as<FIRM::StateType>()->getArmaData();

        //arma::colvec nodeToTargetVec = stateProperty_[targetNode]->as<FIRM::StateType>()->getArmaData() - stateProperty_[node]->as<FIRM::StateType>()->getArmaData();

        //double edgeLength =arma::norm(nodeToTargetVec.subvec(0,1),2); 

        double distToGoalFromTarget = arma::norm(targetToGoalVec.subvec(0,1),2); 

        // REVIEW why adding distToGoalFromTarget to the (default) edgeCostToGo metric?
        // NOTE without this term, DP immediately returns a incomplete solution, while this term ruins the distance metric
//         double singleCostToGo =  (transitionProbability*nextNodeCostToGo + (1-transitionProbability)*obstacleCostToGo_ + edgeWeight.getCost()) + distanceCostWeight_*distToGoalFromTarget;
//         // for debug
//         std::cout << "singleCostToGo[" << node << "->" << targetNode << "] " << singleCostToGo << " = " << transitionProbability << "*" << nextNodeCostToGo << " + " << "(1-" << transitionProbability << ")*" << obstacleCostToGo_ << " + " << edgeWeight.getCost() << " + " << distanceCostWeight_ << "*" << distToGoalFromTarget << std::endl;
        // NOTE use this (correct) metric if costToGo_ is initialized by Dijkstra search beforehand
        double singleCostToGo =  (transitionProbability*nextNodeCostToGo + (1-transitionProbability)*obstacleCostToGo_ + edgeWeight.getCost());

        candidateCostToGo[e] =  singleCostToGo ;

    }

    DoubleValueComp dvc;
    std::pair<Edge,double> bestCandidate =   *min_element(candidateCostToGo.begin(), candidateCostToGo.end(), dvc );

    return bestCandidate;

}

void FIRM::solveDijkstraSearch(const FIRM::Vertex goalVertex)
{
    OMPL_INFORM("FIRM: Solving Dijkstra search");

    auto start_time = std::chrono::high_resolution_clock::now();

    Visualizer::clearMostLikelyPath();

    using namespace arma;

    float discountFactor = discountFactorDP_;   // NOTE assumed to be 1.0

    std::map<Vertex, double> newCostToGo;
    std::map<Vertex, Vertex> bestChildVertexToGoal;
    std::vector<Vertex> closed_list;

    costToGo_ = std::map<Vertex, double>();


    // create a min heap with a user-defined comparator
    auto compareCostToGoFunc = [](const std::pair<Vertex, double>& currentVertexCostToGo, const std::pair<Vertex, double>& otherVertexCostToGo) { return (currentVertexCostToGo.second > otherVertexCostToGo.second); };

    boost::heap::fibonacci_heap<std::pair<Vertex, double>, boost::heap::compare<decltype(compareCostToGoFunc)>> heapCostToGo(compareCostToGoFunc);
    std::map<Vertex, boost::heap::fibonacci_heap<std::pair<Vertex, double>, boost::heap::compare<decltype(compareCostToGoFunc)>>::handle_type> handleCostToGo;


    // initialize the heap and a temporary cost-to-go container
    foreach (Vertex v, boost::vertices(g_))
    {
        if (v == goalVertex)
        {
            newCostToGo[v] = goalCostToGo_;
        }
        else
        {
            newCostToGo[v] = infiniteCostToGo_;
        }

        handleCostToGo[v] = heapCostToGo.push(std::pair<Vertex, double>(v, newCostToGo[v]));
    }
    feedback_.clear();

    // do Dijkstra search (starting from the goal)
    // NOTE this is a PRIORITIZED SINGLE-PASS VALUE ITERATION that can provide good intial values for general value iteration method
    // it some of transitionProbability is not equal to 1, the solution will not be optimal
    // thus, solveDynamicProgram(), which is general value iteration, should be called afterward
    int cnt=0;
    while (!heapCostToGo.empty())
    {
        // the node with the mininum cost-to-go for next expansion
        std::pair<Vertex, double> vertexCostToGo = heapCostToGo.top();
        const Vertex childVertex = vertexCostToGo.first;
        const double childCostToGo = vertexCostToGo.second;    // already closed
        heapCostToGo.pop();
        closed_list.push_back(childVertex);

        // relax the costs of edges coming toward the node
        foreach(Edge edge, boost::in_edges(childVertex, g_))
        {
            const Vertex parentVertex = boost::source(edge, g_);
            const double parentCostToGo = newCostToGo[parentVertex];

            // skip if this parent node is already closed
            //if (parentCostToGo <= childCostToGo)    // if all transitionProbability equals to 1, this condition is sufficient
            //    continue;
            if (std::find(closed_list.begin(), closed_list.end(), parentVertex) != closed_list.end())    // otherwise, we forcefully check for single-pass history
                continue;

            // update the edge cost
            double newParentCostToGo = getNewCostToGoViaChild(parentVertex, childVertex, childCostToGo, edge);

            // update the minimum cost-to-go and the best child on the shortest path to the goal (so far)
            if (newParentCostToGo < parentCostToGo)
            {
                // do not create invalid feedback_[goalVertex] and bestChildVertexToGoal_[goalVertex]
                if (parentVertex != goalVertex)
                {
                    newCostToGo[parentVertex] = newParentCostToGo;
                    bestChildVertexToGoal[parentVertex] = childVertex;
                    feedback_[parentVertex] = edge;
                    // for debug
                    //std::cout << "feedback_[" << parentVertex << "]: " << edge << std::endl;
                }

                // update the min heap
                heapCostToGo.decrease( handleCostToGo[parentVertex], std::pair<Vertex, double>(parentVertex, newParentCostToGo) );
            }
        }
    }

    costToGo_.swap(newCostToGo);                            // costToGo_ = newCostToGo
    bestChildVertexToGoal_.swap(bestChildVertexToGoal);     // bestChildVertexToGoal_ = bestChildVertexToGoal


    auto end_time = std::chrono::high_resolution_clock::now();

    double timeDijkstra = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    OMPL_INFORM("FIRM: Dijkstra Solve Time: %2.3f seconds<----", timeDijkstra/1000.0);

    OMPL_INFORM("FIRM: Solved Dijkstra search");

    if(doSaveLogs_)
    {
        std::ofstream outfile;
        outfile.open(logFilePath_+"DijkstraSolveTime.txt",std::ios::app);
        outfile<<"Time: "<<timeDijkstra<<" ms"<<std::endl;
        outfile.close();
    }

    sendFeedbackEdgesToViz();

    Visualizer::setMode(Visualizer::VZRDrawingMode::FeedbackViewMode);
//     sleep(2);   // for visualization
}

inline bool FIRM::compareCostToGo(const std::pair<Vertex, double>& currentVertexCostToGo, const std::pair<Vertex, double>& otherVertexCostToGo)
{
    double currentCostToGo = currentVertexCostToGo.second;
    double otherCostToGo = otherVertexCostToGo.second;

    return (currentCostToGo > otherCostToGo);
}

double FIRM::getNewCostToGoViaChild(const Vertex parentVertex, const Vertex childVertex, const double childCostToGo, const Edge edge)
{
    const FIRMWeight edgeWeight = boost::get(boost::edge_weight, g_, edge);
    const double transitionProbability = edgeWeight.getSuccessProbability();

    double newCostToGoViaChild = transitionProbability*childCostToGo + (1-transitionProbability)*obstacleCostToGo_ + edgeWeight.getCost();
    return newCostToGoViaChild;
}

double FIRM::evaluateSuccessProbability(const Edge currentEdge, const FIRM::Vertex start, const FIRM::Vertex goal)
{
    const FIRMWeight currentEdgeWeight = boost::get(boost::edge_weight, g_, currentEdge);

    double successProb = currentEdgeWeight.getSuccessProbability();

    Vertex v = boost::target(currentEdge, g_);

    while(v != goal)
    {
        if(feedback_.find(v) == feedback_.end())  // not connected to the goal
            return 0.0;

        Edge edge = feedback_.at(v);

        const FIRMWeight edgeWeight =  boost::get(boost::edge_weight, g_, edge);

        const double transitionProbability  = edgeWeight.getSuccessProbability();

        //OMPL_INFORM("The success probability of edge is %f", transitionProbability);

        successProb = successProb * transitionProbability;

        v = boost::target(edge, g_);

    }

    return successProb;
}

void FIRM::updateEdgeCollisionCost(FIRM::Vertex currentVertex, FIRM::Vertex goalVertex)
{
    // cycle through feedback, update edge costs for edges in collision
    while(currentVertex != goalVertex)
    {
        if(feedback_.find(currentVertex) == feedback_.end())  // not connected to the goal
            return;

        Edge edge = feedback_.at(currentVertex); // get the edge

        Vertex target = boost::target(edge, g_); // get the target of this edge

        // if edge is invalid, increase its cost
        if(!si_->checkMotion(stateProperty_[currentVertex], stateProperty_[target]))
        {
            double pvc1 = weightProperty_[edge].getCost();
    
            weightProperty_[edge].setCost(pvc1 + obstacleCostToGo_*10);

            weightProperty_[edge].setSuccessProbability(0.0);

            // Get outgoing edges of target
            foreach(Edge e, boost::out_edges(target, g_))
            {
                // if any outgoing edge is in collision, update its cost
                if(!si_->checkMotion(stateProperty_[target], stateProperty_[boost::target(e, g_)]))
                {
                    double pvc2 = weightProperty_[e].getCost();
    
                    weightProperty_[e].setCost(pvc2 + obstacleCostToGo_*10);

                    weightProperty_[e].setSuccessProbability(0.0);
                }
            }

        }
           
        currentVertex =  target;

    }

}

void FIRM::executeFeedback(void)
{
    EdgeControllerType edgeController;
    NodeControllerType nodeController;

    bool edgeControllerStatus;
    bool nodeControllerStatus;


    Vertex start = startM_[0];
    Vertex goal = goalM_[0];
    Vertex currentVertex = start;

    ompl::base::State *cstartState = si_->allocState();
    ompl::base::State *cendState = si_->allocState();
    ompl::base::State *goalState = si_->cloneState(stateProperty_[goal]);
    ompl::base::State *tempTrueStateCopy = si_->allocState();

    si_->copyState(cstartState, stateProperty_[start]);
    siF_->setTrueState(stateProperty_[start]);
    siF_->setBelief(stateProperty_[start]);


    Visualizer::setMode(Visualizer::VZRDrawingMode::FIRMMode);
    Visualizer::clearRobotPath();
    sendMostLikelyPathToViz(start, goal);

    Visualizer::doSaveVideo(doSaveVideo_);
    siF_->doVelocityLogging(true);
    nodeReachedHistory_.push_back(std::make_pair(currentTimeStep_, numberofNodesReached_) );


    OMPL_INFORM("FIRM: Running policy execution");

    while(!goalState->as<FIRM::StateType>()->isReached(cstartState))
    {
        // this implicitly means that isReached() for the goal is satisfied (by the node controller)
        if(currentVertex==goal)
            break;

        Edge e = feedback_.at(currentVertex);

        assert(currentVertex < boost::num_vertices(g_));

        // for debug
        if(ompl::magic::PRINT_FEEDBACK_PATH)
            std::cout << "PATH[" << currentVertex;

        // Check if feedback policy is valid 
        while(!isFeedbackPolicyValid(currentVertex, goal))
        {
            OMPL_INFORM("FIRM: Invalid path detected from Vertex %u to %u", currentVertex, boost::target(e, g_));

            updateEdgeCollisionCost(currentVertex, goal);

            // resolve DP
            solveDijkstraSearch(goal);
            solveDynamicProgram(goal, false);

            e = feedback_.at(currentVertex);

            OMPL_INFORM("FIRM: Updated path, moving from Vertex %u to %u", currentVertex, boost::target(e, g_));
        }

        double succProb = evaluateSuccessProbability(e, currentVertex, goal);
        successProbabilityHistory_.push_back(std::make_pair(currentTimeStep_, succProb) );

        OMPL_INFORM("FIRM: Moving from Vertex %u (%2.3f, %2.3f, %2.3f, %2.6f) to %u (%2.3f, %2.3f, %2.3f, %2.6f) with TP = %f", currentVertex, boost::target(e, g_), 
                stateProperty_[currentVertex]->as<FIRM::StateType>()->getX(),
                stateProperty_[currentVertex]->as<FIRM::StateType>()->getY(),
                stateProperty_[currentVertex]->as<FIRM::StateType>()->getYaw(),
                arma::trace(stateProperty_[currentVertex]->as<FIRM::StateType>()->getCovariance()),
                stateProperty_[boost::target(e, g_)]->as<FIRM::StateType>()->getX(),
                stateProperty_[boost::target(e, g_)]->as<FIRM::StateType>()->getY(),
                stateProperty_[boost::target(e, g_)]->as<FIRM::StateType>()->getYaw(),
                arma::trace(stateProperty_[boost::target(e, g_)]->as<FIRM::StateType>()->getCovariance()),
                succProb);


        ompl::base::Cost costCov;
        int stepsExecuted = 0;
        int stepsToStop = 0;


        // NOTE NodeController will be invoked after executing EdgeController until it ends (until the end of sequence or isTerminated() is satisfied)

        // [1] EdgeController
        {
            edgeController = edgeControllers_.at(e);
            edgeController.setSpaceInformation(policyExecutionSI_);
            edgeControllerStatus = edgeController.Execute(cstartState, cendState, costCov, stepsExecuted, stepsToStop, false);


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
        {
            nodeController = nodeControllers_.at(boost::target(e, g_));
            nodeController.setSpaceInformation(policyExecutionSI_);
            nodeControllerStatus = nodeController.Stabilize(cstartState, cendState, costCov, stepsExecuted, false);


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

        } // [2] NodeController


        if(nodeControllerStatus)    // regardless of failure of EdgeController (EdgeControllerStatus) due to high deviation
        {
            numberofNodesReached_++;
            nodeReachedHistory_.push_back(std::make_pair(currentTimeStep_, numberofNodesReached_) );
            currentVertex = boost::target(e, g_);
        }
        else
        {
            Visualizer::doSaveVideo(false);
            siF_->doVelocityLogging(false);

            OMPL_INFORM("Controller stopped due to deviation, need to add new state at: ");
            siF_->printState(cendState);

            currentVertex = addStateToGraph(cendState);

            // set true state back to its correct value after Monte Carlo (happens during adding state to Graph)
            siF_->setTrueState(tempTrueStateCopy);

            solveDijkstraSearch(goal);
            solveDynamicProgram(goal, false);

            Visualizer::doSaveVideo(doSaveVideo_);
            siF_->doVelocityLogging(true);
        }

    } // while()

    // save the visualization
    Visualizer::doSaveVideo(true);

    // for analysis
    // this data is also saved in run-(TIMESTAMP)/RolloutFIRMCostHistory.csv
    std::cout << std::endl;
    std::cout << "Execution time steps: " << currentTimeStep_ << std::endl;
    std::cout << "Execution covariance cost: " << executionCostCov_ << std::endl;
    std::cout << "Execution cost: " << executionCost_ << "  ( = " << informationCostWeight_ << "*" << executionCostCov_ << " + " << timeCostWeight_ << "*" << currentTimeStep_ << " )" << std::endl;     // 1)
    //std::cout << "Execution cost: " << executionCost_ << "  ( = " << informationCostWeight_ << "*" << executionCostCov_ << "/" << currentTimeStep_ << " + " << timeCostWeight_ << "*" << currentTimeStep_ << " )" << std::endl;     // 3)
    //std::cout << "Execution cost: " << executionCost_ << "  ( = " << informationCostWeight_ << "*" << executionCostCov_ << " )" << std::endl;     // 4)
    std::cout << std::endl;


    if(doSaveLogs_)
    {
        std::vector<std::pair<double, double> > velLog;
        siF_->getVelocityLog(velLog);
        for(int i=0; i < velLog.size(); i++)
        {
            //velocityHistory_.push_back(std::make_pair(i, velLog[i].first)); //unicycle
            velocityHistory_.push_back(std::make_pair(i, sqrt( pow(velLog[i].first,2) + pow(velLog[i].second,2) ))); // omni
        }

        writeTimeSeriesDataToFile("StandardFIRMCostHistory.csv", "costToGo");
        writeTimeSeriesDataToFile("StandardFIRMSuccessProbabilityHistory.csv", "successProbability");
        writeTimeSeriesDataToFile("StandardFIRMNodesReachedHistory.csv","nodesReached");
        writeTimeSeriesDataToFile("StandardFIRMVelocityHistory.csv", "velocity");
    }
    Visualizer::doSaveVideo(false);

    // free the memory
    si_->freeState(tempTrueStateCopy);
    si_->freeState(cstartState);
    si_->freeState(cendState);
}

void FIRM::executeFeedbackWithKidnapping(void)
{

    Vertex start = startM_[0];
    Vertex goal  = goalM_[0] ;

    ompl::base::State *goalState = si_->cloneState(stateProperty_[goal]);

    sendMostLikelyPathToViz(start, goal);

    siF_->setTrueState(stateProperty_[start]);
    siF_->setBelief(stateProperty_[start]);

    Vertex currentVertex =  start;

    EdgeControllerType controller;

    ompl::base::State *cstartState = si_->allocState();
    si_->copyState(cstartState, stateProperty_[start]);

    ompl::base::State *cendState = si_->allocState();

    OMPL_INFORM("FIRM: Running policy execution");

    bool kidnapped_flag = false;

    int kidnappingCounter  = 0;

    Visualizer::doSaveVideo(doSaveVideo_);

    while(!goalState->as<FIRM::StateType>()->isReached(cstartState)/*currentVertex != goal*/)
    {

        if(currentVertex==goal)
            break;

        Edge e = feedback_.at(currentVertex);

        double succProb = evaluateSuccessProbability(e, currentVertex, goal);

        OMPL_INFORM("FIRM: Moving from Vertex %u (%2.3f, %2.3f, %2.3f, %2.6f) to %u (%2.3f, %2.3f, %2.3f, %2.6f) with TP = %f", currentVertex, boost::target(e, g_), 
                stateProperty_[currentVertex]->as<FIRM::StateType>()->getX(),
                stateProperty_[currentVertex]->as<FIRM::StateType>()->getY(),
                stateProperty_[currentVertex]->as<FIRM::StateType>()->getYaw(),
                arma::trace(stateProperty_[currentVertex]->as<FIRM::StateType>()->getCovariance()),
                stateProperty_[boost::target(e, g_)]->as<FIRM::StateType>()->getX(),
                stateProperty_[boost::target(e, g_)]->as<FIRM::StateType>()->getY(),
                stateProperty_[boost::target(e, g_)]->as<FIRM::StateType>()->getYaw(),
                arma::trace(stateProperty_[boost::target(e, g_)]->as<FIRM::StateType>()->getCovariance()),
                succProb);

        successProbabilityHistory_.push_back(std::make_pair(currentTimeStep_, succProb) );

        controller = edgeControllers_.at(e);

        ompl::base::Cost costCov(0);

        int stepsExecuted = 0;

        int stepsToStop = 0;

        controller.setSpaceInformation(policyExecutionSI_);

        bool controllerStatus = controller.Execute(cstartState, cendState, costCov, stepsExecuted, stepsToStop, false);


        // NOTE how to penalize uncertainty (covariance) and path length (time steps) in the cost
        //*1) cost = wc * sum(trace(cov_k))  + wt * K  (for k=1,...,K)
        // 2) cost = wc * trace(cov_f)       + wt * K
        // 3) cost = wc * mean(trace(cov_k)) + wt * K
        // 4) cost = wc * sum(trace(cov_k))

        currentTimeStep_ += stepsExecuted;

        executionCostCov_ += costCov.value() - ompl::magic::EDGE_COST_BIAS;    // 1,2,3,4) costCov is actual execution cost but only for covariance penalty (even without weight multiplication)

        executionCost_ = informationCostWeight_*executionCostCov_ + timeCostWeight_*currentTimeStep_;    // 1)
//         executionCost_ = informationCostWeight_*executionCostCov_/(currentTimeStep_==0 ? 1e-10 : currentTimeStep_) + timeCostWeight_*currentTimeStep_;    // 3)
//         executionCost_ = informationCostWeight_*executionCostCov_;    // 4)

        costHistory_.push_back(std::make_tuple(currentTimeStep_, executionCostCov_, executionCost_));


         // get a copy of the true state
        ompl::base::State *tempTrueStateCopy = si_->allocState();

        siF_->getTrueState(tempTrueStateCopy);

        if(!si_->isValid(tempTrueStateCopy))
        {
            OMPL_INFORM("Robot Collided :(");
            return;
        }

        if(controllerStatus)
        {
            currentVertex = boost::target(e, g_);
        }
        else
        {

            Visualizer::doSaveVideo(false);

            currentVertex = addStateToGraph(cendState);

            // set true state back to its correct value after Monte Carlo (happens during adding state to Graph)
            siF_->setTrueState(tempTrueStateCopy);

            solveDijkstraSearch(goal);
            solveDynamicProgram(goal, false);

            Visualizer::doSaveVideo(doSaveVideo_);

        }

        si_->freeState(tempTrueStateCopy);

        /**
        1. Check if innovation i.e. change in trace(cov) between start and end state is high
        2. If the change is high, then switch to lost mode
        3. Sample modes and run policygen till you converge to one mode
        4. get back to policy execution
        */
        if(si_->distance(cendState,stateProperty_[goal]) < si_->distance(cendState,stateProperty_[start]) && !kidnapped_flag && kidnappingCounter < 1)
        {
            std::cout<<"Before Simulated Kidnapping! (Press Enter) \n";
            //std::cin.get();
            this->simulateKidnapping();
            std::cout<<"AFter Simulated Kidnapping! (Press Enter) \n";
            //std::cin.get();
            kidnapped_flag = true;
            kidnappingCounter++;
        }

        if(kidnapped_flag) //if(this->detectKidnapping(cstartState, cendState))
        {
            recoverLostRobot(cendState);

            siF_->setBelief(cendState);

             // get a copy of the true state
            ompl::base::State *tempTrueStateCopy = si_->allocState();

            siF_->getTrueState(tempTrueStateCopy);

            currentVertex = addStateToGraph(cendState);

            // set true state back to its correct value after Monte Carlo (happens during adding state to Graph)
            siF_->setTrueState(tempTrueStateCopy);

            siF_->freeState(tempTrueStateCopy);

            solveDijkstraSearch(goal);
            solveDynamicProgram(goal, false);

            sendMostLikelyPathToViz(currentVertex, goal);

            kidnapped_flag = false;
        }

        si_->copyState(cstartState, cendState);

    }

    Visualizer::doSaveVideo(false);

    // free the memory
    si_->freeState(cstartState);
    si_->freeState(cendState);
}

// Experimental
void FIRM::executeFeedbackWithRollout(void)
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


    OMPL_INFORM("FIRM: Running policy execution");

    // While the robot state hasn't reached the goal state, keep running
    // HACK setting relaxedConstraint argument to false means isReached() condition is exceptionally relaxed for termination
    while(!goalState->as<FIRM::StateType>()->isReached(cstartState, true))
    //while(!goalState->as<FIRM::StateType>()->isReached(cstartState, false))
    {
        targetNode = boost::target(e, g_);

        double succProb = evaluateSuccessProbability(e, tempVertex, goal);
        successProbabilityHistory_.push_back(std::make_pair(currentTimeStep_, succProb ) );

        OMPL_INFORM("FIRM Rollout: Moving from Vertex %u (%2.3f, %2.3f, %2.3f, %2.6f) to %u (%2.3f, %2.3f, %2.3f, %2.6f) with TP = %f", tempVertex, targetNode, 
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
//                 edgeControllers_[edge].freeLinearSystems();
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
            OMPL_INFORM("FIRM Rollout: Reached FIRM Node: %u", targetNode);
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

    // save the visualization
    Visualizer::doSaveVideo(true);

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

void FIRM::showRolloutConnections(const FIRM::Vertex v)
{
    Visualizer::clearRolloutConnections();

    // NOTE redundant call of nearest neighbor search just for visualization!

    //const std::vector<Vertex>& neighbors = connectionStrategy_(v);
    const std::vector<Vertex>& neighbors = connectionStrategy_(v, NNRadius_);     // NOTE to allow a variable (bounding) radius to neighbors
    foreach (Vertex n, neighbors)
    {
        Visualizer::addRolloutConnection(stateProperty_[v], stateProperty_[n]);
    }

    const std::vector<Vertex>& kneighbors = kConnectionStrategy_(v, numNearestNeighbors_);
    foreach (Vertex n, kneighbors)
    {
        Visualizer::addRolloutConnection(stateProperty_[v], stateProperty_[n]);
    }

    boost::this_thread::sleep(boost::posix_time::milliseconds(50));
}

void FIRM::addStateToVisualization(const ompl::base::State *state)
{
    Visualizer::addState(state);
}

void FIRM::sendFeedbackEdgesToViz()
{
    Visualizer::ClearFeedbackEdges();

    for(typename std::map<Vertex, Edge>::const_iterator i=feedback_.begin(), e=feedback_.end(); i!=e; ++i)
    {
        Vertex sourceVertex, targetVertex;
        Edge edge;
        sourceVertex = i->first;
        edge = i->second;
        targetVertex = boost::target(edge, g_);
        //OMPL_INFORM("FIRM: MLP from Vertex %u to %u", sourceVertex,targetVertex);
        Visualizer::addFeedbackEdge(stateProperty_[sourceVertex], stateProperty_[targetVertex], 0);
    }

}

void FIRM::sendMostLikelyPathToViz(const FIRM::Vertex start, const FIRM::Vertex goal)
{
    Visualizer::clearMostLikelyPath();

    Vertex v = start;

    while(v != goal)
    {
        Vertex targetVertex;

        Edge edge = feedback_.at(v);

        targetVertex = boost::target(edge, g_);

        OMPL_INFORM("Most Likely Path from %u to %u", v, targetVertex);
        
        Visualizer::addMostLikelyPathEdge(stateProperty_[v], stateProperty_[targetVertex]);

        v = targetVertex;
    }
}

bool FIRM::isFeedbackPolicyValid(FIRM::Vertex currentVertex, FIRM::Vertex goalVertex)
{
    // for debug
    if(ompl::magic::PRINT_FEEDBACK_PATH)
        std::cout << "->" << currentVertex;

    // cycle through feedback, if feedback edge is invalid, return false
    // NOTE Dynamic Programming may leave feedback_ path information even to the disconnected components
    // to fix this behavior, limit the number of iteration to the total number of vertices in the graph
    int nIter=0;
    while(currentVertex != goalVertex && nIter < boost::num_vertices(g_))
    {
        // to avoid illegal access to non-existing element in feedback_
        if (feedback_.find(currentVertex) == feedback_.end())    // there is no feedback edge coming from this vertex
        {
            OMPL_WARN("Reached a node that is NOT connected to the goal! Quit isFeedbackPolicyValid()!");
            return true;    // not returning false to avoid calling solveDijkstraSearch()/solveDynamicProgram() again!
        }

        // to avoid infinite loop of checkMotion() and effectively ignore disconnected node from rollout candidates
        if (costToGo_[currentVertex] >= infiniteCostToGo_)
        {
            OMPL_WARN("Reached a node that is NOT connected to the goal! Quit isFeedbackPolicyValid()!");
            return true;    // not returning false to avoid calling solveDijkstraSearch()/solveDynamicProgram() again!
        }


        Edge edge = feedback_.at(currentVertex); // get the edge

        Vertex target = boost::target(edge, g_); // get the target of this edge

        // if edge is invalid, increase its cost
        if(!si_->checkMotion(stateProperty_[currentVertex], stateProperty_[target]))
        {
            return false;
        }
           
        currentVertex =  target;

        nIter++;

        // for debug
        if(ompl::magic::PRINT_FEEDBACK_PATH)
            std::cout << "->" << target;
    }

    if(nIter >= boost::num_vertices(g_))
    {
        OMPL_WARN("Reached a node that is NOT connected to the goal! Cleaning up feedback_ and quit isFeedbackPolicyValid()...");

        // remove these unconnected nodes from the feedback_ data
        while (true)
        {
            if (feedback_.find(currentVertex) == feedback_.end())    // there is no feedback edge coming from this vertex
            {
                break;
            }

            Edge edge = feedback_.at(currentVertex); // get the edge
            Vertex target = boost::target(edge, g_); // get the target of this edge

            costToGo_[currentVertex] = infiniteCostToGo_;
            feedback_.erase(currentVertex);

            currentVertex =  target;
        }

        return true;    // not returning false to avoid calling solveDijkstraSearch()/solveDynamicProgram() again!
    }

    // for debug
    if(ompl::magic::PRINT_FEEDBACK_PATH)
        std::cout << "]" << std::endl;

    return true;

}

FIRM::Edge FIRM::generateRolloutPolicy(const FIRM::Vertex currentVertex, const FIRM::Vertex goal)
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

            //OMPL_INFORM("Rollout: Invalid path detected from Vertex %u to %u", targetNode, targetOfNextFIRMEdge);

            updateEdgeCollisionCost(targetNode, goal);

            // resolve Dijkstra/DP
            solveDijkstraSearch(goal);
            solveDynamicProgram(goal, false);

            //targetOfNextFIRMEdge = boost::target(feedback_.at(targetNode), g_);  
            //OMPL_INFORM("Rollout: Updated path, next firm edge moving from Vertex %u to %u", targetNode, targetOfNextFIRMEdge);

        }

        // The cost to go from the target node
        double nextNodeCostToGo = costToGo_[targetNode];

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
//             std::cout << "COST[" << currentVertex << "->" << targetNode << "->G] " << edgeCostToGo << " = " << transitionProbability << "*" << nextNodeCostToGo << " + " << "(1-" << transitionProbability << ")*" << obstacleCostToGo_ << " + " << edgeWeight.getCost() << std::endl;
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

void FIRM::simulateKidnapping()
{
    siF_->setTrueState(kidnappedState_);
}

bool FIRM::detectKidnapping(ompl::base::State *previousState, ompl::base::State *newState)
{

    using namespace arma;

    mat previousCov = previousState->as<FIRM::StateType>()->getCovariance();

    mat newCov = newState->as<FIRM::StateType>()->getCovariance();

    double innovSignal = (trace(newCov) - trace(previousCov)) / trace(previousCov);

    if(innovSignal >= ompl::magic::KIDNAPPING_INNOVATION_CHANGE_THRESHOLD )
    {
        return true;
    }

    return false;

}

void FIRM::savePlannerData()
{

    std::vector<std::pair<int,std::pair<arma::colvec,arma::mat> > > nodes;

    foreach(Vertex v, boost::vertices(g_))
    {

        arma::colvec xVec = stateProperty_[v]->as<FIRM::StateType>()->getArmaData();

        arma::mat cov = stateProperty_[v]->as<FIRM::StateType>()->getCovariance();

        std::pair<int,std::pair<arma::colvec,arma::mat> > nodeToWrite = std::make_pair(v, std::make_pair(xVec, cov)) ;

        nodes.push_back(nodeToWrite);

    }

    std::vector<std::pair<std::pair<int,int>,FIRMWeight> > edgeWeights;

    foreach(Edge e, boost::edges(g_))
    {
        Vertex start = boost::source(e,g_);
        Vertex goal  = boost::target(e,g_);

        const FIRMWeight w = boost::get(boost::edge_weight, g_, e);

        edgeWeights.push_back(std::make_pair(std::make_pair(start,goal),w));

    }

    FIRMUtils::writeFIRMGraphToXML(nodes, edgeWeights);

}


void FIRM::loadRoadMapFromFile(const std::string &pathToFile)
{
    std::vector<std::pair<int, arma::colvec> > FIRMNodePosList;
    std::vector<std::pair<int, arma::mat> > FIRMNodeCovarianceList;

    boost::mutex::scoped_lock _(graphMutex_);

    if(FIRMUtils::readFIRMGraphFromXML(pathToFile,  FIRMNodePosList, FIRMNodeCovarianceList , loadedEdgeProperties_))
    {

        loadedRoadmapFromFile_ = true;

        this->setup();

        for(int i = 0; i < FIRMNodePosList.size() ; i++)
        {
            ompl::base::State *newState = siF_->allocState();

            arma::colvec xVec = FIRMNodePosList[i].second;
            arma::mat     cov = FIRMNodeCovarianceList[i].second;

            newState->as<FIRM::StateType>()->setArmaData(xVec);
            newState->as<FIRM::StateType>()->setCovariance(cov);

            //Vertex v = addStateToGraph(siF_->cloneState(newState));

            Vertex m;

            m = boost::add_vertex(g_);

            stateProperty_[m] = newState;

            NodeControllerType nodeController;

            generateNodeController(newState, nodeController); // Generate the node controller

            nodeControllers_[m] = nodeController; // Add it to the list

            // Initialize to its own (dis)connected component.
            disjointSets_.make_set(m);

            nn_->add(m);

            policyGenerator_->addFIRMNodeToObservationGraph(newState);

            addStateToVisualization(newState);

            assert(m==FIRMNodePosList[i].first && "IDS DONT MATCH !!");
        }

        bool unite = true;

        for(int i=0; i<loadedEdgeProperties_.size(); i++)
        {
            EdgeControllerType edgeController;

            Vertex a = loadedEdgeProperties_[i].first.first;
            Vertex b = loadedEdgeProperties_[i].first.second;

            ompl::base::State* startNodeState = siF_->cloneState(stateProperty_[a]);
//             ompl::base::State* targetNodeState = siF_->cloneState(stateProperty_[b]);
            ompl::base::State* targetNodeState = stateProperty_[b];

            // Generate the edge controller for given start and end state
            generateEdgeController(startNodeState,targetNodeState,edgeController);

            const FIRMWeight weight = loadedEdgeProperties_[i].second;

            const unsigned int id = maxEdgeID_++;

            const Graph::edge_property_type properties(weight, id);

            // create an edge with the edge weight property
            std::pair<Edge, bool> newEdge = boost::add_edge(a, b, properties, g_);

            edgeControllers_[newEdge.first] = edgeController;

            if(unite)
                uniteComponents(a, b);

            unite = !unite;

            Visualizer::addGraphEdge(stateProperty_[a], stateProperty_[b]);

            // free the memory
            siF_->freeState(startNodeState);
//             siF_->freeState(targetNodeState);
        }

    }
}

void FIRM::writeTimeSeriesDataToFile(std::string fname, std::string dataName)
{

    std::ofstream outfile;

    outfile.open(logFilePath_ + fname);

    if(dataName.compare("costToGo")==0)
    {
        for(int i=0; i < costHistory_.size(); i++)
        {
            outfile << std::get<0>(costHistory_[i]) << "," << std::get<1>(costHistory_[i]) << "," << std::get<2>(costHistory_[i]) << std::endl;
        }
    }

    if(dataName.compare("successProbability")==0)
    {
        for(int i=0; i < successProbabilityHistory_.size(); i++)
        {
            outfile<<successProbabilityHistory_[i].first<<","<<successProbabilityHistory_[i].second<<std::endl;
        }
    }

    if(dataName.compare("nodesReached")==0)
    {
        for(int i=0; i < nodeReachedHistory_.size(); i++)
        {
            outfile<<nodeReachedHistory_[i].first<<","<<nodeReachedHistory_[i].second<<std::endl;
        }
    }

    if(dataName.compare("stationaryPenalty")==0)
    {
        for(int i=0; i < stationaryPenaltyHistory_.size(); i++)
        {
            outfile << std::get<0>(stationaryPenaltyHistory_[i]) << "," << std::get<1>(stationaryPenaltyHistory_[i]) << "," << std::get<2>(stationaryPenaltyHistory_[i]) << std::endl;
        }
    }

    if(dataName.compare("velocity")==0)
    {
        for(int i=0; i < velocityHistory_.size(); i++)
        {
            outfile<<velocityHistory_[i].first<<","<<velocityHistory_[i].second<<std::endl;
        }
    }

    if(dataName.compare("multiModalWeights")==0)
    {
        for(int i=0; i < successProbabilityHistory_.size(); i++)
        {
            outfile<<weightsHistory_[i].first<<",";

            for(int j=0; j< weightsHistory_[i].second.size(); j++)
            {
                outfile<<weightsHistory_[i].second[j]<<",";
            }
            outfile<<std::endl;
        }
    }

    outfile.close();
}

void FIRM::loadParametersFromFile(const std::string &pathToFile)
{
   TiXmlDocument doc(pathToFile);

    bool loadOkay = doc.LoadFile();

    if( !loadOkay )
    {
        printf( "FIRM: Could not load setup file. Error='%s'. Exiting.\n", doc.ErrorDesc() );

        exit( 1 );
    }

    TiXmlNode* node = 0;

    TiXmlElement* itemElement = 0;

    node = doc.FirstChild( "FIRM" );
    assert( node );

    TiXmlNode* child = 0;

    // Video
    child = node->FirstChild("Video");
    assert( child );

    itemElement = child->ToElement();
    assert( itemElement );

    int saveVideo = 0;
    itemElement->QueryIntAttribute("save", &saveVideo);
//     if(saveVideo == 1) doSaveLogs_ = true;
    if(saveVideo == 1) doSaveVideo_ = true;     // TODO something is not working with saving videos

    // Logging
    child = node->FirstChild("DataLog");
    assert( child );

    itemElement = child->ToElement();
    assert( itemElement );

    std::string logpath;
    itemElement->QueryStringAttribute("folder", &logpath);
    logFilePath_ = logpath;

    namespace pt = boost::posix_time;

    pt::ptime now = pt::second_clock::local_time();

    std::string timeStamp(to_iso_string(now)) ;

    logFilePath_ = logFilePath_ + "run-" + timeStamp ;

    boost::filesystem::path dir(logFilePath_);

    logFilePath_ = logFilePath_ + "/";

    int saveLog = 0;
    itemElement->QueryIntAttribute("save", &saveLog);
    if(saveLog==1)
    {
        doSaveLogs_ = true;
        boost::filesystem::create_directory(dir);
    }
    else
    {
       doSaveLogs_ = false;
    }

    // Roadmap output
    child = node->FirstChild("Roadmap");
    assert( child );

    itemElement = child->ToElement();
    assert( itemElement );

    int saveRoadmap = 0;
    itemElement->QueryIntAttribute("save", &saveRoadmap);

    if(saveRoadmap == 1)
    {
        doSavePlannerData_ = true;
    }
    else
    {
        doSavePlannerData_ = false;
    }

    // Monte carlo parameters
    child = node->FirstChild("MCParticles");
    assert( child );

    itemElement = child->ToElement();
    assert( itemElement );

    int numP = 0;
    itemElement->QueryIntAttribute("numparticles", &numP);
    numMCParticles_ = numP;

   
    // Rollout steps
    child = node->FirstChild("RolloutSteps");
    assert( child );

    itemElement = child->ToElement();
    assert( itemElement );

    int rolloutSteps = 0;
    itemElement->QueryIntAttribute("rolloutsteps", &rolloutSteps);
    rolloutSteps_ = rolloutSteps;

    // Nearest neighbor radius
    child = node->FirstChild("NNRadius");
    assert( child );

    itemElement = child->ToElement();
    assert( itemElement );

    double nnradius = 0.0;
    itemElement->QueryDoubleAttribute("nnradius", &nnradius);
    NNRadius_ = nnradius;

    // Nearest neighbors num
    child = node->FirstChild("NumNN");
    assert( child );

    itemElement = child->ToElement();
    assert( itemElement );

    int numnn = 0;
    itemElement->QueryIntAttribute("numnn", &numnn);
    numNearestNeighbors_ = numnn;

    // DP params
    double discountFactorDP = 0.0, informationCostWeight = 0.0, timeCostWeight = 0.0, statCostIncrement = 0.0, distanceCostWeight = 0.0, goalCostToGo = 0.0, obstacleCostToGo = 0.0, initialCostToGo = 0.0, convergenceThresholdDP = 0.0;
    int maxDPIterations = 0;

    child = node->FirstChild("DPDiscountFactor");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("discountfac", &discountFactorDP);
    discountFactorDP_ = discountFactorDP;

    child = node->FirstChild("DistCostWeight");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("distcostw", &distanceCostWeight);
    distanceCostWeight_ = distanceCostWeight;


    int connectToFutureNodes = 0, applyStationaryPenalty = 0, borderBeliefSampling = 0;
    child = node->FirstChild("StabilizationHack");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryIntAttribute("connectToFutureNodes", &connectToFutureNodes);
    itemElement->QueryIntAttribute("applyStationaryPenalty", &applyStationaryPenalty);
    itemElement->QueryIntAttribute("borderBeliefSampling", &borderBeliefSampling);
    connectToFutureNodes_ = (bool)connectToFutureNodes;
    applyStationaryPenalty_ = (bool)applyStationaryPenalty;
    borderBeliefSampling_ = (bool)borderBeliefSampling;


    child = node->FirstChild("InfCostWeight");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("infcostw", &informationCostWeight);
    informationCostWeight_ = informationCostWeight;

    child = node->FirstChild("TimeCostWeight");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("timecostw", &timeCostWeight);
    timeCostWeight_ = timeCostWeight;

    child = node->FirstChild("StatCostInc");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("statcostinc", &statCostIncrement);
    statCostIncrement_ = statCostIncrement;


    child = node->FirstChild("GoalCostToGo");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("goalctg", &goalCostToGo);
    goalCostToGo_ = goalCostToGo;

    child = node->FirstChild("ObstCostToGo");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("obsctg", &obstacleCostToGo);
    obstacleCostToGo_ = obstacleCostToGo;

    child = node->FirstChild("InitCostToGo");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("initctg", &initialCostToGo);
    initialCostToGo_ = initialCostToGo;

    child = node->FirstChild("DPConvergenceThreshold");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("dpconvthresh", &convergenceThresholdDP);
    convergenceThresholdDP_ = convergenceThresholdDP;

    child = node->FirstChild("MaxDPIter");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryIntAttribute("dpiter", &maxDPIterations);
    maxDPIterations_ = maxDPIterations;

    OMPL_INFORM("FIRM: NNRadius = %f", NNRadius_);
}

bool FIRM::isStartVertex(const Vertex v)
{

    for(int i=0; i <  startM_.size(); i++)
    {
        if(v == startM_[i])
            return true;
    }

    return false;

}

bool FIRM::isGoalVertex(const Vertex v)
{
    for(int i=0; i <  goalM_.size(); i++)
    {
        if(v == goalM_[i])
            return true;
    }

    return false;

}

void FIRM::recoverLostRobot(ompl::base::State *recoveredState)
{
    Visualizer::doSaveVideo(false);

    Visualizer::clearMostLikelyPath();

    Visualizer::setMode(Visualizer::VZRDrawingMode::MultiModalMode);

    auto start_time_sampling = std::chrono::high_resolution_clock::now();

    policyGenerator_->setPolicyExecutionSpace(policyExecutionSI_);

    policyGenerator_->sampleNewBeliefStates();

    auto end_time_sampling = std::chrono::high_resolution_clock::now();

    std::cout << "Time to sample beliefs: "<<std::chrono::duration_cast<std::chrono::milliseconds>(end_time_sampling - start_time_sampling).count() << " milli seconds."<<std::endl;

    std::cin.get();

    //ompl::base::State *currentTrueState = siF_->allocState();

    //siF_->getTrueState(currentTrueState);

    int counter = 0;

    Visualizer::doSaveVideo(doSaveVideo_);

    int timeSinceKidnap = 0;

    weightsHistory_.push_back(std::make_pair(timeSinceKidnap,policyGenerator_->getWeights()));

    auto start_time_recovery = std::chrono::high_resolution_clock::now();

    while(!policyGenerator_->isConverged())
    {
        std::vector<ompl::control::Control*> policy;

        policyGenerator_->generatePolicy(policy);

        Visualizer::doSaveVideo(doSaveVideo_);

        int rndnum = FIRMUtils::generateRandomIntegerInRange(100, ompl::magic::MAX_MM_POLICY_LENGTH/*policy.size()-1*/);

        //int hzn = policy.size()-1;
        int hzn = ompl::magic::MAX_MM_POLICY_LENGTH > policy.size()? policy.size() : rndnum;

        for(int i=0; i < hzn ; i++)
        {
            policyExecutionSI_->applyControl(policy[i]);

            unsigned int numModesBefore = policyGenerator_->getNumberOfModes();

            policyGenerator_->propagateBeliefs(policy[i]);

            unsigned int numModesAfter = policyGenerator_->getNumberOfModes();

            // If any of the modes gets deleted, break and find new policy
            if(numModesAfter != numModesBefore)
                break;

            //siF_->getTrueState(currentTrueState);

            // If the belief's clearance gets below the threshold, break loop & replan
            if(!policyGenerator_->doCurrentBeliefsSatisfyClearance(i))
            {
                if(counter == 0)
                {
                    counter++;
                    break;
                }

            }

            if(counter > ompl::magic::MIN_STEPS_AFTER_CLEARANCE_VIOLATION_REPLANNING)
                counter = 0;

            if(policyGenerator_->isConverged())
                break;

            //boost::this_thread::sleep(boost::posix_time::milliseconds(20));

            timeSinceKidnap++;

            weightsHistory_.push_back(std::make_pair(timeSinceKidnap,policyGenerator_->getWeights()));


        }

    }

    auto end_time_recovery = std::chrono::high_resolution_clock::now();

    std::cout << "Time to sample recover (exclude sampling): "<<std::chrono::duration_cast<std::chrono::milliseconds>(end_time_recovery - start_time_recovery).count() << " milli seconds."<<std::endl;

    std::cin.get();

    std::vector<ompl::base::State*> bstates;

    policyGenerator_->getCurrentBeliefStates(bstates);

    siF_->copyState(recoveredState, bstates[0]);

    Visualizer::setMode(Visualizer::VZRDrawingMode::PRMViewMode);

    writeTimeSeriesDataToFile("MultiModalWeightsHistory.csv", "multiModalWeights");


}
