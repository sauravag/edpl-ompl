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
#include <boost/foreach.hpp>
#include <boost/thread.hpp>
#include <tinyxml.h>
#include "Visualization/Visualizer.h"
#include "Utils/FIRMUtils.h"
#include "Planner/FIRM.h"

#define foreach BOOST_FOREACH
#define foreach_reverse BOOST_REVERSE_FOREACH
#define SHOW_MONTE_CARLO False

namespace ompl
{
    namespace magic
    {

        /** \brief The number of steps to take for a random bounce
            motion generated as part of the expansion step of PRM. */
        static const unsigned int MAX_RANDOM_BOUNCE_STEPS   = 5;

        /** \brief The number of nearest neighbors to consider by
            default in the construction of the PRM roadmap */
        static const int DEFAULT_NEAREST_NEIGHBORS = 5;

        /** \brief The time in seconds for a single roadmap building operation */
        static const double ROADMAP_BUILD_TIME = 60;  // 60 is a good number    

        /** \brief For a node that is not observable, use a fixed covariance */
        static const double NON_OBSERVABLE_NODE_COVARIANCE = 0.1; // 0.1 is a good number

        /** \brief Discounting factor for the Dynamic Programming solution, helps converge faster if set < 1.0 */
        static const float DEFAULT_DP_DISCOUNT_FACTOR = 1.0;

        /** \brief Maximum allowed number of iterations to solve DP */
        static const int DEFAULT_DP_MAX_ITERATIONS = 20000; // 20000 is a good number

        /** \brief Weighting factor for filtering cost */
        static const double DEFAULT_INFORMATION_COST_WEIGHT = 1.0;

        /** \brief Weighting factor for distance based cost */
        static const double DEFAULT_DISTANCE_TO_GOAL_COST_WEIGHT = 0.01; 

        /** \brief Weighting factor for edge execution time cost */
        static const double TIME_TO_STOP_COST_WEIGHT = 0.001;

        /** \brief The cost to go from goal. */
        static const double DEFAULT_GOAL_COST_TO_GO = 0.0;

        /** \brief The initial cost to go from a non-goal node*/
        static const double DEFAULT_INIT_COST_TO_GO = 2.0; // 2 is a good number

        /** \brief The cost to traverse an obstacle*/
        static const double DEFAULT_OBSTACLE_COST_TO_GO = 200; // 200 is a good number

        /** \brief The minimum difference between cost-to-go from start to goal between two successive DP iterations for DP to coverge*/
        static const double DEFAULT_DP_CONVERGENCE_THRESHOLD = 1e-3; // 1e-3 is a good number

        /** \brief Default neighborhood radius */
        static const double DEFAULT_NEAREST_NEIGHBOUR_RADIUS = 5.0; // 5.0 meters is good

        static const double KIDNAPPING_INNOVATION_CHANGE_THRESHOLD = 5.0; // 50%

        static const unsigned int MAX_MM_POLICY_LENGTH   = 1000;

        static const float MIN_ROBOT_CLEARANCE = 0.10;

        static const unsigned int MIN_STEPS_AFTER_CLEARANCE_VIOLATION_REPLANNING = 10;

        static const int DEFAULT_STEPS_TO_ROLLOUT = 10;

        static const double EDGE_COST_BIAS = 0.01; // In controller.h all edge costs are added up from 0.01 as the starting cost, this helps DP converge
    }
}

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
    specs_.recognizedGoal = ompl::base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;

    minFIRMNodes_ = 25;
    maxFIRMNodes_ = 100;

    currentTimeStep_ = 0;

    numberofNodesReached_ = 0;

    executionCost_ = 0;

    costToGoHistory_.push_back(std::make_pair(currentTimeStep_,executionCost_));

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

    informationCostWeight_ = ompl::magic::DEFAULT_INFORMATION_COST_WEIGHT;

    distanceCostWeight_ = ompl::magic::DEFAULT_DISTANCE_TO_GOAL_COST_WEIGHT;

    goalCostToGo_ = ompl::magic::DEFAULT_GOAL_COST_TO_GO;

    obstacleCostToGo_ = ompl::magic::DEFAULT_OBSTACLE_COST_TO_GO;

    initalCostToGo_ = ompl::magic::DEFAULT_INIT_COST_TO_GO;

    maxDPIterations_ = ompl::magic::DEFAULT_DP_MAX_ITERATIONS;
    
    convergenceThresholdDP_ = ompl::magic::DEFAULT_DP_CONVERGENCE_THRESHOLD;

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
        //connectionStrategy_ = ompl::geometric::KStarStrategy<Vertex>(boost::bind(&FIRM::milestoneCount, this), nn_, si_->getStateDimension());
        connectionStrategy_ = FStrategy<Vertex>(NNRadius_, nn_);
        //connectionStrategy_ = ompl::geometric::KBoundedStrategy<Vertex>(numNearestNeighbors_, NNRadius_, nn_);
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
        si_->freeState(stateProperty_[v]);
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
                stateProperty_[m] = si_->cloneState(workStates[i]);
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
                
                solveDynamicProgram(goal);
                
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

    OMPL_INFORM("FIRM: DP could not converge.");
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

FIRM::Vertex FIRM::addStateToGraph(ompl::base::State *state, bool addReverseEdge, bool shouldCreateNodeController)
{

    boost::mutex::scoped_lock _(graphMutex_);

    // First construct a node stabilizer controller
    NodeControllerType nodeController;

    generateNodeController(state, nodeController); // Generating the node controller at sampled state, this will set stationary covariance at node

    // Now add belief state to graph as FIRM node
    Vertex m;

    m = boost::add_vertex(g_);

    if(addReverseEdge)
        addStateToVisualization(state);

    stateProperty_[m] = state;

    nodeControllers_[m] = nodeController;

    totalConnectionAttemptsProperty_[m] = 1;
    successfulConnectionAttemptsProperty_[m] = 0;

    // Initialize to its own (dis)connected component.
    disjointSets_.make_set(m);

    nn_->add(m);

    // Which milestones will we attempt to connect to?
    std::vector<Vertex> neighbors = connectionStrategy_(m);

    OMPL_INFORM("Adding State, Number of Nearest Neighbors = %u", neighbors.size());

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

                addEdgeToGraph(m, n, forwardEdgeAdded);

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
        Edge edge = feedback_[currentVertex]; // get the edge

        Vertex target = boost::target(edge, g_); // get the target of this edge

        if(target > boost::num_vertices(g_))
            OMPL_ERROR("Error in constructing feedback path. Tried to access vertex ID not in graph.");

        p->append(stateProperty_[currentVertex],edgeControllers_[edge]); // push the state and controller to take

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

void FIRM::addEdgeToGraph(const FIRM::Vertex a, const FIRM::Vertex b, bool &edgeAdded)
{

    EdgeControllerType edgeController;

    const FIRMWeight weight = generateEdgeControllerWithCost(a, b, edgeController);

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

FIRMWeight FIRM::generateEdgeControllerWithCost(const FIRM::Vertex a, const FIRM::Vertex b, EdgeControllerType &edgeController)
{
    ompl::base::State* startNodeState = siF_->cloneState(stateProperty_[a]);
    ompl::base::State* targetNodeState = siF_->cloneState(stateProperty_[b]);

     // Generate the edge controller for given start and end state
    generateEdgeController(startNodeState,targetNodeState,edgeController);

    double successCount = 0;

    // initialize costs to 0
    ompl::base::Cost edgeCost(0);
    ompl::base::Cost nodeStabilizationCost(0);

    // if want/do not want to show monte carlo sim
    siF_->showRobotVisualization(SHOW_MONTE_CARLO);

    for(unsigned int i=0; i< numMCParticles_;i++)
    {

        siF_->setTrueState(startNodeState);

        siF_->setBelief(startNodeState);

        ompl::base::State* endBelief = siF_->allocState(); // allocate the end state of the controller

        ompl::base::Cost filteringCost(0);

        int stepsExecuted = 0;

        int stepsToStop = 0;

        if(edgeController.Execute(startNodeState, endBelief, filteringCost, stepsExecuted, stepsToStop))
        {
            successCount++;

            // compute the edge cost by the weighted sum of filtering cost and time to stop (we use number of time steps, time would be steps*dt)
            //edgeCost.v = edgeCost.v + ompl::magic::INFORMATION_COST_WEIGHT*filteringCost.v + ompl::magic::TIME_TO_STOP_COST_WEIGHT*stepsToStop;
            edgeCost = ompl::base::Cost(edgeCost.value() + informationCostWeight_*filteringCost.value() + ompl::magic::TIME_TO_STOP_COST_WEIGHT*stepsToStop);
        }
    }

    siF_->showRobotVisualization(true);

    //edgeCost.v = edgeCost.v / successCount ;
    edgeCost = ompl::base::Cost(edgeCost.value() / successCount);

    double transitionProbability = successCount / numMCParticles_ ;

    FIRMWeight weight(edgeCost.value(), transitionProbability);

    return weight;
}


void FIRM::generateEdgeController(const ompl::base::State *start, const ompl::base::State* target, FIRM::EdgeControllerType &edgeController)
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

    // create the edge controller
    EdgeControllerType ctrlr(target, intermediates, openLoopControls, siF_);

    // assign the edge controller
    edgeController =  ctrlr;
}

void FIRM::generateNodeController(ompl::base::State *state, FIRM::NodeControllerType &nodeController)
{
    // Create a copy of the node state
    ompl::base::State *node = si_->allocState();
    siF_->copyState(node, state);

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

    // set the covariance
    node->as<FIRM::StateType>()->setCovariance(stationaryCovariance);
    state->as<FIRM::StateType>()->setCovariance(stationaryCovariance);

    // create a node controller with node as the state and zero control as nominal control
    std::vector<ompl::control::Control*> zeroControl; zeroControl.push_back(siF_->getMotionModel()->getZeroControl());

    std::vector<ompl::base::State*> nodeState; nodeState.push_back(node);

    NodeControllerType ctrlr(node, nodeState, zeroControl, siF_);

    // assign the node controller
    nodeController = ctrlr;

}

template<typename Map>
arma::colvec MapToColvec(const Map& _m) {
  arma::colvec mapData(_m.size());
  int ix = 0;
  for(typename Map::const_iterator i= _m.begin(), e= _m.end(); i!=e; ++i, ++ix)
    mapData[ix] = i->second;
  return mapData;
}

void FIRM::solveDynamicProgram(const FIRM::Vertex goalVertex)
{
    OMPL_INFORM("FIRM: Solving DP");

    auto start_time = std::chrono::high_resolution_clock::now();

    Visualizer::clearMostLikelyPath();

    using namespace arma;

    float discountFactor = discountFactorDP_;

    std::map<Vertex, double> newCostToGo;

    costToGo_ = std::map<Vertex, double>();

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
            costToGo_[v] = initalCostToGo_;
            newCostToGo[v] = initalCostToGo_;
        }
    }

    feedback_.clear();

    bool convergenceCondition = false;

    int nIter=0;
    while(!convergenceCondition && nIter < maxDPIterations_)
    {
        nIter++;

        foreach(Vertex v, boost::vertices(g_))
        {

            //value for goal node stays the same or if has no out edges then ignore it
            if( v == goalVertex || boost::out_degree(v,g_) == 0 )
            {
                continue;
            }

            // Update the costToGo of vertex
            std::pair<Edge,double> candidate = getUpdatedNodeCostToGo(v, goalVertex);

            feedback_[v] = candidate.first;

            newCostToGo[v] = candidate.second * discountFactor;

            //assert(costToGo_.size()==newCostToGo.size());

        }

        convergenceCondition = (norm(MapToColvec(costToGo_)-MapToColvec(newCostToGo), "inf") <= convergenceThresholdDP_);

        costToGo_.swap(newCostToGo);   // Equivalent to costToGo_ = newCostToGo

    }

    auto end_time = std::chrono::high_resolution_clock::now();

    double timeDP = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    OMPL_INFORM("FIRM: DP Solve Time: %2.3f seconds<----", timeDP/1000.0);

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

        double singleCostToGo =  (transitionProbability*nextNodeCostToGo + (1-transitionProbability)*obstacleCostToGo_ + edgeWeight.getCost()) + distanceCostWeight_*distToGoalFromTarget;

        candidateCostToGo[e] =  singleCostToGo ;

    }

    DoubleValueComp dvc;
    std::pair<Edge,double> bestCandidate =   *min_element(candidateCostToGo.begin(), candidateCostToGo.end(), dvc );

    return bestCandidate;

}

double FIRM::evaluateSuccessProbability(const Edge currentEdge, const FIRM::Vertex start, const FIRM::Vertex goal)
{
    const FIRMWeight currentEdgeWeight = boost::get(boost::edge_weight, g_, currentEdge);

    double successProb = currentEdgeWeight.getSuccessProbability();

    Vertex v = boost::target(currentEdge, g_);

    while(v != goal)
    {
        Edge edge = feedback_[v];

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
        Edge edge = feedback_[currentVertex]; // get the edge

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

    Visualizer::doSaveVideo(doSaveVideo_);

    nodeReachedHistory_.push_back(std::make_pair(currentTimeStep_, numberofNodesReached_) );

    siF_->doVelocityLogging(true);

    while(!goalState->as<FIRM::StateType>()->isReached(cstartState))
    {

        if(currentVertex==goal)
            break;

        Edge e = feedback_[currentVertex];

        assert(currentVertex < boost::num_vertices(g_));

        OMPL_INFORM("FIRM: Moving from Vertex %u to %u", currentVertex, boost::target(e, g_));

        // Check if feedback policy is valid 
        while(!isFeedbackPolicyValid(currentVertex, goal))
        {

            OMPL_INFORM("FIRM: Invalid path detected from Vertex %u to %u", currentVertex, boost::target(e, g_));

            updateEdgeCollisionCost(currentVertex, goal);

            // resolve DP
            solveDynamicProgram(goal);

            e = feedback_[currentVertex];

            OMPL_INFORM("FIRM: Updated path, moving from Vertex %u to %u", currentVertex, boost::target(e, g_));
        }

        double succProb = evaluateSuccessProbability(e, currentVertex, goal);

        OMPL_INFORM("FIRM: Moving from Vertex %u to %u with TP = %f", currentVertex, boost::target(e, g_), succProb);

        successProbabilityHistory_.push_back(std::make_pair(currentTimeStep_, succProb) );

        controller = edgeControllers_[e];

        ompl::base::Cost cost;

        int stepsExecuted = 0;

        int stepsToStop = 0;

        controller.setSpaceInformation(policyExecutionSI_);

        bool controllerStatus = controller.Execute(cstartState, cendState, cost, stepsExecuted, stepsToStop, false);

        executionCost_ += cost.value() - ompl::magic::EDGE_COST_BIAS;

        costToGoHistory_.push_back(std::make_pair(currentTimeStep_,executionCost_));

        // get a copy of the true state
        ompl::base::State *tempTrueStateCopy = si_->allocState();

        siF_->getTrueState(tempTrueStateCopy);

        if(!si_->isValid(tempTrueStateCopy))
        {
           OMPL_INFORM("Robot Collided :(");

           return;
        }

        currentTimeStep_ += stepsExecuted;

        if(controllerStatus)
        {
            numberofNodesReached_++;

            nodeReachedHistory_.push_back(std::make_pair(currentTimeStep_, numberofNodesReached_) );

            currentVertex = boost::target(e, g_);
        }
        else
        {
            Visualizer::doSaveVideo(false);
            siF_->doVelocityLogging(false);

            //
            OMPL_INFORM("Controller stopped due to deviation, need to add new state at: ");
            siF_->printState(cendState);
            //

            currentVertex = addStateToGraph(cendState);

            // Set true state back to its correct value after Monte Carlo (happens during adding state to Graph)
            siF_->setTrueState(tempTrueStateCopy);

            solveDynamicProgram(goal);

            Visualizer::doSaveVideo(doSaveVideo_);
            siF_->doVelocityLogging(true);

        }

        si_->freeState(tempTrueStateCopy);

        si_->copyState(cstartState, cendState);


    }

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

        Edge e = feedback_[currentVertex];

        double succProb = evaluateSuccessProbability(e, currentVertex, goal);

        OMPL_INFORM("FIRM: Moving from Vertex %u to %u with TP = %f", currentVertex, boost::target(e, g_), succProb);

        successProbabilityHistory_.push_back(std::make_pair(currentTimeStep_, succProb) );

        controller = edgeControllers_[e];

        ompl::base::Cost cost(0);

        int stepsExecuted = 0;

        int stepsToStop = 0;

        controller.setSpaceInformation(policyExecutionSI_);

        bool controllerStatus = controller.Execute(cstartState, cendState, cost, stepsExecuted, stepsToStop, false);

        executionCost_ += cost.value() - ompl::magic::EDGE_COST_BIAS;

        costToGoHistory_.push_back(std::make_pair(currentTimeStep_,executionCost_));

         // get a copy of the true state
        ompl::base::State *tempTrueStateCopy = si_->allocState();

        siF_->getTrueState(tempTrueStateCopy);

        if(!si_->isValid(tempTrueStateCopy))
        {
            OMPL_INFORM("Robot Collided :(");
            return;
        }

        currentTimeStep_ += stepsExecuted;

        if(controllerStatus)
        {
            currentVertex = boost::target(e, g_);
        }
        else
        {

            Visualizer::doSaveVideo(false);

            currentVertex = addStateToGraph(cendState);

            // Set true state back to its correct value after Monte Carlo (happens during adding state to Graph)
            siF_->setTrueState(tempTrueStateCopy);

            solveDynamicProgram(goal);

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

            // Set true state back to its correct value after Monte Carlo (happens during adding state to Graph)
            siF_->setTrueState(tempTrueStateCopy);

            siF_->freeState(tempTrueStateCopy);

            solveDynamicProgram(goal);

            sendMostLikelyPathToViz(currentVertex, goal);

            kidnapped_flag = false;
        }

        si_->copyState(cstartState, cendState);

    }

    Visualizer::doSaveVideo(false);

}

// Experimental
void FIRM::executeFeedbackWithRollout(void)
{

    // Open a file for writing rollout computation time
    std::ofstream outfile;
    if(doSaveLogs_)
    {
        outfile.open(logFilePath_+"RolloutComputationTime.csv");
        outfile<<"RolloutNum, RadiusNN, NumNN, MCParticles, avgTimePerNeighbor, totalTimeSecs" <<std::endl;
    }

    Visualizer::setMode(Visualizer::VZRDrawingMode::RolloutMode);
    Visualizer::clearRobotPath();

    const Vertex start = startM_[0];
    const Vertex goal  = goalM_[0] ;

    sendMostLikelyPathToViz(start, goal) ;

    ompl::base::State *goalState = si_->cloneState(stateProperty_[goal]);

    //===== SET A Custom Init Covariance=====================
    // using namespace arma;

    // mat tempCC(3,3);

    // tempCC<< 0.1 << 0.0 << 0.0 << endr
    //         << 0.0 << 0.1 << 0.0 << endr
    //         << 0.0 << 0.0 << 0.0000001<<endr;

    // stateProperty_[start]->as<StateType>()->setCovariance(tempCC);
    // Visualizer::updateCurrentBelief(stateProperty_[start]);
    //================================

    siF_->setTrueState(stateProperty_[start]);
    siF_->setBelief(stateProperty_[start]);

    Vertex currentVertex =  start;

    ompl::base::State *cstartState = si_->allocState();
    si_->copyState(cstartState, stateProperty_[start]);

    ompl::base::State *cendState = si_->allocState();

    OMPL_INFORM("FIRM: Running policy execution");

    Edge e = feedback_[currentVertex];

    Vertex tempVertex = currentVertex;

    OMPL_INFORM("Goal State is: \n");

    si_->printState(goalState);

    double averageTimeForRolloutComputation = 0;

    int numberOfRollouts = 0;

    Visualizer::doSaveVideo(doSaveVideo_);

    nodeReachedHistory_.push_back(std::make_pair(currentTimeStep_, numberofNodesReached_) );

    // While the robot state hasn't reached the goal state, keep running
    while(!goalState->as<FIRM::StateType>()->isReached(cstartState, true))
    {

        double succProb = evaluateSuccessProbability(e, tempVertex, goal);

        OMPL_INFORM("FIRM Rollout: Moving from Vertex %u to %u with TP = %f", tempVertex, boost::target(e, g_), succProb);

        successProbabilityHistory_.push_back(std::make_pair(currentTimeStep_, succProb ) );

        EdgeControllerType controller = edgeControllers_[e];

        assert(controller.getGoal());

        ompl::base::Cost cost;

        siF_->doVelocityLogging(true);

        /**
            Instead of executing the entire controller, we need to execute N steps, then calculate the cost to go through the neighboring nodes.
            Whichever gives the lowest cost to go, is our new path. Do this at every N steps.
        */
        int stepsExecuted = 0;

        controller.setSpaceInformation(policyExecutionSI_);

        controller.executeUpto(rolloutSteps_, cstartState, cendState, cost, stepsExecuted, false);

        executionCost_ += cost.value() - ompl::magic::EDGE_COST_BIAS;

        costToGoHistory_.push_back(std::make_pair(currentTimeStep_, executionCost_));

        ompl::base::State *tState = si_->allocState();

        siF_->getTrueState(tState);

        if(!si_->isValid(tState))
        {
            OMPL_INFORM("Robot Collided :(");
            return;
        }

        currentTimeStep_ += stepsExecuted;

        // If the robot has already reached a FIRM node then take feedback edge
        // else do rollout
        if(stateProperty_[boost::target(e,g_)]->as<FIRM::StateType>()->isReached(cendState, true))
        {
            OMPL_INFORM("FIRM Rollout: Reached FIRM Node: %u", boost::target(e,g_));

            numberofNodesReached_++;

            nodeReachedHistory_.push_back(std::make_pair(currentTimeStep_, numberofNodesReached_) );

            tempVertex = boost::target(e,g_);

            e = feedback_[tempVertex];

        }

        else
        {

            siF_->doVelocityLogging(false);

            Visualizer::doSaveVideo(false);

            // start profiling time to compute rollout
            auto start_time = std::chrono::high_resolution_clock::now();

            tempVertex = addStateToGraph(cendState, false);

            siF_->setTrueState(tState);

            e = generateRolloutPolicy(tempVertex, goal);

            // end profiling time to compute rollout
            auto end_time = std::chrono::high_resolution_clock::now();

            Visualizer::doSaveVideo(doSaveVideo_);

            numberOfRollouts++;

            int numNN = numNearestNeighbors_;

            double timeToDoRollout = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

            averageTimeForRolloutComputation +=  timeToDoRollout / numNN;

            if(doSaveLogs_)
            {
                outfile<<numberOfRollouts<<","<<NNRadius_<<","
                   <<numNN<<","<<numMCParticles_<<","<<timeToDoRollout / (1000*numNN)<<","<<timeToDoRollout/1000<<std::endl;
            }

            //std::cout << "Time to execute rollout : "<<timeToDoRollout << " milli seconds."<<std::endl;

            showRolloutConnections(tempVertex);

            // clear the rollout candidate connection drawings and show the selected edge
            Visualizer::clearRolloutConnections();

            Visualizer::setChosenRolloutConnection(stateProperty_[tempVertex], stateProperty_[boost::target(e,g_)]);

            boost::remove_vertex(tempVertex, g_);

        }

        si_->freeState(tState);

        si_->copyState(cstartState, cendState);

    }

    nodeReachedHistory_.push_back(std::make_pair(currentTimeStep_, numberofNodesReached_) );

    OMPL_INFORM("FIRM: Number of nodes reached with Rollout: %u", numberofNodesReached_);

    averageTimeForRolloutComputation = averageTimeForRolloutComputation / (1000*numberOfRollouts);

    std::cout<<"Nearest Neighbor Radius: "<<NNRadius_<<", Monte Carlo Particles: "<<numMCParticles_<<", Avg Time/neighbor (seconds): "<<averageTimeForRolloutComputation<<std::endl;    

    if(doSaveLogs_)
    {

        outfile.close();

        writeTimeSeriesDataToFile("RolloutFIRMCostHistory.csv", "costToGo");

        writeTimeSeriesDataToFile("RolloutFIRMSuccessProbabilityHistory.csv", "successProbability");

        writeTimeSeriesDataToFile("RolloutFIRMNodesReachedHistory.csv","nodesReached");

        std::vector<std::pair<double, double> > velLog;

        siF_->getVelocityLog(velLog);

        for(int i=0; i < velLog.size(); i++)
        {
            velocityHistory_.push_back(std::make_pair(i, sqrt( pow(velLog[i].first,2) + pow(velLog[i].second,2) ))); // omni
            //velocityHistory_.push_back(std::make_pair(i, velLog[i].first)); // unicycle
        }

        writeTimeSeriesDataToFile("RolloutFIRMVelocityHistory.csv", "velocity");
    }

    Visualizer::doSaveVideo(false);


}

void FIRM::showRolloutConnections(const FIRM::Vertex v)
{
    Visualizer::clearRolloutConnections();

    const std::vector<Vertex>& neighbors = connectionStrategy_(v);

    foreach (Vertex n, neighbors)
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

        Edge edge = feedback_[v];

        targetVertex = boost::target(edge, g_);

        OMPL_INFORM("Most Likely Path from %u to %u", v, targetVertex);
        
        Visualizer::addMostLikelyPathEdge(stateProperty_[v], stateProperty_[targetVertex]);

        v = targetVertex;
    }
}

bool FIRM::isFeedbackPolicyValid(FIRM::Vertex currentVertex, FIRM::Vertex goalVertex)
{
    // cycle through feedback, if feedback edge is invalid, return false
    while(currentVertex != goalVertex)
    {
        Edge edge = feedback_[currentVertex]; // get the edge

        Vertex target = boost::target(edge, g_); // get the target of this edge

        // if edge is invalid, increase its cost
        if(!si_->checkMotion(stateProperty_[currentVertex], stateProperty_[target]))
        {
            return false;
        }
           
        currentVertex =  target;

    }

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
        Edge nextFIRMEdge = feedback_[targetNode];

        // The node to which next firm edge goes
        Vertex targetOfNextFIRMEdge = boost::target(nextFIRMEdge, g_);

        // Check if feedback from target to goal is valid or not
        if(!isFeedbackPolicyValid(targetNode, goal))
        {

            OMPL_INFORM("Rollout: Invalid path detected from Vertex %u to %u", targetNode, targetOfNextFIRMEdge);

            updateEdgeCollisionCost(targetNode, goal);

            // resolve DP
            solveDynamicProgram(goal);

            targetOfNextFIRMEdge = boost::target(feedback_[targetNode], g_);  

            OMPL_INFORM("Rollout: Updated path, next firm edge moving from Vertex %u to %u", targetNode, targetOfNextFIRMEdge);

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

        // the cost of taking the edge
        double edgeCostToGo = transitionProbability*nextNodeCostToGo + (1-transitionProbability)*obstacleCostToGo_ + edgeWeight.getCost();


        // for debug
        std::cout << "COST[" << currentVertex << "->" << targetNode << "->" << targetOfNextFIRMEdge << "->G] " << edgeCostToGo << " = " << transitionProbability << "*" << nextNodeCostToGo << " + " << "(1-" << transitionProbability << ")*" << obstacleCostToGo_ << " + " << edgeWeight.getCost() << std::endl;


        if(edgeCostToGo < minCost)
        {
            minCost  = edgeCostToGo;
            edgeToTake = e;

            // for debug
            minCostVertCurrent = currentVertex;
            minCostVertNext = targetNode;
            minCostVertNextNext = targetOfNextFIRMEdge;
        }
    }

    // for debug
    std::cout << "minC[" << minCostVertCurrent << "->" << minCostVertNext << "->" << minCostVertNextNext << "->G] " << minCost << std::endl;

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
            ompl::base::State* targetNodeState = siF_->cloneState(stateProperty_[b]);

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

        }

    }
}

void FIRM::writeTimeSeriesDataToFile(std::string fname, std::string dataName)
{

    std::ofstream outfile;

    outfile.open(logFilePath_ + fname);

    if(dataName.compare("costToGo")==0)
    {
        for(int i=0; i < costToGoHistory_.size(); i++)
        {
            outfile<<costToGoHistory_[i].first<<","<<costToGoHistory_[i].second<<std::endl;
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
    double discountFactorDP = 0.0, informationCostWeight = 0.0, distanceCostWeight = 0.0, goalCostToGo = 0.0, obstacleCostToGo = 0.0, initalCostToGo = 0.0, convergenceThresholdDP = 0.0;
    int maxDPIterations = 0;

    child = node->FirstChild("DPDiscountFactor");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("discountfac", &discountFactorDP);
    discountFactorDP_ = discountFactorDP;

    child = node->FirstChild("InfCostWeight");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("infcostw", &informationCostWeight);
    informationCostWeight_ = informationCostWeight;

    child = node->FirstChild("DistCostWeight");
    assert( child );
    itemElement = child->ToElement();
    assert( itemElement );
    itemElement->QueryDoubleAttribute("distcostw", &distanceCostWeight);
    distanceCostWeight_ = distanceCostWeight;

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
    itemElement->QueryDoubleAttribute("initctg", &initalCostToGo);
    initalCostToGo_ = initalCostToGo;

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
