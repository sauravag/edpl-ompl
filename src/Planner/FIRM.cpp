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

#include "../../include/Planner/FIRM.h"
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
#include "../../include/Visualization/Visualizer.h"
#include "../../include/Utils/FIRMUtils.h"

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
        static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 8;

        /** \brief The time in seconds for a single roadmap building operation */
        static const double ROADMAP_BUILD_TIME = 60;

        /** \brief Number of monte carlo simulations to run for one edge when adding an edge to the roadmap */
        static const double NUM_MONTE_CARLO_PARTICLES = 2; // minimum 10 for FIRM, 4 for rollout

        /** \brief For a node that is not observable, use a fixed covariance */
        static const double NON_OBSERVABLE_NODE_COVARIANCE = 0.1;

        /** \brief Discounting factor for the Dynamic Programming solution, helps converge faster if set < 1.0 */
        static const float DYNAMIC_PROGRAMMING_DISCOUNT_FACTOR = 1.0;

        /** \brief Maximum allowed number of iterations to solve DP */
        static const int DP_MAX_ITERATIONS = 20000;

        /** \brief Weighting factor for filtering cost */
        static const double INFORMATION_COST_WEIGHT = 0.9999 ;

        /** \brief Weighting factor for edge execution time cost */
        static const double TIME_TO_STOP_COST_WEIGHT = 0.0001;

        /** \brief The cost to go from goal. */
        static const double GOAL_COST_TO_GO = 0.0;

        /** \brief The initial cost to go from a non-goal node*/
        static const double INIT_COST_TO_GO = 2.0;

        /** \brief The cost to traverse an obstacle*/
        static const double OBSTACLE_COST_TO_GO = 200;

        /** \brief The minimum difference between cost-to-go from start to goal between two successive DP iterations for DP to coverge*/
        static const double DP_CONVERGENCE_THRESHOLD = 1e-3;

        /** \brief Default neighborhood radius */
        static const double DEFAULT_NEAREST_NEIGHBOUR_RADIUS = 1.5; // meters

        static const double KIDNAPPING_INNOVATION_CHANGE_THRESHOLD = 5.0; // 50%

        static const unsigned int MAX_MM_POLICY_LENGTH   = 1000;

        static const float MIN_ROBOT_CLEARANCE = 0.05; // 0.1 for create

        static const unsigned int MIN_STEPS_AFTER_CLEARANCE_VIOLATION_REPLANNING = 10;

        static const int STEPS_TO_ROLLOUT = 30;
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

    currentTimeStep_ = 0;

    numberofNodesReached_ = 0;

    executionCost_ = 0;

    costToGoHistory_.push_back(std::make_pair(currentTimeStep_,executionCost_));

    policyGenerator_ = new NBM3P(si);

    loadedRoadmapFromFile_ = false;

    policyExecutionSI_ = siF_; // by default policies are executed in the same space that the roadmap is generated

    makeDataLogPath();

}

FIRM::~FIRM(void)
{
    freeMemory();
    delete policyGenerator_;
}

void FIRM::setup(void)
{
    Planner::setup();
    if (!nn_)
    {
        nn_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(si_->getStateSpace()));
        nn_->setDistanceFunction(boost::bind(&FIRM::distanceFunction, this, _1, _2));
    }
    if (!connectionStrategy_)
    {
        //connectionStrategy_ = ompl::geometric::KStarStrategy<Vertex>(boost::bind(&FIRM::milestoneCount, this), nn_, si_->getStateDimension());
        connectionStrategy_ = FStrategy<Vertex>(ompl::magic::DEFAULT_NEAREST_NEIGHBOUR_RADIUS, nn_);
        //connectionStrategy_ = ompl::geometric::KBoundedStrategy<Vertex>(ompl::magic::DEFAULT_NEAREST_NEIGHBORS, ompl::magic::DEFAULT_NEAREST_NEIGHBOUR_RADIUS, nn_);
    }

    int np = ompl::magic::NUM_MONTE_CARLO_PARTICLES;
    numParticles_ = np;

}

void FIRM::setMaxNearestNeighbors(unsigned int k)
{
    if (!nn_)
    {
        nn_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(si_->getStateSpace()));
        nn_->setDistanceFunction(boost::bind(&FIRM::distanceFunction, this, _1, _2));
    }

    if (!userSetConnectionStrategy_)
        connectionStrategy_.clear();
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
                    }
                    catch(int e)
                    {
                        stateStable = false;
                    }

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

    if(boost::num_vertices(g_) < minFIRMNodes_) return false;

    foreach (Vertex start, starts)
    {
        foreach (Vertex goal, goals)
        {
            graphMutex_.lock();
            bool same_component = sameComponent(start, goal);
            graphMutex_.unlock();

            if (same_component && g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
            {
                boost::mutex::scoped_lock _(graphMutex_);
                solveDynamicProgram(goal);
                solution = constructFeedbackPath(start, goal);
                sendFeedbackEdgesToViz();
                return true; // return true if solution is found
            }
        }
    }

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

    OMPL_INFORM("%s: Adding start state to roadmap.", getName().c_str());

    // Add the valid start states as milestones
    while (const ompl::base::State *st = pis_.nextStart())
        startM_.push_back(addStateToGraph(si_->cloneState(st)));

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
    boost::thread slnThread(boost::bind(&FIRM::checkForSolution, this, ptc, boost::ref(sol)));

    ompl::base::PlannerTerminationCondition ptcOrSolutionFound =
        ompl::base::plannerOrTerminationCondition(ptc, ompl::base::PlannerTerminationCondition(boost::bind(&FIRM::addedNewSolution, this)));

    // If no roadmap was loaded, then construct one
    if(!loadedRoadmapFromFile_)
    {
        constructRoadmap(ptcOrSolutionFound);
    }

    // If roadmap wasn't loaded from file, then save the newly constructed roadmap
    if(!loadedRoadmapFromFile_)
    {
        this->savePlannerData();
    }

    slnThread.join();

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
            growRoadmap(ompl::base::plannerOrTerminationCondition(ptc, ompl::base::timedPlannerTerminationCondition(2*ompl::magic::ROADMAP_BUILD_TIME)), xstates[0]);
        }
        else
        {
            expandRoadmap(ompl::base::plannerOrTerminationCondition(ptc, ompl::base::timedPlannerTerminationCondition(ompl::magic::ROADMAP_BUILD_TIME)), xstates);
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

    addStateToVisualization(state);

    stateProperty_[m] = state;

    nodeControllers_[m] = nodeController;

    totalConnectionAttemptsProperty_[m] = 1;
    successfulConnectionAttemptsProperty_[m] = 0;

    // Initialize to its own (dis)connected component.
    disjointSets_.make_set(m);

    nn_->add(m);

    // Which milestones will we attempt to connect to?
    const std::vector<Vertex>& neighbors = connectionStrategy_(m);

    std::vector<boost::thread> monteCarloThreads;

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

ompl::base::PathPtr FIRM::constructFeedbackPath(const Vertex &start, const Vertex &goal)
{
    sendFeedbackEdgesToViz();

    FeedbackPath *p = new FeedbackPath(siF_);

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
        }

    }

    return ompl::base::PathPtr(p);
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

    for(unsigned int i=0; i< numParticles_;i++)
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
            edgeCost = ompl::base::Cost(edgeCost.value() + ompl::magic::INFORMATION_COST_WEIGHT*filteringCost.value() + ompl::magic::TIME_TO_STOP_COST_WEIGHT*stepsToStop);
        }
    }

    siF_->showRobotVisualization(true);

    //edgeCost.v = edgeCost.v / successCount ;
    edgeCost = ompl::base::Cost(edgeCost.value() / successCount);

    double transitionProbability = successCount / numParticles_ ;

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

   if(siF_->getObservationModel()->isStateObservable(node))
   {
        // Contruct a linear kalman filter
        LinearizedKF linearizedKF(siF_);

        //Construct a linear system
        LinearSystem linearSystem(siF_, node, siF_->getMotionModel()->getZeroControl(),siF_->getObservationModel()->getObservation(state, false), siF_->getMotionModel(), siF_->getObservationModel());

        // Compute the stationary cov at node state using LKF
        arma::mat stationaryCovariance = linearizedKF.computeStationaryCovariance(linearSystem);

        // set the covariance
        node->as<SE2BeliefSpace::StateType>()->setCovariance(stationaryCovariance);
        state->as<SE2BeliefSpace::StateType>()->setCovariance(stationaryCovariance);

        // create a node controller
        std::vector<ompl::control::Control*> dummyControl;
        std::vector<ompl::base::State*> dummyStates;
        NodeControllerType ctrlr(node, dummyStates, dummyControl, siF_);

        // assign the node controller
        nodeController = ctrlr;
    }

    else
    {
        // set a high stationary cov at node state
        int stateDim = si_->getStateDimension();
        arma::mat stationaryCovariance = arma::eye(stateDim,stateDim)*ompl::magic::NON_OBSERVABLE_NODE_COVARIANCE;

        // set the covariance
        node->as<SE2BeliefSpace::StateType>()->setCovariance(stationaryCovariance);
        state->as<SE2BeliefSpace::StateType>()->setCovariance(stationaryCovariance);

        // create a node controller
        std::vector<ompl::control::Control*> dummyControl;
        std::vector<ompl::base::State*> dummyStates;
        NodeControllerType ctrlr(node, dummyStates, dummyControl, siF_);

        // assign the node controller
        nodeController = ctrlr;
    }

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

    Visualizer::clearMostLikelyPath();

    using namespace arma;

    float discountFactor = ompl::magic::DYNAMIC_PROGRAMMING_DISCOUNT_FACTOR;

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
            costToGo_[v] = ompl::magic::GOAL_COST_TO_GO;
            newCostToGo[v] = ompl::magic::GOAL_COST_TO_GO;
        }
        else
        {
            costToGo_[v] = ompl::magic::INIT_COST_TO_GO;
            newCostToGo[v] = ompl::magic::INIT_COST_TO_GO;
        }
    }

    feedback_.clear();

    bool convergenceCondition = false;

    int nIter=0;
    while(!convergenceCondition && nIter < ompl::magic::DP_MAX_ITERATIONS)
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
            std::pair<Edge,double> candidate = getUpdatedNodeCostToGo(v);

            feedback_[v] = candidate.first;

            newCostToGo[v] = candidate.second * discountFactor;

            //assert(costToGo_.size()==newCostToGo.size());

        }

        convergenceCondition = (norm(MapToColvec(costToGo_)-MapToColvec(newCostToGo), "inf") <= ompl::magic::DP_CONVERGENCE_THRESHOLD);

        costToGo_.swap(newCostToGo);   // Equivalent to costToGo_ = newCostToGo

    }

    OMPL_INFORM("FIRM: Solved DP");

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


std::pair<typename FIRM::Edge,double> FIRM::getUpdatedNodeCostToGo(const FIRM::Vertex node)
{

    std::map<Edge,double > candidateCostToGo;

    foreach(Edge e, boost::out_edges(node, g_))
    {
        // the target of given edge "e"
        const Vertex targetNode = boost::target(e, g_);

        double nextNodeCostToGo = costToGo_[targetNode];

        const FIRMWeight edgeWeight =  boost::get(boost::edge_weight, g_, e);

        const double transitionProbability  = edgeWeight.getSuccessProbability();

        double singleCostToGo = ( transitionProbability*nextNodeCostToGo + (1-transitionProbability)*ompl::magic::OBSTACLE_COST_TO_GO) + edgeWeight.getCost();

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

    Visualizer::doSaveVideo(true);

    nodeReachedHistory_.push_back(std::make_pair(currentTimeStep_, numberofNodesReached_) );

    siF_->doVelocityLogging(true);

    while(!goalState->as<SE2BeliefSpace::StateType>()->isReached(cstartState))
    {
        //costToGoHistory_.push_back(std::make_pair(currentTimeStep_,costToGo_[currentVertex]));

        if(currentVertex==goal)
            break;

        Edge e = feedback_[currentVertex];

        assert(currentVertex < boost::num_vertices(g_));

        OMPL_INFORM("FIRM: Moving from Vertex %u to %u", currentVertex, boost::target(e, g_));

        double succProb = evaluateSuccessProbability(e, currentVertex, goal);

        OMPL_INFORM("FIRM: Moving from Vertex %u to %u with TP = %f", currentVertex, boost::target(e, g_), succProb);

        successProbabilityHistory_.push_back(std::make_pair(currentTimeStep_, succProb) );

        controller = edgeControllers_[e];

        ompl::base::Cost cost;

        int stepsExecuted = 0;

        int stepsToStop = 0;

        controller.setSpaceInformation(policyExecutionSI_);

        bool controllerStatus = controller.Execute(cstartState, cendState, cost, stepsExecuted, stepsToStop, false);

        executionCost_ += cost.value();

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
            // dont record video of sim while adding current state to graph
            Visualizer::doSaveVideo(false);

            // dont log velocity
            siF_->doVelocityLogging(false);

            // apply stop command to robot
            policyExecutionSI_->applyControl(policyExecutionSI_->getMotionModel()->getZeroControl());

            OMPL_INFORM("Controller stopped due to deviation, need to add new state at: ");
            siF_->printState(cendState);

            currentVertex = addStateToGraph(cendState);

            // Set true state back to its correct value after Monte Carlo (happens during adding state to Graph)
            siF_->setTrueState(tempTrueStateCopy);

            solveDynamicProgram(goal);

            Visualizer::doSaveVideo(true);
            siF_->doVelocityLogging(true);

        }

        si_->freeState(tempTrueStateCopy);

        si_->copyState(cstartState, cendState);


    }

    //Stop robot after policy has been executed
    policyExecutionSI_->applyControl(policyExecutionSI_->getMotionModel()->getZeroControl());

    //costToGoHistory_.push_back(std::make_pair(currentTimeStep_,0));

    writeTimeSeriesDataToFile("StandardFIRMCostHistory.csv", "costToGo");

    writeTimeSeriesDataToFile("StandardFIRMSuccessProbabilityHistory.csv", "successProbability");

    writeTimeSeriesDataToFile("StandardFIRMNodesReachedHistory.csv","nodesReached");

    std::vector<std::pair<double, double> > velLog;

    siF_->getVelocityLog(velLog);

    for(int i=0; i < velLog.size(); i++)
    {
        //velocityHistory_.push_back(std::make_pair(i, velLog[i].first)); //unicycle
        velocityHistory_.push_back(std::make_pair(i, sqrt( pow(velLog[i].first,2) + pow(velLog[i].second,2) ))); // omni
    }

    writeTimeSeriesDataToFile("StandardFIRMVelocityHistory.csv", "velocity");

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

    Visualizer::doSaveVideo(true);

    while(!goalState->as<SE2BeliefSpace::StateType>()->isReached(cstartState)/*currentVertex != goal*/)
    {
        //costToGoHistory_.push_back(std::make_pair(currentTimeStep_,costToGo_[currentVertex]));

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

        executionCost_ += cost.value();

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
            //Stop robot after policy has been executed
            policyExecutionSI_->applyControl(policyExecutionSI_->getMotionModel()->getZeroControl());

            Visualizer::doSaveVideo(false);

            currentVertex = addStateToGraph(cendState);

            // Set true state back to its correct value after Monte Carlo (happens during adding state to Graph)
            siF_->setTrueState(tempTrueStateCopy);

            solveDynamicProgram(goal);

            Visualizer::doSaveVideo(true);

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
            //Stop robot after policy has been executed
            policyExecutionSI_->applyControl(policyExecutionSI_->getMotionModel()->getZeroControl());

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

    //costToGoHistory_.push_back(std::make_pair(currentTimeStep_,0));

    //writeTimeSeriesDataToFile("StandardFIRMCostHistory.csv", "costToGo");

    //writeTimeSeriesDataToFile("StandardFIRMSuccessProbabilityHistory", "successProbability");

    //Stop robot after policy has been executed
    policyExecutionSI_->applyControl(policyExecutionSI_->getMotionModel()->getZeroControl());

    Visualizer::doSaveVideo(false);

}

void FIRM::executeFeedbackWithRollout(void)
{
    Visualizer::setMode(Visualizer::VZRDrawingMode::RolloutMode);
    Visualizer::clearRobotPath();

    const Vertex start = startM_[0];
    const Vertex goal  = goalM_[0] ;

    sendMostLikelyPathToViz(start, goal);

    ompl::base::State *goalState = si_->cloneState(stateProperty_[goal]);

    siF_->setTrueState(stateProperty_[start]);
    siF_->setBelief(stateProperty_[start]);

    Vertex currentVertex =  start;

    ompl::base::State *cstartState = si_->allocState();
    si_->copyState(cstartState, stateProperty_[start]);

    ompl::base::State *cendState = si_->allocState();

    OMPL_INFORM("FIRM: Running policy execution");

    Edge e = feedback_[currentVertex];

    Vertex tempVertex;

    OMPL_INFORM("Goal State is: \n");

    si_->printState(goalState);

    //costToGoHistory_.push_back(std::make_pair(currentTimeStep_, costToGo_[start]));

    double averageTimeForRolloutComputation = 0;

    int numberOfRollouts = 0;

    Visualizer::doSaveVideo(true);

    connectionStrategy_ = FStrategy<Vertex>(1.2*ompl::magic::DEFAULT_NEAREST_NEIGHBOUR_RADIUS, nn_);

    nodeReachedHistory_.push_back(std::make_pair(currentTimeStep_, numberofNodesReached_) );

    // While the robot state hasn't reached the goal state, keep running
    while(!goalState->as<SE2BeliefSpace::StateType>()->isReached(cstartState, true))
    {

        double succProb = evaluateSuccessProbability(e, tempVertex, goal);

        OMPL_INFORM("FIRM: Moving from Vertex %u to %u with TP = %f", tempVertex, boost::target(e, g_), succProb);

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

        controller.executeUpto(ompl::magic::STEPS_TO_ROLLOUT, cstartState, cendState, cost, stepsExecuted, false);

        executionCost_ += cost.value();

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
        if(stateProperty_[boost::target(e,g_)]->as<SE2BeliefSpace::StateType>()->isReached(cendState, true))
        {
            numberofNodesReached_++;

            nodeReachedHistory_.push_back(std::make_pair(currentTimeStep_, numberofNodesReached_) );

            tempVertex = boost::target(e,g_);

            e = feedback_[tempVertex];

            //costToGoHistory_.push_back(std::make_pair(currentTimeStep_, costToGo_[tempVertex]));
        }

        else
        {
            //Stop robot to compute rollout, future versions will not require robot to stop to compute rollout
            policyExecutionSI_->applyControl(policyExecutionSI_->getMotionModel()->getZeroControl());

            siF_->doVelocityLogging(false);

            Visualizer::doSaveVideo(false);

            // start profiling time to compute rollout
            auto start_time = std::chrono::high_resolution_clock::now();

            tempVertex = addStateToGraph(cendState, false);

            siF_->setTrueState(tState);

            e = generateRolloutPolicy(tempVertex);

            // end profiling time to compute rollout
            auto end_time = std::chrono::high_resolution_clock::now();

            Visualizer::doSaveVideo(true);

            numberOfRollouts++;

            int numNN = connectionStrategy_(tempVertex).size();

            averageTimeForRolloutComputation += (std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()) / numNN;

            std::cout << "Time to execute rollout : "<<std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << " milli seconds."<<std::endl;

            showRolloutConnections(tempVertex);

            // clear the rollout candidate connection drawings and show the selected edge
            Visualizer::clearRolloutConnections();

            Visualizer::setChosenRolloutConnection(stateProperty_[tempVertex], stateProperty_[boost::target(e,g_)]);

            boost::remove_vertex(tempVertex, g_);

        }

        si_->freeState(tState);

        si_->copyState(cstartState, cendState);

    }

    //Stop robot after policy has been executed
    policyExecutionSI_->applyControl(policyExecutionSI_->getMotionModel()->getZeroControl());

    nodeReachedHistory_.push_back(std::make_pair(currentTimeStep_, numberofNodesReached_) );

    OMPL_INFORM("FIRM: Number of nodes reached with Rollout: %u", numberofNodesReached_);

    averageTimeForRolloutComputation = averageTimeForRolloutComputation / (1000*numberOfRollouts);

    std::ofstream outfile;

    outfile.open("RolloutComputationTime.txt",  std::ios::app );

    outfile<<"Nearest Neighbor Radius: "<<ompl::magic::DEFAULT_NEAREST_NEIGHBOUR_RADIUS<<", Monte Carlo Particles: "<<ompl::magic::NUM_MONTE_CARLO_PARTICLES<<", Avg Time/neighbor (seconds): "<<averageTimeForRolloutComputation<<std::endl;

    outfile.close();

    //costToGoHistory_.push_back(std::make_pair(currentTimeStep_,0));

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

void FIRM::addStateToVisualization(ompl::base::State *state)
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

        Visualizer::addMostLikelyPathEdge(stateProperty_[v], stateProperty_[targetVertex]);

        v = targetVertex;
    }
}

FIRM::Edge FIRM::generateRolloutPolicy(const FIRM::Vertex currentVertex)
{
    /**
        For the given node, find the out edges and see the total cost of taking that edge
        The cost of taking the edge is cost to go from the target of the edge + the cost of the edge itself
    */
    double minCost = 1e10;
    Edge edgeToTake;

    // Iterate over the out edges
    foreach(Edge e, boost::out_edges(currentVertex, g_))
    {

        // Get the target node of the edge
        Vertex targetNode = boost::target(e, g_);

        // The cost to go from the target node
        double nextNodeCostToGo = costToGo_[targetNode];

        // Find the weight of the edge
        FIRMWeight edgeWeight =  boost::get(boost::edge_weight, g_, e);

        // The transition prob of the edge
        double transitionProbability  = edgeWeight.getSuccessProbability();

        // the cost of taking the edge
        double edgeCostToGo = (transitionProbability*nextNodeCostToGo + (1-transitionProbability)*ompl::magic::OBSTACLE_COST_TO_GO) + edgeWeight.getCost();

        if(edgeCostToGo < minCost)
        {
            minCost  = edgeCostToGo;
            edgeToTake = e;

        }

    }

    //costToGoHistory_.push_back(std::make_pair(currentTimeStep_,minCost));

    return edgeToTake;
}

void FIRM::simulateKidnapping()
{
    siF_->setTrueState(kidnappedState_);
}

bool FIRM::detectKidnapping(ompl::base::State *previousState, ompl::base::State *newState)
{

    using namespace arma;

    mat previousCov = previousState->as<SE2BeliefSpace::StateType>()->getCovariance();

    mat newCov = newState->as<SE2BeliefSpace::StateType>()->getCovariance();

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

        arma::colvec xVec = stateProperty_[v]->as<SE2BeliefSpace::StateType>()->getArmaData();

        arma::mat cov = stateProperty_[v]->as<SE2BeliefSpace::StateType>()->getCovariance();

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


void FIRM::loadRoadMapFromFile(const std::string pathToFile)
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

            newState->as<SE2BeliefSpace::StateType>()->setXYYaw(xVec(0),xVec(1),xVec(2));
            newState->as<SE2BeliefSpace::StateType>()->setCovariance(cov);

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

    int counter = 0;

    Visualizer::doSaveVideo(true);

    int timeSinceKidnap = 0;

    weightsHistory_.push_back(std::make_pair(timeSinceKidnap,policyGenerator_->getWeights()));

    auto start_time_recovery = std::chrono::high_resolution_clock::now();

    while(!policyGenerator_->isConverged())
    {
        std::vector<ompl::control::Control*> policy;

        policyGenerator_->generatePolicy(policy);

        Visualizer::doSaveVideo(true);

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
