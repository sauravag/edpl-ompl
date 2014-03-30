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
/* Authors: Saurav Agarwal, Ali-akbar Agha-mohammadi */

#ifndef FIRM_PLANNER_
#define FIRM_PLANNER_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <utility>
#include <vector>
#include <map>
#include "../Weight/FIRMWeight.h"
#include "ompl/control/ControlSpace.h"
#include "ompl/control/SpaceInformation.h"
#include "../Controllers/Controller.h"
#include "../SeparatedControllers/RHCICreate.h"
#include "../Filters/ExtendedKF.h"
#include "../Filters/LinearizedKF.h"
#include "../Path/FeedbackPath.h"
#include "../ConnectionStrategy/FStrategy.h"
#include "../Path/FeedbackPath.h"
//#include "../SpaceInformation/SpaceInformation.h"
//#include "../Visualization/Visualizer.h"

namespace base
{
    // Forward declare for use in implementation
    OMPL_CLASS_FORWARD(OptimizationObjective);
}

/**
   @anchor FIRM
   @par Short description
   Feedback Information RoadMap is a method that generates a graph in belief space 
   and returns a policy that guides that robot from the start to goal.

   @par External documentation
   Please refer to the publications at Ali's research page.
  
   <a href="http://www.mit.edu/~aliagha/Web/publications.htm">[Link to Ali Agha's publications]</a>
*/

/** \brief Feedback Information RoadMap planner */
class FIRM : public ompl::base::Planner
{

public:

    struct vertex_state_t {
        typedef boost::vertex_property_tag kind;
    };

    struct vertex_total_connection_attempts_t {
        typedef boost::vertex_property_tag kind;
    };

    struct vertex_successful_connection_attempts_t {
        typedef boost::vertex_property_tag kind;
    };

    struct vertex_flags_t {
        typedef boost::vertex_property_tag kind;
    };

    struct edge_flags_t {
        typedef boost::edge_property_tag kind;
    };

    /**
     @brief The underlying roadmap graph.

     @par Edges are directed and have a weight property called FIRMWeight. This weight property 
          stores information about the edge controller identification, transition probability and 
          execution cost.
     */
    typedef boost::adjacency_list <
        boost::vecS, boost::vecS, boost::bidirectionalS,
        boost::property < vertex_state_t, ompl::base::State*,
        boost::property < vertex_total_connection_attempts_t, unsigned int,
        boost::property < vertex_successful_connection_attempts_t, unsigned int,
        boost::property < vertex_flags_t, unsigned int,
        boost::property < boost::vertex_predecessor_t, unsigned long int,
        boost::property < boost::vertex_rank_t, unsigned long int > > > > > >,
        boost::property < boost::edge_weight_t, FIRMWeight /* ompl::base::Cost */,
        boost::property < boost::edge_index_t, unsigned int,
        boost::property < edge_flags_t, unsigned int > > >
    > Graph;

    typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
    typedef boost::graph_traits<Graph>::edge_descriptor   Edge;

    typedef boost::shared_ptr< ompl::NearestNeighbors<Vertex> > RoadmapNeighbors;

    /** @brief A function returning the milestones that should be
     *         attempted to connect to
     */
    typedef boost::function<std::vector<Vertex>&(const Vertex)> ConnectionStrategy;

    /** @brief A function that can reject connections.
     This is called after previous connections from the neighbor list
     have been added to the roadmap.
     */
    typedef boost::function<bool(const Vertex&, const Vertex&)> ConnectionFilter;

    /** Defining the edge and node controller types*/
    typedef Controller<RHCICreate, ExtendedKF> EdgeControllerType;
    typedef Controller<RHCICreate, LinearizedKF> NodeControllerType;

    /** \brief Constructor */
    FIRM(const firm::SpaceInformation::SpaceInformationPtr &si, bool debugMode=false, bool starStrategy = false);

    virtual ~FIRM(void);

    virtual void setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef);

    /** \brief Set the connection strategy function that specifies the
     milestones that connection attempts will be made to for a
     given milestone.

     \par The behavior and performance of PRM can be changed drastically
     by varying the number and properties of the milestones that are
     connected to each other.

     \param pdef A function that takes a milestone as an argument and
     returns a collection of other milestones to which a connection
     attempt must be made. The default connection strategy is to connect
     a milestone's 10 closest neighbors.
     */
    void setConnectionStrategy(const ConnectionStrategy& connectionStrategy)
    {
        connectionStrategy_ = connectionStrategy;
        userSetConnectionStrategy_ = true;
    }
    /** \brief Convenience function that sets the connection strategy to the
     default one with k nearest neighbors.
     */
    void setMaxNearestNeighbors(unsigned int k);

    /** \brief Set the function that can reject a milestone connection.

     \par The given function is called immediately before a connection
     is checked for collision and added to the roadmap. Other neighbors
     may have already been connected before this function is called.
     This allows certain heuristics that use the structure of the
     roadmap (like connected components or useful cycles) to be
     implemented by changing this function.

     \param connectionFilter A function that takes the new milestone,
     a neighboring milestone and returns whether a connection should be
     attempted.
     */
    void setConnectionFilter(const ConnectionFilter& connectionFilter)
    {
        connectionFilter_ = connectionFilter;
    }

    virtual void getPlannerData(ompl::base::PlannerData &data) const;

    /** \brief While the termination condition allows, this function will construct the roadmap (using growRoadmap() and expandRoadmap(),
        maintaining a 2:1 ratio for growing/expansion of roadmap) */
    virtual void constructRoadmap(const ompl::base::PlannerTerminationCondition &ptc);

    /** \brief If the user desires, the roadmap can be
        improved for the given time (seconds). The solve()
        method will also improve the roadmap, as needed.*/
    virtual void growRoadmap(double growTime);

    /** \brief If the user desires, the roadmap can be
        improved until a given condition is true. The solve()
        method will also improve the roadmap, as needed.*/
    virtual void growRoadmap(const ompl::base::PlannerTerminationCondition &ptc);

    /** \brief Attempt to connect disjoint components in the roadmap
        using random bouncing motions (the PRM expansion step) for the
        given time (seconds). */
    virtual void expandRoadmap(double expandTime);

    /** \brief Attempt to connect disjoint components in the roadmap
        using random bouncing motions (the PRM expansion step) until the
        given condition evaluates true. */
    virtual void expandRoadmap(const ompl::base::PlannerTerminationCondition &ptc);

    /** \brief  */
    virtual ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc);

    /** \brief  */
    void clearQuery(void);

    virtual void clear(void);

    /** \brief Set a different nearest neighbors datastructure */
    template<template<typename T> class NN>
    void setNearestNeighbors(void)
    {
        nn_.reset(new NN<Vertex>());
        if (!userSetConnectionStrategy_)
            connectionStrategy_.clear();
        if (isSetup())
            setup();
    }

    virtual void setup(void);

    const Graph& getRoadmap(void) const
    {
        return g_;
    }

    /** \brief Compute distance between two milestones (this is simply distance between the states of the milestones) */
    double distanceFunction(const Vertex a, const Vertex b) const
    {
        return si_->distance(stateProperty_[a], stateProperty_[b]);
    }

    /** \brief Compute distance between two milestones (this is simply distance between the states of the milestones) */
    unsigned int milestoneCount(void) const
    {
        return boost::num_vertices(g_);
    }

    const RoadmapNeighbors& getNearestNeighbors(void)
    {
        return nn_;
    }

     /** \brief Executes the generated policy on the system */
    void executeFeedback(void);
protected:

    /** \brief Free all the memory allocated by the planner */
    void freeMemory(void);

    /** \brief Construct a milestone for a given state (\e state), store it in the nearest neighbors data structure
        and then connect it to the roadmap in accordance to the connection strategy. */
    virtual Vertex addMilestone(ompl::base::State *state);

    /** \brief Make two milestones (\e m1 and \e m2) be part of the same connected component. The component with fewer elements will get the id of the component with more elements. */
    void uniteComponents(Vertex m1, Vertex m2);

    /** \brief Check if two milestones (\e m1 and \e m2) are part of the same connected component. This is not a const function since we use incremental connected components from boost */
    bool sameComponent(Vertex m1, Vertex m2);

    /** \brief Randomly sample the state space, add and connect milestones
         in the roadmap. Stop this process when the termination condition
         \e ptc returns true.  Use \e workState as temporary memory. */
    virtual void growRoadmap(const ompl::base::PlannerTerminationCondition &ptc, ompl::base::State *workState);

    /** \brief Attempt to connect disjoint components in the
        roadmap using random bounding motions (the PRM
        expansion step) */
    virtual void expandRoadmap(const ompl::base::PlannerTerminationCondition &ptc, std::vector<ompl::base::State*> &workStates);

    /** Thread that checks for solution */
    void checkForSolution(const ompl::base::PlannerTerminationCondition &ptc, ompl::base::PathPtr &solution);

    /** \brief Check if there exists a solution, i.e., there exists a pair of milestones such that the first is in \e start and the second is in \e goal, and the two milestones are in the same connected component. If a solution is found, the path is saved. */
    bool haveSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals, ompl::base::PathPtr &solution);

    /** \brief Returns the value of the addedSolution_ member. */
    bool addedNewSolution(void) const;

    /** \brief Given two milestones from the same connected component, construct a path connecting them and set it as the solution */
    //virtual ompl::base::PathPtr constructSolution(const Vertex &start, const Vertex &goal);

    /** \brief Given a solution represented as a vector of predecesors in the roadmap, construct a geometric path */
    virtual ompl::base::PathPtr constructFeedbackPath(const Vertex &start, const Vertex &goal);

    /** \brief Add an edge from vertex a to b in graph */
    virtual void addEdgeToGraph(const Vertex a, const Vertex b);

    /** \brief Generates the cost of the edge */
    virtual FIRMWeight generateControllersWithEdgeCost(ompl::base::State* startNodeState,
                                                             ompl::base::State* targetNodeState,
                                                             EdgeControllerType &edgeController,
                                                             NodeControllerType &nodeController);

    /** \brief Generates the edge controller that drives the robot from start to end of edge */
    virtual void generateEdgeController(const ompl::base::State *start, const ompl::base::State* target, EdgeControllerType &edgeController);

    /** \brief Generates the node controller that stabilizes the robot to the node */
    virtual void generateNodeController(const ompl::base::State *state, NodeControllerType &nodeController);

    /** \brief Solves the dynamic program to return a feedback policy */
    virtual void solveDynamicProgram(const Vertex goalVertex);

    void addStateToVisualization(ompl::base::State *state) ;

    void sendFeedbackEdgesToViz();

    /** \brief Calculates the new cost to go from a node*/
    std::pair<typename FIRM::Edge,double> getUpdatedNodeCostToGo(Vertex node);

    /** \brief Flag indicating whether the default connection strategy is the Star strategy */
    bool                                                   starStrategy_;

    /** \brief Sampler user for generating valid samples in the state space */
    ompl::base::ValidStateSamplerPtr                             sampler_;

    /** \brief Sampler user for generating random in the state space */
    ompl::base::StateSamplerPtr                                  simpleSampler_;

    /** \brief Nearest neighbors data structure */
    RoadmapNeighbors                                       nn_;

    /** \brief Connectivity graph */
    Graph                                                  g_;

    /** \brief Array of start milestones */
    std::vector<Vertex>                                    startM_;

    /** \brief Array of goal milestones */
    std::vector<Vertex>                                    goalM_;

    /** \brief Access to the internal ompl::base::state at each Vertex */
    boost::property_map<Graph, vertex_state_t>::type       stateProperty_;

    /** \brief Access to the number of total connection attempts for a vertex */
    boost::property_map<Graph,
        vertex_total_connection_attempts_t>::type          totalConnectionAttemptsProperty_;

    /** \brief Access to the number of successful connection attempts for a vertex */
    boost::property_map<Graph,
        vertex_successful_connection_attempts_t>::type     successfulConnectionAttemptsProperty_;

    /** \brief Access to the weights of each Edge */
    boost::property_map<Graph, boost::edge_weight_t>::type weightProperty_;

    /** \brief Access to the indices of each Edge */
    boost::property_map<Graph, boost::edge_index_t>::type  edgeIDProperty_;

    /** \brief Data structure that maintains the connected components */
    boost::disjoint_sets<
        boost::property_map<Graph, boost::vertex_rank_t>::type,
        boost::property_map<Graph, boost::vertex_predecessor_t>::type >
                                                           disjointSets_;

    /** \brief Maximum unique id number used so for for edges */
    unsigned int                                           maxEdgeID_;

    /** \brief Function that returns the milestones to attempt connections with */
    ConnectionStrategy                                     connectionStrategy_;

    /** \brief Function that can reject a milestone connection */
    ConnectionFilter                                       connectionFilter_;

    /** \brief Flag indicating whether the employed connection strategy was set by the user (or defaults are assumed) */
    bool                                                   userSetConnectionStrategy_;

    /** \brief Random number generator */
    ompl::RNG                                                    rng_;

    /** \brief A flag indicating that a solution has been added during solve() */
    bool                                                   addedSolution_;

    /** \brief Mutex to guard access to the Graph member (g_) */
    mutable boost::mutex                                   graphMutex_;

    /** \brief Objective cost function for PRM graph edges */
    ompl::base::OptimizationObjectivePtr                         opt_;

    /** \brief Given two vertices, returns a heuristic on the cost of the path connecting them. This method wraps OptimizationObjective::motionCostHeuristic */
    ompl::base::Cost costHeuristic(Vertex u, Vertex v) const;

    /** \brief The base::SpaceInformation cast as firm::SpaceInformation, for convenience */
    const firm::SpaceInformation::SpaceInformationPtr            siF_;

    /** \brief A table that stores the edge controllers according to the edges */
    std::map <Edge, EdgeControllerType > edgeControllers_;

    /** \brief A table that stores the node controllers according to the node (vertex) ids */
    std::map <Vertex, NodeControllerType > nodeControllers_;

    std::map <Vertex, double> costToGo_;

    // This feedback will eventually be in a feedbackpath class
    std::map <Vertex, Edge> feedback_;
    /** \brief The number of particles to use for monte carlo simulations*/
    unsigned int numParticles_;

    bool debug_;

    unsigned int minFIRMNodes_;

};


#endif
