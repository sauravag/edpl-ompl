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

#ifndef NBM3P_H
#define NBM3P_H


#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/thread.hpp>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/control/Control.h>
#include "../SpaceInformation/SpaceInformation.h"
#include <ompl/base/Cost.h>
#include "../Filters/ExtendedKF.h"

/** \para
NBM3P is a planner for Non-Gaussian Belief State. Stated simply, its job is to generate the best next control to disambiguate the belief
given the current modes.
Key features:

1) We represent the belief with a Gaussian Mixture Model (GMM) rather than particles.

2) Instead of basing actions on the most-likely hypothesis, we create candidate actions based on each mode and evaluate
the best one.

3) We use a sampling based planner i.e. RRT* to plan candidate trajectories. (One can also simply use RRTs but
due to insignificant overhead in using RRT* over RRT we prefer RRT* as it gives us the benefit of optimality)

4) We introduce a switching behavior in the belief representation during the online-phase from Gaussian to non-Gaussian,
and back, as required. Our argument is that most of the times, the belief is well represented by a Gaussian, wherever
this is not the case, we switch to a GMM and our algorithm creates plans that converge back to a uni-modal Gaussian.

5) We present simulation results for a 2D navigation problem in which a robot is kidnapped.
*/

class NBM3P
{

    public:

        struct vertex_state_t {
            typedef boost::vertex_property_tag kind;
        };

        typedef boost::adjacency_list <
            boost::vecS, boost::vecS, boost::undirectedS,
            boost::property < vertex_state_t, ompl::base::State*>,
            boost::property < boost::edge_weight_t, unsigned int ,
            boost::property < boost::edge_index_t, unsigned int > >
        > Graph;

        typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

        typedef boost::graph_traits<Graph>::edge_descriptor   Edge;

        typedef boost::shared_ptr< ompl::NearestNeighbors<Vertex> > RoadmapNeighbors;

        /** \brief Constructor */
        NBM3P(firm::SpaceInformation::SpaceInformationPtr si):si_(si),
        maxEdgeID_(0),
        stateProperty_(boost::get(vertex_state_t(), g_)),
        weightProperty_(boost::get(boost::edge_weight, g_)),
        edgeIDProperty_(boost::get(boost::edge_index, g_))
        {
            previousPolicy_.push_back(si_->getMotionModel()->getZeroControl());
            policyExecutionSI_ = si_;
        }

        /** \brief Destructor */
        virtual ~NBM3P()
        {
            while(!currentBeliefStates_.empty())
            {
                si_->freeState(currentBeliefStates_.back()), currentBeliefStates_.pop_back();
            }

        }

        /** \brief This function samples the beliefs, which form the starting point for the multi-modal scenario*/
        void sampleNewBeliefStates();

        /** \brief Compute distance between two milestones (this is simply distance between the states of the milestones) */
        double distanceFunction(const Vertex a, const Vertex b) const
        {
            return si_->distance(stateProperty_[a], stateProperty_[b]);
        }

        /** \brief Set the current belief states based on robot's belief states*/
        void setCurrentBeliefStates(const std::vector<ompl::base::State*> states)
        {
            currentBeliefStates_.clear();
            weights_.clear();

            for(unsigned int i = 0; i < states.size(); i++)
            {
                currentBeliefStates_.push_back(si_->cloneState(states[i]));
                weights_.push_back(1.0/states.size()); // assign equal weights to all
            }

        }

        /** \brief Add the FIRM node as a state to the observation graph*/
        void addFIRMNodeToObservationGraph(ompl::base::State *state)
        {
            this->addStateToObservationGraph(si_->cloneState(state));
        }

        /** \brief For each belief state, there is a target node to go to, set those here */
        void setBeliefTargetStates(const std::vector<ompl::base::State*> states)
        {
            targetStates_.clear();

            for(unsigned int i = 0; i < states.size(); i++)
            {
                addFIRMNodeToObservationGraph(si_->cloneState(states[i]));
                targetStates_.push_back(si_->cloneState(states[i]));
            }

        }

        /** brief Get the current belief states*/
        void getCurrentBeliefStates(std::vector<ompl::base::State*> &states)
        {
            states.clear();

            for(unsigned int i = 0; i < currentBeliefStates_.size(); i++)
            {
                states.push_back(si_->cloneState(currentBeliefStates_[i]));
            }

        }

        /** \brief clear out the internal state containers */
        void clearInternalStates()
        {
            currentBeliefStates_.clear();
            targetStates_.clear();
        }

        /** \brief Given the current belief states and target states, output the best policy. */
        /** \para This function does the majority work. It will use an RRT to generate open loop control policies
        based on each mode to its target. Then evaluate each policy on each mode and return the best policy to be followed. We will
        then apply this policy to the true state i.e. the robot and update all the beliefs. This process would then get
        repeated.*/
        virtual void generatePolicy(std::vector<ompl::control::Control*> &policy);

        /** \brief Runs the open loop policy on a given mode and outputs the cost*/
        virtual ompl::base::Cost executeOpenLoopPolicyOnMode(std::vector<ompl::control::Control*> controls, const ompl::base::State* state);

        /** \brief advances the beliefs/modes by applying the given controls*/
        virtual void propagateBeliefs(const ompl::control::Control *control, bool isSimulation = false);

        /** \brief Updates the weights of the Gaussians in the mixture */
        virtual void updateWeights(const arma::colvec trueObservation);

        /** \brief Compute the innovation between the robot's and mode's observation*/
        virtual arma::colvec computeInnovation(const int currentBeliefIndx, const arma::colvec trueObservation, double &weightFactor);

        /** \brief Remove modes that are no longer considered important*/
        void removeBeliefs(const std::vector<int> Indxs);

        /** \brief Print the mode weights*/
        void printWeights() const
        {
            OMPL_INFORM("NBM3P: Printing Weights");

            for(unsigned int i=0; i < weights_.size(); i++)
            {
                OMPL_INFORM("NBM3P: The Weight of mode #%u = %f ",i,weights_[i]);
            }

        }

        /** \brief checks if the beliefs have converged */
        bool isConverged();

        /** \brief Returns true if all beliefs are valid i.e. collision free */
        bool areCurrentBeliefsValid();

        /** \brief Returns true if all beliefs satisfy a certain minimum clearance for n steps from now, else false. */
        bool doCurrentBeliefsSatisfyClearance(int currentStep);

        /** \brief Get the state with the max weight and its weight */
        void getStateWithMaxWeight(ompl::base::State *state, float &weight);

        /** \brief Get the weights of the modes */
        std::vector<float> getWeights()
        {
            return weights_;
        }

        /** \brief Get the numbder of modes currently alive */
        int getNumberOfModes()
        {
            return weights_.size();
        }

        /** \brief Set the space information in which the policies are executed. This space may or not be same as the variable si_.
                   Use this feature to change where the control action is executed (ex. real robot, ros/gazebo).
        */
        void setPolicyExecutionSpace(firm::SpaceInformation::SpaceInformationPtr executionSI)
        {
            policyExecutionSI_ = executionSI;
        }

    private:

        //float computeWeightForMode(const int currentBeliefIndx,const arma::colvec trueObservation);

         /** \brief Add the a state to the observation graph*/
        void addStateToObservationGraph(ompl::base::State *state);

        /** \brief Add an edge between the two vertices */
        void addEdgeToObservationGraph(const Vertex a, const Vertex b);

        /** \brief Returns the list of observations that are observed from a Vertex*/
        void evaluateObservationListForVertex(const Vertex v);

        /** \brief Get the overlap in observation for two vertices*/
        virtual bool getObservationOverlap(const Vertex a, const Vertex b, unsigned int &weight);

        /** \brief Get the nodes within some radius "r" to state */
        std::vector<Vertex> getNeighbors(const ompl::base::State *state);

        /** \brief Find the target location for the given state*/
        Vertex findTarget(const unsigned int beliefStateIndx);

        /** \brief Find the total weight of the edges from v to nodes in the given neighbor set*/
        int calculateIntersectionWithNeighbor(const Vertex v, std::vector<Vertex> neighbors);

        /** \brief Returns true if all weights are same, false otherwise*/
        bool areSimilarWeights();

        /** \brief if 2 modes have the same weight and pose, then they are duplicates */
        void removeDuplicateModes();

        /** \brief draw the current beliefs */
        void drawBeliefs();

        /** \brief Normalize the weights of modes */
        void normalizeWeights();

        /** \brief sets a uniform weight for all modes */
        void assignUniformWeight();

        /** \brief Container for the current modes/beliefs*/
        std::vector<ompl::base::State*> currentBeliefStates_;

        /** \brief The target location for each corresponding belief*/
        std::vector<ompl::base::State*> targetStates_;

        /** \brief Pointer to the state space information*/
        firm::SpaceInformation::SpaceInformationPtr si_;

        /** \brief This is the space in which the policy is executed. By default it is set to the same space that is passed to the constructer.
                If this space is changed, the observations and controls are both in the context of this new space. Use the setPolicyExecutionSpace
                function to change this parameter. Particularly useful if you wish to drive a real robot and get sensor readings.*/
        firm::SpaceInformation::SpaceInformationPtr policyExecutionSI_;

        /** \brief Container for the previous open loop policy*/
        std::vector<ompl::control::Control*> previousPolicy_;

        /** \brief Container for the current weights of the beliefs */
        std::vector<float> weights_;

        /** \brief connectivity graph (observation graph) */
        Graph                                                  g_;

        /** \brief Access to the internal ompl::base::state at each Vertex */
        boost::property_map<Graph, vertex_state_t>::type       stateProperty_;

        /** \brief Access to the weights of each Edge */
        boost::property_map<Graph, boost::edge_weight_t>::type weightProperty_;

        /** \brief Access to the indices of each Edge */
        boost::property_map<Graph, boost::edge_index_t>::type  edgeIDProperty_;


        /** \brief Data structure that maintains the connected components */
        /*boost::disjoint_sets<
            boost::property_map<Graph, boost::vertex_rank_t>::type,
            boost::property_map<Graph, boost::vertex_predecessor_t>::type >
                                                                    disjointSets_;*/
        /** \brief Stores the landmarks that a vertex in the graph can see*/
        std::map<Vertex, std::vector<arma::colvec> > stateObservationProperty_;

        /** \brief Mutex to guard access to the Graph member (g_) */
        mutable boost::mutex                                   graphMutex_;

        /** \brief Used to track the maximum number of Edges in the Uniqueness graph */
        unsigned int maxEdgeID_;

        /** \brief keep a track of how long a belief has been predicting to observe something that is not seen by the robot*/
        std::vector<double> timeSinceDivergence_;

};
#endif
