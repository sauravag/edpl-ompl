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

/* Authors: Saurav Agarwal */

#ifndef SE2BELIEF_SPACE_H_
#define SE2BELIEF_SPACE_H_

// OMPL includes
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/spaces/SE3StateSpace.h"

//other includes
#include <boost/math/constants/constants.hpp>
#include <armadillo>

using namespace ompl::base;
class SE2BeliefSpace : public ompl::base::CompoundStateSpace
{

    public:

        /** \brief A belief in SE(2): (x, y, yaw, covariance) */
        class StateType : public CompoundStateSpace::StateType
        {
        public:

            typedef unsigned long int Vertex;    // HACK from include/Planner/FIRM.h
            static const unsigned long int INVALID_VERTEX_ID = (unsigned long int) -1;  // HACK from include/Planner/FIRM.h

            StateType(void) : CompoundStateSpace::StateType()
            {
              covariance_ = arma::zeros<arma::mat>(3,3);
              controllerID_ = -1;

              clearThisQVvisit();
              clearThisQVmincosttogo();
              clearChildQexpanded();
            }

            /** \brief Get the X component of the state */
            double getX(void) const
            {
                return as<RealVectorStateSpace::StateType>(0)->values[0];
            }

            /** \brief Get the Y component of the state */
            double getY(void) const
            {
                return as<RealVectorStateSpace::StateType>(0)->values[1];
            }

            /** \brief Get the yaw component of the state. This is
                the rotation in plane, with respect to the Z
                axis. */
            double getYaw(void) const
            {
                return as<SO2StateSpace::StateType>(1)->value;
            }

            arma::colvec getArmaData(void) const
            {
                arma::colvec stateVec(3);

                stateVec[0] = getX();
                stateVec[1] = getY();
                stateVec[2] = getYaw();
                return stateVec;
            }

            arma::mat getCovariance(void) const
            {
                return covariance_;
            }

            double getTraceCovariance(void) const
            {
                return arma::trace(covariance_);
            }


            /** \brief Set the X component of the state */
            void setX(double x)
            {
                as<RealVectorStateSpace::StateType>(0)->values[0] = x;
            }

            /** \brief Set the Y component of the state */
            void setY(double y)
            {
                as<RealVectorStateSpace::StateType>(0)->values[1] = y;
            }

            /** \brief Set the X and Y components of the state */
            void setXY(double x, double y)
            {
                setX(x);
                setY(y);
            }

            /** \brief Set the yaw component of the state. This is
                the rotation in plane, with respect to the Z
                axis. */
            void setYaw(double yaw)
            {
                as<SO2StateSpace::StateType>(1)->value = yaw;
            }

            void setXYYaw(double x, double y, double yaw)
            {
                setX(x);
                setY(y);
                setYaw(yaw);
            }

            void setArmaData(const arma::colvec &x)
            {
                setX(x[0]);
                setY(x[1]);
                setYaw(x[2]);
            }

            void setCovariance(arma::mat cov){
                covariance_ = cov;
            }


            void clearThisQVvisit(){
                thisQVvisit_ = 0.0;
            }

            void clearThisQVmincosttogo(){
                thisQVmincosttogo_ = 1000000000.0;    // ompl::magic::DEFAULT_INF_COST_TO_GO
            }

            void clearChildQexpanded(){
                childQexpanded_ = false;
            }

            void clearChildQnodes(){
                childQnodes_.clear();
            }

//             void clearChildQweights(){
//                 childQweights_.clear();
//             }

//             void clearChildQvalues(){
//                 childQvalues_.clear();
//             }
//
//             void clearChildQpenalties(){
//                 childQpenalties_.clear();
//             }

            void clearChildQcosttogoes(){
                childQcosttogoes_.clear();
            }

            void clearChildQvisits(){
                childQvisits_.clear();
            }

            void clearChildQmisses(){
                childQmisses_.clear();
            }

            void clearChildQVnodes(){
                childQVnodes_.clear();
            }


            void addThisQVvisit(const double visit=1.0){
                thisQVvisit_ += visit;
            }

            void setThisQVmincosttogo(const double mincosttogo){
                thisQVmincosttogo_ = mincosttogo;
            }

            void updateThisQVmincosttogo(const double costtogo){
                if (costtogo < thisQVmincosttogo_)
                    thisQVmincosttogo_ = costtogo;
            }

            void setChildQexpanded(const bool expanded=true){
                childQexpanded_ = expanded;
            }

            void addChildQnode(const Vertex childQnode){
                childQnodes_.push_back(childQnode);
            }

//             void setChildQweight(const Vertex childQnode, const double weight){
//                 childQweights_[childQnode] = weight;
//             }

//             void setChildQvalue(const Vertex childQnode, const double value){
//                 childQvalues_[childQnode] = value;
//             }
//
//             void updateChildQvalue(const Vertex childQnode, const double value){
//                 childQvalues_[childQnode] = std::min(getChildQvalue(childQnode), value);
//             }
//
//             void setChildQpenalty(const Vertex childQnode, const double penalty){
//                 childQpenalties_[childQnode] = penalty;
//             }
//
//             void updateChildQpenalty(const Vertex childQnode, const double penalty){
//                 childQpenalties_[childQnode] = std::max(getChildQpenalty(childQnode), penalty);
//             }

            void setChildQcosttogo(const Vertex childQnode, const double costtogo){
                childQcosttogoes_[childQnode] = costtogo;
            }

//             void updateChildQcosttogo(const Vertex childQnode){
//                 childQcosttogoes_[childQnode] = getChildQvalue(childQnode) + getChildQpenalty(childQnode);
//             }

            void addChildQvisit(const Vertex childQnode, const double visit=1.0){
                childQvisits_[childQnode] += visit;
            }

            void addChildQmiss(const Vertex childQnode, const double miss=1.0){
                childQmisses_[childQnode] += miss;
            }

            void addChildQVnode(const Vertex childQnode, const Vertex childQVnode){
//                 childQVnodes_[childQnode].push_back(childQVnode);
                childQVnodes_[childQnode] = childQVnode;
            }


            const double getThisQVvisit() const {
                return thisQVvisit_;
            }

            double getThisQVmincosttogo(){
                return thisQVmincosttogo_;
            }

            const bool getChildQexpanded() const {
                return childQexpanded_;
            }

            const std::vector<Vertex> getChildQnodes() const {
                return childQnodes_;
            }

//             double getChildQweight(const Vertex childQnode){
//                 if (childQweights_.find(childQnode) == childQweights_.end())
//                 {
//                     OMPL_ERROR("childQnode key is not found in childQweights_!");
//                     return 0.0;   // not to allow this action to be selected
//                 }
//                 return childQweights_.at(childQnode);
//             }

//             const std::map<Vertex, double> getChildQvalues() const {
//                 return childQvalues_;
//             }
//
//             double getChildQvalue(const Vertex childQnode){
//                 if (childQvalues_.find(childQnode) == childQvalues_.end())
//                 {
//                     OMPL_ERROR("childQnode key is not found in childQvalues_!");
//                     //return ompl::magic::DEFAULT_INF_COST_TO_GO;   // not to allow this action to be selected
//                     return 1000000000.0;
//                 }
//                 return childQvalues_.at(childQnode);
//             }
//
//             const std::map<Vertex, double> getChildQpenalties() const {
//                 return childQpenalties_;
//             }
//
//             double getChildQpenalty(const Vertex childQnode){
//                 if (childQpenalties_.find(childQnode) == childQpenalties_.end())
//                 {
//                     //OMPL_ERROR("childQnode key is not found in childQpenalties_!");
//                     return 0.0;  // default value should be the minimum since C(ha) = max(C(ha), penalty)
//                 }
//                 return childQpenalties_.at(childQnode);
//             }

            const std::map<Vertex, double> getChildQcosttogoes() const {
                return childQcosttogoes_;
            }

            double getChildQcosttogo(const Vertex childQnode){
                if (childQcosttogoes_.find(childQnode) == childQcosttogoes_.end())
                {
                    OMPL_ERROR("childQnode key is not found in childQcosttogoes_!");
//                     updateChildQcosttogo(childQnode);  // J(ha) = V(ha) + C(ha)
                }
                return childQcosttogoes_.at(childQnode);
            }

            double getChildQvisit(const Vertex childQnode){
                if (childQvisits_.find(childQnode) == childQvisits_.end())
                {
                    //OMPL_ERROR("childQnode key is not found in childQvisits_!");
                    return 0.0;  // not yet expanded, so no visit
                }
                return childQvisits_.at(childQnode);
            }

            double getChildQmiss(const Vertex childQnode){
                if (childQmisses_.find(childQnode) == childQmisses_.end())
                {
                    //OMPL_ERROR("childQnode key is not found in childQmisses_!");
                    return 0.0;  // not yet expanded, so no miss (collision)
                }
                return childQmisses_.at(childQnode);
            }

//             const std::vector<Vertex> getChildQVnodes(const Vertex selectedChildQnode) const {
//                 if (childQVnodes_.find(selectedChildQnode) == childQVnodes_.end())
//                 {
//                     //OMPL_INFO("selectedChildQnode key is not found in childQVnodes_!");
//                     return std::vector<Vertex>();  // return an empty vector
//                 }
//                 return childQVnodes_.at(selectedChildQnode);
//             }
            const Vertex getChildQVnode(const Vertex selectedChildQnode) const {
                if (childQVnodes_.find(selectedChildQnode) == childQVnodes_.end())
                {
                    //OMPL_WARN("selectedChildQnode key is not found in childQVnodes_!");
                    //return std::vector<Vertex>();  // return an empty vector
                    return INVALID_VERTEX_ID;
                }
                return childQVnodes_.at(selectedChildQnode);
            }


            /** \brief Checks if the input state has stabilized to this state (node pose and covariance reachability check) */
            bool isReached(ompl::base::State *state, bool relaxedConstraint=false) const;

            bool isReachedWithinNEps(const ompl::base::State *state, const double nEps=1.0) const;

            /** \brief Checks if the input state's pose has reached this state (node pose reachability check) */
            bool isReachedPose(const ompl::base::State *state, const double nEps=1.0) const;

            /** \brief Checks if the input state's covariance has reached this state (node covariance reachability check) */
            bool isReachedCov(const ompl::base::State *state, const double nEps=1.0) const;


            /** \brief Sample a border belief state from this belief state (mainly for Monte Carlo simulation) */
            bool sampleBorderBeliefState(ompl::base::State* borderBelief) const;

            /** \brief Sample a new state from this belief state (mainly for Monte Carlo simulation) */
            bool sampleTrueStateFromBelief(ompl::base::State* sampState, const double nsigma=2.0) const;


            double getStateDistanceTo(const ompl::base::State *state) const;
            double getPosDistanceTo(const ompl::base::State *state) const;
            double getOriDistanceTo(const ompl::base::State *state) const;
            double getCovDistanceTo(const ompl::base::State *state) const;


            static double meanNormWeight_, covNormWeight_, reachDist_, reachDistPos_, reachDistOri_, reachDistCov_;

            static arma::colvec normWeights_;

        private:
              arma::mat covariance_;
              size_t controllerID_;


              // FIRMCP
              // SE2BeliefSpace state will represent a VNODE in POMCP (a belief state from {a1, o1, a2, o2, ..., at, ot})
              // SE2BeliefSpace state will also contain the information of QNODEs in POMCP (a belief state from {a1, o1, a2, o2, ..., at, ot, a(t+1)} for each action)
              // QNODES will not be explicitly saved in the graph

              double thisQVvisit_;                                  // N(h)   // size: [1]
              double thisQVmincosttogo_;                            // J(h) = min_a(Q(ha))   // size: [1]

              bool childQexpanded_;                                 // true if this node is added to POMCP tree
              std::vector<Vertex> childQnodes_;                     // T(ha)  // size: [number of actions (controllers to the connected neighbors)]
//               std::map<Vertex, double> childQweights_;              // w(ha)  // size: [number of actions]  // heuristic value only for POMCP-Rollout
//               std::map<Vertex, double> childQvalues_;               // V(ha)  // size: [number of actions]  // min(V(ha)) for pure cost-to-go   // XXX
//               std::map<Vertex, double> childQpenalties_;            // C(ha)  // size: [number of actions]  // max(C(ha)) for discounted collition penalty  // XXX
              std::map<Vertex, double> childQcosttogoes_;           // J(ha)  // size: [number of actions]  // J(ha) = V(ha) + C(ha)
              std::map<Vertex, double> childQvisits_;               // N(ha)  // size: [number of actions]
              std::map<Vertex, double> childQmisses_;               // M(ha)  // size: [number of actions]  // number of collisions out of N(ha) visits

//               std::map<Vertex, std::vector<Vertex>> childQVnodes_;  // T(hao) // size: [number of actions] x [number of (distinctive) observations] // XXX
              std::map<Vertex, Vertex> childQVnodes_;               // T(hao) // size: [number of actions]  // NOTE assuming all childQVnodes_[selectedChildQnode] except collision can be merged into one Gaussian belief state

        };


        SE2BeliefSpace(void) : CompoundStateSpace()
        {
            setName("SE2_BELIEF" + getName());
            type_ = STATE_SPACE_SE2;
            addSubspace(StateSpacePtr(new RealVectorStateSpace(2)), 1.0);
            addSubspace(StateSpacePtr(new SO2StateSpace()), 0.5);
            lock();
        }

        virtual ~SE2BeliefSpace(void)
        {
        }

        /** \copydoc RealVectorStateSpace::setBounds() */
        void setBounds(const RealVectorBounds &bounds)
        {
            as<RealVectorStateSpace>(0)->setBounds(bounds);
        }

        /** \copydoc RealVectorStateSpace::getBounds() */
        const RealVectorBounds& getBounds(void) const
        {
            return as<RealVectorStateSpace>(0)->getBounds();
        }

        virtual State* allocState(void) const;
        virtual void copyState(State *destination,const State *source) const;
        virtual void freeState(State *state) const;

        //virtual void registerProjections(void);
        virtual double distance(const State* state1, const State *state2);

        // gets the relative vector between "from" and "to"
        // equivalent to result = vectorA-vectorB
        void getRelativeState(const State *from, const State *to, State *state);

        void printBeliefState(const State *state);

};
#endif
