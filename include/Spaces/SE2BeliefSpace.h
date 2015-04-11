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

#ifndef BELIEF_SPACE_H_
#define BELIEF_SPACE_H_

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
            StateType(void) : CompoundStateSpace::StateType()
            {
              covariance_ = arma::zeros<arma::mat>(3,3);
              controllerID_ = -1;

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

            arma::mat getCovariance(void) const
            {
                return covariance_;
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

            void setCovariance(arma::mat cov){
                covariance_ = cov;
            }

            arma::colvec getArmaData(void) const
            {
                arma::colvec stateVec(3);

                stateVec[0] = getX();
                stateVec[1] = getY();
                stateVec[2] = getYaw();
                return stateVec;
            }

            /** \brief Checks if the input state has stabilized to this state (node reachability check) */
            bool isReached(ompl::base::State *state, bool relaxedConstraint=false) const;

            static double meanNormWeight_, covNormWeight_, reachDist_;

            static arma::colvec normWeights_;

        private:
              arma::mat covariance_;
              size_t controllerID_;

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
