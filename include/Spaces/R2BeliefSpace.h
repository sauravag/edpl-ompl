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

#ifndef R2BELIEF_SPACE_H_
#define R2BELIEF_SPACE_H_

// OMPL includes
#include "ompl/base/spaces/RealVectorStateSpace.h"

//other includes
#include <boost/math/constants/constants.hpp>
#include <armadillo>

using namespace ompl::base;
class R2BeliefSpace : public ompl::base::RealVectorStateSpace
{

    public:

        /** \brief A belief in R(2): (x, y, covariance) */
        class StateType : public RealVectorStateSpace::StateType
        {
        public:
            StateType(void) : RealVectorStateSpace::StateType()
            {
              covariance_ = arma::zeros<arma::mat>(2,2);
              controllerID_ = -1;

            }

            /** \brief Get the X component of the state */
            double getX(void) const
            {
                return this->values[0];
            }

            /** \brief Get the Y component of the state */
            double getY(void) const
            {
                return this->values[1];
            }           

            arma::mat getCovariance(void) const
            {
                return covariance_;
            }


            /** \brief Set the X component of the state */
            void setX(double x)
            {
                this->values[0] = x;
            }

            /** \brief Set the Y component of the state */
            void setY(double y)
            {
                this->values[1] = y;
            }

            /** \brief Set the X and Y components of the state */
            void setXY(double x, double y)
            {
                setX(x);
                setY(y);
            }    

            void setArmaData(const arma::colvec &x)
            {
                setX(x[0]);
                setY(x[1]);
            }

            void setCovariance(arma::mat cov){
                covariance_ = cov;
            }

            arma::colvec getArmaData(void) const
            {
                arma::colvec stateVec(2);

                stateVec[0] = getX();
                stateVec[1] = getY();
                return stateVec;
            }

            /** \brief Checks if the input state has stabilized to this state (node reachability check) */
            bool isReached(ompl::base::State *state, bool relaxedConstraint=false) const;


            /** \brief Sample a new state from this belief state (mainly for Monte Carlo simulation) */
            bool sampleFromBelief(ompl::base::State* sampState) const
            {
                // Cholesky decomposition such that covariance_ = transform * transform.t()
                arma::mat transform;
                if(!arma::chol(transform, covariance_, "lower"))
                {
                    OMPL_ERROR("Failed to decompose the covariance matrix for random sampling!");
                    return false;
                }

                // draw a random sample from standard normal distribution
                arma::colvec randvec(2, arma::fill::randn);

                // transform this random sample for this Gaussian distribution
                arma::colvec mean = getArmaData();
                arma::colvec randvec_transformed = mean + transform * randvec;

                // set the state property as
                sampState->as<StateType>()->setX(randvec_transformed[0]);
                sampState->as<StateType>()->setY(randvec_transformed[1]);
                sampState->as<StateType>()->setCovariance(covariance_);    // REVIEW set the covariance as the same with this state

                return true;
            }


            static double meanNormWeight_, covNormWeight_, reachDist_;

            static arma::colvec normWeights_;

        private:
              arma::mat covariance_;
              size_t controllerID_;

        };


        R2BeliefSpace(void) : RealVectorStateSpace(2)
        {
            setName("R2_BELIEF" + getName());
            type_ = STATE_SPACE_REAL_VECTOR;
        }

        virtual ~R2BeliefSpace(void)
        {
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
