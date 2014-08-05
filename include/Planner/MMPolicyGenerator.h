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

#ifndef FIRM_OMPL_MM_POLICY_GENERATOR_H
#define FIRM_OMPL_MM_POLICY_GENERATOR_H

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/control/Control.h>
#include "../SpaceInformation/SpaceInformation.h"
#include <ompl/base/Cost.h>
#include "../Filters/ExtendedKF.h"

/** \para
The MMPolicyGenerator class's job is to generate the best next control to disambiguate the belief
given the current modes.
*/

class MMPolicyGenerator
{

    public:

        /** \brief Constructor */
        MMPolicyGenerator(firm::SpaceInformation::SpaceInformationPtr si):si_(si)
        {
            previousPolicy_.push_back(si_->getMotionModel()->getZeroControl());
        }

        /** \brief Destructor */
        ~MMPolicyGenerator()
        {
            while(!currentBeliefStates_.empty()) si_->freeState(currentBeliefStates_.back()), currentBeliefStates_.pop_back();

            //weights_.clear();
            //currentBeliefStates_.clear();
            //targetStates_.clear();
            //previousPolicy_.clear();
            //si_.reset();
        }

        /** \brief Set the current belief states based on robot's belief states*/
        void setCurrentBeliefStates(const std::vector<ompl::base::State*> states)
        {
            currentBeliefStates_.clear();
            weights_.clear();
            //currentBeliefStates_.insert(currentBeliefStates_.end(), states.begin(), states.end());

            for(int i = 0; i < states.size(); i++)
            {
                currentBeliefStates_.push_back(si_->cloneState(states[i]));
                weights_.push_back(1.0/states.size()); // assign equal weights to all
            }

        }

        /** \brief For each belief state, there is a target node to go to, set those here */
        void setBeliefTargetStates(const std::vector<ompl::base::State*> states)
        {
            targetStates_.clear();
            //targetStates_.insert(targetStates_.end(), states.begin(), states.end());

            for(int i = 0; i < states.size(); i++)
            {
                targetStates_.push_back(si_->cloneState(states[i]));
            }

        }

        /** brief Get the current belief states*/
        void getCurrentBeliefStates(std::vector<ompl::base::State*> &states)
        {
            states.clear();

            for(int i = 0; i < currentBeliefStates_.size(); i++)
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
        void generatePolicy(std::vector<ompl::control::Control*> &policy);

        /** \brief Runs the open loop policy on a given mode and outputs the cost*/
        ompl::base::Cost executeOpenLoopPolicyOnMode(std::vector<ompl::control::Control*> controls, const ompl::base::State* state);

        /** \brief advances the beliefs/modes by applying the given controls*/
        void propagateBeliefs(const ompl::control::Control *control);

        /** \brief Updates the weights of the Gaussians in the mixture */
        void updateWeights();

        arma::colvec computeInnovation(const arma::colvec Zprd, const arma::colvec Zg);

        void removeBelief(const int Indx);

        void printWeights() const
        {
            OMPL_INFORM("~~Printing Weights~~");
            for(unsigned int i=0; i < weights_.size(); i++)
            {
                std::cout<<"The Weight of mode #"<<i<<"  ->  "<<weights_[i]<<std::endl;
            }
            OMPL_INFORM("~~End Weights~~ \n");
        }

    private:

        /** \brief Returns true if all weights are same, false otherwise*/
        bool areSimilarWeights();

        /** \brief Container for the current modes/beliefs*/
        std::vector<ompl::base::State*> currentBeliefStates_;

        /** \brief The target location for each corresponding belief*/
        std::vector<ompl::base::State*> targetStates_;

        /** \brief Pointer to the state space information*/
        firm::SpaceInformation::SpaceInformationPtr si_;

        /** \brief Container for the previous open loop policy*/
        std::vector<ompl::control::Control*> previousPolicy_;

        /** \brief Container for the current weights of the beliefs */
        std::vector<float> weights_;

};
#endif
