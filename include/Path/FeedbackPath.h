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
/* Author: Saurav Agarwal, Ali-akbar Agha-mohammadi */

#ifndef FIRM_FEEDBACK_PATH_
#define FIRM_FEEDBACK_PATH_

#include "ompl/base/Path.h"
#include "../SpaceInformation/SpaceInformation.h"
#include "../Controllers/Controller.h"
#include "../SeparatedControllers/RHCICreate.h"
#include "../Filters/ExtendedKF.h"

class FeedbackPath : public ompl::base::Path
{
    public:

        /** \brief Constructor, needs information about space */
        FeedbackPath(const firm::SpaceInformation::SpaceInformationPtr &si) : ompl::base::Path(si), siF_(si) {}

        //destructor
        ~FeedbackPath(void) {}



        /** \brief Append a state to the path, these states are the nodes to be visited */
        void append(const ompl::base::State *state)
        {
            ompl::base::State *temp = si_->allocState();
            si_->copyState(temp, state);
            nodesToVisit_.push_back(temp);
        }

        /** \brief Append a controller and its goal node to the path*/
        void append(const ompl::base::State *state, Controller<RHCICreate, ExtendedKF> controller)
        {
            ompl::base::State *temp = si_->allocState();
            si_->copyState(temp, state);
            nodesToVisit_.push_back(temp);
            feedbackControllers_.push_back(controller);
        }

        /** \brief Clear the feedback states and controllers */
        void clearFeedback()
        {
            feedbackControllers_.clear();
            nodesToVisit_.clear();
        }

        /** \brief Returns the sequence of controllers to follow in the path */
        std::vector< Controller<RHCICreate, ExtendedKF> > getControllers(void)
        {
            return feedbackControllers_;
        }

        std::vector<ompl::base::State*> getStates(void)
        {
            return nodesToVisit_;
        }

        std::size_t getStateCount(void)
        {
            return nodesToVisit_.size();
        }

        double length(void) const
        {
            return nodesToVisit_.size();
        }

        bool check(void) const
        {
            return true;
        }

        ompl::base::Cost cost(const OptimizationObjectivePtr &obj) const
        {
            ompl::base::Cost c(0);
            return c;
        }

        void print(std::ostream &out) const
        {
            for(unsigned int i=0;i<nodesToVisit_.size();i++)
            {
                siF_->printState(nodesToVisit_[i]);
            }
        }

    protected:

        /** \brief A vector containing the feedback controllers, which define the path in belief space */
        std::vector<Controller<RHCICreate, ExtendedKF> > feedbackControllers_;

        /** \brief The sequence of nodes to visit */
        std::vector<ompl::base::State*> nodesToVisit_;

        /** \brief Pointer to the belief space information */
        firm::SpaceInformation::SpaceInformationPtr siF_;

};
#endif // Feedback_PATH_
