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

#ifndef FIRM_OPTIMIZATION_OBJECTIVE_
#define FIRM_OPTIMIZATION_OBJECTIVE_

#include "ompl/base/OptimizationObjective.h"
#include "ompl/control/SpaceInformation.h"
#include "../Controllers/Controller.h"
#include "../SeparatedControllers/RHCICreate.h"
#include "../Filters/ExtendedKF.h"

/**
The cost to execute an edge i.e. going from some node A to some node B
is defined as the FIRM optimization objective. The cost is a measure of the
uncertainty in the system (e.g. trace(covariance) for all intermediate states on the edge).
FIRM gives us paths that have the least uncertainty.
*/
// We can template this class on the filter and controller
class FIRMOptimizationObjective : public ompl::base::OptimizationObjective
{
    public:

        FIRMOptimizationObjective(const ompl::control::SpaceInformationPtr &si);

        /** \brief Motion cost for this objective is defined as
            the sum of trace(covariance)for all intermediate states
            between \e s1 and \e s2, using the method SpaceInformation::distance(). */
        virtual ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2);


    protected:

        Controller<RHCICreate, ExtendedKF> controller_;


}
#endif
