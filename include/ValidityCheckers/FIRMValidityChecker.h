/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Texas A&M University
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
#include "../ObservationModels/ObservationModelMethod.h"
/*
Used to check if a state is in collission or not moreover,
in FIRM we need to know if a state is observable or not
before adding it to the graph.
*/

class FIRMValidityChecker : public ompl::base::StateValidityChecker
{
  public:
    typedef ObservationModelMethod::ObservationModelPointer ObservationModelPointer;
    typedef SE2BeliefSpace::StateType StateType;

    FIRMValidityChecker(const firm::SpaceInformation::SpaceInformationPtr &si/**, ObservationModelPointer om*/) :
    siF_(si), ompl::base::StateValidityChecker(si)/**, observationModel_(om)*/
    {
    }

    virtual bool isValid(const ompl::base::State *state) const
    {
      // states within a box are invalid
      int x_l =  3;
      int x_r =  15;
      int y_b =  5;
      int y_t =  15;

      arma::colvec pos = state->as<StateType>()->getArmaData();

      if(pos[0] >= x_l && pos[0] <= x_r )
      {
        if(pos[1] >= y_b && pos[1] <= y_t)
        {
            return false;
        }
      }
      return true;//siF_->getObservationModel()->isStateObservable(state);
    }

    protected:
        firm::SpaceInformation::SpaceInformationPtr siF_;
};
