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

#ifndef RHC_ICREATE_
#define RHC_ICREATE_

#include "SeparatedControllerMethod.h"
#include <deque>

class RHCICreate : public SeparatedControllerMethod
{

  public:
    typedef typename SeparatedControllerMethod::ControlType   ControlType;
    typedef typename MotionModelMethod::MotionModelPointer MotionModelPointer;
    //typedef typename MPTraits::LinearSystem   LinearSystem;

    RHCICreate() {}

    RHCICreate(ompl::base::State *goal,
        const std::vector<ompl::base::State*> &nominalXs,
        const std::vector<ompl::control::Control*> &nominalUs,
        const std::vector<LinearSystem>& linearSystems,  // Linear systems are not used in this class but it is here to unify the interface
        const MotionModelPointer mm) :
        SeparatedControllerMethod(goal, nominalXs, nominalUs, linearSystems, mm)
        {
          assert(controlQueueSize_ > 0 && "Error: RHCICreate control queue size not valid. Please initialize by calling SetControlQueueSize");

         //this->m_reachedFlag = false;
        }

    ~RHCICreate() {}

    virtual ompl::control::Control* generateFeedbackControl(const ompl::base::State *state, const size_t& _t = 0) ;

    static void setControlQueueSize(const int queueSize)
    {
      controlQueueSize_ = queueSize;
    }

    static void setTurnOnlyDistance(const double turnDist)
    {
      turnOnlyDistance_ = turnDist;
    }

   private:
    static int controlQueueSize_;
    static double turnOnlyDistance_;
    std::deque<ompl::control::Control*> openLoopControls_;
};
#endif
