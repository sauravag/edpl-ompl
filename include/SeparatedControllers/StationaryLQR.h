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

#ifndef STATIONARY_LQR_
#define STATIONARY_LQR_

#include "SeparatedControllerMethod.h"

/** \brief  Finite time LQR controller */
class StationaryLQR : public SeparatedControllerMethod
{

  public:
    typedef typename SeparatedControllerMethod::ControlType   ControlType;
    typedef typename MotionModelMethod::MotionModelPointer MotionModelPointer;
    //typedef typename MPTraits::LinearSystem   LinearSystem;

    StationaryLQR(){}

    StationaryLQR(ompl::base::State *goal,
        const std::vector<ompl::base::State*> &nominalXs,
        const std::vector<ompl::control::Control*> &nominalUs,
        const std::vector<LinearSystem>& linearSystems,
        const MotionModelPointer mm);

    ~StationaryLQR() {}

  ompl::control::Control* generateFeedbackControl(const ompl::base::State *state, const size_t& Ts = 0) ;
  
  private:

    /** \brief Generate the set of feedback gain by solving ricatti equation backwards*/
    void generateFeedbackGain();

    /** \brief The sequence of gains*/
    arma::mat feedbackGain_;

    /** \brief The cost weight matrix for terminal error, i.e., final state cost */
    arma::mat Wxf_;

    /** \brief intermediate state cost weight*/
    arma::mat Wx_;

    /** \brief Control cost weight matrix */
    arma::mat Wu_;

};
#endif
