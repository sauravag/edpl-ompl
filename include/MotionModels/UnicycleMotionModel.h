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

#ifndef UNICYCLE_MOTIONMODEL_
#define UNICYCLE_MOTIONMODEL_

//#include "ValidityChecker.hpp"
#include "MotionModelMethod.h"
#include <limits>
#include <cassert>

class UnicycleMotionModel : public MotionModelMethod
{

    //Dimensions of stat vector, control vector, motion noise vector
    //These are constant and specific to the motion model. This also implies
    //the motion model will only work with configurations of a particular dimension
    static const int stateDim = 3;
    static const int controlDim = 2;
    static const int motionNoiseDim = 5;

  public:
    typedef typename MotionModelMethod::ControlType ControlType;
    typedef typename MotionModelMethod::NoiseType NoiseType;
    typedef typename MotionModelMethod::JacobianType JacobianType;

    // XML-based constructor
    UnicycleMotionModel(const ompl::control::SpaceInformationPtr si, const char *pathToSetupFile) :
    MotionModelMethod(si, motionNoiseDim)
    {

       //In here, should check if the cfg type is compatible with this
       //motion model, and exit with error otherwise
        this->loadParameters(pathToSetupFile);

    }

    /** \brief Destructor. */
    ~UnicycleMotionModel() {}

    /** \brief Propagate the system to the next state, given the current state, a control and a noise. */
    void Evolve(const ompl::base::State *state, const ompl::control::Control *control, const NoiseType& w, ompl::base::State *result);


    /** \brief  Generate open loop control that drives robot from start to end state. */
    void generateOpenLoopControls(const ompl::base::State *startState,
                                                  const ompl::base::State *endState,
                                                  std::vector<ompl::control::Control*> &openLoopControls);

    /** \brief Generate noise according to specified state and control input. */
    NoiseType generateNoise(const ompl::base::State *state, const ompl::control::Control* control);

    /** \brief Calculate the state transition Jacobian i.e. df/dx where f is the transition function and x is the state. */
    JacobianType getStateJacobian(const ompl::base::State *state, const ompl::control::Control* control, const NoiseType& w);
    
    /** \brief Calculate the control transition Jacobian i.e. df/du where f is the transition function and u is the control. */
    JacobianType getControlJacobian(const ompl::base::State *state, const ompl::control::Control* control, const NoiseType& w);
    
    /** \brief Calculate the noise transition Jacobian i.e. df/dw where f is the transition function and w is the noise. */
    JacobianType getNoiseJacobian(const ompl::base::State *state, const ompl::control::Control* control, const NoiseType& w);

    /** \brief Calculate the process noise covariance. */    
    arma::mat processNoiseCovariance(const ompl::base::State *state, const ompl::control::Control* control);

  private:

    /** \brief Generate the control noise covariance.*/
    arma::mat controlNoiseCovariance(const ompl::control::Control* control);

    /** \brief Load parameters from XML. */
    void loadParameters(const char *pathToSetupFile);

    /** \brief Bias standard deviation of the motion noise */
    arma::colvec sigma_; // 
    
    /** \brief Proportional standard deviation of the motion noise */
    arma::colvec eta_; // 
    
    /** \brief  Covariance of state additive noise */
    arma::mat    P_Wg_; //

    /** \brief max rotational velocity */
    double maxAngularVelocity_; //
    
    /** \brief max translational velocity */
    double maxLinearVelocity_; //
    
    /** \brief max translational velocity */
    double orbitRadius_;
    
    /** \brief min translational velocity */
    double minLinearVelocity_;

};

#endif
