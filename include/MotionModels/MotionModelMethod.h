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
*   * Neither the name of the Rice University nor the names of its
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

/////////////////////////////////////////////
// Abstract class interface for motion models
// This class defines the operations that can be performed on motion models,
// as well as the data they are expected to have
/////////////////////////////////////////////

#ifndef MOTIONMODELMETHOD_
#define MOTIONMODELMETHOD_

#include "armadillo"
#include "../Spaces/SE2BeliefSpace.h"

class MotionModelMethod
{


  public:

		typedef arma::colvec  ControlType;
		typedef arma::colvec  NoiseType;
        typedef SE2BeliefSpace SpaceType;
		typedef SpaceType::StateType StateType;
		typedef arma::mat     JacobianType;
        typedef boost::shared_ptr<MotionModelMethod> MotionModelPointer;

        ///////////////////////////////////////////
        //		Constructors and destructors 			 //
        ///////////////////////////////////////////
        //Default constructor
        //MotionModelMethod() : stateDim_(0), controlDim_(0), noiseDim_(0) {}


		MotionModelMethod(int sDim=0, int cDim=0, int nDim=0):
        //TODO: can the noise dimension actually be different from the control dimension?
			stateDim_(sDim),
			controlDim_(cDim),
			zeroControl_(arma::zeros<ControlType>(controlDim_)),
			noiseDim_(nDim),
			zeroNoise_(arma::zeros<NoiseType>(noiseDim_)),
			dt_(0.0) {}

        void setTimeStep(double timeStep){ dt_ = timeStep;}

		//Destructor
		virtual ~MotionModelMethod() {};


		//Produce the next state, given the current state, a control and a noise
		//Implementation is specific to particular motion model
		virtual void Evolve(const ompl::base::State *state, const ControlType& u, const NoiseType& w, ompl::base::State *result) = 0;

		//Generate open loop control between two specified Cfgs/states
		//Implementation is specific to particular motion model
		virtual std::vector<ControlType> generateOpenLoopControls(const ompl::base::State *startState,
                                                        const ompl::base::State *endState) = 0;



		//Generate noise according to specified state and control input
		//Appears to be common to all motion models
		virtual NoiseType generateNoise(const ompl::base::State *state, const ControlType& u) = 0;

		// df/dx
		virtual JacobianType
		getStateJacobian(const ompl::base::State *state, const ControlType& u, const NoiseType& w) = 0;
		// df/du
		virtual JacobianType
		getControlJacobian(const ompl::base::State *state, const ControlType& u, const NoiseType& w) = 0;
		// df/dw
		virtual JacobianType
		getNoiseJacobian(const ompl::base::State *state, const ControlType& u, const NoiseType& w) = 0;

		virtual arma::mat
		processNoiseCovariance(const ompl::base::State *state, const ControlType& u) = 0;

        // Get zero control which affects no change on the robot
		virtual const ControlType& getZeroControl() { return zeroControl_; }

		// Zero noise
		virtual const NoiseType& getZeroNoise()     { return zeroNoise_; }

        virtual const size_t controlDim()           { return controlDim_; }


        virtual double getTimestepSize() { return dt_; }

	protected:

		const int stateDim_;

		//control vector dimension is specific to each motion model subclass
		const int controlDim_;
		const ControlType zeroControl_;

		//noise vector dimension is specific to each motion model subclass
		const int noiseDim_;
		const NoiseType zeroNoise_;

		//timestep size, which is used to generate the next state by applying
		//a control for that period of time
		double dt_;

};

#endif
