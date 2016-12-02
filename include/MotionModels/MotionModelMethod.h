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

/* Authors: Ali-akbar Agha-mohammadi, Saurav Agarwal*/

#ifndef MOTIONMODELMETHOD_
#define MOTIONMODELMETHOD_

#include <armadillo>
#include "Spaces/SE2BeliefSpace.h"
#include <ompl/control/Control.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>

/**
 \brief Abstract class interface for defining a motion model.
**/

class MotionModelMethod
{


  public:

		typedef arma::colvec  ControlType;
		typedef arma::colvec  NoiseType;
        typedef SE2BeliefSpace SpaceType;
		typedef SpaceType::StateType StateType;
		typedef arma::mat     JacobianType;
        typedef boost::shared_ptr<MotionModelMethod> MotionModelPointer;

        /** \brief Constructor */
		MotionModelMethod(const ompl::control::SpaceInformationPtr si, int nDim=0):
            si_(si),
			stateDim_(si->getStateDimension()),
			controlDim_((si->getControlSpace())->getDimension()),
			noiseDim_(nDim),
            zeroNoise_(arma::zeros<NoiseType>(noiseDim_)),
			dt_(0.0)
        {
            zeroControl_ = si_->allocControl();
            for(unsigned int i = 0; i < controlDim_ ; i++)
            {
                zeroControl_->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i] = 0;
            }

        }

        /** \brief Set the timestep for motion. */
        void setTimeStep(double timeStep){ dt_ = timeStep;}

		/** \brief Destructor */
		virtual ~MotionModelMethod() {};


		/** \brief Propagate the system to the next state, given the current state, a control and a noise. */
		virtual void Evolve(const ompl::base::State *state, const ompl::control::Control *control, const NoiseType& w, ompl::base::State *result) = 0;

		/** \brief  Generate open loop control that drives robot from start to end state. */
		virtual void generateOpenLoopControls(const ompl::base::State *startState,
                                              const ompl::base::State *endState,
                                              std::vector<ompl::control::Control*> &openLoopControls) = 0;

        /** \brief Generate open loop controls for a geometric path. */
        virtual void generateOpenLoopControlsForPath(const ompl::geometric::PathGeometric path,
                                              std::vector<ompl::control::Control*> &openLoopControls) = 0;

		/** \brief Generate noise according to specified state and control input. */
		virtual NoiseType generateNoise(const ompl::base::State *state, const ompl::control::Control* control) = 0;

		/** \brief Calculate the state transition Jacobian i.e. df/dx where f is the transition function and x is the state. */
		virtual JacobianType
		getStateJacobian(const ompl::base::State *state, const ompl::control::Control* control, const NoiseType& w) = 0;

        /** \brief Calculate the control transition Jacobian i.e. df/du where f is the transition function and u is the control. */
		virtual JacobianType
		getControlJacobian(const ompl::base::State *state, const ompl::control::Control* control, const NoiseType& w) = 0;

        /** \brief Calculate the noise transition Jacobian i.e. df/dw where f is the transition function and w is the noise. */
		virtual JacobianType
		getNoiseJacobian(const ompl::base::State *state, const ompl::control::Control* control, const NoiseType& w) = 0;

        /** \brief Calculate the process noise covariance. */
		virtual arma::mat
		processNoiseCovariance(const ompl::base::State *state, const ompl::control::Control* control) = 0;

        /** \brief Get zero control which produces no change in the robot state. */
		virtual ompl::control::Control* getZeroControl() { return zeroControl_; }

		/** \brief Get the zero noise. */
		virtual const NoiseType& getZeroNoise()
		{
            return zeroNoise_;
        }

        /** \brief Get the control dimension. */
        virtual const size_t controlDim()           { return controlDim_; }

        /** \brief Get the time step value. */
        virtual double getTimestepSize() { return dt_; }

        /** \brief Convert a control from OMPL format to armadillo vector. */
        arma::colvec OMPL2ARMA(const ompl::control::Control *control)
        {

            arma::colvec u(controlDim_);

            if(!control) control = si_->allocControl();

            const double *conVals = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

            for (unsigned int i = 0; i < controlDim_; i++)
            {
                u[i] = conVals[i];
            }

            return u;
        }

        /** \brief Convert a control from aradillo vector to ompl::control::Control* . */
        void ARMA2OMPL(arma::colvec u, ompl::control::Control *control)
        {
            if(!control) control = si_->allocControl();

            for (unsigned int i = 0; i < controlDim_; i++)
            {
                control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i] = u[i];
            }
        }

	protected:

        /** \brief A pointer to the space information. */
	    ompl::control::SpaceInformationPtr si_;

        /** \brief Dimension of the system state. */
		const unsigned int stateDim_;

		/** \brief control vector dimension is specific to each motion model subclass. */
		const unsigned int controlDim_;

        /** \brief Zero control. */
		ompl::control::Control* zeroControl_;

		/** \brief noise vector dimension is specific to each motion model subclass. */
		const int noiseDim_;

        /** \brief Zero noise. */
		NoiseType zeroNoise_;

		/** \brief timestep size, used to generate the next state by applying a control for this period of time. */
		double dt_;

};

#endif
