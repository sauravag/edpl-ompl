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

#ifndef FIRM_OMPL_
#define FIRM_OMPL_

// #define USE_ROS // REMOVE COMMENT TO PLAN WITH ROS, Access simulated/real sensor and robot through ROS

#include <iostream>
#include <fstream>

// OMPL includes
#include <ompl/base/SpaceInformation.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/DiscreteMotionValidator.h>

//Spaces
#include "Spaces/SE2BeliefSpace.h"
#include "SpaceInformation/SpaceInformation.h"

#ifdef USE_ROS
    #include "SpaceInformation/ROSSpaceInformation.h"
#endif

//Observation Models
#include "ObservationModels/ObservationModelMethod.h"
#include "ObservationModels/CamAruco2DObservationModel.h"

//Motion Models
#include "MotionModels/MotionModelMethod.h"
#include "MotionModels/UnicycleMotionModel.h"
#include "MotionModels/OmnidirectionalMotionModel.h"

//State Propagators
#include "MotionModels/UnicycleStatePropagator.h"

//LinearSystem
#include "LinearSystem/LinearSystem.h"

//Filters
#include "Filters/dare.h"
#include "Filters/KalmanFilterMethod.h"
#include "Filters/ExtendedKF.h"

//Separated Controllers
#include "SeparatedControllers/SeparatedControllerMethod.h"
#include "SeparatedControllers/RHCICreate.h"


//Controllers
#include "Controllers/Controller.h"

// Samplers
#include "Samplers/GaussianValidBeliefSampler.h"
#include "Samplers/UniformValidBeliefSampler.h"

// Validity checkers
#include "ValidityCheckers/FIRMValidityChecker.h"

//Multi-Modal
#include "Planner/NBM3P.h"

// FIRM Optimization Objective
//#include "OptimizationObjectives/FIRMOptimizationObjective.h"

//#include "Spaces/ICreateControlSampler.h"

// Utilities
#include "Utils/FIRMUtils.h"

// ROS
#ifdef USE_ROS
    #include "ros/ros.h"
#endif

#endif
