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

#ifndef FIRM_OMPL_
#define FIRM_OMPL_

#include <iostream>
#include <fstream>

// OMPL includes
#include <ompl/base/SpaceInformation.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/DiscreteMotionValidator.h>

//Spaces
#include "include/Spaces/SE2BeliefSpace.h"
#include "include/SpaceInformation/SpaceInformation.h"

//Observation Models
#include "include/ObservationModels/ObservationModelMethod.h"
#include "include/ObservationModels/CamAruco2DObservationModel.h"

//Motion Models
#include "include/MotionModels/MotionModelMethod.h"
#include "include/MotionModels/UnicycleMotionModel.h"

//State Propagators
#include "include/MotionModels/UnicycleStatePropagator.h"

//LinearSystem
#include "include/LinearSystem/LinearSystem.h"

//Filters
#include "include/Filters/dare.h"
#include "include/Filters/KalmanFilterMethod.h"
#include "include/Filters/ExtendedKF.h"

//Separated Controllers
#include "include/SeparatedControllers/SeparatedControllerMethod.h"
#include "include/SeparatedControllers/RHCICreate.h"

//ActuationSystems
//#include "include/ActuationSystems/ActuationSystemMethod.h"
//#include "include/ActuationSystems/SimulatedActuationSystem.h"

//Controllers
#include "include/Controllers/Controller.h"

// Samplers
//#include "include/Samplers/GaussianValidBeliefSampler.h"
//#include "include/Samplers/UniformValidBeliefSampler.h"

// Validity checkers
#include "include/ValidityCheckers/FIRMValidityChecker.h"

// FIRM Optimization Objective
//#include "include/OptimizationObjectives/FIRMOptimizationObjective.h"

#endif
