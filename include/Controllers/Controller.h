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

#ifndef CONTROLLER_
#define CONTROLLER_

#include "../SeparatedControllers/SeparatedControllerMethod.h"
#include "../Filters/KalmanFilterMethod.h"
#include "../MotionModels/MotionModelMethod.h"
#include "../ObservationModels/ObservationModelMethod.h"


template <class SeparatedControllerType, class FilterType>
class Controller
{

  public:
    typedef MotionModelMethod::SpaceType SpaceType;
    typedef MotionModelMethod::StateType StateType;
    typedef typename MotionModelMethod::ControlType   ControlType;
  	typedef typename ObservationModelMethod::ObservationType ObservationType;
  	typedef typename MotionModelMethod::MotionModelPointer MotionModelPointer;
  	typedef typename ObservationModelMethod::ObservationModelPointer ObservationModelPointer;
  	typedef typename ActuationSystemMethod::ActuationSystemPointer ActuationSystemPointer;

	//Controller() {};

  Controller(const ompl::base::State *goal,
		     const std::vector<ompl::base::State*>& nominalXs,
			const std::vector<ControlType>& nominalUs,
            MotionModelPointer mm,
			ObservationModelPointer om,
			ActuationSystemPointer as);

  double  Execute(const ompl::base::State *startState, bool& isFailed,
                ompl::base::State *endState, bool constructionMode=true, double sleepTime=0.0);

  double  Stabilize(const ompl::base::State *startState, ompl::base::State *endState);

  bool    isTerminated(const ompl::base::State *state, const size_t t);

  ompl::base::State* Evolve(const ompl::base::State *state, size_t t, bool isConstructionMode);

  ompl::base::State* getGoal() {return goal_; }

  void setActuationSystem(ActuationSystemPointer as) { actuationSystem_ = as ; }

  bool isValid();


/*
  double ControllerCost(const CfgType& _x, const size_t& _t) {

			assert(!"Not supported yet!");
	}
*/
  static void setNodeReachedAngle(double angle) {nodeReachedAngle_ = angle; }
  static void setNodeReachedDistance(double d) {nodeReachedDistance_ = d; }
  static void setMaxTries(double maxtries) {maxTries_ = maxtries; }

  size_t Length() { return lss_.size(); }

  private:
    MotionModelPointer motionModel_;
		ObservationModelPointer observationModel_;
		ActuationSystemPointer actuationSystem_;
		std::vector<LinearSystem> lss_;
		SeparatedControllerType separatedController_;
		FilterType filter_;
		ompl::base::State *goal_;   // last nominal point
		int tries_;
		static double nodeReachedAngle_;
		static double nodeReachedDistance_;
		static double maxTries_;
		double maxExecTime_;
		bool obstacleMarkerObserved_;

};


#endif
