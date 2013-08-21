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

#include "SeparatedControllers/SeparatedControllerMethod.h"
#include "Filters/KalmanFilterMethod.h"
#include "MotionModels/MotionModelMethod.h"
#include "ObservationModels/ObservationModelMethod.h"



class Controller 
{

  public:
  
  typedef typename MotionModelMethod::ControlType   ControlType;
	typedef typename ObservationModelMethod::ObservationType ObservationType;
	typedef typename MotionModelMethod::MotionModelPointer MotionModelPointer;
	typedef typename ObservationModelMethod::ObservationModelPointer ObservationModelPointer;
	
	Controller() {};
	
  Controller(const CfgType& _goal,
		  const std::vector<CfgType>& _nominalXs,
			const std::vector<ControlType>& _nominalUs,
      MotionModelPointer _mm,
			ObservationModelPointer _om,
			ActuationSystemPointer _as,
      string _vcLabel);

  double  Execute(const CfgType& _b, bool& _isFailed, CfgType& _endBelief,bool constructionMode=true, double _sleepTime=0.0);
  
  double  Stabilize(const CfgType& _b, CfgType& _finalBelief);
  
  bool    IsTerminated(const CfgType& _b, const size_t _t);
  
  CfgType Evolve(const CfgType& _b, size_t _t, bool _isConstructionMode);
  
  CfgType GetGoal() {return m_goal; }
  
  void SetActuationSystem(ActuationSystemPointer _as) { m_actuationSystem = _as ; }

  bool IsValid();


/*
  double ControllerCost(const CfgType& _x, const size_t& _t) {
  
			assert(!"Not supported yet!");
	}
*/
  static void SetNodeReachedAngle(double _angle) {m_nodeReachedAngle = _angle; }
  static void SetNodeReachedDistance(double _d) {m_nodeReachedDistance = _d; }
  static void SetMaxTries(double _maxtries) {m_maxTries = _maxtries; }
  
  size_t Length() { return m_lss.size(); }
    
  private:
    MotionModelPointer m_motionModel;
		ObservationModelPointer m_observationModel;
		ActuationSystemPointer m_actuationSystem;
		vector< LinearSystem<MPTraits> > m_lss;
		SeparatedControllerType m_separatedController;
		FilterType m_filter;
		CfgType m_goal;   // last nominal point
		int m_tries;
		static double m_nodeReachedAngle;
		static double m_nodeReachedDistance;
		static double m_maxTries;
		double m_maxExecTime;
		bool m_obstacleMarkerObserved;
    string m_vcLabel;

};


#endif 
