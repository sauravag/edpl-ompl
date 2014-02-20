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
#include "../SpaceInformation/SpaceInformation.h"

template <class SeparatedControllerType, class FilterType>
class Controller
{

    public:
        typedef MotionModelMethod::SpaceType SpaceType;
        typedef MotionModelMethod::StateType StateType;
        typedef firm::SpaceInformation::SpaceInformationPtr SpaceInformationPtr;
        typedef MotionModelMethod::ControlType   ControlType;
        typedef ObservationModelMethod::ObservationType ObservationType;
        typedef MotionModelMethod::MotionModelPointer MotionModelPointer;
        typedef ObservationModelMethod::ObservationModelPointer ObservationModelPointer;

        Controller() {};

        Controller(const ompl::base::State *goal,
                 const std::vector<ompl::base::State*>& nominalXs,
                 const std::vector<ompl::control::Control*>& nominalUs,
                 const firm::SpaceInformation::SpaceInformationPtr si);

        bool Execute(const ompl::base::State *startState,
                   ompl::base::State* endState,
                   ompl::base::Cost &executionCost,
                   bool constructionMode=true);

        ompl::base::State*  Stabilize(const ompl::base::State *startState);

        bool isTerminated(const ompl::base::State *state, const size_t t);

        void Evolve(const ompl::base::State *state, size_t t, ompl::base::State* nextState);

        ompl::base::State* getGoal() {return goal_; }

        void setSpaceInformation(SpaceInformationPtr si) { si_ = si; }

        bool isValid();

        static void setNodeReachedAngle(double angle) {nodeReachedAngle_ = angle; }
        static void setNodeReachedDistance(double d) {nodeReachedDistance_ = d; }
        static void setMaxTries(double maxtries) {maxTries_ = maxtries; }

        size_t Length() { return lss_.size(); }

    private:

        SpaceInformationPtr si_; // Instead of the actuation system, in OMPL we have the spaceinformation
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
		bool debug_;
		double nominalTrajDeviationThreshold_;

};

template <class SeparatedControllerType, class FilterType>
double Controller<SeparatedControllerType, FilterType>::nodeReachedAngle_ = -1;

template <class SeparatedControllerType, class FilterType>
double Controller<SeparatedControllerType, FilterType>::nodeReachedDistance_ = -1;

template <class SeparatedControllerType, class FilterType>
double Controller<SeparatedControllerType, FilterType>::maxTries_ = -1;

template <class SeparatedControllerType, class FilterType>
Controller<SeparatedControllerType, FilterType>::Controller(const ompl::base::State *goal,
            const std::vector<ompl::base::State*>& nominalXs,
            const std::vector<ompl::control::Control*>& nominalUs,
            const firm::SpaceInformation::SpaceInformationPtr si): si_(si)
{

  goal_ = si_->allocState();
  si_->copyState(goal_, goal);

  lss_.reserve(nominalXs.size());

  for(size_t i=0; i<nominalXs.size(); ++i)
  {

    LinearSystem ls(nominalXs[i], nominalUs[i], si_->getMotionModel(), si_->getObservationModel());

    lss_.push_back(ls);
  }

  //copy construct separated controller
  SeparatedControllerType sepController(goal_, nominalXs, nominalUs, lss_,si_->getMotionModel());

  separatedController_ = sepController;

  FilterType filter(si);
  filter_ = filter;

  //lastNominalPoint = actuationSystem_->getMotionModel()->Evolve(_nominalXs.back(),
  //_nominalUs.back(), actuationSystem_->getMotionModel()->GetZeroNoise());
  tries_ = 0;

  //nominalXs is scaled by <3> to allow a bit more steps for robot to execute edge.
  //Otherwise may not get good performance
  maxExecTime_ = ceil(nominalXs.size()*3);
  obstacleMarkerObserved_ = false;
  debug_ = false;
  nominalTrajDeviationThreshold_ = 4.0; // This value of 4 should not be hard coded

}


template <class SeparatedControllerType, class FilterType>
bool Controller<SeparatedControllerType, FilterType>::Execute(const ompl::base::State *startState,
                                                              ompl::base::State* endState,
                                                              ompl::base::Cost &executionCost,
                                                              bool constructionMode)
{
  using namespace std;

  unsigned int k = 0;

  /**HOW TO SET INITAL VALUE OF COST
    cost = 1 - > for time based only if time per execution is "1"
    cost = 0.01 -> for covariance based
  */
  double cost = 0.01;
  //cout<<"!!-----Executing-----!!"<<endl;

  ompl::base::State *internalState = si_->allocState();
  si_->copyState(internalState, startState);

  while(!this->isTerminated(endState, k))
  {
    //std::cout << "time: "<< k << std::endl;
    this->Evolve(internalState, k, endState) ;

    internalState = si_->cloneState(endState);

    // if the propagated state is not valid, return
    if( !si_->checkTrueStateValidity())
    {
      //cout << k << " of " << m_maxExecTime << " steps used." << endl;
      //isFailed = true;
      //failureCode = -1;
      return false;
    }

    ompl::base::State  *nominalX_K = si_->allocState();

    if(k<lss_.size())
      nominalX_K = lss_[k].getX();

    else nominalX_K = lss_[lss_.size()-1].getX();

    arma::colvec nomXVec = nominalX_K->as<StateType>()->getArmaData();
    arma::colvec endStateVec = endState->as<StateType>()->getArmaData();
    arma::colvec deviation = nomXVec.subvec(0,1) - endStateVec.subvec(0,1);

    if(debug_)
    {
        std::cout<<"The nominal trajectory point is:" <<nomXVec<<std::endl;
        std::cout<<"The current state is           :"<<endStateVec<<std::endl;
        std::cout<<"The deviation from nominal trajectory is: "<<abs(norm(deviation,2))<<std::endl;
        //std::cout<<"The size of LSS is :"<<lss_.size()<<std::endl;
        //std::cout<<"The k is : "<<k<<std::endl;
        std::cin.get();
    }
    if(abs(norm(deviation,2)) > nominalTrajDeviationThreshold_)
    {
      //isFailed = true;
      //failureCode = -3;
      return false;
    }

    k++;

    /**
     Increment cost by:
     -> 0.01 for time based
     -> trace(Covariance) for FIRM
    */
    cost += arma::trace(endState->as<StateType>()->getCovariance());

    if(debug_) cout<<"Trace of covariance: "<<arma::trace(endState->as<StateType>()->getCovariance())<<std::endl;
  }

  executionCost.v = cost;

  return true ;
}

template <class SeparatedControllerType, class FilterType>
void
Controller<SeparatedControllerType, FilterType>::
Evolve(const ompl::base::State *state, size_t t, ompl::base::State* nextState)
{

  //std::cout << "Do not forget is_reliable for feedback controls." << std::endl;
  ompl::control::Control* control = separatedController_.generateFeedbackControl(state/*, t*/);

  si_->applyControl(control);

  //---------------//
  //cout<<"The observations from actuation system are: "<<endl<<z<<endl;

  unsigned int singleobservationdim = 4;
  //cout<<" The construction mode is: "<<_isConstructionMode<<endl;

  ObservationType zCorrected = si_->getObservation();

  //cout << "Observation Z: " << endl << Z << endl;
  LinearSystem current;// = lss_[_t];
  LinearSystem next; //= lss_[_t+1];

  current = next = LinearSystem(goal_,
                                si_->getMotionModel()->getZeroControl(),
                                zCorrected,
                                si_->getMotionModel(),
                                si_->getObservationModel());

  if( (Length() > 0) && (t <= Length()-1) )
  {

    if( t == Length() - 1 )
    {
      current = lss_[t];
    }
    else
    {
      current = lss_[t];
      next = lss_[t+1];
    }
  }

  ompl::base::State *nextBelief = si_->allocState();

  filter_.Evolve(state, control, zCorrected, current, next, nextBelief);

  si_->copyState(nextState, nextBelief);
  si_->setBelief(nextBelief);

  //cout << "nextBelief: " << nextBelief << endl;
  //cout << "-----------***----***---***---------------" << endl;

}


template <class SeparatedControllerType, class FilterType>
ompl::base::State* Controller<SeparatedControllerType, FilterType>::Stabilize(const ompl::base::State *startState)
{

    int k =0;
    double cost=0;

    ompl::base::StateSpacePtr space(new SpaceType());
    ompl::base::State  *b = space->allocState();

    space->copyState(b, startState);
     /*
    cout<<"!!-----Stabilizing-----!!"<<endl;
    //cout<<"Controller.h -> stabilize : belief covariance" <<endl<<_b.m_covariance<<endl;
    //cout<<"Controller.h -> stabilize : goal covariance" <<endl<<goal_.m_covariance<<endl;

    while(!goal_.IsReached(b))
    {
      //usleep(0.01*1e6);
      //cout << "time: "<< k << endl;
      b = this->Evolve(startState, k, true) ;
      k++;

      cost += arma::trace(b.GetCovariance());
    }
    //cout<<"The Filter estimate currently is:  "<< b.GetArmaData()(0)<<" "<< b.GetArmaData()(1)<<" "<<b.GetArmaData()(2)*180/PI<<endl;
    //cout << "Going on edge" << endl;
    _finalBelief = b ;
    cout<<"!!-----End Stabilizing-----!!"<<endl;
    return cost ;
    */
    return b;
}

template <class SeparatedControllerType, class FilterType>
bool Controller<SeparatedControllerType, FilterType>::isTerminated(const ompl::base::State *state,
                                                                   const size_t t )
{

  using namespace arma;

  colvec diff = state->as<StateType>()->getArmaData() - goal_->as<StateType>()->getArmaData();

  double distance_to_goal = norm(diff.subvec(0,1),2);

  //std::cout<<"The distance to goal is: "<<distance_to_goal<<std::endl;
  //std::usleep(1e5);

  if( distance_to_goal > nodeReachedDistance_)   {
      return false;
   }

   if( distance_to_goal <= nodeReachedDistance_)
   {

    if( abs(diff[2]) > nodeReachedAngle_*boost::math::constants::pi<double>()/180 && tries_ < maxTries_ )
    {
      //cout<<"m_tries :"<<m_tries<<endl;
      tries_++;
      return false;
    }

   }

  tries_ = 0;
  return true;

}


/*
template <class SeparatedControllerType, class FilterType>
bool Controller<SeparatedControllerType, FilterType>::isValid()
{

  ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
  Environment* env = this->GetMPProblem()->GetEnvironment();
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  string callee = this->GetName();
  CDInfo cdInfo;

  for(size_t i = 0; i < lss_.size(); ++i) {
    CfgType x = lss_[i].GetX();

    if(!x.InBoundary(env) || !vc->IsValid(x, env, *stats, cdInfo, &callee)) {
      return false;
    }
  }


  return true;
}
*/

#endif
