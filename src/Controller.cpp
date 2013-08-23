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

#include "../include/ActuationSystems/SimulatedActuationSystem.h"
#include "../include/Controllers/Controller.h"

template <class SeparatedControllerType, class FilterType>
double Controller<SeparatedControllerType, FilterType>::nodeReachedAngle_ = -1;

template <class SeparatedControllerType, class FilterType>
double Controller<SeparatedControllerType, FilterType>::nodeReachedDistance_ = -1;

template <class SeparatedControllerType, class FilterType>
double Controller<SeparatedControllerType, FilterType>::maxTries_ = -1;

template <class SeparatedControllerType, class FilterType>
Controller<SeparatedControllerType, FilterType>::Controller(const ompl::base::State *goal,
      const std::vector<ompl::base::State*>& nominalXs,
      const std::vector<ControlType>& nominalUs,
      MotionModelPointer mm,
      ObservationModelPointer om,
      ActuationSystemPointer as): goal_(goal),  actuationSystem_(as),  motionModel_(mm), observationModel_(om)
{
  //assert(_nominalXs.size() == _nominalUs.size()+1);

  //create vector of linear systems

  lss_.reserve(nominalXs.size());

  for(size_t i=0; i<nominalXs.size(); ++i)
  {

    LinearSystem ls(nominalXs[i], nominalUs[i], motionModel_, observationModel_);

    lss_.push_back(ls);
  }


  //copy construct separated controller
  SeparatedControllerType sepController(goal, nominalXs, nominalUs, lss_, mm);

  separatedController_ = sepController;

  FilterType filter(motionModel_, observationModel_);
  filter_ = filter;

  //lastNominalPoint = motionModel_->Evolve(_nominalXs.back(),
  //_nominalUs.back(), motionModel_->GetZeroNoise());
  tries_ = 0;

  //nominalXs is scaled by <3> to allow a bit more steps for robot to execute edge.
  //Otherwise may not get good performance
  maxExecTime_ = ceil(nominalXs.size()*3);

}

template <class SeparatedControllerType, class FilterType>
ompl::base::State*
Controller<SeparatedControllerType, FilterType>::Evolve(const ompl::base::State *state, size_t t, bool isConstructionMode)
{

  //cout << "==========================================" << endl;
  //cout << "Timestep: " << _t << endl;
  //cout << "HState::: " << endl;
  //cout << "\t trueState: " << _h.m_trueState << endl;
  //cout << "\t belief: " << _h.m_belief << endl;

  //std::cout << "Do not forget is_reliable for feedback controls." << std::endl;
  ControlType u = separatedController_.generateFeedbackControl(state, t);

  //cout << "Generated control: " << endl << u << endl;

  actuationSystem_->applyControl(u);

  ObservationType z = actuationSystem_->getObservation();

  //---- WARNING---//
  //TODO: The singleobservationdim needs to be retrieved from the observation model
  //---------------//
  //cout<<"The observations from actuation system are: "<<endl<<z<<endl;

  int singleobservationdim = 4;
  //cout<<" The construction mode is: "<<_isConstructionMode<<endl;

  //Adding obstacle markers
  obstacleMarkerObserved_ = false;  // !!---Defaulting to false for development phase---!!
  /*
  if(!isConstructionMode)
  {
    for(int i=0; i<z.n_rows/singleobservationdim; i++)
    {
      int markerID = z[i*singleobservationdim];
      //cout<<"Checking if marker ID :" <<markerID <<" Is an obstacle marker"<<endl;
      if(this->GetMPProblem()->IsObstacleMarker(markerID))
      {
        //add obstacle to environment if it doesn't already exist
        //flag that you have observed an obstacle marker
        //cout<<"Marker ID :"<<markerID<<" is an obstacle"<<endl;
        if(this->GetMPProblem()->AddObstacle(markerID))
          m_obstacleMarkerObserved = true;
      }
    }
  }
  */

  ObservationType zCorrected = observationModel_->removeSpuriousObservations(z);

  //cout << "Observation Z: " << endl << Z << endl;
  LinearSystem current;// = lss_[_t];
  LinearSystem next; //= lss_[_t+1];

  current = next = LinearSystem(goal_,
                                motionModel_->getZeroControl(),
                                zCorrected,
                                motionModel_,
                                observationModel_);

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

  ompl::base::StateSpacePtr space(new SpaceType());
  ompl::base::State *nextBelief = space->allocState();

  nextBelief = filter_.Evolve(state, u, zCorrected, current, next, isConstructionMode);

  actuationSystem_->setBelief(nextBelief);

  //cout << "nextBelief: " << nextBelief << endl;
  //cout << "-----------***----***---***---------------" << endl;
  return nextBelief;

}


template <class SeparatedControllerType, class FilterType>
bool Controller<SeparatedControllerType, FilterType>::isValid()
{
  /*
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
  */

  return true;
}

template <class SeparatedControllerType, class FilterType>
double Controller<SeparatedControllerType, FilterType>::Execute(const ompl::base::State *state,
                                                                bool& isFailed,
                                                                ompl::base::State *currentBelief,
                                                                bool constructionMode,
                                                                double sleepTime)
{

   currentBelief = state;

    int k = 0;
    /*
    cost = 1 - > for time based only if time per execution is "1"
    cost = 0.01 -> for covariance based
    */
    double cost = 0.01;
    //cout<<"!!-----Executing-----!!"<<endl;

    if(!this->isValid())
    {
      isFailed = true;
      return -2;
    }

    //cout<<"lss_ size is :" << lss_.size() <<endl;

    //cout<<"The robot goal is:  "<<goal_<<endl;
    while(!this->isTerminated(currentBelief, k))
    {

      usleep(sleepTime*1e6);
      //cout << "time: "<< k << endl;
      currentBelief = this->Evolve(currentBelief, k, constructionMode) ;

      if( actuationSystem_->checkCollision())
      {
        //cout << k << " of " << m_maxExecTime << " steps used." << endl;
        isFailed = true;
        return -1;
      }

      if(obstacleMarkerObserved_ == true && constructionMode)
      {
        if(!this->isValid())
        {
          isFailed = true;
          return -2;
        }

      }
      ompl::base::StateSpacePtr space(new SpaceType());
      ompl::base::State  *nominalX_K = space->allocState();

      if(k<lss_.size())
        nominalX_K = lss_[k].getX();

      else nominalX_K = lss_[lss_.size()-1].getX();

      arma::colvec deviation = (nominalX_K->as<StateType>()->getArmaData()).subvec(0,1) - currentBelief->as<StateType>()->getArmaData().subvec(0,1);

      // This value of 4 should not be hard coded
      if(norm(deviation,2) > 4.0)
      {
        isFailed = true;
        return -3;
      }

      k++;

      if(!constructionMode)
      {
        //this->GetMPProblem()->RemoveDecayedObstacles(); // check and remove decayed obstacles
      }
      /*
       Increment cost by:
       0.01 for time based
       trace of covariance for FIRM
      */
      cost += arma::trace(currentBelief->as<StateType>()->getCovariance());
    }
    //cout<<"The Filter estimate currently is:  "<< b.GetArmaData()(0)<<" "<< b.GetArmaData()(1)<<" "<<b.GetArmaData()(2)*180/PI<<endl;
    //cout << "Going on edge" << endl;
    //cout<<"!!-----End Executing-----!!"<<endl;
    obstacleMarkerObserved_ = false;
    return cost ;
}

template <class SeparatedControllerType, class FilterType>
double Controller<SeparatedControllerType, FilterType>::Stabilize(const ompl::base::State *startState,
                                                                  ompl::base::State *endState)
{
    /*
    int k =0;
    double cost=0;

    ompl::base::StateSpacePtr space(new SpaceType());
    ompl::base::State  *b = space->allocState();

    space->copyState(b, startState);
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
    return 0;
}

template <class SeparatedControllerType, class FilterType>
bool Controller<SeparatedControllerType, FilterType>::isTerminated(const ompl::base::State *state,
                                                                   const size_t t )
{

  using namespace arma;

  colvec diff = state->as<StateType>()->getArmaData() - goal_->as<StateType>()->getArmaData();

  double distance_to_goal = norm(diff.subvec(0,1),2);

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
