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

template <class SeparatedControllerType, class FilterType>
double Controller<SeparatedControllerType, FilterType>::nodeReachedAngle_ = -1;

template <class SeparatedControllerType, class FilterType>
double Controller<SeparatedControllerType, FilterType>::nodeReachedDistance_ = -1;

template <class SeparatedControllerType, class FilterType>
double Controller<SeparatedControllerType, FilterType>::maxTries_ = -1;

template <class SeparatedControllerType, class FilterType>
Controller<SeparatedControllerType, FilterType>::
Controller(const ompl::base::State *goal,
      const std::vector<ompl::base::State*>& nominalXs,
      const std::vector<ControlType>& nominalUs,
      MotionModelPointer mm,
      ObservationModelPointer om): 
      goal_(goal),
      actuationSystem_(as),
      motionModel_(mm),
      observationModel_(om)
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

  //lastNominalPoint = m_motionModel->Evolve(_nominalXs.back(),
  //_nominalUs.back(), m_motionModel->GetZeroNoise());
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
  ControlType u = separatedController_.GenerateFeedbackControl(_b, _t);
  
  //cout << "Generated control: " << endl << u << endl;
  
  //-->>>>>>>actuationSystem_->ApplyControl(u); 
  
  ObservationType z = actuationSystem_->GetObservation();

  //---- WARNING---//
  //TODO: The singleobservationdim needs to be retrieved from the observation model
  //---------------//
  //cout<<"The observations from actuation system are: "<<endl<<z<<endl;
  int singleobservationdim = 4;
  //cout<<" The construction mode is: "<<_isConstructionMode<<endl;
  if(!_isConstructionMode) {
    for(int i=0; i<z.n_rows/singleobservationdim; i++){
      int markerID = z[i*singleobservationdim];
      //cout<<"Checking if marker ID :" <<markerID <<" Is an obstacle marker"<<endl;
      if(this->GetMPProblem()->IsObstacleMarker(markerID)) {
        //add obstacle to environment if it doesn't already exist
        //flag that you have observed an obstacle marker
        //cout<<"Marker ID :"<<markerID<<" is an obstacle"<<endl;
        if(this->GetMPProblem()->AddObstacle(markerID))
          m_obstacleMarkerObserved = true;        
      }
    }
  }
  
  ObservationType zCorrected = m_observationModel->RemoveSpuriousObservations(z);
  
  //cout << "Observation Z: " << endl << Z << endl;
  LinearSystem<MPTraits> current;// = m_lss[_t];
  LinearSystem<MPTraits> next; //= m_lss[_t+1];
                                  
  current = next = LinearSystem<MPTraits>(m_goal, //lastNominalPoint,
                                          m_motionModel->GetZeroControl(),
                                          zCorrected,
                                          m_motionModel,
                                          m_observationModel); 
  if( (Length() > 0) && (_t <= Length()-1) ){
        
    if( _t == Length() - 1 ) {
       current = m_lss[_t];
     } 
     else {
       current = m_lss[_t];
       next = m_lss[_t+1];
     }
  }
  
  CfgType nextBelief = m_filter.Evolve(_b, u, zCorrected, current, next, _isConstructionMode);
  
  actuationSystem_->SetBelief(nextBelief);
  
  //cout << "nextBelief: " << nextBelief << endl;
  //cout << "-----------***----***---***---------------" << endl;
  return nextBelief;

}


template <class SeparatedControllerType, class FilterType>
bool Controller<SeparatedControllerType, FilterType>::isValid() 
{

  ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
  Environment* env = this->GetMPProblem()->GetEnvironment();
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  string callee = this->GetName();
  CDInfo cdInfo;
  
  for(size_t i = 0; i < m_lss.size(); ++i) {
    CfgType x = m_lss[i].GetX();
    
    if(!x.InBoundary(env) || !vc->IsValid(x, env, *stats, cdInfo, &callee)) {
      return false;
    }
  }
  
  return true; 
}

template <class SeparatedControllerType, class FilterType>
double Controller<SeparatedControllerType, FilterType>::Execute(const CfgType& _b,
bool& _isFailed,
CfgType& _currentBelief,
bool constructionMode,
double _sleepTime) {
    
    _currentBelief = _b;

    int k = 0;
    double cost = 1;//0.01; //1: for time based only if time per execution is "1", 0.01 for covariance based
    //cout<<"!!-----Executing-----!!"<<endl;

    if(!this->IsValid()) {
      _isFailed = true;
      return -2;
    }
    //cout<<"m_lss size is :" << m_lss.size() <<endl;
    
    //cout<<"The robot goal is:  "<<m_goal<<endl;
    while(!this->IsTerminated(_currentBelief, k)) {
      
      usleep(_sleepTime*1e6);
      //cout << "time: "<< k << endl;
      _currentBelief = this->Evolve(_currentBelief, k, constructionMode) ;
      
      if( actuationSystem_->CheckCollision()) {
        //cout << k << " of " << m_maxExecTime << " steps used." << endl;
        _isFailed = true;
        return -1;
      }
      if(m_obstacleMarkerObserved == true && !constructionMode){
        if(!this->IsValid()) {
          _isFailed = true;
          return -2;
        }

      }

      CfgType nominalX_K;
      if(k<m_lss.size())
        nominalX_K = m_lss[k].GetX();

      else nominalX_K = m_lss[m_lss.size()-1].GetX();
      
      arma::colvec deviation = nominalX_K.GetArmaData().subvec(0,1) - _currentBelief.GetArmaData().subvec(0,1);

      if(norm(deviation,2) > 4.0){
        _isFailed = true;
        return -3;
      }
      
      k++;

      if(!constructionMode){
        this->GetMPProblem()->RemoveDecayedObstacles(); // check and remove decayed obstacles
      }
      // 0.01 for cost based and trace of covariance for FIRM
      cost += 0.01;//arma::trace(_currentBelief.GetCovariance());
    }
    //cout<<"The Filter estimate currently is:  "<< b.GetArmaData()(0)<<" "<< b.GetArmaData()(1)<<" "<<b.GetArmaData()(2)*180/PI<<endl;
    //cout << "Going on edge" << endl;
    //cout<<"!!-----End Executing-----!!"<<endl;
    m_obstacleMarkerObserved = false;
    return cost ;
}

template <class SeparatedControllerType, class FilterType>
double
Controller<SeparatedControllerType, FilterType>::Stabilize(const CfgType& _b, CfgType& _finalBelief) {

    
    int k =0;
    double cost=0;
    
    CfgType b = _b;
    cout<<"!!-----Stabilizing-----!!"<<endl;
    //cout<<"Controller.h -> stabilize : belief covariance" <<endl<<_b.m_covariance<<endl;
    //cout<<"Controller.h -> stabilize : goal covariance" <<endl<<m_goal.m_covariance<<endl;
    
    while(!m_goal.IsReached(b)) {
    
      //usleep(0.01*1e6); 
      //cout << "time: "<< k << endl;
      b = this->Evolve(b, k, true) ;
      k++;
    
      cost += arma::trace(b.GetCovariance());
    }
    //cout<<"The Filter estimate currently is:  "<< b.GetArmaData()(0)<<" "<< b.GetArmaData()(1)<<" "<<b.GetArmaData()(2)*180/PI<<endl;
    //cout << "Going on edge" << endl;
    _finalBelief = b ;
    cout<<"!!-----End Stabilizing-----!!"<<endl;
    return cost ;
}

template <class MPTraits, class SeparatedControllerType, class FilterType>
bool
Controller<MPTraits, SeparatedControllerType, FilterType>::IsTerminated(const CfgType& _b,
const size_t _t ) {

  using namespace arma;
  
  colvec diff = _b.GetArmaData() - m_goal.GetArmaData() ;
  
  double distance_to_goal = norm(diff.subvec(0,1),2);
  
  if( distance_to_goal > m_nodeReachedDistance)   {
      return false;
   }
   
   if( distance_to_goal <= m_nodeReachedDistance)   {
  
    if( abs(diff[2]) > m_nodeReachedAngle*PI/180 && m_tries < m_maxTries ){
      //cout<<"m_tries :"<<m_tries<<endl;
      m_tries++;
      return false;
    }
   
   }

  m_tries = 0;
  return true;
   
  
}
