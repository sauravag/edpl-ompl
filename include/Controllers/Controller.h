#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "SeparatedControllers/SeparatedControllerMethod.h"
#include "Filters/KalmanFilterMethod.h"
#include "MotionModels/MotionModelMethod.h"
#include "ObservationModels/ObservationModelMethod.h"
#include "Utilities/MPUtils.h"



template <class MPTraits, class SeparatedControllerType, class FilterType>
class Controller : public MPBaseObject<MPTraits> {

  public:
  
  typedef typename MotionModelMethod<MPTraits>::ControlType   ControlType;
	typedef typename MPTraits::CfgType CfgType;
  typedef typename MPTraits::MPProblemType MPProblemType;
	typedef typename ObservationModelMethod<MPTraits>::ObservationType ObservationType;
	typedef typename MPTraits::MPProblemType::MotionModelPointer MotionModelPointer;
	typedef typename MPTraits::MPProblemType::ObservationModelPointer ObservationModelPointer;
	typedef typename MPTraits::MPProblemType::ActuationSystemPointer ActuationSystemPointer;
	typedef typename MPTraits::MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
  //typedef typename MPTraits::LinearSystem<MPTraits>   LinearSystem;
	
	Controller() {};
	
  Controller(MPProblemType* _problem, 
      const CfgType& _goal,
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

template <class MPTraits, class SeparatedControllerType, class FilterType>
double Controller<MPTraits, SeparatedControllerType, FilterType>::m_nodeReachedAngle = -1;

template <class MPTraits, class SeparatedControllerType, class FilterType>
double Controller<MPTraits, SeparatedControllerType, FilterType>::m_nodeReachedDistance = -1;

template <class MPTraits, class SeparatedControllerType, class FilterType>
double Controller<MPTraits, SeparatedControllerType, FilterType>::m_maxTries = -1;

template <class MPTraits, class SeparatedControllerType, class FilterType>
Controller<MPTraits, SeparatedControllerType, FilterType>::
Controller(MPProblemType* _problem, const CfgType& _goal,
		  const std::vector<CfgType>& _nominalXs,
			const std::vector<ControlType>& _nominalUs,
      MotionModelPointer _mm,
			ObservationModelPointer _om,
			ActuationSystemPointer _as,
      string _vcLabel)
  : MPBaseObject<MPTraits>(_problem), m_goal(_goal),
    m_actuationSystem(_as),
    m_motionModel(_mm),
		m_observationModel(_om),
    m_vcLabel(_vcLabel)
		 {


  this->SetName("Controller");
  //assert(_nominalXs.size() == _nominalUs.size()+1);

  //create vector of linear systems
  
  m_lss.reserve(_nominalXs.size());
  
  for(size_t i=0; i<_nominalXs.size(); ++i) {
  
    LinearSystem<MPTraits> ls(_nominalXs[i], _nominalUs[i],	m_motionModel, m_observationModel);
    
    m_lss.push_back(ls);
  }


  //copy construct separated controller
  SeparatedControllerType sepController(_goal,
        _nominalXs, _nominalUs, m_lss, _mm);
  m_separatedController = sepController;
  
	FilterType filter(m_motionModel, m_observationModel);
	m_filter = filter;

	//lastNominalPoint = m_motionModel->Evolve(_nominalXs.back(),
	//_nominalUs.back(), m_motionModel->GetZeroNoise());
  m_tries = 0;
  
  //nominalXs is scaled by <1.2> to allow a bit more steps for robot to execute edge. Otherwise may not get good performance
  m_maxExecTime = ceil(_nominalXs.size()*3); 
                                              
}

template <class MPTraits, class SeparatedControllerType, class FilterType>
typename Controller<MPTraits, SeparatedControllerType, FilterType>::CfgType
Controller<MPTraits, SeparatedControllerType, FilterType>::Evolve(const CfgType& _b, size_t _t, bool _isConstructionMode) {
	
	//cout << "==========================================" << endl;
	//cout << "Timestep: " << _t << endl;
	//cout << "HState::: " << endl;
	//cout << "\t trueState: " << _h.m_trueState << endl;
	//cout << "\t belief: " << _h.m_belief << endl;
	
	//std::cout << "Do not forget is_reliable for feedback controls." << std::endl;
  ControlType u = m_separatedController.GenerateFeedbackControl(_b, _t);
  
	//cout << "Generated control: " << endl << u << endl;
	
	m_actuationSystem->ApplyControl(u);	
	
	ObservationType z = m_actuationSystem->GetObservation();

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
  
	CfgType nextBelief = m_filter.Evolve(_b, u, zCorrected,	current, next, _isConstructionMode);
	
	m_actuationSystem->SetBelief(nextBelief);
	
	//cout << "nextBelief: " << nextBelief << endl;
	//cout << "-----------***----***---***---------------" << endl;
	return nextBelief;

}


template <class MPTraits, class SeparatedControllerType, class FilterType>
bool
Controller<MPTraits, SeparatedControllerType, FilterType>::IsValid() {

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

template <class MPTraits, class SeparatedControllerType, class FilterType>
double
Controller<MPTraits, SeparatedControllerType, FilterType>::Execute(const CfgType& _b,
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
      
      if( m_actuationSystem->CheckCollision()) {
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

template <class MPTraits, class SeparatedControllerType, class FilterType>
double
Controller<MPTraits, SeparatedControllerType, FilterType>::Stabilize(const CfgType& _b, CfgType& _finalBelief) {

    
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

#endif 
