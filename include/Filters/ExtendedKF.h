#ifndef EXTENDED_KF_H_
#define EXTENDED_KF_H_

#include "KalmanFilterMethod.h"


template<class MPTraits>
class ExtendedKF : public  KalmanFilterMethod<MPTraits>  {



	public:
	
	typedef typename MotionModelMethod<MPTraits>::ControlType   ControlType;
	typedef typename MPTraits::CfgType CfgType;
   //	typedef typename MotionModelMethod::CfgType           CfgType;
	
	typedef typename ObservationModelMethod<MPTraits>::ObservationType ObservationType;
	typedef typename MPTraits::MPProblemType::MotionModelPointer MotionModelPointer;
	typedef typename MPTraits::MPProblemType::ObservationModelPointer ObservationModelPointer;	
        
	ExtendedKF() { }

	ExtendedKF(MotionModelPointer _motionModel,	ObservationModelPointer _observationModel);
	
	//gets a belief and control, returns predicted belief if control
	//were to be applied
	CfgType Predict(const CfgType& _belief,
	const ControlType& _control,
	const LinearSystem<MPTraits>& _ls, const bool _isConstruction=false);

	//gets a belief and observation, returns 
	CfgType Update(const CfgType& _belief,
	const ObservationType& _obs,
        const	LinearSystem<MPTraits>& _ls, const bool _isConstruction=false);

	CfgType Evolve(const CfgType& _belief,
	const ControlType& _control,
	const ObservationType& _obs,
	const LinearSystem<MPTraits>& _lsPred, 
	const LinearSystem<MPTraits>& _lsUpdate,
	const bool _isConstruction=false);
	
	
	arma::mat ComputeStationaryCovariance (const LinearSystem<MPTraits>& _ls){}

};

template <class MPTraits>
ExtendedKF<MPTraits>::ExtendedKF(MotionModelPointer _motionModel,
ObservationModelPointer _observationModel) :
KalmanFilterMethod<MPTraits>(_motionModel, _observationModel) {

}

template <class MPTraits>
typename ExtendedKF<MPTraits>::CfgType
ExtendedKF<MPTraits>::Predict(const CfgType& _belief, 
const typename MotionModelMethod<MPTraits>::ControlType& _control, 
const LinearSystem<MPTraits>& _ls,const bool _isConstruction=false) {

  using namespace arma;
  
	CfgType xPred = this->m_motionModel->Evolve(_belief, _control,
		this->m_motionModel->GetZeroNoise());

	mat covPred = _ls.GetA() * _belief.GetCovariance() * trans(_ls.GetA()) + 
		_ls.GetG() * _ls.GetQ() * trans(_ls.GetG());

	if(!_isConstruction)
  	{
	  ofstream myfile;
	  myfile.open("DataOut/PredictStepOutput.txt", std::fstream::app);
	  assert(myfile.is_open());
	  myfile <<"<--ThisStepOutput---->\n";
	  myfile <<"<-----Control-------->\n";
	  myfile <<_control<<endl;
	  myfile <<"<----Predicted Mean------>\n";
	  myfile <<xPred<<endl;
	  myfile <<"<----Predicted Covariance---->\n";
	  myfile <<covPred<<endl;  
	  //cout<<"Writing"<<endl;
	  myfile.close();
	}

	return CfgType(xPred, covPred);

}

template <class MPTraits>
typename ExtendedKF<MPTraits>::CfgType
ExtendedKF<MPTraits>::Update(const CfgType& _belief, const typename ObservationModelMethod<MPTraits>::ObservationType& _obs,
const LinearSystem<MPTraits>& _ls, const bool _isConstruction=false) {

  using namespace arma;

  CfgType xPred = _belief;

  colvec innov = this->m_observationModel->ComputeInnovation(xPred, _obs);
  
 // cout<<"innovation calculated"<<endl;
  if(!innov.n_rows || !innov.n_cols){
	return _belief; // return the prediction if you don't have any innovation
  }

  assert(innov.n_rows);
  assert(innov.n_cols);

  mat covPred = _belief.GetCovariance();
		
  mat rightMatrix = _ls.GetH() * covPred * trans(_ls.GetH()) + _ls.GetR(); 
  mat leftMatrix = covPred * trans(_ls.GetH());
  mat KalmanGain = solve(trans(rightMatrix), trans(leftMatrix)); 
  KalmanGain = KalmanGain.t();
  
  //cout<<"Kalman gain calculated"<<endl;

  colvec xPredVec = xPred.GetArmaData();

  colvec xEstVec = xPredVec + KalmanGain*innov;

  //cout<<"The innovation (in EKF update) :"<<endl<<innov<<endl;
  //cout<<"The Kalman Gain (in EKF update) :"<<endl<<KalmanGain<<endl;

  //cout<<"New State estimated"<<endl;

  CfgType xEst;
  xEst.SetArmaData(xEstVec);
	
  mat covEst = covPred - KalmanGain* _ls.GetH() * covPred;

  //------Printing Data to text file-----
  if(!_isConstruction)
  {
	  ofstream myfile;
	  myfile.open("DataOut/UpdateStepOutput.txt", std::fstream::app);
	  assert(myfile.is_open());

	  myfile <<"<--ThisStepOutput---->\n";
	  myfile <<"<---Observation------>\n";
	  myfile <<_obs<<endl;
	  myfile <<"<----Innovation------>\n";
	  myfile <<innov<<endl;
	  myfile <<"<-----Estimated Mean----->\n";
	  myfile << xEst <<endl;
	  myfile <<"<--- Estimated Covariance---->\n";
	  myfile <<covEst<<endl;  
	  //cout<<"Writing"<<endl;
	  myfile.close();
	}
  
  return CfgType(xEst, covEst);
	
}

template <class MPTraits>
typename ExtendedKF<MPTraits>::CfgType
ExtendedKF<MPTraits>::Evolve(const CfgType& _belief,
const typename MotionModelMethod<MPTraits>::ControlType& _control,
const typename ObservationModelMethod<MPTraits>::ObservationType& _obs,
const LinearSystem<MPTraits>& _lsPred, 
const LinearSystem<MPTraits>& _lsUpdate,
const bool _isConstruction=false) {

	// In the EKF we do not use the linear systems passed to the filter, instead we generate the linear systems on the fly

  using namespace arma;
  
  LinearSystem<MPTraits> lsPred(_belief, _control, this->m_motionModel, this->m_observationModel) ;

  CfgType bPred = Predict(_belief, _control, lsPred, _isConstruction);
	
	
  if(!_obs.n_rows || !_obs.n_cols) {
    return bPred;
  }
	
  LinearSystem<MPTraits> lsUpdate(bPred, _control, _obs, this->m_motionModel, this->m_observationModel) ;

  CfgType bEst = Update(bPred, _obs, lsUpdate, _isConstruction);

  //cout<<"Returning updated estimate form EKF"<<endl;

  return bEst;

}
#endif

