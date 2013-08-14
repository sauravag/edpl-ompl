#ifndef KALMAN_FILTER_METHOD_
#define KALMAN_FILTER_METHOD_

#include "MotionModels/MotionModelMethod.h"
#include "ObservationModels/ObservationModelMethod.h"
#include "LinearSystem/LinearSystem.h"
#include "dare.h"
//#include "HState.h"


class KalmanFilterMethod 
{
  typedef arma::colvec ObservationType;
  typedef arma::colvec ControlType;
  typedef arma::colvec NoiseType;
  //typedef arma::mat JacobianType;
  typedef ObservationModelMethod::ObservationModelPointer ObservationModelPointer; 

	public:

  	KalmanFilterMethod() {}

  	KalmanFilterMethod (MotionModelPointer motionModel, ObservationModelPointer observationModel ) : 
  	motionModel_(motionModel), 
  	observationModel_(observationModel) {}
  	
  	//virtual ~KalmanFilterMethod() {}
  	
  	//gets a belief and control, returns predicted belief if control
  	//were to be applied
  	
  	MotionModelPointer GetMotionModelPointer(){return motionModel_;}
  	
  	void SetMotionModelPointeer(const MotionModelPointer& mm) { motionModel_ = mm;}
  	
  	ObservationModelPointer GetObservationModelPointer(){return observationModel_;}
  	
  	void SetObservationModelPointer(const ObservationModelPointer& om) { observationModel_ = om;}

  	virtual
  	ompl::base::State Predict(const ompl::base::State *belief,
  	const ControlType& control,
  	const LinearSystem& ls,
  	const bool isConstruction=false) = 0;

  	//gets a belief and observation, returns
  	virtual 
  	ompl::base::State Update(const ompl::base::State *belief,
  	const ObservationType& obs,
    const	LinearSystem& ls,
    const bool isConstruction=false) = 0;

    virtual
  	ompl::base::State Evolve(const ompl::base::State *belief,
  	const ControlType& control,
  	const ObservationType& obs, 
  	const LinearSystem& lsPred, 
  	const LinearSystem& lsUpdate,
  	const bool isConstruction=false) = 0;
  	
  	virtual
  	arma::mat ComputeStationaryCovariance(const LinearSystem& ls) = 0;


	protected:
	
  	MotionModelPointer motionModel_;
  	ObservationModelPointer observationModel_;

};


#endif

