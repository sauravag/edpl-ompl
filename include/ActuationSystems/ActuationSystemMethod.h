#ifndef ACTUATION_SYSTEM_METHOD_
#define ACTUATION_SYSTEM_METHOD_

#include "../MotionModels/MotionModelMethod.h"
#include "../ObservationModels/ObservationModelMethod.h"

class ActuationSystemMethod
{

  public:
    typedef MotionModelMethod::SpaceType SpaceType;
    typedef MotionModelMethod::StateType StateType;
    typedef typename MotionModelMethod::ControlType ControlType;
    typedef typename ObservationModelMethod::ObservationType ObservationType;
    typedef boost::shared_ptr<ActuationSystemMethod> ActuationSystemPointer;
    typedef ObservationModelMethod::ObservationModelPointer ObservationModelPointer;
    typedef MotionModelMethod::MotionModelPointer MotionModelPointer;


    ActuationSystemMethod() {}

    ActuationSystemMethod(MotionModelPointer mm, ObservationModelPointer om):motionModel_(mm), observationModel_(om){}

    virtual void applyControl(ControlType& u) = 0;

    virtual ObservationType getObservation() = 0;

    virtual bool checkCollision() = 0;

    virtual void setBelief(const ompl::base::State *state) = 0;

    virtual void setTrueState(const ompl::base::State *state) = 0;

    virtual ompl::base::State* getTrueState() = 0;

    virtual MotionModelPointer getMotionModel() = 0;
    virtual ObservationModelPointer getObservationModel() = 0;

  protected:

    //Environment* environment_;
    MotionModelPointer motionModel_;
    ObservationModelPointer observationModel_;

};


#endif
