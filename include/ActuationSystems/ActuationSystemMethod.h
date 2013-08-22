#ifndef ACTUATION_SYSTEM_METHOD_
#define ACTUATION_SYSTEM_METHOD_


class ActuationSystemMethod 
{

  public:
    typedef MotionModelMethod::SpaceType SpaceType;
    typedef MotionModelMethod::StateType StateType;
    typedef typename MotionModelMethod::ControlType ControlType;
    typedef typename ObservationModelMethod::ObservationType ObservationType;
    typedef boost::shared_ptr<ActuationSystemMethod> ActuationSystemPointer;


    ActuationSystemMethod() {}

    virtual void applyControl(ControlType& u) = 0;

    virtual ObservationType getObservation() = 0;

    virtual bool checkCollision() = 0;
    
    virtual void setBelief(const ompl::base::State *state) = 0;
     
    virtual void setTrueState(const ompl::base::State *state) = 0;
                
  //protected:
  
    //Environment* environment_;
};


#endif
