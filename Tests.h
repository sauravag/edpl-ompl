#ifndef FIRM_OMPL_TESTS_
#define FIRM_OMPL_TESTS_

#include "FIRMOMPL.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace arma;
using namespace std;

void TestSE2BeliefSpace()
{
    typedef SE2BeliefSpace::StateType StateType;
    SE2BeliefSpace *space;
    space =  new SE2BeliefSpace();

    space->sanityChecks();

    ob::RealVectorBounds bounds(2);
    bounds.setLow(-5);
    bounds.setHigh(5);

    space->setBounds(bounds);

    ob::State *from = space->allocState();
    ob::State *to   = space->allocState();

    from->as<StateType>()->setXYYaw(1,1,3.14157);
    to->as<StateType>()->setXYYaw(5,5,1.57);

    cout<<"The from State is: "<<from->as<StateType>()->getX()<<endl;
    cout<<"The to state is: "<<to->as<StateType>()->getX()<<endl;

    arma::mat testCov(3,3);

    testCov<<1<<2<<3<<endr
        <<1<<2<<3<<endr
        <<1<<2<<3<<endr;

    from->as<StateType>()->setCovariance(testCov);
    to->as<StateType>()->setCovariance(testCov*-1);

    ob::State *result = space->allocState();
    space->getRelativeState(from, to, result);

    cout<<"The Resultant State X    : "<<result->as<StateType>()->getX()<<endl;
    cout<<"The Resultant State Y    : "<<result->as<StateType>()->getY()<<endl;
    cout<<"The Resultant State Yaw  : "<<result->as<StateType>()->getYaw()<<endl;
    cout<<"The Resultant State Cov  : "<<result->as<StateType>()->getCovariance()<<endl;

    cout<<"-----------------------"<<endl;
    space->printState(result);
}

void TestObservationModel()
{
    
}
#endif
