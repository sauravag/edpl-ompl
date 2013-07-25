#ifndef FIRM_OMPL_TESTS_H
#define FIRM_OMPL_TESTS_H

#include "FIRMOMPL.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace arma;
using namespace std;

void TestSE2BeliefSpace()
{
    typedef SE2BeliefSpace::StateType BeliefStateType;
    SE2BeliefSpace *space;
    space =  new SE2BeliefSpace();

    space->sanityChecks();

    ob::RealVectorBounds bounds(2);
    bounds.setLow(-5);
    bounds.setHigh(5);

    space->setBounds(bounds);

    ob::State *from = space->allocState();
    ob::State *to   = space->allocState();

    from->as<SE2BeliefSpace::StateType>()->setXYYaw(1,1,3.14157);
    to->as<SE2BeliefSpace::StateType>()->setXYYaw(5,5,1.57);

    cout<<"The from State is: "<<from->as<SE2BeliefSpace::StateType>()->getX()<<endl;
    cout<<"The to state is: "<<to->as<SE2BeliefSpace::StateType>()->getX()<<endl;

    arma::mat testCov(3,3);

    testCov<<1<<2<<3<<endr
        <<1<<2<<3<<endr
        <<1<<2<<3<<endr;

    from->as<SE2BeliefSpace::StateType>()->setCovariance(testCov);
    to->as<SE2BeliefSpace::StateType>()->setCovariance(testCov*-1);

    ob::State *result = space->allocState();

    space->getRelativeState(from, to, result);

    cout<<"The Resultant State X    : "<<result->as<BeliefStateType>()->getX()<<endl;
    cout<<"The Resultant State Y    : "<<result->as<BeliefStateType>()->getY()<<endl;
    cout<<"The Resultant State Yaw  : "<<result->as<BeliefStateType>()->getYaw()<<endl;
    cout<<"The Resultant State Cov  : "<<result->as<BeliefStateType>()->getCovariance()<<endl;

    cout<<"-----------------------"<<endl;
    space->printState(result);
}
#endif
