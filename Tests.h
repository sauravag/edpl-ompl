#ifndef FIRM_OMPL_TESTS_
#define FIRM_OMPL_TESTS_

#include "FIRMOMPL.h"
#include "tinyxml.h"

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
    cout<<"State Space Passed Tests"<<endl;
}

void TestObservationModel()
{
    CamAruco2DObservationModel om( "/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml" );

    typedef SE2BeliefSpace::StateType StateType;
    SE2BeliefSpace *space;
    space =  new SE2BeliefSpace();

    ob::RealVectorBounds bounds(2);
    bounds.setLow(-5);
    bounds.setHigh(5);

    space->setBounds(bounds);

    ob::State *from = space->allocState();

    from->as<StateType>()->setXYYaw(1.3,3,0);

    cout<<"The from State is: "<<from->as<StateType>()->getX()<<endl;

    ObservationModelMethod::ObservationType obs =  om.getObservation(from, true);

    cout<<"The observation is: "<<obs<<endl;

    ObservationModelMethod::ObservationType pred_obs =  om.getObservationPrediction(from, obs);



    assert(obs.n_rows == pred_obs.n_rows && obs.n_cols == pred_obs.n_cols);
    // Check if the IDs of the predicted and seen observations match.
    for(int i=0; i<obs.n_rows/4; i++)
    {
        //cout<<"The observation is :"<< obs[i*4] <<endl;
        //cout<<"The predicted observation is :"<< pred_obs[i*4]<<endl;
        assert(obs[i*4] ==  pred_obs[i*4]);
    }

    cout<<"Observation Model passed tests"<<endl;
}
#endif
