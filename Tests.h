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
    space->printBeliefState(result);
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


void TestMotionModel()
{
    UnicycleMotionModel mm( "/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml" );

    typedef SE2BeliefSpace::StateType StateType;
    SE2BeliefSpace *space;
    space =  new SE2BeliefSpace();

    ob::RealVectorBounds bounds(2);
    bounds.setLow(-5);
    bounds.setHigh(5);

    space->setBounds(bounds);

    ob::State *from = space->allocState();

    from->as<StateType>()->setXYYaw(1.3,3,0);

    ob::State *to = space->allocState();

    to->as<StateType>()->setXYYaw(5,3,1.57);

    cout<<"The from State is: "<<from->as<StateType>()->getX()<<endl;

    colvec u(2);
    u[0] = 0.1;
    u[1] = 0.1;


    colvec noise = mm.generateNoise(from, u);

    cout<<"Noise :"<<noise<<endl;

    mat stateJacobian = mm.getStateJacobian(from, u , noise);

    mat controlJacobian = mm.getControlJacobian(from, u, noise);

    mat noiseJacobian = mm.getNoiseJacobian(from, u, noise);

    mat processNoiseCovariance = mm.processNoiseCovariance(from , u);

    vector<colvec> openLoopControls = mm.generateOpenLoopControls(from, to);

    ompl::base::State *nextState =  from;

    for(int i=0; i< openLoopControls.size() ; i++)
    {
        colvec w = mm.generateNoise(nextState, openLoopControls[i]);
        nextState = mm.Evolve(nextState, openLoopControls[i], w);

    }

    colvec diff = to->as<StateType>()->getArmaData() - nextState->as<StateType>()->getArmaData();

    assert(norm(diff.subvec(0,1),2) < 1e-1); // the distance between final state and goal is less than eps

    cout<<"The final evolved State is :"<<nextState->as<MotionModelMethod::StateType>()->getArmaData()<<endl;
    cout<<"the final commanded state was :"<<to->as<MotionModelMethod::StateType>()->getArmaData()<<endl;

    cout<<"Motion Model passed tests"<<endl;

}

void TestKalmanFilter()
{

    MotionModelMethod::MotionModelPointer mm(new UnicycleMotionModel( "/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml"));

    ObservationModelMethod::ObservationModelPointer om(new CamAruco2DObservationModel( "/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml" ));

    ExtendedKF kf(mm, om);

    typedef SE2BeliefSpace::StateType StateType;
    ompl::base::StateSpacePtr space(new SE2BeliefSpace());
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));

    ob::RealVectorBounds bounds(2);
    bounds.setLow(-5);
    bounds.setHigh(5);

    //space->setBounds(bounds);

    ob::State *from = space->allocState();

    from->as<StateType>()->setXYYaw(1.3,3,0);

    ob::State *to = space->allocState();

    to->as<StateType>()->setXYYaw(5,3,1.57);

    cout<<"The from State is: "<<from->as<StateType>()->getX()<<", "<<from->as<StateType>()->getY()<<endl;

    colvec u(2);
    u[0] = 0.1;
    u[1] = 0.1;

    arma::mat testCov(3,3);

    testCov<<0.01<<0<<0<<endr
           <<0<<0.01<<0<<endr
           <<0<<0<<0.01<<endr;

    from->as<StateType>()->setCovariance(testCov);

    vector<colvec> openLoopControls = mm->generateOpenLoopControls(from, to);

    ompl::base::State *nextState = space->allocState();

    si->copyState(nextState, from);

    ompl::base::State *kfEstimate = space->allocState();
    si->copyState(kfEstimate, from);
    //corrupting the initial belief
    kfEstimate->as<StateType>()->setXYYaw(1.45,3.2,0.025);

    LinearSystem dummy;

    for(int i=0; i< openLoopControls.size() ; i++)
    {
        colvec w = mm->generateNoise(from, openLoopControls[i]);
        nextState = mm->Evolve(from, openLoopControls[i], w);
        colvec obs = om->getObservation(nextState, true);
        kfEstimate = kf.Evolve(kfEstimate, openLoopControls[i], obs, dummy, dummy);
        si->copyState(from, nextState);


    }

    colvec diff = to->as<StateType>()->getArmaData() - nextState->as<StateType>()->getArmaData();

    assert(norm(diff.subvec(0,1),2) < 0.05); // the distance between final state and goal is less than eps

    cout<<"The final evolved State is :"<<nextState->as<MotionModelMethod::StateType>()->getArmaData()<<endl;
    cout<<"The final filtered State is :"<<kfEstimate->as<MotionModelMethod::StateType>()->getArmaData()<<endl;
    cout<<"the final commanded state was :"<<to->as<MotionModelMethod::StateType>()->getArmaData()<<endl;

    cout<<"Kalman Filter passed tests, check if the values make sense to you!"<<endl;

}


void TestRHCICreate()
{

    MotionModelMethod::MotionModelPointer mm(new UnicycleMotionModel( "/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml"));

    ObservationModelMethod::ObservationModelPointer om(new CamAruco2DObservationModel( "/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml" ));

    ExtendedKF kf(mm, om);

    typedef SE2BeliefSpace::StateType StateType;
    ompl::base::StateSpacePtr space(new SE2BeliefSpace());
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));

    ob::RealVectorBounds bounds(2);
    bounds.setLow(-5);
    bounds.setHigh(5);

    //space->setBounds(bounds);

    ob::State *from = space->allocState();

    from->as<StateType>()->setXYYaw(1.3,3,0);

    ob::State *to = space->allocState();

    to->as<StateType>()->setXYYaw(5,3,1.57);

    cout<<"The from State is: "<<from->as<StateType>()->getX()<<", "<<from->as<StateType>()->getY()<<endl;

    colvec u(2);
    u[0] = 0.1;
    u[1] = 0.1;

    arma::mat testCov(3,3);

    testCov<<0.5<<0<<0<<endr
           <<0<<0.5<<0<<endr
           <<0<<0<<0.02<<endr;

    from->as<StateType>()->setCovariance(testCov);

    vector<colvec> openLoopControls = mm->generateOpenLoopControls(from, to);

    ompl::base::State *nextState = space->allocState();;

    si->copyState(nextState, from);

    ompl::base::State *kfEstimate = space->allocState();
    si->copyState(kfEstimate, from);
    kfEstimate->as<StateType>()->setXYYaw(1.0,3.2,0.1);

    LinearSystem dummy;

    std::vector<ompl::base::State*> nmx;
    std::vector<colvec> nmu;
    std::vector<LinearSystem> lss;

    RHCICreate *sepController = new RHCICreate(to, nmx, nmu, lss, mm);
    colvec diff = to->as<StateType>()->getArmaData() - from->as<StateType>()->getArmaData();

    while(norm(diff.subvec(0,1), 2) > 0.08)
    {
        cout<<"KF Estimate"<<endl;
        space->as<SE2BeliefSpace>()->printBeliefState(kfEstimate);
        cout<<"True State"<<endl;
        space->as<SE2BeliefSpace>()->printBeliefState(nextState);
        cin.get();

        // generate control based on belief
        colvec rhcU = sepController->generateFeedbackControl(kfEstimate);
        // generate motion noise based on true state
        colvec w = mm->generateNoise(from, rhcU);
        // evolve true state
        nextState = mm->Evolve(from, rhcU, w);
        //get observation based on true state
        colvec obs = om->getObservation(nextState, true);
        //evolve kalman filter using control and obs
        kfEstimate = kf.Evolve(kfEstimate, rhcU, obs, dummy, dummy);

        si->copyState(from, nextState);

        diff = to->as<StateType>()->getArmaData() - kfEstimate->as<StateType>()->getArmaData();

    }


    assert(norm(diff.subvec(0,1),2) < 0.10);

    cout<<"The final evolved State is :"<<nextState->as<MotionModelMethod::StateType>()->getArmaData()<<endl;
    cout<<"The final filtered State is :"<<kfEstimate->as<MotionModelMethod::StateType>()->getArmaData()<<endl;
    cout<<"the final commanded state was :"<<to->as<MotionModelMethod::StateType>()->getArmaData()<<endl;

    cout<<"RHC ICreate passed tests only if the values make sense to you!"<<endl;

}

void TestActuationSystem()
{

    MotionModelMethod::MotionModelPointer mm(new UnicycleMotionModel( "/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml"));

    ObservationModelMethod::ObservationModelPointer om(new CamAruco2DObservationModel( "/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml" ));

    ExtendedKF kf(mm, om);

    typedef SE2BeliefSpace::StateType StateType;
    ompl::base::StateSpacePtr space(new SE2BeliefSpace());
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));

    ob::RealVectorBounds bounds(2);
    bounds.setLow(-5);
    bounds.setHigh(5);

    //space->setBounds(bounds);

    ob::State *from = space->allocState();

    from->as<StateType>()->setXYYaw(1.3,3,0);

    ob::State *to = space->allocState();

    to->as<StateType>()->setXYYaw(5,3,1.57);

    cout<<"The from State is: "<<from->as<StateType>()->getX()<<", "<<from->as<StateType>()->getY()<<endl;

    colvec u(2);
    u[0] = 0.1;
    u[1] = 0.1;

    arma::mat testCov(3,3);

    testCov<<0.5<<0<<0<<endr
           <<0<<0.5<<0<<endr
           <<0<<0<<0.02<<endr;

    from->as<StateType>()->setCovariance(testCov);

    vector<colvec> openLoopControls = mm->generateOpenLoopControls(from, to);

    ompl::base::State *nextState = space->allocState();;

    si->copyState(nextState, from);

    ompl::base::State *kfEstimate = space->allocState();
    si->copyState(kfEstimate, from);
    kfEstimate->as<StateType>()->setXYYaw(1.0,3.2,0.1);

    LinearSystem dummy;

    std::vector<ompl::base::State*> nmx;
    std::vector<MotionModelMethod::ControlType> nmu;
    std::vector<LinearSystem> lss;

    RHCICreate *sepController = new RHCICreate(to, nmx, nmu, lss, mm);
    colvec diff = to->as<StateType>()->getArmaData() - from->as<StateType>()->getArmaData();

    ActuationSystemMethod::ActuationSystemPointer as(new SimulatedActuationSystem(mm, om));

    as->setTrueState(from);

    //Controller<RHCICreate, ExtendedKF> myController(to, nmx, nmu, mm, om,as );

    while(norm(diff.subvec(0,1), 2) > 0.08)
    {
        cout<<"KF Estimate"<<endl;
        space->as<SE2BeliefSpace>()->printBeliefState(kfEstimate);
        cout<<"True State"<<endl;
        space->as<SE2BeliefSpace>()->printBeliefState(nextState);
        //cin.get();

        // generate control based on belief
        colvec rhcU = sepController->generateFeedbackControl(kfEstimate);

        as->applyControl(rhcU);

        // evolve true state
        nextState = as->getTrueState();
        //get observation based on true state
        colvec obs = om->getObservation(nextState, true);
        //evolve kalman filter using control and obs
        kfEstimate = kf.Evolve(kfEstimate, rhcU, obs, dummy, dummy);

        si->copyState(from, nextState);

        diff = to->as<StateType>()->getArmaData() - kfEstimate->as<StateType>()->getArmaData();

    }


    assert(norm(diff.subvec(0,1),2) < 0.10);

    cout<<"The final evolved State is :"<<nextState->as<MotionModelMethod::StateType>()->getArmaData()<<endl;
    cout<<"The final filtered State is :"<<kfEstimate->as<MotionModelMethod::StateType>()->getArmaData()<<endl;
    cout<<"the final commanded state was :"<<to->as<MotionModelMethod::StateType>()->getArmaData()<<endl;

    cout<<"RHC ICreate passed tests only if the values make sense to you!"<<endl;

}
#endif
