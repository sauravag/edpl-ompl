/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Texas A&M University
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
*   * Neither the name of the Texas A&M University nor the names of its
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

/* Authors: Saurav Agarwal */

#ifndef FIRM_OMPL_TESTS_
#define FIRM_OMPL_TESTS_

#include "FIRMOMPL.h"
#include "tinyxml/tinyxml.h"


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

bool isTheStateValid(const ob::State *state)
{
    ObservationModelMethod::ObservationModelPointer om(new CamAruco2DObservationModel( "/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml" ));

    return om->isStateObservable(state);
}
/*
void TestBeliefStateSampler()
{
    //SE2BeliefSpace *space;
    //space =  new SE2BeliefSpace();
    ompl::base::StateSpacePtr statespace(new SE2BeliefSpace());

    ompl::control::ControlSpacePtr controlspace( new ompl::control::RealVectorControlSpace(statespace,2) ) ;

    // construct an instance of space information from this state space
    firm::SpaceInformation::SpaceInformationPtr si(new firm::SpaceInformation(statespace, controlspace));

    ob::RealVectorBounds bounds(2);
    bounds.setLow(-5);
    bounds.setHigh(5);

    //space->setBounds(bounds);
    MotionModelMethod::MotionModelPointer mm(new UnicycleMotionModel(si, "/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml"));

    ObservationModelMethod::ObservationModelPointer om(new CamAruco2DObservationModel( "/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml" ));
    //ActuationSystemMethod::ActuationSystemPointer as(new SimulatedActuationSystem(mm, om));

    si->setStateValidityChecker(boost::bind(&isTheStateValid, _1));

    GaussianValidBeliefSampler *sampler = new GaussianValidBeliefSampler(si);

    ompl::base::State *sampleState = si->allocState();

    sampler->setStdDev(0.2);
    //sampler->setActuationSystem(as);

    sampler->sample(sampleState);

    space->as<SE2BeliefSpace>()->printBeliefState(sampleState);

}
*/


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
    for(unsigned int i=0; i<obs.n_rows/4; i++)
    {
        //cout<<"The observation is :"<< obs[i*4] <<endl;
        //cout<<"The predicted observation is :"<< pred_obs[i*4]<<endl;
        assert(obs[i*4] ==  pred_obs[i*4]);
    }

    cout<<"Observation Model passed tests"<<endl;
}

/*
void TestStatePropagator()
{
    namespace ob = ompl::base;
    namespace oc = ompl::control;
    typedef SE2BeliefSpace::StateType StateType;

    ob::StateSpacePtr space(new SE2BeliefSpace());

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-6);
    bounds.setHigh(6);

    space->as<SE2BeliefSpace>()->setBounds(bounds);

    // construct an instance of space information from this state space
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    ob::State *from = space->allocState();

    from->as<StateType>()->setXYYaw(1,1,0);

    ob::State *to = space->allocState();

    cout<<"The from State is: "<<from->as<StateType>()->getX()<<endl;

    unsigned int dim = 2;

    ompl::control::ControlSpacePtr controlSpace( new ompl::control::RealVectorControlSpace(space,2) ) ;

    ompl::control::SpaceInformationPtr controlSpaceInfo(new oc::SpaceInformation(space, controlSpace));

    ompl::control::Control *command = controlSpace->allocControl();

    command->as<oc::RealVectorControlSpace::ControlType>()->values[0] = 1;
    command->as<oc::RealVectorControlSpace::ControlType>()->values[1] = 1;

    std::cout<<"The set control values are ->"<<std::endl;
    controlSpace->printControl(command, std::cout) ;

    UnicycleStatePropagator sp(controlSpaceInfo);

    sp.propagate(from, command, 1, to);

    space->as<SE2BeliefSpace>()->printBeliefState(to);

}
*/


void TestMotionModel()
{
    typedef SE2BeliefSpace::StateType StateType;

    //SE2BeliefSpace *space;
    //space =  new SE2BeliefSpace();
    ompl::base::StateSpacePtr statespace(new SE2BeliefSpace());

    std::cout<<"Printing the dimension of statespace :"<<statespace->getDimension()<<std::endl;

    ompl::control::ControlSpacePtr controlspace( new ompl::control::RealVectorControlSpace(statespace,2) ) ;

    // construct an instance of space information from this state space
    firm::SpaceInformation::SpaceInformationPtr si(new firm::SpaceInformation(statespace, controlspace));

    UnicycleMotionModel mm(si,  "/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml" );

    ob::State *from = si->allocState();

    from->as<StateType>()->setXYYaw(1.3,3,0);

    ob::State *to = si->allocState();

    to->as<StateType>()->setXYYaw(5,3,1.57);

    cout<<"The from State is: "<<from->as<StateType>()->getX()<<endl;

    colvec uvec(2);
    uvec[0] = 0.1;
    uvec[1] = 0.1;

    ompl::control::Control* u;

    u = si->allocControl();

    mm.ARMA2OMPL(uvec, u);

    colvec noise = mm.generateNoise(from, u);

    cout<<"Noise :"<<noise<<endl;

    mat stateJacobian = mm.getStateJacobian(from, u , noise);

    mat controlJacobian = mm.getControlJacobian(from, u, noise);

    mat noiseJacobian = mm.getNoiseJacobian(from, u, noise);

    mat processNoiseCovariance = mm.processNoiseCovariance(from , u);

    vector<ompl::control::Control*> openLoopControls;

    mm.generateOpenLoopControls(from, to, openLoopControls);

    cout<<"The size of open loop controls is: "<<openLoopControls.size()<<endl;

    ompl::base::State *nextState =  from;

    for(unsigned int i=0; i< openLoopControls.size() ; i++)
    {
        cout<<"The control is :"<<endl;
        si->printControl(openLoopControls[i], std::cout);
        colvec w = mm.getZeroNoise();
        mm.Evolve(from, openLoopControls[i], w, nextState);
        si->copyState(from, nextState);
    }

    colvec diff = to->as<StateType>()->getArmaData() - nextState->as<StateType>()->getArmaData();

    assert(norm(diff.subvec(0,1),2) < 1e-1); // the distance between final state and goal is less than eps

    cout<<"The final evolved State is :"<<nextState->as<MotionModelMethod::StateType>()->getArmaData()<<endl;
    cout<<"the final commanded state was :"<<to->as<MotionModelMethod::StateType>()->getArmaData()<<endl;

    cout<<"Motion Model passed tests"<<endl;

}

void TestKalmanFilter()
{
    typedef SE2BeliefSpace::StateType StateType;

    ompl::base::StateSpacePtr statespace(new SE2BeliefSpace());

    std::cout<<"Printing the dimension of statespace :"<<statespace->getDimension()<<std::endl;

    ompl::control::ControlSpacePtr controlspace( new ompl::control::RealVectorControlSpace(statespace,2) ) ;

    // construct an instance of space information from this state space
    firm::SpaceInformation::SpaceInformationPtr si(new firm::SpaceInformation(statespace, controlspace));

    MotionModelMethod::MotionModelPointer mm(new UnicycleMotionModel(si, "/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml"));

    ObservationModelMethod::ObservationModelPointer om(new CamAruco2DObservationModel( "/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml" ));

    si->setMotionModel(mm);
    si->setObservationModel(om);

    ExtendedKF kf(si);

    ob::RealVectorBounds bounds(2);
    bounds.setLow(-5);
    bounds.setHigh(5);

    //space->setBounds(bounds);

    ompl::base::State *from = si->allocState();

    from->as<StateType>()->setXYYaw(1.3,3,0);

    ompl::base::State *to = si->allocState();

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

    vector<ompl::control::Control*> openLoopControls;
    mm->generateOpenLoopControls(from, to, openLoopControls);

    ompl::base::State *nextState = si->allocState();

    si->copyState(nextState, from);

    ompl::base::State *kfEstimate = si->allocState();
    si->copyState(kfEstimate, from);
    //corrupting the initial belief
    kfEstimate->as<StateType>()->setXYYaw(1.45,3.2,0.025);

    LinearSystem dummy;

    for(unsigned int i=0; i< openLoopControls.size() ; i++)
    {
        colvec w = mm->generateNoise(from, openLoopControls[i]);
        mm->Evolve(from, openLoopControls[i], w, nextState);
        si->copyState(from, nextState);
        colvec obs = om->getObservation(nextState, true);
        kf.Evolve(kfEstimate, openLoopControls[i], obs, dummy, dummy, kfEstimate);

    }

    colvec diff = kfEstimate->as<StateType>()->getArmaData() - nextState->as<StateType>()->getArmaData();
    double error = norm(diff.subvec(0,1),2) ;

    cout<<"========================================================================"<<endl;
    cout<<"The final true State is       : \n"<<nextState->as<MotionModelMethod::StateType>()->getArmaData()<<endl;
    cout<<"The final filtered State is   : \n"<<kfEstimate->as<MotionModelMethod::StateType>()->getArmaData()<<endl;
    cout<<"The final commanded state was : \n"<<to->as<MotionModelMethod::StateType>()->getArmaData()<<endl;
    cout<<"The error in final estimate   : \n"<<diff<<std::endl;

    assert(error<0.05); // the distance between final state and goal is less than eps
    cout<<"Kalman Filter passed tests, check if the values make sense to you!"<<endl;

}


void TestRHCICreate()
{
    typedef SE2BeliefSpace::StateType StateType;

    ompl::base::StateSpacePtr statespace(new SE2BeliefSpace());

    ompl::control::ControlSpacePtr controlspace( new ompl::control::RealVectorControlSpace(statespace,2) ) ;

    // construct an instance of space information from this state space
    firm::SpaceInformation::SpaceInformationPtr si(new firm::SpaceInformation(statespace, controlspace));

    MotionModelMethod::MotionModelPointer mm(new UnicycleMotionModel(si, "/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml"));

    ObservationModelMethod::ObservationModelPointer om(new CamAruco2DObservationModel( "/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml" ));


    si->setMotionModel(mm);
    si->setObservationModel(om);

    ExtendedKF kf(si);

    ob::RealVectorBounds bounds(2);
    bounds.setLow(-5);
    bounds.setHigh(5);

    //space->setBounds(bounds);

    ob::State *from = si->allocState();

    from->as<StateType>()->setXYYaw(1.3,3,0);

    ob::State *to = si->allocState();

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

    vector<ompl::control::Control*> openLoopControls;
    mm->generateOpenLoopControls(from, to, openLoopControls);

    ompl::base::State *nextState = si->allocState();;

    si->copyState(nextState, from);

    ompl::base::State *kfEstimate = si->allocState();
    ompl::base::State *kfUpdate = si->allocState();
    si->copyState(kfEstimate, from);
    //kfEstimate->as<StateType>()->setXYYaw(1.0,3.2,0.1);

    LinearSystem dummy;

    std::vector<ompl::base::State*> nmx;
    std::vector<ompl::control::Control*> nmu;
    std::vector<LinearSystem> lss;

    RHCICreate *sepController = new RHCICreate(to, nmx, nmu, lss, mm);
    colvec diff = to->as<StateType>()->getArmaData() - from->as<StateType>()->getArmaData();

    while(norm(diff.subvec(0,1), 2) > 0.06 || abs(diff[2]) > 2*3.142/180 )
    {
        cout<<"KF Estimate"<<endl;
        statespace->as<SE2BeliefSpace>()->printBeliefState(kfEstimate);
        cout<<"True State"<<endl;
        statespace->as<SE2BeliefSpace>()->printBeliefState(nextState);

        //cin.get();

        // generate control based on belief
        ompl::control::Control *rhcU = sepController->generateFeedbackControl(kfEstimate);
        // generate motion noise based on true state
        colvec w = mm->generateNoise(from, rhcU);
        // evolve true state
        mm->Evolve(from, rhcU, w, nextState);
        //get observation based on true state
        colvec obs = om->getObservation(nextState, true);
        //evolve kalman filter using control and obs
        kf.Evolve(kfEstimate, rhcU, obs, dummy, dummy, kfUpdate);

        si->copyState(from, nextState);
        si->copyState(kfEstimate, kfUpdate);

        diff = to->as<StateType>()->getArmaData() - kfEstimate->as<StateType>()->getArmaData();

    }


    assert(norm(diff.subvec(0,1),2) < 0.10);

    cout<<"The final true State is :"<<nextState->as<MotionModelMethod::StateType>()->getArmaData()<<endl;
    cout<<"The final filtered State is :"<<kfEstimate->as<MotionModelMethod::StateType>()->getArmaData()<<endl;
    cout<<"the final commanded state was :"<<to->as<MotionModelMethod::StateType>()->getArmaData()<<endl;

    cout<<"RHC ICreate passed tests only if the values make sense to you!"<<endl;

}

/*
void TestController()
{

    typedef SE2BeliefSpace::StateType StateType;

    ompl::base::StateSpacePtr statespace(new SE2BeliefSpace());

    ompl::control::ControlSpacePtr controlspace( new ompl::control::RealVectorControlSpace(statespace,2) ) ;

    // construct an instance of space information from this state space
    firm::SpaceInformation::SpaceInformationPtr si(new firm::SpaceInformation(statespace, controlspace));

    MotionModelMethod::MotionModelPointer mm(new UnicycleMotionModel(si, "/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml"));

    ObservationModelMethod::ObservationModelPointer om(new CamAruco2DObservationModel( "/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml" ));

    si->setMotionModel(mm);
    si->setObservationModel(om);

    si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(new FIRMValidityChecker(si)));

    ExtendedKF kf(si);

    //space->setBounds(bounds);

    ob::State *from = si->allocState();

    from->as<StateType>()->setXYYaw(1.3,3,0);

    ob::State *to = si->allocState();

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

    std::vector<ompl::control::Control*> openLoopControls;
    mm->generateOpenLoopControls(from, to, openLoopControls);

    std::vector<ompl::base::State*> intermediates;

    ompl::base::State *intermediate = si->allocState();

    si->copyState(intermediate, from);
    for(typename vector<ompl::control::Control*>::iterator c=openLoopControls.begin(), e=openLoopControls.end(); c!=e; ++c)
    {
        ompl::base::State *x = si->allocState();
        mm->Evolve(intermediate,*c,mm->getZeroNoise(), x);
        intermediates.push_back(x);
        si->copyState(intermediate, x);
    }

    ompl::base::State *nextState = si->allocState();;

    si->copyState(nextState, from);

    ompl::base::State *kfEstimate = si->allocState();

    si->copyState(kfEstimate, from);
    kfEstimate->as<StateType>()->setXYYaw(1.0,3.2,0.1);

    LinearSystem dummy;

    std::vector<ompl::base::State*> nmx;
    std::vector<ompl::control::Control*> nmu;
    std::vector<LinearSystem> lss;

    //RHCICreate *sepController = new RHCICreate(to, nmx, nmu, lss, mm);
    colvec diff = to->as<StateType>()->getArmaData() - from->as<StateType>()->getArmaData();

    si->setTrueState(from);

    Controller<RHCICreate, ExtendedKF> *myController;
    myController =  new Controller<RHCICreate,ExtendedKF>(to, intermediates, openLoopControls, si);
    bool isFailed = false;
    ompl::base::Cost cost(0);
    int failureCode=0;

    ompl::base::State* finalEstimate = si->allocState();

    bool success = myController->Execute(kfEstimate, finalEstimate, cost);

    if(success) cout<<"Controller executed completely \n";
    else cout<<"Controller failed !! \n";

    diff = to->as<StateType>()->getArmaData() - finalEstimate->as<StateType>()->getArmaData();
    cout<<"The final filtered State from controller is :"<<finalEstimate->as<MotionModelMethod::StateType>()->getArmaData()<<endl;
    cout<<"the final commanded state was :"<<to->as<MotionModelMethod::StateType>()->getArmaData()<<endl;

    assert(norm(diff.subvec(0,1),2) < 0.10 && "Controller failed to take robot to goal");

    cout<<"Controller passed tests only if the values make sense to you!"<<endl;

}
*/
/*
void TestFIRMWeight()
{
    FIRMWeight weight(2);

    cout<<"The weight properties are: "<<endl;
    cout<<weight.getCost()<<endl;
    cout<<weight.getControllerID()<<endl;
    cout<<weight.getSuccessProbability()<<endl;

}
*/
/**
void TestPlotting()
{

    QCustomPlot *customPlot;

    QVector<double> x(101), y(101); // initialize with entries 0..100
    for (int i=0; i<101; ++i)
    {
        x[i] = i/50.0 - 1; // x goes from -1 to 1
        y[i] = x[i]*x[i];  // let's plot a quadratic function
    }
    // create graph and assign data to it:
    customPlot->addGraph();
    customPlot->graph(0)->setData(x, y);
    // give the axes some labels:
    customPlot->xAxis->setLabel("x");
    customPlot->yAxis->setLabel("y");
    // set axes ranges, so we see all data:
    customPlot->xAxis->setRange(-1, 1);
    customPlot->yAxis->setRange(0, 1);
    customPlot->replot();
}
*/
#endif

