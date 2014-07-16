#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
//#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/control/PathControl.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/base/goals/GoalState.h>
#include <iostream>
#include <istream>
#include "include/Planner/FIRM.h"
#include "include/Spaces/SE2BeliefSpace.h"
#include "FIRMOMPL.h"
#include "Tests.h"
#include <QApplication>
#include <QtGui/QDesktopWidget>
#include "include/Visualization/Window.h"
#include "include/Visualization/Visualizer.h"
#include <boost/thread.hpp>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;
using namespace arma;
using namespace std;

bool isStateValid(const ob::State *state)
{
    //No collision check involved right now

    ObservationModelMethod::ObservationModelPointer om(new CamAruco2DObservationModel( "/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml" ));

    return om->isStateObservable(state);
}

/*
ob::ValidStateSamplerPtr allocGaussianValidBeliefSampler(const ompl::base::SpaceInformation *si)
{
    // we can perform any additional setup / configuration of a sampler here,
    // but there is nothing to tweak in case of the ObstacleBasedValidStateSampler.
    ObservationModelMethod::ObservationModelPointer om(new CamAruco2DObservationModel("/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml" ));
    //MotionModelMethod::MotionModelPointer mm(new UnicycleMotionModel(*si, "/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml"));

    GaussianValidBeliefSampler *sampler = new GaussianValidBeliefSampler(si);
    //sampler->setMotionModel(mm);
    sampler->setObservationModel(om);
    return ob::ValidStateSamplerPtr(sampler);
}
*/

/*
ob::ValidStateSamplerPtr allocUniformValidBeliefSampler(const ompl::base::SpaceInformation *si, const MotionModelMethod::MotionModelPointer mm,
                                                        const ObservationModelMethod::ObservationModelPointer om)
{

    UniformValidBeliefSampler *unisampler = new UniformValidBeliefSampler(si);
    unisampler->setObservationModel(om);
    unisampler->setMotionModel(mm);
    return ob::ValidStateSamplerPtr(unisampler);
}
*/
/*
oc::DirectedControlSamplerPtr allocDirectedControlSampler(const ompl::control::SpaceInformation *si)
{
    ICreateControlSampler *csampler = new ICreateControlSampler(si);
    return ompl::control::DirectedControlSamplerPtr(csampler);
}
*/

void plan(void)
{
    typedef SE2BeliefSpace::StateType StateType;

    // construct the state space we are planning in
    ob::StateSpacePtr statespace(new SE2BeliefSpace());

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(2);
    // set X bound
    bounds.setLow(0,0);
    bounds.setHigh(0,17);
    //set Y bound
    bounds.setLow(1,0);
    bounds.setHigh(1,7);

    statespace->as<SE2BeliefSpace>()->setBounds(bounds);

    //Construct the control space
    oc::ControlSpacePtr controlspace( new oc::RealVectorControlSpace(statespace,2) ) ;

    // construct an instance of space information from this state space
    //oc::SpaceInformationPtr si(new FIRMSpaceInformation(statespace, controlspace));
    firm::SpaceInformation::SpaceInformationPtr si(new firm::SpaceInformation(statespace, controlspace));

    // provide the observation model to the space
    ObservationModelMethod::ObservationModelPointer om(new CamAruco2DObservationModel("/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml" ));
    si->setObservationModel(om);

    // Provide the motion model to the space
    MotionModelMethod::MotionModelPointer mm(new UnicycleMotionModel(si, "/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml"));
    si->setMotionModel(mm);

    // set state validity checking for this space
    si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(new FIRMValidityChecker(si)));

    // set motion validator
    si->setMotionValidator(ob::MotionValidatorPtr(new ob::DiscreteMotionValidator(si)));

    //set the state sampler
    //si->setValidStateSamplerAllocator(allocGaussianValidBeliefSampler);

    //si->setDirectedControlSamplerAllocator(allocDirectedControlSampler);
    //set the state propagator
    si->setStatePropagator(oc::StatePropagatorPtr(new UnicycleStatePropagator(si))) ;
    si->setPropagationStepSize(0.1); // this is the duration that a control is applied
    si->setMinMaxControlDuration(1,1000); // minimum and maximum integer multiples of timestep that can be applied
    //ob::ValidStateSamplerPtr   simpleSampler;

    // allocate a valid state sampler, by default, a uniform sampler is allocated
    //simpleSampler = si->allocValidStateSampler();

    // create a random start state
    ob::State *start = statespace->allocState();

    start->as<StateType>()->setXYYaw(15,4,0);
    //start->as<StateType>()->setXYYaw(4,1,0);
    //start->as<StateType>()->setXYYaw(6,6,0);

    Visualizer::updateSpaceInformation(si);
    //Visualizer::addState(start);

    //Visualizer::updateCurrentBelief(start);
    //Visualizer::updateTrueState(start);

    //simpleSampler->sample(start);

    cout<<"The start state is :"<<endl;
    statespace->as<SE2BeliefSpace>()->printBeliefState(start);

    // create a random goal state
    ob::State *goal = statespace->allocState();
    goal->as<StateType>()->setXYYaw(0.4,4.8,1.57);
    //goal->as<StateType>()->setXYYaw(1.5,5.5,1.57);
    //Visualizer::addState(goal);

    cout<<"The goal state is:"<<endl;
    statespace->as<SE2BeliefSpace>()->printBeliefState(goal);
    //cin.get();

    // create a problem instance
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal, 1.0);


    // create a planner for the defined space
    ob::PlannerPtr planner(new FIRM(si, false));

    // RRT planner
    //ob::PlannerPtr planner(new ompl::geometric::PRM(si));

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    //planner->as<oc::RRT>()->setIntermediateStates(true);
    //const oc::RRT rrt = static_cast<oc::RRT&>(*planner);

    //set the maximum number of nearest neighbors
    //planner->setMaxNearestNeighbors(5);

    // perform setup steps for the planner
    planner->setup();

    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    //------------------------------------------------

    std::cout<<"------ATTEMPTING SOLUTION------------"<<std::endl;

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->solve(100);

    cout<<"------COMPLETED ATTEMPT--------------"<<std::endl;

    cout<<"The min control duration is :"<<si->getMinControlDuration()<<endl;
    cout<<"The max control duration is :"<<si->getMaxControlDuration()<<endl;


    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path

        //oc::PathControl *cpath = new oc::PathControl(si);
        planner->as<FIRM>()->executeFeedback();
        /*
        const ob::PathPtr &path = pdef->getSolutionPath();

        //oc::PathControl cpath = static_cast<oc::PathControl&>(*path);
        og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path);
        //cpath.interpolate();

        std::cout << "Found solution, The path is:" << std::endl;

        // print the path to screen
        path->print(std::cout);

        //planner->as<FIRM>()->executeFeedback();
        //std::vector<ob::State*> solnStates = cpath.getStates();

        //cout<<"Number of states"<<solnStates.size()<<std::endl;

        si->setTrueState(start);
        si->setBelief(start);

        for(int i=0;i<gpath.getStateCount();i++)
        {
            Visualizer::addState(gpath.getState(i));
        }

        for(int i=0;i<gpath.getStateCount()-1;i++)
        {
            si->setTrueState(gpath.getState(i));
            si->setBelief(gpath.getState(i));
            //cout<<"The control duration is :"<<cpath.getControlDuration(i)<<endl;
            vector<oc::Control*> OLC;
            mm->generateOpenLoopControls(gpath.getState(i),gpath.getState(i+1),OLC) ;

            for(int j=0;j<OLC.size();j++)
            {
                si->applyControl(OLC[j]);
                boost::this_thread::sleep(boost::posix_time::milliseconds(33));
            }
        }
        */
    }
    else
        std::cout << "No solution found" << std::endl;

    cout << "DONE" << std::endl;
}

int main(int argc, char *argv[])
{
    cout << "OMPL version: " << OMPL_VERSION << endl;

    // set static variables
    RHCICreate::setControlQueueSize(10);
    RHCICreate::setTurnOnlyDistance(0.05);
    Controller<RHCICreate, ExtendedKF>::setNodeReachedAngle(5); // degrees
    Controller<RHCICreate, ExtendedKF>::setNodeReachedDistance(0.05);// meters
    Controller<RHCICreate, ExtendedKF>::setMaxTries(50);
    Controller<RHCICreate, ExtendedKF>::setMaxTrajectoryDeviation(4.0); // meters

    //TestSE2BeliefSpace();
    //TestBeliefStateSampler();
    //TestObservationModel();
    //TestMotionModel();
    //TestKalmanFilter();
    //TestRHCICreate();
    //TestController();
    //TestFIRMWeight();
    //TestStatePropagator();

    QApplication app(argc, argv);
    MyWindow window;
    window.resize(window.sizeHint());
    int desktopArea = QApplication::desktop()->width() * \
        QApplication::desktop()->height();
    int widgetArea = window.width() * window.height();

    window.showMaximized();

    //plan();
    boost::thread solveThread(plan);

    app.exec();
    solveThread.join();

    cout<<"--------------Press Enter to Exit------------- \n";
    return 0;

}
