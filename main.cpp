#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include "include/Planner/FIRM.h"
#include "include/Spaces/SE2BeliefSpace.h"
#include "FIRMOMPL.h"
#include "Tests.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace arma;
using namespace std;

bool isStateValid(const ob::State *state)
{
    //No collision check involved right now

    ObservationModelMethod::ObservationModelPointer om(new CamAruco2DObservationModel( "/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml" ));

    return om->isStateObservable(state);
}

void plan(void)
{
    typedef SE2BeliefSpace::StateType StateType;

    // construct the state space we are planning in
    ob::StateSpacePtr space(new SE2BeliefSpace());

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-6);
    bounds.setHigh(6);

    space->as<SE2BeliefSpace>()->setBounds(bounds);

    // construct an instance of  space information from this state space
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    ObservationModelMethod::ObservationModelPointer om(new CamAruco2DObservationModel( "/home/saurav/Research/Development/OMPL/FIRM-OMPL/Setup.xml" ));

    // set state validity checking for this space
    si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(new FIRMValidityChecker(si, om)));

    // create a random start state
    ob::State *start = space->allocState();

    start->as<StateType>()->setXYYaw(1.3,3.2,0);

    cout<<"The start state is :"<<endl;
    space->as<SE2BeliefSpace>()->printBeliefState(start);
    //cin.get();
    // create a random goal state
    ob::State *goal = space->allocState();
    goal->as<StateType>()->setXYYaw(5,2,0);
    cout<<"The goal state is:"<<endl;
    space->as<SE2BeliefSpace>()->printBeliefState(goal);
    //cin.get();

    // create a problem instance
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    ob::PlannerPtr planner(new ompl::FIRM(si));

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();

    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    std::cout<<"------ATTEMPTING SOLUTION------------"<<std::endl;

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->solve(10);

    std::cout<<"------COMPLETED ATTEMPT--------------"<<std::endl;

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution, The path is:" << std::endl;

        // print the path to screen
        path->print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int, char **)
{
  std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    // set static variables
  RHCICreate::setControlQueueSize(10);
  RHCICreate::setTurnOnlyDistance(0.05);
  Controller<RHCICreate, ExtendedKF>::setNodeReachedAngle(10); // degrees
  Controller<RHCICreate, ExtendedKF>::setNodeReachedDistance(0.05);// meters
  Controller<RHCICreate, ExtendedKF>::setMaxTries(40);

  plan();

  //TestSE2BeliefSpace();
  //TestBeliefStateSampler();
  //TestObservationModel();
  //TestMotionModel();
  //TestKalmanFilter();
  //TestRHCICreate();
  //TestController();
  //TestFIRMWeight();

  return 0;
}
