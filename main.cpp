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

/* Authors: Saurav Agarwal, Ali-akbar Agha-mohammadi */

#include "FIRM2DSetup.h"
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/control/PathControl.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/base/goals/GoalState.h>
#include "include/Planner/FIRM.h"
#include "include/Spaces/SE2BeliefSpace.h"
#include "FIRMOMPL.h"
#include "Tests.h"
#include <QApplication>
#include <QtGui/QDesktopWidget>
#include "include/Visualization/Window.h"
#include "include/Visualization/Visualizer.h"
#include <boost/thread.hpp>
#include <iostream>
#include <istream>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;
using namespace arma;
using namespace std;

void plan()
{
    FIRM2DSetup *mySetup(new FIRM2DSetup);

    std::string setupFilePath = "/home/saurav/Research/Development/FIRM-OMPL/Setup.xml";
    std::string robot_fname = "/home/saurav/Research/Development/FIRM-OMPL/Models/simpleICreate2.obj";
    std::string env_fname = "/home/saurav/Research/Development/FIRM-OMPL/Models/simpleFIRMEnv.obj";

    mySetup->setPathToSetupFile(setupFilePath.c_str());

    assert(mySetup->setRobotMesh(robot_fname.c_str()));

    assert(mySetup->setEnvironmentMesh(env_fname.c_str()));

     // define starting state
    mySetup->setStartState(16,3,0);
    mySetup->setGoalState(1,4,1.57);

    mySetup->setup();

    Visualizer::updateRenderer(*dynamic_cast<const ompl::app::RigidBodyGeometry*>(mySetup), mySetup->getGeometricStateExtractor());

    Visualizer::updateSpaceInformation(mySetup->getSpaceInformation());

    // Specify the maximum time allowed to solve problem
    double solveTime = 90;


    if(mySetup->solve(solveTime))
    {
        mySetup->executeSolution();
    }
    else
    {
        OMPL_INFORM("Unable to find Solution in given time.");

        exit(1);
    }

}

int main(int argc, char *argv[])
{

    QApplication app(argc, argv);

    MyWindow window;

    window.resize(window.sizeHint());

    int desktopArea = QApplication::desktop()->width() * \
                            QApplication::desktop()->height();

    int widgetArea = window.width() * window.height();

    window.showMaximized();

    boost::thread solveThread(plan);

    app.exec();

    solveThread.join();

    OMPL_INFORM("Task Complete");

    return 0;

}
