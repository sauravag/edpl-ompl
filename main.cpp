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
    FIRM2DSetup mySetup;

    std::string setupFilePath = "/home/saurav/Research/Development/FIRM-OMPL/Setup.xml";
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/car1_planar_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/Maze_planar_env.dae";

    mySetup.setPathToSetupFile(setupFilePath.c_str());

    mySetup.setRobotMesh(robot_fname.c_str());

    mySetup.setEnvironmentMesh(env_fname.c_str());

     // define starting state
    mySetup.setStartState(15,5,0);
    mySetup.setGoalState(0.4,4.8,1.57);

    if(mySetup.solve(90))
    {
        mySetup.executeSolution();
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
