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

/* Author: Saurav Agarwal */

/*! \mainpage EDPL-OMPL
 *
 * \section Introduction
 *
 * Belief space planning with OMPL as the backend.
 * 
 * \section Tips
 * 
 *  Look at the top of FIRMOMPL.h to see how to include ROS and build. Over there, uncomment #define USE_ROS to allow ROS dependent files to compiled.
 *
 * \section install_sec Installation
 *
 *  See README
 *  
 */

#include <QApplication>
#include <QtGui/QDesktopWidget>
#include <boost/thread.hpp>
#include <iostream>
#include <istream>

//#include "Setup/FIRM2DSetup.h"
#include "Setup/TwoDPointRobotSetup.h"

#ifdef USE_ROS
    #include "Setup/FIRMAruco2DROSSetup.h"
#endif

#include "Setup/MultiModalSetup.h"
//#include "Testing/Tests.h"

using namespace std;

void plan(const std::string &setupFilePath)
{
    //FIRM2DSetup *mySetup(new FIRM2DSetup);

    TwoDPointRobotSetup *mySetup(new TwoDPointRobotSetup);

    mySetup->setPathToSetupFile(setupFilePath.c_str());

    mySetup->setup();
    
    Visualizer::setMode(Visualizer::VZRDrawingMode::PRMViewMode);

    int keepTrying = 1;

    while(keepTrying)
    {
        if(mySetup->solve())
        {

#ifdef __linux__
            // HACK to make a beep sound (linux only)
            bool ret;
            //ret = system("sh -c \"echo '\a'> $(tty)\" 2>/dev/null");
#endif

            mySetup->Run();
            OMPL_INFORM("Plan Executed.");
            Visualizer::doSaveVideo(true);  // true: save the final trajectory executed by the robot

#ifdef __linux__
            // HACK to make a beep sound (linux only)
            //ret = system("sh -c \"echo '\a'> $(tty)\" 2>/dev/null");
#endif

            keepTrying = 0;

        }
        else
        {
            OMPL_INFORM("Unable to find Solution in given time. Either more nodes are needed, or DP could not converge.");
            
            break;

            //std::cin>>keepTrying;
        }

    }

    delete mySetup;

    OMPL_INFORM("Execution Terminated.");

    QApplication::quit();

    return;
}

#ifdef USE_ROS
void planROS(const std::string &setupFilePath)
{

    FIRMAruco2DROSSetup *mySetup(new FIRMAruco2DROSSetup);

    mySetup->setPathToSetupFile(setupFilePath.c_str());

    mySetup->setup();

    Visualizer::setMode(Visualizer::VZRDrawingMode::PRMViewMode);

    int mode = 0;

    OMPL_INFORM("Choose what mode (0: Standard FIRM, 1 : Rollout , 2: Kidnapping-Multi-Modal)? : ");

    //cin>>mode;

    int keepTrying = 1;

    ros::spinOnce();

    while(keepTrying)
    {
        if(mySetup->solve())
        {
            //mySetup->executeSolution(mode);
            mySetup->Run(mode);

            OMPL_INFORM("Plan Executed.");

            Visualizer::doSaveVideo(false);

            keepTrying = 0;

        }
        else
        {
            OMPL_INFORM("Unable to find Solution in given time, would you like to continue attempt. (1: yes, 0 :no) ? :");

            std::cin>>keepTrying;
        }

    }

    delete mySetup;

    OMPL_INFORM("Execution Terminated, Close Terminal");

}
#endif

int main(int argc, char *argv[])
{

    #ifdef USE_ROS
        // Initialize the ros node
        ros::init(argc, argv, "firm_planner");
    #endif

    //srand(239645);

    //arma_rng::set_seed(239645);
    arma::arma_rng::set_seed_random();

    QApplication app(argc, argv);

    MyWindow window;

    window.resize(window.sizeHint());

    window.showMaximized();

    window.resetCamera();

    /**
     Below you have 2 options:

        1. To plan just using the provided simulation

        2. To plan with ROS integration, the provided example listens for aruco_marker_publisher and advertises robot commands to geometry::twist
    */
    #ifndef USE_ROS
        boost::thread solveThread(plan,argv[1]);
    #endif

    #ifdef USE_ROS
        boost::thread solveThread(planROS,argv[1]);
    #endif

    app.exec();

    solveThread.join();

    OMPL_INFORM("Task Complete");

    exit(0);


}
