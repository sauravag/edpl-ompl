/**************************************
Feedback Information RoadMaps Using Open Motion Planning Library

Authors:
Saurav Agarwal (sauravag@tamu.edu)
Ali-akbar Agha-mohammadi (aliagha@mit.edu)

Texas A&M University
Copyright 2014
**************************************/

----------------------------------------
Brief: 
----------------------------------------
This application is an implementation of Feedback Information Road Maps (FIRM) with OMPL and the M3P Non-Gaussian Planner.
FIRM is a multi-query approach for planning under uncertainty which is a belief-space variant of probabilistic roadmap. FIRM relies on a Gaussian
representation of the belief state. We provide a new planner M3P, that enables planning in Non-Gaussian belief spaces. Such a planner is particularly useful
if your system has a multi-modal hypothesis about its state. Such a situation could arise from ambigious data-association where the robot sees some information which makes it believe that it could be in one of multiple places (ex. A laser scanner can get confused by geometrically identical rooms in a building).

----------------------------------------
References:
----------------------------------------
1. A. Agha-mohammadi, Suman Chakravorty, Nancy Amato, "FIRM: Sampling-based Feedback Motion Planning Under Motion Uncertainty and Imperfect Measurements", International Journal of Robotics Research, 33(2):268-304, February 2014.

2. A. Agha-mohammadi, Saurav Agarwal, Aditya Mahadevan, Suman Chakravorty, Daniel Tomkins, Jory Denny, Nancy Amato, "Robust Online Belief Space Planning in Changing Environments: Application to Physical Mobile Robots," In Proc. IEEE Int. Conf. Robot. Autom. (ICRA), Hong Kong, China, May 2014.

3. ompl.kavrakilab.org : OMPL documentation and code

----------------------------------------
Compilation
----------------------------------------
The application is shared as a codeblocks project. You can use codeblocks to
directly compile the application. For ease of use, two codeblocks projects are provided one for OSX and another for Linux (tested on Ubuntu 14.04), 
be sure to link to the correct libraries and set the include paths for your compiler build settings.

External Depencies: [All of these are hard requirements for this app to run]

1. Open Motion Planning Library (OMPL): Excellent instructions provided on the ompl website [http://ompl.kavrakilab.org/] for installation. Follow the instructions to build and install the full omplapp and QT will automatically be installed as part of that.  
 
2. QT & OpenGL (freeglut): For Visualization

3. Armadillo C++ Matrix Algebra Library: Recommended to download and build from source

4. tinyxml: Needed for reading landmark/setup parameters

---------------------------------------
How To Use This Application
---------------------------------------
We have developed FIRM as a planner based on the design philosophy of the planner class in OMPL. On top of the base FIRM planner,
we have added additional functionaility to the package such as a motion model class, observation model class, filter class etc. 
that are required within FIRM and not provided explicity in OMPL.

1. Use the provided application as is: Simply compile the code and run it. By changing the names of the setup paramter file (xml files)
to use in main.cpp, you can control the motion/observation model parameters, environment geometry file, robot geometry file, landmark
locations etc. 

2.  The planner looks for a FIRMRoadMap.xml file in the top directory. We use this file to store a pre-computed roadmap. If this map is found, the planner will load it and use it. If not, a new roadmap will be generated and saved to the same file name. You can then move this xml file to the SavedRoadmaps folder for later use.

3. Build you own application by calling FIRM in your scenario. To do this you would need to write a motion/observation model by deriving the MotionModelMethod and ObservationModeMethod class. Currently, we have only defined the SE2BeliefSpace (x,y,yaw). If your robot/system has a different state 
space, then you would need to define a new belief space class. For example, if you're working with a quadrotor such that your state X = [x,y,z,roll,pitch,yaw] you would need to defie a new SE3BeliefSpace.

---------------------------------------
How To Integrate With ROS
---------------------------------------

Integration with a real robot through ROS or any other system, simply requires that during policy execution, the apply control and get observation commands are sent to the right "SpaceInformation". For example, we provide a ROSSpaceInformation class that subscribes 

---------------------------------------
How To Contribute
---------------------------------------

We welcome contributions to our work. Feel free to fork this code and add new features. Once you're ready to integrate it to our project,
send us a pull request.

Some new features that would be exciting to work on:

1. Adding new belief space classes for higher dimensional states ex. quadrotor.

2. Integration with ROS-MoveIt.

3. Adding new observation and motion models.

---------------------------------------
License:
----------------------------------------
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
