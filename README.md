----------------------------------------
Belief Space Planning Using Open Motion Planning Library

Authors:
Saurav Agarwal (sauravag@tamu.edu)
Ali-akbar Agha-mohammadi (aliagha@mit.edu)

Estimation, Decision and Planning Lab
Texas A&M University
Copyright 2014
----------------------------------------

----------------------------------------
Brief: 
----------------------------------------
This application is an implementation of Feedback Information Road Maps (FIRM) and the Node-Based Multi-Modal Motion Planner (NBM3P) planners with OMPL. FIRM is a multi-query approach for planning under uncertainty which is a belief-space variant of probabilistic roadmap. FIRM relies on a Gaussian representation of the belief state. We also provide a new planner NBM3P, that enables planning in Non-Gaussian belief spaces. Such a planner is particularly useful if your system has a multi-modal hypothesis about its state. Such a situation could arise from ambigious data-association where the robot sees some information which makes it believe that it could be in one of multiple places (ex. A laser scanner can get confused by geometrically identical rooms in a building).

---------------------------------------
Tested Configuration
---------------------------------------
1. Ubuntu 16.04.2
2. Boost: 1.58
3. OMPL 1.2.1

----------------------------------------
Compiling & Running
----------------------------------------

Quick Instructions: 

From the folder containing this readme
 1. $chmod +x scripts/build_bsp.sh
 2. $./scripts/build_bsp.sh
 3. $./bsp-app-demo "./SetupFiles/SetupTROSims.xml"

Details:

A CMakeLists.txt file is provided for easy compilation with cmake.

External Depencies: [All of these are hard requirements]

1. Open Motion Planning Library (OMPL v1.2.1 minimum): Excellent instructions provided on the ompl website [http://ompl.kavrakilab.org/] for installation. Follow the instructions to build and install the full omplapp and QT will automatically be installed as part of that.  
 
2. QT4 & OpenGL (freeglut): For Visualization (Ubuntu: sudo apt-get install freeglut3-dev libqt4-dev)

3. Armadillo C++ (version 7.5) Matrix Algebra Library: Recommended to download and build from source (http://arma.sourceforge.net/download.html)

4. tinyxml: Needed for reading landmark/setup parameters (Ubuntu: sudo apt-get install libtinyxml-dev)

5. Cmake and a C++ compiler (Use GNU 5.4.1 which is tested or newer GNU versions, GNU 4.X.X will not work!)

Build:

1. $mkdir build
2. $cd build
3. $cmake ..
4. $make -j4

Run: In project directory do $./bsp-app-demo

----------------------------------------
FAQs, Tips, How To etc. 
----------------------------------------
Please visit my webpage which answers most common questions about this library.

http://sauravag.com/2015/10/faq-belief-space-planning-with-ompl/

----------------------------------------
References:
----------------------------------------
1. A. Agha-mohammadi, Suman Chakravorty, Nancy Amato, "FIRM: Sampling-based Feedback Motion Planning Under Motion Uncertainty and Imperfect Measurements", International Journal of Robotics Research, 33(2):268-304, February 2014.

2. A. Agha-mohammadi, Saurav Agarwal, Aditya Mahadevan, Suman Chakravorty, Daniel Tomkins, Jory Denny, Nancy Amato, "Robust Online Belief Space Planning in Changing Environments: Application to Physical Mobile Robots," In Proc. IEEE Int. Conf. Robot. Autom. (ICRA), Hong Kong, China, May 2014.

3. Saurav Agarwal, Amirhossein Tamjidi and Suman Chakravorty, “Motion Planning in Non-Gaussian Belief Spaces for Mobile Robots“, IEEE Int. Conf. Robot. Autom. (ICRA), 2016 [submitted for review]

4. http://edplab.org :  Estimation, Decision and Planning Lab Webpage (publications, related work, videos)

5. http://ompl.kavrakilab.org : OMPL documentation and code

---------------------------------------
How To Use This Application
---------------------------------------
We have developed these planner based on the design philosophy of the planner class in OMPL. On top of the base planners, we have added additional functionaility to the package such as a motion model class, observation model class, filter class etc. that are required within FIRM and not provided explicity in OMPL.

1. Use the provided application as is: Simply compile the code and run it. By changing the names of the setup paramter file (xml files) to use in main.cpp, you can control the motion/observation model parameters, environment geometry file, robot geometry file, landmark locations etc. 

2. In FIRM, we can load a previously computed map or have the planner generate a new one. When the planner starts up, it looks for a FIRMRoadMap.xml file in the top directory. We use this file to load a pre-computed roadmap (the nodes and the edges of the FIRM graph). If this file is found, the planner will load it and use it (so you get to re-use an old map). If there is no such file, a new roadmap will be generated and saved to the same file name (FIRMRoadMap.xml). You can then move this xml file to the SavedRoadmaps folder for later use. 

3. Build you own application by calling FIRM in your scenario. To do this you would need to write a motion/observation model by deriving the MotionModelMethod and ObservationModeMethod class. Currently, we have only defined the SE2BeliefSpace (x,y,yaw). If your robot/system has a different state 
space, then you would need to define a new belief space class. For example, if you're working with a quadrotor such that your state X = [x,y,z,roll,pitch,yaw] you would need to defie a new SE3BeliefSpace.

---------------------------------------
How To Integrate With ROS
---------------------------------------

Integration with a real robot through ROS or any other system, simply requires that during policy execution, the apply control and get observation commands are sent to the right "SpaceInformation". For example, we provide a ROSSpaceInformation class that is able to send twist messages. The example should be sufficient for you to get started with ROS development. You may need to update the CMakeLists accordingly. There is a variable USE_ROS in edplompl.h that needs to be defined which will
use FIRMAruco2DROSSetup.h, this setup file has not been fully tested recently but should be fairly easy to use.

---------------------------------------
How To Contribute
---------------------------------------

We welcome contributions to our work. Feel free to fork this code and add new features. Once you're ready to integrate it to our project, send us a pull request.

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
