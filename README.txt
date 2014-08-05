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
This application is an implementation of Feedback Information Road Maps (FIRM) with OMPL.

"FIRM is a multi-query approach for planning under uncertainty which is a belief-space variant of probabilistic roadmap 

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
directly compile the application. For ease of use, two code::blocks projects are provided for OSX and Linux (tested on Ubuntu), 
be sure to link to the correct libraries and set the include paths for your compiler build settings.

External Depencies: [All of these are hard requirements for this app to run]

1. Open Motion Planning Library: Excellent instructions provided on the ompl website for installation. Follow the instructions to build
and install the full omplapp and QT will automatically be installed as part of that. 
 
2. QT & OpenGL (freeglut): For Visualization

3. Armadillo C++ Matrix Algebra Library: Recommended to download and build from source


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
