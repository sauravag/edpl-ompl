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
methods. The crucial feature of FIRM is that the costs associated with the edges are independent of each other, and in this sense it is the first method that generates a graph in belief space that preserves the optimal substructure property. From a practical point of view, FIRM is a robust and reliable planning framework. It is robust since the solution is a feedback and there is no need for expensive replanning. It is reliable because accurate collision probabilities can be computed along the edges. In addition, FIRM is a scalable framework, where the complexity of planning with FIRM is a constant multiplier of the complexity of planning with PRM."  

-----------------------------------------
References: 
-----------------------------------------
1. A. Agha-mohammadi, Suman Chakravorty, Nancy Amato, "FIRM: Sampling-based Feedback Motion Planning Under Motion Uncertainty and Imperfect Measurements", International Journal of Robotics Research, 33(2):268-304, February 2014.

2. A. Agha-mohammadi, Saurav Agarwal, Aditya Mahadevan, Suman Chakravorty, Daniel Tomkins, Jory Denny, Nancy Amato, "Robust Real-time Planning in Belief Space using Multi-query Graphs: Application to Physical Mobile Robots," In Proc. IEEE Int. Conf. Robot. Autom. (ICRA), Hong Kong, China, May 2014. 

3. A. Agha-mohammadi, Suman Chakravorty, Nancy M. Amato, "Sampling-based Nonholonomic Motion Planning in Belief Space via Dynamic Feedback Linearization-based FIRM", In Proc. IEEE Int. Conf. Intel. Rob. Syst. (IROS), Vilamoura, Portugal, Oct 2012

4. A. Agha-mohammadi, Suman Chakravorty, Nancy M. Amato, "FIRM: Feedback Controller-Based Information-State Roadmap, A Framework for Motion Planning Under Uncertainty," In Proc. IEEE Int. Conf. Intel. Rob. Syst. (IROS), San Francisco, CA, Sep 2011. 

5. ompl.kavrakilab.org : OMPL documentation and code

6. http://www.mit.edu/~aliagha : Ali-akbar Agha-momhammadi's research page

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
