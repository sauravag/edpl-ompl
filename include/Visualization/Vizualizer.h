/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Texas A&M University
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

#ifndef FIRM_OMPL_VISUALIZER_H
#define FIRM_OMPL_VISUALIZER_H

#include <X11/X.h>
#include <X11/Xlib.h>
#include <GL/gl.h>
#include <GL/glx.h>
#include <GL/glu.h>
#include <pthread.h>

#include <armadillo>
#include <list>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include "../Spaces/SE2BeliefSpace.h"
#include "../SpaceInformation/SpaceInformation.h"

class Visualizer
{

    public:

        Visualizer(){}

        ~Visualizer(){}

        static void addLandmarks(std::vector<arma::colvec>& landmarks)
        {
            boost::mutex::scoped_lock sl(drawMutex_);
            landmarks_.insert(landmarks_.end(), landmarks.begin(), landmarks.end());
        }


        static void addState(ompl::base::State *state)
        {
            boost::mutex::scoped_lock sl(drawMutex_);
            states_.push_back(state);
        }

        static void updateTrueState(const ompl::base::State *state)
        {
            boost::mutex::scoped_lock sl(drawMutex_);
            si_->copyState(trueState_,state);
        }

        static void updateCurrentBelief(const ompl::base::State *state)
        {
            boost::mutex::scoped_lock sl(drawMutex_);
            si_->copyState(currentBelief_,state);
        }

        static void drawLandmark(arma::colvec& landmark);

        static void drawState(const ompl::base::State* state);


    private:

        static std::list<ompl::base::State*> states_;

        static boost::mutex drawMutex_;

        static ompl::base::State* trueState_;

        static ompl::base::State* currentBelief_;

        static std::vector<arma::colvec> landmarks_;

        static firm::SpaceInformation::SpaceInformationPtr si_;


};
#endif // FIRM_OMPL_VISUALIZER_H
