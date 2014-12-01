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

/* Authors: Aditya Mahadevan, Saurav Agarwal, Ali-akbar Agha-mohammadi */

#include "../../include/Visualization/Visualizer.h"

boost::mutex Visualizer::drawMutex_;

std::list<ompl::base::State*> Visualizer::states_;

std::list<ompl::base::State*> Visualizer::beliefModes_;

ompl::base::State* Visualizer::trueState_;

ompl::base::State* Visualizer::currentBelief_;

std::vector<arma::colvec> Visualizer::landmarks_;

firm::SpaceInformation::SpaceInformationPtr Visualizer::si_;

std::vector<std::pair<const ompl::base::State*, const ompl::base::State*> > Visualizer::graphEdges_;

std::vector<std::pair<const ompl::base::State*, const ompl::base::State*> > Visualizer::rolloutConnections_;

std::vector<std::pair<const ompl::base::State*, const ompl::base::State*> > Visualizer::mostLikelyPath_;

std::vector<Visualizer::VZRFeedbackEdge> Visualizer::feedbackEdges_;

boost::optional<std::pair<const ompl::base::State*, const ompl::base::State*> > Visualizer::chosenRolloutConnection_;

std::vector<const ompl::base::State*> Visualizer::robotPath_;

std::vector<ompl::geometric::PathGeometric> Visualizer::openLoopRRTPaths_;

Visualizer::VZRDrawingMode Visualizer::mode_;

ompl::app::RenderGeometry* Visualizer::renderGeom_;

int Visualizer::robotIndx_ = -1;

int Visualizer::envIndx_   = -1;

bool Visualizer::saveVideo_ = false;

void Visualizer::drawLandmark(arma::colvec& landmark)
{

    double scale = 0.15;

    glPushMatrix();

    glColor3d(0.0,0.0,0.0);

    glTranslated(landmark[1], landmark[2], 0.0);
    glVertex3f(0.8,0.8,0.8);

    glBegin(GL_TRIANGLE_FAN);
        glVertex3f(0, scale, 0);
        glVertex3f(0.5*scale, 0, 0);
        glVertex3f(0, -scale, 0);
        glVertex3f(-0.5*scale, 0, 0);
    glEnd();

    glPopMatrix();

}

void Visualizer::drawRobot(const ompl::base::State *state)
{

    if(robotIndx_ <=0)
    {
        robotIndx_ = renderGeom_->renderRobot();
    }
    else
    {
        arma::colvec x = state->as<SE2BeliefSpace::StateType>()->getArmaData();

        glPushMatrix();
            glTranslated(x[0], x[1], 0.0);
            glRotated(-90+(180/3.14157)*x[2],0,0,1);
            glCallList(robotIndx_);
        glPopMatrix();
    }
}

void Visualizer::drawState(const ompl::base::State *state, VZRStateType stateType)
{
    using namespace arma;

    switch(stateType)
    {
        case TrueState:
            glColor3d(1,0,0);
            break;

        case BeliefState:
            glColor3d(1,0,0); // red
            break;

        case GraphNodeState:
            glColor3d(0,0,1); // blue
            break;

        default:
            glColor3d(1,1,1); // grey
            break;
    }

    arma::colvec x = state->as<SE2BeliefSpace::StateType>()->getArmaData();
    mat covariance = state->as<SE2BeliefSpace::StateType>()->getCovariance();

    glPushMatrix();
        glTranslated(x[0], x[1], 0);

        //draw a black disk
        GLUquadric *disk = gluNewQuadric();
        gluDisk(disk, 0, 0.12, 12, 1);
        gluDeleteQuadric(disk);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(0.5*cos(x[2]), 0.5*sin(x[2]), 0);
        glEnd();

        double fovRadius = 2.5; //meters
        double fovAngle = 22.5*3.14157/180; //radians
        arma::colvec fovLeft(3), fovRight(3);

        fovLeft	<< fovRadius*cos(x[2] + fovAngle) << endr
                << fovRadius*sin(x[2] + fovAngle) << endr
                << 0                              << endr;

        fovRight << fovRadius*cos(x[2] - fovAngle) 	<< endr
                 << fovRadius*sin(x[2] - fovAngle) 	<< endr
                 <<	0								<< endr;
        //glColor3d(0.5,0.5,0.5);

        //Remove comment to show field of view of robot
        /*
        glBegin(GL_LINE_LOOP);
            glVertex3f(0,0,0);
            glVertex3f(fovRight[0], fovRight[1], 0);
            glVertex3f(fovLeft[0], fovLeft[1], 0);
        glEnd();
        */
    glPopMatrix();

    if(trace(covariance) != 0 && stateType == VZRStateType::BeliefState)
    {
        double chi2 = 9.21034;
        double magnify = 20.0; // scaled up for viewing
        mat pos;
        for(double th = 0; th < 2*boost::math::constants::pi<double>(); th += 0.05*boost::math::constants::pi<double>())
        {
            mat tmpRow(1,2);
            tmpRow << cos(th) << sin(th) << endr;
            pos = join_cols(pos, tmpRow);
        }

        pos *= sqrt(chi2);
        pos *= magnify;

        int nPoints = pos.n_rows;

        //glColor3d(1.0,0.0,0.0); // grey

        try
        {
            mat K = trans(chol(covariance.submat(0,0,1,1)));
            mat shift;

            shift = join_cols((ones(1,nPoints)*x[0]),(ones(1,nPoints)*x[1]));

            mat transformed = K*trans(pos) + shift;

            glPushMatrix();

            glBegin(GL_LINES);

            for(unsigned int i = 0; i < transformed.n_cols; ++i)
            {
                glVertex2f(transformed(0,i), transformed(1,i));
            }

            glEnd();

            glPopMatrix();
        }
        catch(int e)
        {
            OMPL_INFORM("Visualizer: Covariance cholesky did not converge...proceeding");
        }

    }

}

void Visualizer::refresh()
{
    boost::mutex::scoped_lock sl(drawMutex_);

    glPushMatrix();

    glEnable(GL_DEPTH_TEST);
    glClear(GL_DEPTH_BUFFER_BIT);

    drawRobotPath();

    drawEnvironment();

    if(trueState_)
    {
        drawRobot(trueState_);
    }

    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    switch(mode_)
    {
        case NodeViewMode:

            drawGraphBeliefNodes();

            break;

        case FeedbackViewMode:

            drawGraphBeliefNodes();

            drawMostLikelyPath();

            if(feedbackEdges_.size()>0) drawFeedbackEdges();

            break;

        case PRMViewMode:

            drawGraphBeliefNodes();

            if(graphEdges_.size()>0) drawGraphEdges();

            drawMostLikelyPath();

            break;

        case RolloutMode:

            drawRolloutConnections();

            drawMostLikelyPath();

            // draw the rollout connection with lowest cost
            if(chosenRolloutConnection_)
            {
                glColor3d(0.0 , 1.0 , 0.0);
                glLineWidth(3.0);
                    drawEdge(chosenRolloutConnection_->first, chosenRolloutConnection_->second);
                glLineWidth(1.0);
            }

            robotPath_.push_back(si_->cloneState(trueState_));

            break;

        case MultiModalMode:

            drawGraphBeliefNodes();

            drawOpenLoopRRTPaths();

            drawBeliefModes();

            break;


        default:

            assert(!"There is no default drawing mode for OGLDisplay");

            exit(1);
    }

    //glDisable(GL_LIGHTING);
    //glDisable(GL_DEPTH_TEST);
    //glClear(GL_DEPTH_BUFFER_BIT);
    //glDepthMask(GL_FALSE);

    //draw landmarks
    for(size_t i = 0 ; i < landmarks_.size(); ++i)
    {
        drawLandmark(landmarks_[i]);
    }

    if(currentBelief_ && mode_ !=MultiModalMode)
    {
        drawState(currentBelief_, (VZRStateType)1);
    }

    glPopMatrix();
}

void Visualizer::drawEdge(const ompl::base::State* source, const ompl::base::State* target)
{
    using namespace arma;

    colvec::fixed<2> sourceData = source->as<SE2BeliefSpace::StateType>()->getArmaData().subvec(0,1);
    colvec::fixed<2> targetData = target->as<SE2BeliefSpace::StateType>()->getArmaData().subvec(0,1);

    glBegin(GL_LINES);
        glVertex2d(sourceData[0],sourceData[1]);
        glVertex2d(targetData[0],targetData[1]);
    glEnd();
}

void Visualizer::drawEnvironment()
{
    if(renderGeom_)
    {
        if(envIndx_ <= 0)
        {
            envIndx_ = renderGeom_->renderEnvironment();
        }
        else
        {
            glCallList(envIndx_);
        }

    }

}

void Visualizer::drawObstacle()
{

    double x_l =  2.4;
    double x_r =  15.29;
    double y_b =  2.72;
    double y_t =  5.13;

    //glTranslated(x_l, y_b, 0);
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //glLoadIdentity();//load identity matrix
    glColor3f(0.0f,0.0f,1.0f); //blue color
    glBegin(GL_POLYGON);
            glVertex3f(x_l,y_b,0);
            glVertex3f(x_l, y_t, 0);
            glVertex3f(x_r, y_t, 0);
            glVertex3f(x_r,y_b,0);
    glEnd();

}

void Visualizer::drawGraphBeliefNodes()
{
    for(typename std::list<ompl::base::State*>::iterator s=states_.begin(), e=states_.end(); s!=e; ++s)
    {
          drawState(*s, (VZRStateType)2);
    }

}

void Visualizer::drawBeliefModes()
{
    for(typename std::list<ompl::base::State*>::iterator s=beliefModes_.begin(), e=beliefModes_.end(); s!=e; ++s)
    {
          drawState(*s, (VZRStateType)1);
    }

}

void Visualizer::drawGraphEdges()
{
    for(unsigned int i=0; i<graphEdges_.size();i++)
    {
        glColor3d(0.5,0.5,0.5);
        drawEdge(graphEdges_[i].first,graphEdges_[i].second);
    }
}


void Visualizer::drawRolloutConnections()
{
    glDisable(GL_LIGHTING);

    for(int i=0; i<rolloutConnections_.size();i++)
    {
        glColor3d(1.0,0.0,0);
        glLineWidth(3.0);
        drawEdge(rolloutConnections_[i].first,rolloutConnections_[i].second);
        glLineWidth(1.0);

    }
}

void Visualizer::drawFeedbackEdges()
{
    using namespace arma;

    double maxCost = 0;
    for(typename std::vector<VZRFeedbackEdge>::iterator i=feedbackEdges_.begin(), e=feedbackEdges_.end(); i!=e; ++i)
    {
        maxCost = std::max(maxCost, i->cost);
    }

    glLineWidth(2.0);
    for(typename std::vector<VZRFeedbackEdge>::iterator i=feedbackEdges_.begin(), e=feedbackEdges_.end();i!=e; ++i)
    {
        std::pair<const ompl::base::State*, const ompl::base::State*> vertexPair = std::make_pair(i->source, i->target);

        std::vector<std::pair<const ompl::base::State*, const ompl::base::State*> >::iterator it = std::find(mostLikelyPath_.begin(), mostLikelyPath_.end(), vertexPair);

        // if this pair does not exist in most likely path, then draw it, other we will only draw it in most likely path
        if( it == mostLikelyPath_.end())
        {
            double costFactor = sqrt(i->cost/maxCost);
            glColor3d(0.0,1.0,0.0);

            drawEdge(i->source, i->target);

        }
    }
    glLineWidth(1.0);
}

void Visualizer::drawMostLikelyPath()
{
    glDisable(GL_LIGHTING);

    glLineWidth(4.0);
    for(int i=0; i<mostLikelyPath_.size();i++)
    {
        glColor3d(1.0 , 1.0 , 0.0);
        drawEdge(mostLikelyPath_[i].first,mostLikelyPath_[i].second);
    }
    glLineWidth(1.f);
}

void Visualizer::drawRobotPath()
{
    if(robotPath_.size()>=2)
    {
        glDisable(GL_LIGHTING);
        glColor3d(0.0 , 1.0 , 1.0);

        for(int i=0; i<robotPath_.size()-1;i++)
        {
            glColor3d(1.0 , 1.0 , 1.0);

            glLineWidth(4.0);
                drawEdge(robotPath_[i],robotPath_[i+1]);
            glLineWidth(1.f);
        }
    }
}

void Visualizer::drawGeometricPath(ompl::geometric::PathGeometric path)
{
    for(int i=0;i<path.getStateCount()-1;i++)
    {
        glColor3d(0 , 1.0, 0);
        glLineWidth(2.0);
        drawEdge(path.getState(i),path.getState(i+1)) ;
        glLineWidth(1.0);
    }
}

void Visualizer::drawOpenLoopRRTPaths()
{
    for(int i=0;i<openLoopRRTPaths_.size();i++)
    {
        drawGeometricPath(openLoopRRTPaths_[i]);
    }
}

