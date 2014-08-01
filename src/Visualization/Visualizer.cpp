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

/* Authors: Ali-akbar Agha-mohammadi, Saurav Agarwal, Aditya Mahadevan */

#include "../../include/Visualization/Visualizer.h"

boost::mutex Visualizer::drawMutex_;

std::list<ompl::base::State*> Visualizer::states_;

ompl::base::State* Visualizer::trueState_;

ompl::base::State* Visualizer::currentBelief_;

std::vector<arma::colvec> Visualizer::landmarks_;

firm::SpaceInformation::SpaceInformationPtr Visualizer::si_;

std::vector<std::pair<const ompl::base::State*, const ompl::base::State*> > Visualizer::graphEdges_;

std::vector<Visualizer::VZRFeedbackEdge> Visualizer::feedbackEdges_;

Visualizer::VZRDrawingMode Visualizer::mode_;

void Visualizer::drawLandmark(arma::colvec& landmark)
{
  //cout << "++++++++++++++++++++++++++" << endl;
  //cout << "Drawing " << m_landmarks.size() << " landmarks" << endl;
  glColor3d(1.0,1.0,1.0);

  double scale = 0.15;

  glPushMatrix();
    //glLoadIdentity();
    glTranslated(landmark[1], landmark[2], 0);
    glVertex3f(0.8,0.8,0.8);
    glBegin(GL_TRIANGLE_FAN);
      glVertex3f(0, scale, 0);
      glVertex3f(0.5*scale, 0, 0);
      glVertex3f(0, -scale, 0);
      glVertex3f(-0.5*scale, 0, 0);
    glEnd();

  glPopMatrix();
}

void Visualizer::drawState(const ompl::base::State *state, VZRStateType stateType)
{
    using namespace arma;

    switch(stateType)
    {
        case TrueState:
            glColor3d(1.0,0.0,0.0); // red
            break;

        case BeliefState:
            glColor3d(0.5,0.5,0.5); // grey
            break;

        case GraphNodeState:
            glColor3d(1.0,1.0,1.0); // white
            break;

        default:
            glColor3d(1.0,1.0,1.0); // white
            break;
    }

    arma::colvec x = state->as<SE2BeliefSpace::StateType>()->getArmaData();
    mat covariance = state->as<SE2BeliefSpace::StateType>()->getCovariance();

    glPushMatrix();
        glTranslated(x[0], x[1], 0);

        //draw a black disk
        GLUquadric *disk = gluNewQuadric();
        gluDisk(disk, 0, 0.15, 15, 1);
        //glColor3ub(155, 205, 55);
        gluDeleteQuadric(disk);
        //glRotated(m_v[2], 0, 0, 1);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(1.0*cos(x[2]), 1.0*sin(x[2]), 0);
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
        glColor3d(0.5,0.5,0.5);

        //Remove comment to show field of view of robot
        /*
        glBegin(GL_LINE_LOOP);
            glVertex3f(0,0,0);
            glVertex3f(fovRight[0], fovRight[1], 0);
            glVertex3f(fovLeft[0], fovLeft[1], 0);
        glEnd();
        */
    glPopMatrix();

    if(trace(covariance) != 0)
    {
    double chi2 = 9.21034;
    double magnify = 4.0; // scaled up for viewing
    mat pos;
    for(double th = 0; th < 2*boost::math::constants::pi<double>(); th += 0.05*boost::math::constants::pi<double>())
    {
        //cout << "th: " << th <<  endl;
        mat tmpRow(1,2);
        tmpRow << cos(th) << sin(th) << endr;
        pos = join_cols(pos, tmpRow);
    }

    pos *= sqrt(chi2);
    pos *= magnify;

    //cout << "pos size: "<< pos.n_rows << " rows, " << pos.n_cols << " cols" << endl;
    int nPoints = pos.n_rows;

    mat K = trans(chol(covariance.submat(0,0,1,1)));
    //cout << "K size: " << K.n_rows << " rows, " << K.n_cols << " cols " <<endl;
    mat shift;
    shift = join_cols((ones(1,nPoints)*x[0]),(ones(1,nPoints)*x[1]));

    // cout << "shift size: " << shift.n_rows << ", " << shift.n_cols << endl;
    mat transformed = K*trans(pos) + shift;

    glPushMatrix();
    //glColor3d(1,0,0);
    //glTranslated(m_mean[0], m_mean[1], 0);
    //glBegin(GL_TRIANGLE_FAN);
    glBegin(GL_LINES);
    //glVertex2f((*this)[0], (*this)[1]);
    for(unsigned int i = 0; i < transformed.n_cols; ++i)
    {
        glVertex2f(transformed(0,i), transformed(1,i));
    }
    glEnd();

    glPopMatrix();
    }

}

void Visualizer::refresh()
{
    boost::mutex::scoped_lock sl(drawMutex_);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //glClearColor(1.0f, 1.0f, 1.0f, 0.0f);

    glPushMatrix();

    drawEnvironmentBoundary();

    drawObstacle();
    //draw roadmap based on current mode

    switch(mode_)
    {
        case NodeViewMode:
            drawGraphBeliefNodes();
            //DrawNodeViewMode();
            break;
        case FeedbackViewMode:
            drawGraphBeliefNodes();
            if(feedbackEdges_.size()>0) drawFeedbackEdges();
            //DrawFeedbackViewMode();
            break;
        case PRMViewMode:
            drawGraphBeliefNodes();
            if(graphEdges_.size()>0) drawGraphEdges();
            //DrawPRMViewMode();
            break;
        default:
            assert(!"There is no default drawing mode for OGLDisplay");
            exit(1);
    }

    //draw landmarks
    for(size_t i = 0 ; i < landmarks_.size(); ++i)
    {
        drawLandmark(landmarks_[i]);
    }

    if(trueState_) drawState(trueState_, (VZRStateType)0);

    if(currentBelief_) drawState(currentBelief_, (VZRStateType)1);

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

void Visualizer::drawEnvironmentBoundary()
{

    double xmax = 7.14;
    double ymax = 3.05;
    //glTranslated(0, 0, 0);
    glColor3f(0.0f,1.0f,0.0f); //blue color
    glBegin(GL_POLYGON);
            glVertex3f(0,0,0);
            glVertex3f(0, ymax, 0);
            glVertex3f(xmax, ymax, 0);
            glVertex3f(xmax,0,0);
    glEnd();

}

void Visualizer::drawObstacle()
{

    double x_l =  2.0;
    double x_r =  17.5;
    double y_b =  2.0;
    double y_t =  5.0;

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

void Visualizer::drawGraphEdges()
{
    for(unsigned int i=0; i<graphEdges_.size();i++)
    {
        drawEdge(graphEdges_[i].first,graphEdges_[i].second);
    }
}

/**
void Visualizer::drawFeedbackEdges()
{
    for(int i=0; i<feedbackEdges_.size();i++)
    {
        drawEdge(feedbackEdges_[i].source,feedbackEdges_[i].target);
    }
}
*/
void Visualizer::drawFeedbackEdges()
{
    using namespace arma;

    double maxCost = 0;
    for(typename std::vector<VZRFeedbackEdge>::iterator i=feedbackEdges_.begin(), e=feedbackEdges_.end(); i!=e; ++i)
    {
        maxCost = std::max(maxCost, i->cost);
    }

    for(typename std::vector<VZRFeedbackEdge>::iterator i=feedbackEdges_.begin(), e=feedbackEdges_.end();i!=e; ++i)
    {
        double costFactor = sqrt(i->cost/maxCost);
        glColor3d(costFactor,1.0-costFactor,0.0);
        glLineWidth(3.0);
            drawEdge(i->source, i->target);
        glLineWidth(1.f);
    }
}





