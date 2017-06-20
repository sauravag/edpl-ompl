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

/* Authors: Saurav Agarwal, Aditya Mahadevan, Ali-akbar Agha-mohammadi */

/* Handles all the GL functions, camera, and drawing for the general window.
 * Owns the simulation so that it can draw its contents.
 */


#include <iostream>
#include <sstream>
#include <cmath>

using namespace std;

#include <QtGui/QtGui>
#include <QtOpenGL/QtOpenGL>

#include "GL/glu.h"

#include "Visualization/GLWidget.h"

GLWidget::GLWidget(QWidget *parent)
  : QGLWidget(QGLFormat(QGL::SampleBuffers), parent),
    m_drawAxes(false),
    m_camZoom(27), m_view(false),
    m_snapshotPath(tr("")), m_framePath(tr(""))
{
    using namespace arma;
    arma::colvec campos(3);
    campos<<11.0<<endr
         <<11.0<<endr
         <<-1.0<<endr;
    m_camPos =campos;

    arma::colvec camat(3);
    camat<<0.0<<endr
        <<0.0<<endr
        <<-1.5<<endr;
    m_camAt = camat;

    //GetEnvironmentPolygons();
    m_long = -boost::math::constants::pi<double>()/2.0; m_lat = 0.0;

}



QSize GLWidget::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize GLWidget::sizeHint() const
{
    return QSize(WIDTH, HEIGHT);
}

void GLWidget::resetCam()
{
//     m_camPos[0] = 7.5;
//     m_camPos[1] = 7.5;
    m_camPos[0] = 10.0;
    m_camPos[1] = 10.0;
    m_camPos[2] = -250.0;
    m_camAt[0] = 0.0;
    m_camAt[1] = 0.0;
    m_camAt[2] = -1.0;
//     m_camZoom = 40; // 75 for 6 cordr world
    m_camZoom = 45; // 75 for 6 cordr world
    m_long = -boost::math::constants::pi<double>()/2.0;
    m_lat = 0.0;
    updateGL();
}

void GLWidget::moveCam(double _delta)
{
    m_camPos = m_camPos + m_camAt * _delta;
    updateGL();
}

void GLWidget::strafeCam(double _delta)
{
    using namespace arma;
    arma::colvec up(3);
    up<<0.0<<endr
    <<0.0<<endr
    <<1.0<<endr;

    arma::colvec right = m_camAt%up;
    m_camPos = m_camPos + right * _delta;
    updateGL();
}

void GLWidget::zoomCam(double _delta)
{
    m_camZoom += _delta;
    updateGL();
}

void GLWidget::drawAxes(bool _draw)
{
    m_drawAxes = _draw;
}

void GLWidget::saveSnapshot()
{
    static unsigned int snapshotNum = 0;

    //the first time, make a good snapshot directory
    if(m_snapshotPath == "")
    {
        QString dateTime = QDateTime::currentDateTime().toString("MMM.dd.yyyy_hh.mmap");

        QDir currPath(QDir::currentPath());
        currPath.mkpath(tr("Snapshots/") + dateTime);
        m_snapshotPath = QDir::currentPath() + QString("/Snapshots/") + dateTime + QString("/");
    }

    char number[4];
    sprintf(number, "%03u", snapshotNum);
    ostringstream oss;
    oss << m_snapshotPath.toStdString() << "Snapshot" << "_" << number << ".png";

    QString fileName(oss.str().c_str());
    saveImage(fileName);

    ++snapshotNum;
}

void GLWidget::saveFrame()
{
  static unsigned int frameNum = 0;

   //the first time, make a good snapshot directory
    if(m_framePath == "")
    {
        QString dateTime = QDateTime::currentDateTime().toString("MMM.dd.yyyy_hh.mmap");

        QDir currPath(QDir::currentPath());
        currPath.mkpath(tr("VideoFrames/") + dateTime);
        m_framePath = QDir::currentPath() + QString("/VideoFrames/") + dateTime + QString("/");
    }

    char number[8];
    sprintf(number, "%07u", frameNum);
    ostringstream oss;
    oss << m_framePath.toStdString() << "Frame" << "_" << number << ".png";

    QString fileName(oss.str().c_str());
    saveImage(fileName);

    ++frameNum;
}


void GLWidget::ChangeMode(int mode)
{
    Visualizer::setMode((Visualizer::VZRDrawingMode)mode);
}

//initialize GL defaults and construct the simulator
void GLWidget::initializeGL()
{
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glDisable(GL_DEPTH_TEST);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
    glEnable(GL_LINE_SMOOTH);
    glShadeModel(GL_FLAT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

//Update function for GL Scene
void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glClearColor(1.0, 1.0, 1.0, 1.0);

    //projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(m_camZoom, (GLfloat) width()/(GLfloat) height(), 1.0, 1000.0);
    glOrtho(-10.0f, 10.0f, -10.0f, 10.0f, 1.0f, 300.0f);

    //model
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    //camera at m_camPos looking down z-axis with positive y (0,1,0) as up (was
    //m_camAt with up direction of 0, 0, 1)
    if(m_view)
    {
        vector<double> pos(3);
        pos[0] = 0;
        pos[1] = 0;
        gluLookAt(pos[0], pos[1], 0, pos[0], pos[1], 0, 0, 1, 0);
    }
    else
    {
        gluLookAt(m_camPos[0], m_camPos[1], m_camPos[2], m_camPos[0] + m_camAt[0], m_camPos[1] + m_camAt[1],
                m_camPos[2] + m_camAt[2],0, 1, 0);
    }

    ///////////////////////
    //Draw Next Frame
    //////////////////////
    Display::refresh();

    if(m_drawAxes)
    {
        glPushMatrix();

            glLineWidth(1);
            glBegin(GL_LINES);
            glColor3f(0.0, 1.0, 0.0);
            glVertex3f(0.0, 0.0, 0.0);
            glVertex3f(15.0, 0.0, 0.0);
            glColor3f(1.0, 0.0, 0.0);
            glVertex3f(0.0, 0.0, 0.0);
            glVertex3f(0.0, 15.0, 0.0);
            glColor3f(0.0, 0.0, 1.0);
            glVertex3f(0.0, 0.0, 0.0);
            glVertex3f(0.0, 0.0, 15.0);
            glEnd();

        glPopMatrix();
    }


    if(Display::saveVideo())
    {
        saveFrame();
    }


    glPopMatrix();

}

void GLWidget::resizeGL(int width, int height)
{
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    gluPerspective(m_camZoom, (GLfloat) width/(GLfloat) height, 1.0, 1000.0);
    glViewport (0, 0, (GLsizei) width, (GLsizei) height);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    if(m_view)
    {
        vector<double> pos(2);
        pos[0] = pos[1] = 0.0;
        gluLookAt(pos[0], pos[1], 10, pos[0], pos[1], 0, 0, 1, 0);
    }
    else
    {
        gluLookAt(m_camPos[0], m_camPos[1], m_camPos[2],
            m_camPos[0] + m_camAt[0],
            m_camPos[1] + m_camAt[1],
            m_camPos[2] + m_camAt[2],
            0, 1, 0);
    }
}

void GLWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
    m_view = !m_view;
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
    m_lastPos = event->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    if(m_view)
        return;

    double dx = event->x() - m_lastPos.x();
    double dy = event->y() - m_lastPos.y();

    if(event->buttons() & Qt::MiddleButton)
    {
    }
    else if(event->buttons() & Qt::LeftButton)
    {
        m_camPos[0] += -.2 * dx;
        m_camPos[1] += .2 * dy;
        updateGL();
    }
    else if(event->buttons() & Qt::RightButton)
    {
        // ZoomCam(0.5*dy);
        m_camPos[2] += 0.2*dy;
        updateGL();
    }
    m_lastPos = event->pos();
}

//saves a QImage in PNG format.
void GLWidget::saveImage(QString& _fileName)
{
    QImageWriter writer(_fileName,"png");

    QImage im = grabFrameBuffer();

    if(!writer.write(im))
        qDebug() << writer.errorString();
}

