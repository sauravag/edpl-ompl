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

#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <string>

using namespace std;

#include <QtOpenGL/QGLWidget>

//width and height of window
#ifndef WIDTH
#define WIDTH 400
#define HEIGHT 600
#endif

#include "Vector.h"
#include "Matrix.h"
#include "Point.h"
#include "Visualizer.h"


using namespace mathtool;
//typedef mathtool::Point<double, 3> Point3D;

class GLWidget : public QGLWidget {
  Q_OBJECT

  public:
    typedef Visualizer Display;

    GLWidget(QWidget *parent = 0);
    ~GLWidget(){}

    //For Window to know how big to draw the GL scene in the window
    QSize minimumSizeHint() const;
    QSize sizeHint() const;

    //Simulator* GetSimulator(){return m_simulator;}

    //variable determining whether axes are drawn
    bool m_drawAxes;

    //handles camera movement from input
    void resetCam();
    void moveCam(double _delta);
    void strafeCam(double _delta);
    void zoomCam(double _delta);

  public Q_SLOTS:
    void drawAxes(bool);
    void saveSnapshot();
    void ChangeMode(int);

  protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);

    /*
    void GetEnvironmentPolygons();
    void DrawMarkers();
    void DrawRoadmap();
    void DrawViewRegion(vector<double> rPos, double _distance, double _angle);
    void DrawRobot();
    void DrawOtherRobots();
    */
    void saveFrame();
    void saveImage(QString&);

  private:
    arma::colvec m_camPos; //camera position
    arma::colvec m_camAt; //camera rotation
    double m_camZoom; //camera zoom (field of view angle)

    //Environment* m_env; //environment pointer
    //RobotInfo* m_rArgs; //pointer to robot we are viewing
    //ArUco_Shared* m_aArgs; //pointer to webcam image
    //vector <vector<Vector3d> > m_polygons; //environment polygons

    string m_filename; //input to simulation. Saved only to pass to the simulator.
    //Simulator* m_simulator; //SIMULATOR!!!!!
    double m_long, m_lat; //mouse tracking for the camAt vector.
    QPoint m_lastPos; //for mouse handling
    bool m_view; //true represents overhead over robot. False is for viewing whole environment.

    QString m_snapshotPath;
    QString m_framePath;
};

#endif
