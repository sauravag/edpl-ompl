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

#include "../../include/Visualization/GLWidget.h"


GLWidget::GLWidget(QWidget *parent)
  : QGLWidget(QGLFormat(QGL::SampleBuffers), parent),
    m_drawAxes(false),
    m_camZoom(27), m_view(false),
    m_snapshotPath(tr("")), m_framePath(tr(""))
{
   using namespace arma;
   arma::colvec campos(3);
   campos<<13.5<<endr
         <<3.5<<endr
         <<32.0<<endr;
   m_camPos =campos;

   arma::colvec camat(3);
   camat<<0.0<<endr
        <<0.0<<endr
        <<-1.0<<endr;
   m_camAt = camat;

  //GetEnvironmentPolygons();
  m_long = -PI/2.0; m_lat = 0.0;
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
  m_camPos[0] = 10.0;
  m_camPos[1] = 10.0;

  m_camPos[2] = 30.0;
  m_camAt[0] = 0.0; m_camAt[1] = 0.0; m_camAt[2] = -1.0;
  m_camZoom = 45; m_long = -PI/2.0; m_lat = 0.0;
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
  if(m_framePath == "") {
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

//initalize GL defaults and construct the simulator
void GLWidget::initializeGL()
{
//  glEnable(GL_DEPTH_TEST);
//  glDepthMask(GL_TRUE);
//  glDepthFunc(GL_LEQUAL);
//  glDepthRange(-1.0, 1.0);
//  glEnable(GL_CULL_FACE);
//  glCullFace(GL_BACK);
//  glEnable(GL_COLOR_MATERIAL);
//  glShadeModel(GL_FLAT);
  glClearColor(1,1, 1, 0);

//  m_simulator = new Simulator(this, 0.05, 0.03, m_filename);
}

//Update function for GL Scene
void GLWidget::paintGL()
{
  glClearColor(0.0, 0.0, 0.0, 0.0);
  glClear(GL_COLOR_BUFFER_BIT);

  ///////////////////////
  //Set Camera
  //////////////////////

  //projection
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(m_camZoom, (GLfloat) width()/(GLfloat) height(), 1, 100.0);
  //model
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  //camera at m_camPos looking down z-axis with positive y (0,1,0) as up (was
  //m_camAt with up direction of 0, 0, 1)
  if(m_view){
    vector<double> pos(3);
    pos[0] = 10.0;
    pos[1] = 10.0;
    gluLookAt(pos[0]+2, pos[1], 6,
        pos[0]+2, pos[1], 0,
        0, 1, 0);
  }
  else{
    gluLookAt(m_camPos[0], m_camPos[1], m_camPos[2],
        m_camPos[0] + m_camAt[0],
        m_camPos[1] + m_camAt[1],
        m_camPos[2] + m_camAt[2],
        0, 1, 0);
  }

  ///////////////////////
  //Draw Next Frame
  //////////////////////
  Display::refresh();

  if(m_drawAxes) {
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
/**
#ifndef SIMULATION
  ///////////////////////
  //Draw ArUco Image
  //////////////////////
  if(true){
    cv::Mat image = ICreateActuationSystem<RobotTraits>::GetImage();
    cv::Mat* img = &image;
    if(!img->empty()){
      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();
      gluOrtho2D(0, WIDTH, 0, HEIGHT);
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      QImage qframe = QImage((const unsigned char*)(img->data),
          img->cols, img->rows, img->step, QImage::Format_RGB888).rgbSwapped();
      qframe = QGLWidget::convertToGLFormat(qframe);
      qframe = qframe.scaled(QSize(500, 375), Qt::KeepAspectRatio, Qt::SmoothTransformation);
      glRasterPos2f( 370, 450 ); //WIDTH - qframe.width(), HEIGHT - qframe.height() );
      glDrawPixels(qframe.width(),qframe.height(), GL_RGBA, GL_UNSIGNED_BYTE, qframe.bits());
    }
  }
#endif
*/
  //if(Display::saveVideo()) {
    //saveFrame();
  //}

}

void GLWidget::resizeGL(int width, int height)
{
  glViewport (0, 0, (GLsizei) width, (GLsizei) height);
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();
  gluPerspective(m_camZoom, (GLfloat) width/(GLfloat) height, -1, 1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  if(m_view) {
    vector<double> pos(2);
    pos[0] = pos[1] = 0.0;
    gluLookAt(pos[0], pos[1], 10,
        pos[0], pos[1], 0,
        0, 1, 0);
  }
  else {
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

  if(event->buttons() & Qt::MiddleButton) {
  }
  else if(event->buttons() & Qt::LeftButton){
    m_camPos[0] += -.2 * dx;
    m_camPos[1] += .2 * dy;
    updateGL();
  }
  else if(event->buttons() & Qt::RightButton) {
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

