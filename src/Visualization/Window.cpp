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

#include <iostream>
#include <QtGui/QtGui>

using namespace std;

#include "../../include/Visualization/GLWidget.h"
#include "../../include/Visualization/Window.h"

//When given a list of polygons
MyWindow::MyWindow()
{
  //define all buttons
  QPushButton* resetCamButton = new QPushButton("Reset Camera", this);
  QPushButton* snapshotButton = new QPushButton("Take Snapshot", this);
  QPushButton* axisButton = new QPushButton("Draw Axes", this);
  axisButton->setCheckable(true);

  QComboBox* modeComboBox = new QComboBox();
  modeComboBox->addItem("Nodes");
  modeComboBox->addItem("Feedback");
  modeComboBox->addItem("PRM");

  glWidget_ = new GLWidget(this);

  //define shortcuts
  resetCamButton->setShortcut(QKeySequence("c"));
  axisButton->setShortcut(QKeySequence("x"));

  //connect to call backs
  connect(&timer_, SIGNAL(timeout()), this, SLOT(Simulate()));
  connect(resetCamButton, SIGNAL(clicked()), this, SLOT(ResetCamera()));
  connect(snapshotButton, SIGNAL(clicked()), glWidget_, SLOT(SaveSnapshot()));
  connect(axisButton, SIGNAL(clicked(bool)), glWidget_, SLOT(DrawAxes(bool)));
  connect(modeComboBox, SIGNAL(currentIndexChanged(int)), glWidget_, SLOT(ChangeMode(int)));

  //Set up layout
  QGridLayout *mainLayout = new QGridLayout;
  mainLayout->addWidget(resetCamButton, 0, 0);
  mainLayout->addWidget(snapshotButton, 0, 1);
  mainLayout->addWidget(axisButton, 0, 2);
  mainLayout->addWidget(modeComboBox, 0, 3);
  mainLayout->addWidget(glWidget_, 1, 0, 1, 4);
  this->setLayout(mainLayout);

  setWindowTitle(tr("FIRM"));
  this->setFocus();
  timer_.start(0.033*1000);
  modeComboBox->setCurrentIndex(1);
}

void MyWindow::simulate(){
  glWidget_->updateGL();

  //if(quit->ShouldQuit()) {
  //  timer_.stop();
  //  close();
  //}
}

void MyWindow::resetCamera(){
  glWidget_->resetCam();
}

void MyWindow::keyPressEvent(QKeyEvent *e) {
  switch(e->key()){
    case Qt::Key_Escape:
      cout << "User pressed Esc to close " << endl;
      close();
      break;
    case 'a': case 'A':
      glWidget_->strafeCam(-1);
      break;
    case 'd': case 'D':
      glWidget_->strafeCam(1);
      break;
    case 's': case 'S':
      glWidget_->moveCam(-1);
      break;
    case 'w': case 'W':
      glWidget_->moveCam(1);
      break;
    case 'i': case 'I':
      glWidget_->zoomCam(-1);
      break;
    case 'o': case 'O':
      glWidget_->zoomCam(1);
      break;
    default:
      QWidget::keyPressEvent(e);
  }
}
