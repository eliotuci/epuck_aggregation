#ifndef VIEWER_H
#define VIEWER_H

#include <Qt>
//#include <glut.h> // use this on my mac
//#include <OpenGL/glu.h> // use this on my mac
#include <GL/glut.h> // use this on rooster
#include <GL/glu.h> // use this on rooster
#include <QApplication>
#include <QGLViewer/qglviewer.h>
#if QT_VERSION >= 0x040000
# include <QKeyEvent>
#endif

#include "../MISC/general.h"
#include "../EXP/experiment.h"
#include "../WORLD/simple_agents.h"
#include "../WORLD/simple_objects.h"

//#include "video.h"
#include "sky.h"

class Viewer : public QGLViewer
{
  Q_OBJECT;
 public:
  Viewer(QWidget *parent, Experiment *_exp, int argc, char** argv  );
  ~Viewer();
  void keyPressEvent(QKeyEvent *e);
  void glDrawCircle(double x1, double y1, double radius);
  Experiment *exp;
  SKY mySky;
  //Video *video;

 protected:
  virtual void init();
  virtual void draw();

  
 private:
  unsigned char *pRGB;
  double Xcam,Ycam,Zcam,theta,phi;
  //double viewangle;
};

#endif
