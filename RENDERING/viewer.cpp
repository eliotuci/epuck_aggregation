#include "viewer.h"

using namespace std;
using namespace qglviewer;
Viewer::Viewer(QWidget *parent, Experiment *_exp, int argc, char** argv /*, Video *_video */)
    : QGLViewer(QGLFormat(QGL::Rgba),parent)
{
    exp = _exp;
    //video = _video;
    glutInit(&argc, argv);
    Xcam  = 0.0;
    Ycam  = 1.0;
    Zcam  = 3.0;
    theta = 0.0;
    phi   = -100.0;
}


Viewer::~Viewer()
{
    //  delete[] exp;
}

void Viewer::init()
{

    //  resizeGL(200, 200);
    glClearColor(0.0, 0.0, 0.0, 0.0);
    //glClearDepth(0.0);
    glClear(GL_COLOR_BUFFER_BIT );//| GL_DEPTH_BUFFER_BIT );
    glFlush();
    //setGridIsDrawn();
    setSceneRadius( 3.0 );//300 cm
    setSceneCenter(Vec(0.0, 0.0, 0.0) );
    //showEntireScene();

    camera()->setPosition(Vec(Xcam,Ycam,Zcam));
    //camera()->setPosition(Vec(0,100,200));
    //theta rotates the Camera around its Y axis\, and then phi rotates it around its X axis.
    //float theta=M_PI, phi=(3.0*M_PI)/2.0;
    //camera()->setOrientation(theta, phi );
    camera()->setOrientation(theta, phi);

    //initialiser le ciel
    mySky.LoadTexture( SKY_FRONT, "/home/elt7/Simulations/dynaSim/RENDERING/IMAGES/sky.jpg" );
    mySky.LoadTexture( SKY_BACK, "/home/elt7/Simulations/dynaSim/RENDERING/IMAGES/ground.jpg" );
    mySky.LoadTexture( SKY_RIGHT,"/home/elt7/Simulations/dynaSim/RENDERING/IMAGES/brick.jpg" );
    mySky.LoadTexture( SKY_LEFT, "/home/elt7/Simulations/dynaSim/RENDERING/IMAGES/sky.jpg" );
    mySky.LoadTexture( SKY_TOP, "/home/elt7/Simulations/dynaSim/RENDERING/IMAGES/sky.jpg" );
    mySky.LoadTexture( SKY_BOTTOM, "/home/elt7/Simulations/dynaSim/RENDERING/IMAGES/brick.jpg" );

    /* turn on default lighting */
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    GLfloat ambient[] = { 0.5f, 0.5f, 0.5f };
    GLfloat diffuse[] = { 0.5f, 0.5f, 0.5f , 0.0f};
    GLfloat specular[] = { 0.25f, 0.25f, 0.25f , 0.0f};
    //GLfloat lightcolor[] = { 0.8, 0.8, 0.8 };

    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
}

void Viewer::draw()
{
    GLfloat lightpos[] = { 0.0, 0.0, 7.0, 1.0 };
    glLightfv(GL_LIGHT0, GL_POSITION, lightpos);

    glColor3f(0.0, 0.0, 0.0);
    if (mySky.wantSky())
    {
        //mySky.Set( -18.0, -18.0, 0.0, 36.0f );
        //mySky.Set( -150.0, -150.0, 0.0, 300.0f );
        //mySky.Render( );
    }
    exp->getPlane()[0]->render();
    for(int r = 0; r < exp->getAgents().size(); r++ ){
        //       if(!exp->param->agent[r]->is_removed()){
      exp->getAgents()[r]->render( );
      //printf("\nrobot%d rendered\n",r);
      //      }
    }
    //   glColor3f( exp->param->agent[r]->get_colour()[0],
    // 	       exp->param->agent[r]->get_colour()[1],
    // 	       exp->param->agent[r]->get_colour()[2]);
    //   //exp->param->camera->render( exp->param->agent[r] );
    // }
    
    for(int ob = 0; ob < exp->getObjects().size(); ob++){
      if( !exp->getObjects()[ob]->is_removed() )
	exp->getObjects()[ob]->render( );
    }
}
void Viewer::keyPressEvent(QKeyEvent *e)
{
#if QT_VERSION < 0x040000
    if (e->state() == Qt::NoButton)
#else
    if (e->modifiers() == Qt::NoModifier)
#endif
        switch (e->key())
        {
        /* case Qt::Key_Left:  theta += 0.01; camera()->setOrientation(theta, phi );      updateGL(); break;
      case Qt::Key_Right: theta -= 0.01; camera()->setOrientation(theta, phi );      updateGL(); break;
      case Qt::Key_Up:    phi -= 0.01;   camera()->setOrientation(theta, phi );      updateGL(); break;
      case Qt::Key_Down:  phi += 0.01;   camera()->setOrientation(theta, phi );      updateGL(); break;
      case Qt::Key_L:     Xcam += 0.01;  camera()->setPosition(Vec(Xcam,Ycam,Zcam)); updateGL(); break;
      case Qt::Key_R:     Xcam -= 0.01;  camera()->setPosition(Vec(Xcam,Ycam,Zcam)); updateGL(); break;
      case Qt::Key_U:     Ycam -= 0.01;  camera()->setPosition(Vec(Xcam,Ycam,Zcam)); updateGL(); break;
      case Qt::Key_D:     Ycam += 0.01;  camera()->setPosition(Vec(Xcam,Ycam,Zcam)); updateGL(); break;
      case Qt::Key_I:     Zcam -= 0.01;  camera()->setPosition(Vec(Xcam,Ycam,Zcam)); updateGL(); break;
      case Qt::Key_O:     Zcam += 0.01;  camera()->setPosition(Vec(Xcam,Ycam,Zcam)); updateGL(); break; */
        default: QGLViewer::keyPressEvent(e);           //handler de la classe superieure
        }
    else
        QGLViewer::keyPressEvent(e);
}

void Viewer::glDrawCircle(double x1, double y1, double radius) {

    double x2,y2,angle;

    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(x1,y1);

    for (angle=1.0f;angle<361.0f;angle+=0.2)
    {
        x2 = x1+sin(angle)*radius;
        y2 = y1+cos(angle)*radius;
        glVertex2f(x2,y2);
    }

    glEnd();
}

