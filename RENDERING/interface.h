#ifndef _INTERFACE_
#define _INTERFACE_

#include <QWidget>
#include <QtGui>
#include <QSpinBox>
#include <QImage>
#include <QPainter>
#include "viewer.h"
#include "engine.h"


class QCheckBox;
class QGridLayout;
class QHBoxLayout;
class QLabel;
class QMovie;
class QToolButton;
class QVBoxLayout;

class Interface : public QWidget
{
  Q_OBJECT
    
    public:
  Interface(QWidget *parent, Experiment *_exp, Viewer *_viewer );
  ~Interface();
  
  private slots:
  void open();
  //void active_reset_eval_loop();
  void active_play();
  void active_advance();
  void active_pause();
  void active_stop();
  void active_quit();
  void active_rec();
  void active_stop_rec();

  void updateButtons();
  void setGenotypeNumber( int );
  void setFrameRate( int );
  void changeWindowTitle( );
    
 private:
  Engine* m_engine;   // Engine that run an experiment
  Experiment *exp;
  Viewer *viewer;
  //Video *video;
  int frameRate;

  bool flag_activate_loadGeno_buttons;
  //bool flag_activate_reset_eval_loop_botton;
  bool flag_activate_play_buttons;
  bool flag_activate_step_buttons;
  bool flag_activate_pause_buttons;
  bool flag_activate_next_buttons;
  bool flag_activate_quit_buttons;
  bool flag_activate_rec_buttons;
  bool flag_activate_stop_rec_buttons;

  void createButtons();
  
  QToolButton *openButton;
  //QToolButton *reset_eval_loopButton;
  QToolButton *playButton;
  QToolButton *oneStepButton;
  QToolButton *pauseButton;
  QToolButton *stopButton;
  QToolButton *quitButton;
  QToolButton *recButton;
  QToolButton *stopRecButton;

  QHBoxLayout *buttonsLayout;
  QVBoxLayout *mainLayout;
  
  QByteArray ba;
  QString windowTitle;
  QString pathDirImages;
};

#endif 
