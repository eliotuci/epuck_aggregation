#ifndef ENGINE_H
#define ENGINE_H

#include <QObject>
#include <QEvent>
#include <QTimerEvent>

#include "../experiment.h"

/*!
 *  Engine implementato senza usare i QThread
 *
 *  \par Motivation
 *    Si basa sull'uso di eventi per non impegnare troppo il GUI-Thread
 *  \par Description
 *    Description coming soon
 *  \par Warnings
 *    If the method advance() is called manually while the Engine playing then the result is unpredictable<br>
 *
 */
class Engine : public QObject
{
    Q_OBJECT
public:
    /*! Construct an Engine object */
    Engine(Experiment *experiment);
    virtual ~Engine();
    typedef enum { playingS, pausedS, stoppedS } e_state;
    /*! return the actual state */
    e_state status() {
        return state;
    };
public slots:

    /*! Initialize the Advancable object and emit the signal initSig if it is activated */
    void initialize();
    /*! Advance the object and emit the signal advanceSig if it is activated */
    void advance();
    /*! Operation performed at the end of simulation of Advancable object (emit the signal if it is activated) */
    void finalize();
    /*! Avvia l'Engine */
    void play();
    /*! Pause the Engine */
    void pause();
    /*! Arresta l'esecuzione dell'Engine */
    void stop();
    /*! Setta il delay in millisecondi */
    void setDelay( unsigned int );
    /*! possible state of the Engine */
signals:
    /*! When Advancable exit from initialize method */
    void initialized();
    /*! When Advancable exit from finalize method */
    void finished();
    /*! When Advancable exit from advance method */
    void advanced();
    /*! When the stop method is called */
    void paused();
    /*! When the stopped method is called */
    void stopped();
protected:

    e_state state;
    bool playone;
    unsigned int timerId;
    bool timerInited;
    unsigned int delayms;

    Experiment* m_experiment;

    void timerEvent( QTimerEvent* e );

private:
    /*! true if it initialized */
    bool isInit;
    /*! true if it finalized */
    bool isFinish;
};


#endif
