#include "engine.h"
#include <QCoreApplication>
#include <QEvent>
#include <QTimerEvent>


Engine::Engine(Experiment *experiment)
    : m_experiment(experiment)
{
    isInit = false;
    isFinish = true;
    state = stoppedS;
    playone = false;
    this->delayms = 0;
    timerInited = false;
    return;
}

Engine::~Engine(){};

void Engine::setDelay( unsigned int x ) {
    if ( timerInited ) {
        killTimer( timerId );
        timerInited = false;
    }
    //if( x < 5 ) x = 0;
    delayms = x;
}

void Engine::play() {
    state = playingS;
    timerId = startTimer( delayms );
    timerInited = true;
    return;
}

void Engine::pause() {
    state = pausedS;
    if ( timerInited ) {
        killTimer( timerId );
        timerInited = false;
    }
    return;
}

void Engine::stop() {
    finalize();
    state = stoppedS;
    if ( timerInited ) {
        killTimer( timerId );
        timerInited = false;
    }
    return;
}

void Engine::timerEvent( QTimerEvent* ) {
    advance();
    if ( playone ) {
        state = pausedS;
    }
    switch( state ) {
    case playingS:
        break;
    case pausedS:
        emit paused();
        break;
    case stoppedS:
        emit stopped();
        break;
    }
    return;
}

void Engine::initialize() {
    if ( isInit ) return;
    if ( !isFinish ) {
        finalize();
    }
    m_experiment->init_single_evaluation();
    isInit = true;
    isFinish = false;
#ifdef S_DEBUG
    qDebug( QString("== Initialized %1").arg(this->metaObject()->className()).toLocal8Bit().data() );
#endif
    emit initialized();
}

void Engine::advance() {
    if ( !isInit ) {
        initialize();
    }
    m_experiment->adv();
#ifdef S_DEBUG
    qDebug( QString("== Advanced %1").arg(this->metaObject()->className()).toLocal8Bit().data() );
#endif
    emit advanced();
}

void Engine::finalize() {
    if ( isFinish ) return;
    // this remain commented 23 Nov 2016 finalise_trial();
    m_experiment->stop_evaluations_loop( );
    isFinish = true;
    isInit = false;
#ifdef S_DEBUG
    qDebug( QString("== Finished %1").arg(this->metaObject()->className()).toLocal8Bit().data() );
#endif
    emit finished();
}
