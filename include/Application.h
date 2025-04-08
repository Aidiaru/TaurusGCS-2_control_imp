#pragma once

#ifndef APPLICATION_H
#define APPLICATION_H

#include "Swarm.h"
#include "DroneControl.h"

#include <QtGui/QGuiApplication>
#include <QtQml/QQmlApplicationEngine>

#include <memory>

#define APP_NAME    "TaurusGCS"
#define ORG_NAME    "Taurus"
#define ORG_DOMAIN  "taurus.team"

class Application : public QGuiApplication {
    Q_OBJECT

public:
    Application(int argc, char *argv[]);
    ~Application();

    void init();
    void shutdown();

    QQmlApplicationEngine* qmlAppEngine() const { return m_engine; }
signals:

private:
    QQmlApplicationEngine *m_engine;
    Swarm *m_swarm;
    DroneControl *m_droneControl;
};

#endif // APPLICATION_H