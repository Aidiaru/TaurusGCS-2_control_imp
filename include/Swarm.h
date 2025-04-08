#pragma once

#ifndef SWARM_H
#define SWARM_H

#include "Drone.h"

#include <QObject>
#include <memory>
#include <vector>

class Swarm : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QVector<std::shared_ptr<Drone>> drones READ getDrones NOTIFY swarmUpdated)

public:
    explicit Swarm(QObject *parent = nullptr);
    ~Swarm();

    Q_INVOKABLE void addDrone(const QString& connection_url);
    Q_INVOKABLE void addMultipleDrones(int count, int startPort);
    Q_INVOKABLE void removeDrone(int id);
    Q_INVOKABLE void setLeader(int id);
    Q_INVOKABLE TelemetryData* getTelemetry(int droneId);
    Q_INVOKABLE QVector<std::shared_ptr<Drone>> getDrones();
    Q_INVOKABLE int getLeaderId() { return leaderId; }
    Q_INVOKABLE int size() { return drones.size(); }
signals:
    void swarmUpdated();

private:
    QVector<std::shared_ptr<Drone>> drones;
    int leaderId = -1;

};

#endif // SWARM_H