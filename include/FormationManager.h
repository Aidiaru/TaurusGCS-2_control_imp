#pragma once

#ifndef FORMATION_MANAGER_H
#define FORMATION_MANAGER_H

#include "Swarm.h"

#include <QObject>
#include <QVector>

enum class FormationType {
    NONE,
    STRAIGHT_LINE,
    REVERSE_V,
    NORMAL_V
};

class FormationManager : public QObject
{
    Q_OBJECT
    Q_PROPERTY(FormationType currentFormation READ currentFormation NOTIFY currentFormationChanged)
    
public:
    explicit FormationManager(Swarm* swarm, QObject *parent = nullptr);
    ~FormationManager();
    
    Q_INVOKABLE bool setFormation(int formationType);
    Q_INVOKABLE bool changeAltitude(float newAltitude);
    Q_INVOKABLE bool changeFormationDistance(double newDistance);
    Q_INVOKABLE bool landAll();
    
    FormationType currentFormation() const { return m_currentFormation; }
    
signals:
    void currentFormationChanged();
    void formationCommandExecuted(bool success, QString message);
    
private:
    Swarm* m_swarm;
    FormationType m_currentFormation;
    double m_formationDistance = 2.0; // Metre cinsinden
    float m_formationAltitude = 10.0f; // Metre cinsinden
    double v_formation_angle = 30.0; // Derece cinsinden
    
    bool executeFormation(FormationType type);
};

#endif // FORMATION_MANAGER_H 