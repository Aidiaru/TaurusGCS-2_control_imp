#pragma once

#ifndef DRONE_CONTROL_H
#define DRONE_CONTROL_H

#include "Swarm.h"
#include "FormationVerifier.h"
#include "CollisionAvoidance.h"

#include <QObject>
#include <QVector>
#include <QString>
#include <QTimer>

#include <memory>
#include <vector>
#include <future>
#include <deque>

enum class FormationType {
    NONE,
    STRAIGHT_LINE,
    REVERSE_V,
    NORMAL_V
};

// Define a structure for formation mission
struct FormationMission {
    FormationType formationType;
    int durationSeconds;
    
    FormationMission(FormationType type = FormationType::NONE, int duration = 0)
        : formationType(type), durationSeconds(duration) {}
};

class DroneControl : public QObject
{
    Q_OBJECT
    Q_PROPERTY(FormationType currentFormation READ currentFormation NOTIFY currentFormationChanged)
    Q_PROPERTY(QString statusMessage READ statusMessage NOTIFY statusMessageChanged)
    
public:
    explicit DroneControl(Swarm* swarm, QObject *parent = nullptr);
    ~DroneControl();
    
    // API Metotları
    Q_INVOKABLE bool armAndTakeoff();
    Q_INVOKABLE bool setFormation(int formationType, int durationSeconds = 0);
    Q_INVOKABLE bool changeAltitude(float newAltitude);
    Q_INVOKABLE bool changeFormationDistance(double newDistance);
    Q_INVOKABLE bool landAllDrones();
    
    // Mission queue methods
    Q_INVOKABLE bool addMissionToQueue(int formationType, int durationSeconds);
    Q_INVOKABLE bool clearMissionQueue();
    Q_INVOKABLE bool executeMissionQueue();
    Q_INVOKABLE int missionQueueSize() { return static_cast<int>(m_missionQueue.size()); }
    
    // Getter fonksiyonları
    FormationType currentFormation() const { return m_currentFormation; }
    QString statusMessage() const { return m_statusMessage; }
    
signals:
    void currentFormationChanged();
    void statusMessageChanged();
    void commandExecuted(bool success, QString message);
    void missionQueueChanged();
    void missionStarted(int missionIndex, int totalMissions, int formationType, int duration);
    void missionCompleted(int missionIndex, int totalMissions);
    void allMissionsCompleted();
    
    // FormationVerifier signals (relayed)
    void formationDeviation(int droneId, double distance);
    void formationCorrectionStarted();
    void formationCorrectionCompleted();
    
    // CollisionAvoidance signals (relayed)
    void collisionDetected(int drone1Id, int drone2Id, double distance);
    void collisionResolved(int drone1Id, int drone2Id);
    
    // Log message signal to relay messages from verifiers
    void logMessage(QString message, QString type);
    
private slots:
    void checkFormationStatus();
    void onFormationDeviation(int droneId, double distance);
    void onFormationCorrectionStarted();
    void onFormationCorrectionCompleted();
    void onCollisionDetected(int drone1Id, int drone2Id, double distance);
    void onCollisionResolved(int drone1Id, int drone2Id);
    void initializeVerifiersIfNeeded();
    void executeMissionAtIndex(int index);
    
private:
    Swarm* m_swarm;
    FormationType m_currentFormation;
    QString m_statusMessage;
    double m_formationDistance = 2.0;       // Metre cinsinden
    float m_formationAltitude = 10.0f;      // Metre cinsinden
    double v_formation_angle = 30.0;        // Derece cinsinden
    QTimer* m_statusTimer;
    std::unique_ptr<FormationVerifier> m_formationVerifier;
    std::unique_ptr<CollisionAvoidance> m_collisionAvoidance;
    
    // Mission queue
    std::deque<FormationMission> m_missionQueue;
    bool m_isMissionRunning = false;
    int m_currentMissionIndex = -1;
    
    bool executeFormation(FormationType type, int durationSeconds = 0);
    void handle_formation_change(FormationType choice, int duration = 0);
    void updateFormationTargets();
    void setStatusMessage(const QString& message);
    QString getTimeString() const;
    
    // İç fonksiyonlar
    bool generateTargetPositions(FormationType targetFormation, 
                               std::map<int, std::tuple<double, double, float>>& targetPositions);
};

#endif // DRONE_CONTROL_H