#pragma once

#include "Swarm.h"

#include <QObject>

#include <thread>
#include <mutex>
#include <atomic>
#include <map>
#include <tuple>

class FormationVerifier : public QObject {
    Q_OBJECT
    
public:
    FormationVerifier(QVector<std::shared_ptr<Drone>>& drones, double position_tolerance = 0.5, QObject* parent = nullptr);
    ~FormationVerifier();

    void start();
    void stop();
    void setTargetPositions(const std::map<int, std::tuple<double, double, float>>& positions);
    void updateDroneTarget(int droneId, double lat, double lon, float alt);
    bool isRunning() const;
    
signals:
    void formationDeviation(int droneId, double distance);
    void formationCorrectionStarted();
    void formationCorrectionCompleted();
    void logMessage(QString message, QString type); // New signal for log messages
    
private:
    QVector<std::shared_ptr<Drone>> drones;
    std::thread verifierThread;
    std::atomic<bool> running;
    double positionTolerance;
    std::mutex targetMutex;
    std::map<int, std::tuple<double, double, float>> targetPositions;
    
    void verificationLoop();
    void checkAndCorrectPositions();
    void correctFormation();
};
