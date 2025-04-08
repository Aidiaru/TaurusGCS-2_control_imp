#pragma once

#ifndef COLLISION_AVOIDANCE_H
#define COLLISION_AVOIDANCE_H

#include "Swarm.h"

#include <QObject>
#include <vector>
#include <memory>
#include <mutex>
#include <functional>
#include <map>

// Define collision status enum
enum class CollisionStatus {
    NO_COLLISION,
    CRITICAL_COLLISION
};

class CollisionAvoidance : public QObject {
    Q_OBJECT
    
public:
    CollisionAvoidance(QVector<std::shared_ptr<Drone>>& drones, double safety_distance = 0.5, QObject* parent = nullptr);
    ~CollisionAvoidance();

    // Check if a collision will occur between two drone paths
    bool willDronesCollide(
        int drone1_id, double start_lat1, double start_lon1, float start_alt1, double end_lat1, double end_lon1, float end_alt1,
        int drone2_id, double start_lat2, double start_lon2, float start_alt2, double end_lat2, double end_lon2, float end_alt2);
    
    // Check collision between two drones at their current positions
    CollisionStatus checkCollisionBetweenDrones(int drone1_id, int drone2_id, double* distance = nullptr);
    
    // Process multiple drones moving simultaneously to detect and resolve collisions
    // Returns true if any collisions were detected and handled
    bool processFormationTransition(const std::map<int, std::tuple<double, double, float>>& targetPositions,
                                   std::map<int, std::tuple<double, double, float>>& adjustedPositions);

    // Get current safety distance
    double getSafetyDistance() const;
    
signals:
    void collisionDetected(int drone1Id, int drone2Id, double distance);
    void collisionResolved(int drone1Id, int drone2Id);
    void logMessage(QString message, QString type); // New signal for log messages
    
private:
    QVector<std::shared_ptr<Drone>>& drones;
    double safetyDistance;
    std::mutex avoidanceMutex;
    
    // Resolve a potential collision between two drones moving to target positions
    void resolveCollision(int drone1_id, int drone2_id,
                         const std::tuple<double, double, float>& target1,
                         const std::tuple<double, double, float>& target2,
                         std::tuple<double, double, float>& adjusted_target1,
                         std::tuple<double, double, float>& adjusted_target2);

    // Find the drone index by system ID
    int findDroneIndex(int drone_id) const;
};

#endif // COLLISION_AVOIDANCE_H