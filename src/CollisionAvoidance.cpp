#include "CollisionAvoidance.h"
#include "GeoUtils.h"

#include <QDebug>

#include <algorithm>

CollisionAvoidance::CollisionAvoidance(QVector<std::shared_ptr<Drone>>& drones, double safety_distance, QObject* parent)
    : QObject(parent), drones(drones), safetyDistance(safety_distance)
{
    emit logMessage(QString("Çarpışma önleme sistemi %1m güvenlik mesafesi ile başlatıldı").arg(safetyDistance), "info");
}

CollisionAvoidance::~CollisionAvoidance()
{
    emit logMessage("Çarpışma önleme sistemi kapatılıyor", "info");
}

bool CollisionAvoidance::willDronesCollide(
    int drone1_id, double start_lat1, double start_lon1, float start_alt1, double end_lat1, double end_lon1, float end_alt1,
    int drone2_id, double start_lat2, double start_lon2, float start_alt2, double end_lat2, double end_lon2, float end_alt2)
{
    // Simplified collision detection: check if the paths' minimum distance is less than safety distance
    // For a real implementation, we would need more complex path intersection algorithms
    
    // Here we just check if endpoints are too close
    double distance = GeoUtils::calculateDistance3D(
        end_lat1, end_lon1, end_alt1,
        end_lat2, end_lon2, end_alt2
    );
    
    return distance < safetyDistance;
}

CollisionStatus CollisionAvoidance::checkCollisionBetweenDrones(int drone1_id, int drone2_id, double* distance)
{
    std::lock_guard<std::mutex> lock(avoidanceMutex);
    
    int idx1 = findDroneIndex(drone1_id);
    int idx2 = findDroneIndex(drone2_id);
    
    if (idx1 == -1 || idx2 == -1) {
        if (distance) *distance = 9999.0; // Large value to indicate no collision
        return CollisionStatus::NO_COLLISION;
    }
    
    auto pos1 = drones[idx1]->telemetry_manager->telemetryData().m_position;
    auto pos2 = drones[idx2]->telemetry_manager->telemetryData().m_position;
    
    double dist = GeoUtils::calculateDistance3D(
        pos1.latitude_deg, pos1.longitude_deg, pos1.absolute_altitude_m,
        pos2.latitude_deg, pos2.longitude_deg, pos2.absolute_altitude_m
    );
    
    if (distance) *distance = dist;
    
    if (dist < safetyDistance) {
        return CollisionStatus::CRITICAL_COLLISION;
    }
    
    return CollisionStatus::NO_COLLISION;
}

bool CollisionAvoidance::processFormationTransition(const std::map<int, std::tuple<double, double, float>>& targetPositions,
                                                  std::map<int, std::tuple<double, double, float>>& adjustedPositions)
{
    std::lock_guard<std::mutex> lock(avoidanceMutex);
    bool collisionsDetected = false;
    
    emit logMessage("Formasyon geçişi için çarpışma analizi yapılıyor", "info");
    
    // Initialize adjusted positions with target positions
    adjustedPositions = targetPositions;
    
    // Check each pair of drones for potential collisions in their final positions
    for (auto it1 = targetPositions.begin(); it1 != targetPositions.end(); ++it1) {
        int drone1_id = it1->first;
        auto target1 = it1->second;
        
        for (auto it2 = std::next(it1); it2 != targetPositions.end(); ++it2) {
            int drone2_id = it2->first;
            auto target2 = it2->second;
            
            double distance = GeoUtils::calculateDistance3D(
                std::get<0>(target1), std::get<1>(target1), std::get<2>(target1),
                std::get<0>(target2), std::get<1>(target2), std::get<2>(target2)
            );
            
            if (distance < safetyDistance) {
                collisionsDetected = true;
                
                emit logMessage(QString("Drone %1 ve %2 arasında potansiyel çarpışma tespit edildi. Mesafe: %3m")
                               .arg(drone1_id).arg(drone2_id).arg(distance, 0, 'f', 2), "error");
                
                // Emit signal about collision detection
                emit collisionDetected(drone1_id, drone2_id, distance);
                
                // Resolve collision by adjusting intermediate waypoints
                resolveCollision(drone1_id, drone2_id, target1, target2,
                               adjustedPositions[drone1_id], adjustedPositions[drone2_id]);
                
                // Emit signal about collision resolution
                emit collisionResolved(drone1_id, drone2_id);
                emit logMessage(QString("Drone %1 ve %2 için çarpışma riski çözüldü")
                               .arg(drone1_id).arg(drone2_id), "info");
            }
        }
    }
    
    if (!collisionsDetected) {
        emit logMessage("Çarpışma riski tespit edilmedi", "info");
    } else {
        emit logMessage("Çarpışma riskleri çözüldü, güvenli rota oluşturuldu", "info");
    }
    
    return collisionsDetected;
}

double CollisionAvoidance::getSafetyDistance() const
{
    return safetyDistance;
}

void CollisionAvoidance::resolveCollision(int drone1_id, int drone2_id,
                                       const std::tuple<double, double, float>& target1,
                                       const std::tuple<double, double, float>& target2,
                                       std::tuple<double, double, float>& adjusted_target1,
                                       std::tuple<double, double, float>& adjusted_target2)
{
    // Find the midpoint between the two targets
    double mid_lat = (std::get<0>(target1) + std::get<0>(target2)) / 2.0;
    double mid_lon = (std::get<1>(target1) + std::get<1>(target2)) / 2.0;
    float mid_alt = (std::get<2>(target1) + std::get<2>(target2)) / 2.0;
    
    emit logMessage(QString("Drone %1 ve %2 için çarpışma çözümü hesaplanıyor")
                   .arg(drone1_id).arg(drone2_id), "info");
    
    // Create adjusted waypoints that are at least safetyDistance apart
    // We'll do this by adding an offset perpendicular to the line between targets
    
    // Calculate vector from midpoint to target1
    double vec_lat1 = std::get<0>(target1) - mid_lat;
    double vec_lon1 = std::get<1>(target1) - mid_lon;
    
    // Normalize this vector
    double length = std::sqrt(vec_lat1*vec_lat1 + vec_lon1*vec_lon1);
    if (length > 0) {
        vec_lat1 /= length;
        vec_lon1 /= length;
    }
    
    // Convert minimum safety distance to degrees (approximate)
    double safety_degree = safetyDistance / 111000.0;
    
    // Scale the vector to ensure minimum safety distance
    vec_lat1 *= safety_degree * 1.5;
    vec_lon1 *= safety_degree * 1.5;
    
    // Create adjusted waypoints
    adjusted_target1 = std::make_tuple(
        mid_lat + vec_lat1,
        mid_lon + vec_lon1,
        mid_alt + 1.0f  // Add 1 meter to altitude for extra safety
    );
    
    adjusted_target2 = std::make_tuple(
        mid_lat - vec_lat1,
        mid_lon - vec_lon1,
        mid_alt - 1.0f  // Subtract 1 meter from altitude for extra safety
    );
    
    emit logMessage(QString("Drone %1 ve %2 için güvenli yörünge hesaplandı")
                   .arg(drone1_id).arg(drone2_id), "info");
}

int CollisionAvoidance::findDroneIndex(int drone_id) const
{
    for (int i = 0; i < drones.size(); ++i) {
        if (drones[i]->system && static_cast<int>(drones[i]->system->get_system_id()) == drone_id) {
            return i;
        }
    }
    return -1;
}
