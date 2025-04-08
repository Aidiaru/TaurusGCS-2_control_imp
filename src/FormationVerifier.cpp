#include <FormationVerifier.h>
#include <GeoUtils.h>

#include <QDebug>

FormationVerifier::FormationVerifier(QVector<std::shared_ptr<Drone>>& drones, double position_tolerance, QObject* parent)
    : QObject(parent), 
      drones(drones), 
      running(false),
      positionTolerance(position_tolerance)
{
}

FormationVerifier::~FormationVerifier()
{
    qDebug() << "Destroying formation verifier...";
    stop();
}

void FormationVerifier::start()
{
    qDebug() << "Starting formation verifier...";
    if (!running) {
        running = true;
        verifierThread = std::thread(&FormationVerifier::verificationLoop, this);
    }
}

void FormationVerifier::stop()
{
    if (running) {
        running = false;
        if (verifierThread.joinable()) {
            verifierThread.join();
        }
    }
}

void FormationVerifier::setTargetPositions(const std::map<int, std::tuple<double, double, float>>& positions)
{
    std::lock_guard<std::mutex> lock(targetMutex);
    targetPositions = positions;
}

void FormationVerifier::updateDroneTarget(int droneId, double lat, double lon, float alt)
{
    std::lock_guard<std::mutex> lock(targetMutex);
    targetPositions[droneId] = std::make_tuple(lat, lon, alt);
}

bool FormationVerifier::isRunning() const
{
    return running;
}

void FormationVerifier::verificationLoop()
{
    using namespace std::chrono_literals;
    
    emit logMessage("Formasyon doğrulayıcı başlatıldı", "info");
    
    while (running) {
        checkAndCorrectPositions();
        std::this_thread::sleep_for(3s); // Check every 3 seconds
    }
    
    emit logMessage("Formasyon doğrulayıcı durduruldu", "info");
}

void FormationVerifier::checkAndCorrectPositions()
{
    std::lock_guard<std::mutex> lock(targetMutex);
    
    if (targetPositions.empty()) {
        // No target positions defined yet
        return;
    }
    
    bool correctionNeeded = false;
    std::map<int, double> deviations;
    
    // Check each drone's position against its target
    for (const auto& drone : drones) {
        if (!drone->system || !drone->telemetry_manager) continue;
        
        int droneId = static_cast<int>(drone->system->get_system_id());
        
        auto targetIter = targetPositions.find(droneId);
        if (targetIter == targetPositions.end()) {
            // No target position for this drone
            continue;
        }
        
        auto currentPosition = drone->telemetry_manager->telemetryData().m_position;
        auto targetPosition = targetIter->second;
        
        double targetLat = std::get<0>(targetPosition);
        double targetLon = std::get<1>(targetPosition);
        float targetAlt = std::get<2>(targetPosition);
        
        double distance = GeoUtils::calculateDistance3D(
            currentPosition.latitude_deg, currentPosition.longitude_deg, currentPosition.absolute_altitude_m,
            targetLat, targetLon, targetAlt
        );
        
        // If distance exceeds tolerance, mark for correction
        if (distance > positionTolerance) {
            correctionNeeded = true;
            deviations[droneId] = distance;
            emit formationDeviation(droneId, distance);
            emit logMessage(QString("Drone %1: Formasyondan %.2f m sapma tespit edildi").arg(droneId).arg(distance), "warning");
        }
    }
    
    // If any drone needs correction, initiate formation adjustment
    if (correctionNeeded) {
        emit logMessage(QString("Formasyon sapması tespit edildi: %1 drone pozisyon dışında").arg(deviations.size()), "warning");
        
        // Trigger formation re-adjustment
        correctFormation();
    }
}

void FormationVerifier::correctFormation()
{
    emit logMessage("Formasyon düzeltme işlemi başlıyor...", "warning");
    emit formationCorrectionStarted();
    
    // Move all drones back to their target positions simultaneously
    for (auto& drone : drones) {
        if (!drone->system || !drone->action) continue;
        
        int droneId = static_cast<int>(drone->system->get_system_id());
        
        auto targetIter = targetPositions.find(droneId);
        if (targetIter == targetPositions.end()) {
            // No target position for this drone
            continue;
        }
        
        auto targetPosition = targetIter->second;
        double targetLat = std::get<0>(targetPosition);
        double targetLon = std::get<1>(targetPosition);
        float targetAlt = std::get<2>(targetPosition);
        
        // Send goto_location command
        auto result = drone->action->goto_location(
            targetLat, 
            targetLon, 
            targetAlt, 
            0.0f // Arbitrary heading
        );
        
        if (result != mavsdk::Action::Result::Success) {
            emit logMessage(QString("Drone %1 için goto_location komutu başarısız!").arg(droneId), "error");
        } else {
            emit logMessage(QString("Drone %1 hedef konumuna yeniden yönlendiriliyor").arg(droneId), "info");
        }
    }
    
    emit logMessage("Formasyon düzeltme komutları tüm drone'lara gönderildi", "info");
    emit formationCorrectionCompleted();
}
