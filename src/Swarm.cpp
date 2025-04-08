#include <Drone.h>
#include <Swarm.h>

#include <thread>
#include <chrono>

using namespace mavsdk;

Swarm::Swarm(QObject *parent) : QObject(parent), leaderId(-1) {
    drones = QVector<std::shared_ptr<Drone>>();
}

Swarm::~Swarm() {
    for (auto &drone : drones) {
        drone->mavsdk.reset();
    }
}

void Swarm::addDrone(const QString&  connection_url) {
    std::shared_ptr<Drone> drone = std::make_shared<Drone>();
    if (init_drone(drone, connection_url)) {
        if (drones.empty()) {
            leaderId = 1;
        }
        drones.push_back(drone);
        emit swarmUpdated();
    }
}

void Swarm::addMultipleDrones(int count, int startPort) {
    qDebug() << "Connecting to" << count << "drones starting at port" << startPort;
    
    for (int i = 0; i < count; i++) {
        int port = startPort + i;
        QString connection_url = QString("udp://127.0.0.1:%1").arg(port);
        qDebug() << "Trying to connect to drone at" << connection_url;
        
        std::shared_ptr<Drone> drone = std::make_shared<Drone>();
        if (init_drone(drone, connection_url)) {
            qDebug() << "Successfully connected to drone at" << connection_url;
            drones.push_back(drone);
        }
    }
    
    // İlk bağlanan dronu lider olarak ata (eğer hiç lider yoksa)
    if (!drones.empty() && leaderId < 0) {
        leaderId = 1;
    }
    
    emit swarmUpdated();
}

// Ortadan bir drone silinirse hata verir !! 
void Swarm::removeDrone(int droneId) {  //düzeltilmesi  gerek (id ataması neye göre yapılıyor ona bağlı)
    for (int i = 0; i <= drones.size(); ++i) {
        if (i+1 == droneId) {
            drones[i]->mavsdk.reset();
            drones.remove(i);
            drones[i]->telemetry_manager->stop();
            if (droneId == leaderId) {
                leaderId = -1;
            }
            break;
        }
    }
    
    emit swarmUpdated();

}

void Swarm::setLeader(int droneId) {
    if (droneId < drones.size()) {
        for (int i = 0; i < drones.size(); ++i) {
            if (i == droneId) {
                leaderId = i;
                emit swarmUpdated();
                return;
            }
        }
    }
    qDebug() << "Invalid drone id";
}
TelemetryData* Swarm::getTelemetry(int droneId) {
    if (drones.empty() || droneId < 0 || droneId > drones.size()) {
        qDebug() << "Invalid Drone ID: " << droneId;
        return nullptr;
    }

    auto& drone = drones[droneId-1];
    if (!drone || !drone->telemetry_manager) {
        qDebug() << "TelemetryManager not found! Drone ID: " << droneId;
        return nullptr;
    }

    return &(drone->telemetry_manager->telemetryData());
}


QVector<std::shared_ptr<Drone>> Swarm::getDrones() {
    return drones;
}