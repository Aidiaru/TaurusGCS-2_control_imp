#include <DroneControl.h>
#include <Formation.h>
#include <GeoUtils.h>

#include <QDebug>
#include <QDateTime>

#include <cmath>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <future>
#include <thread>

using namespace std::chrono_literals;
using namespace mavsdk;

// Yardımcı fonksiyon - drone arm ve takeoff
void arm_and_takeoff(std::shared_ptr<Drone> drone) {
    if (!drone || !drone->system) {
        qDebug() << "Drone veya system nesnesi bulunamadı!";
        return;
    }
    
    while (drone->system->get_system_id() == 0) {
        std::this_thread::sleep_for(100ms);
    }
    
    auto system_id = static_cast<int>(drone->system->get_system_id());

    if (drone->action->arm() != Action::Result::Success) {
        qDebug() << "Drone" << system_id << "arm edilemedi!";
        return;
    }

    if (drone->action->takeoff() != Action::Result::Success) {
        qDebug() << "Drone" << system_id << "takeoff komutu başarısız!";
        return;
    }
}

DroneControl::DroneControl(Swarm* swarm, QObject *parent)
    : QObject(parent), 
      m_swarm(swarm), 
      m_currentFormation(FormationType::NONE),
      m_statusMessage("Hazır"),
      m_isMissionRunning(false),
      m_currentMissionIndex(-1)
{
    // Durum kontrolü için zamanlayıcı oluştur
    m_statusTimer = new QTimer(this);
    connect(m_statusTimer, &QTimer::timeout, this, &DroneControl::checkFormationStatus);
    m_statusTimer->start(5000); // Her 5 saniyede bir kontrol et
    
    // Swarm güncellenince FormationVerifier ve CollisionAvoidance'ı yeniden başlat
    connect(m_swarm, &Swarm::swarmUpdated, this, &DroneControl::initializeVerifiersIfNeeded);
    
    // İlk başlatma
    initializeVerifiersIfNeeded();
}

DroneControl::~DroneControl()
{
    m_statusTimer->stop();
    
    // Formation verifier'ı durdur
    if (m_formationVerifier && m_formationVerifier->isRunning()) {
        m_formationVerifier->stop();
    }
    
    // Collision avoidance sistemini durdur (gerekirse)
    if (m_collisionAvoidance) {
        // CollisionAvoidance sınıfında bir stop metodu yoksa, bu satırı kaldırın
        // m_collisionAvoidance->stop();
    }
}

void DroneControl::initializeVerifiersIfNeeded()
{
    auto drones = m_swarm->getDrones();
    
    // FormationVerifier başlatma veya güncelleme
    if (!m_formationVerifier) {
        // İlk kez oluşturma
        m_formationVerifier = std::make_unique<FormationVerifier>(drones);
        connect(m_formationVerifier.get(), &FormationVerifier::formationDeviation,
                this, &DroneControl::onFormationDeviation);
        connect(m_formationVerifier.get(), &FormationVerifier::formationCorrectionStarted,
                this, &DroneControl::onFormationCorrectionStarted);
        connect(m_formationVerifier.get(), &FormationVerifier::formationCorrectionCompleted,
                this, &DroneControl::onFormationCorrectionCompleted);
        
        // Connect log messages to our relay signal
        connect(m_formationVerifier.get(), &FormationVerifier::logMessage,
                this, &DroneControl::logMessage);
        
        // Dronlar varsa başlat
        if (!drones.isEmpty()) {
            m_formationVerifier->start();
            qDebug() << "FormationVerifier başlatıldı";
        } else {
            qDebug() << "FormationVerifier oluşturuldu, ancak drone olmadığı için beklemede";
        }
    } else {
        // Verifier zaten varsa, durumunu kontrol et
        bool wasRunning = m_formationVerifier->isRunning();
        
        // Önce durdur
        if (wasRunning) {
            m_formationVerifier->stop();
        }
        
        // Dronlar varsa ve daha önce çalışıyorsa tekrar başlat
        // if (!drones.isEmpty() && wasRunning) {
        if (!drones.isEmpty()) {
            m_formationVerifier->start();
            qDebug() << "FormationVerifier yeniden başlatıldı";
        }
    }
    
    // CollisionAvoidance başlatma veya güncelleme
    if (!m_collisionAvoidance) {
        // İlk kez oluşturma
        m_collisionAvoidance = std::make_unique<CollisionAvoidance>(drones, 0.7);
        connect(m_collisionAvoidance.get(), &CollisionAvoidance::collisionDetected,
                this, &DroneControl::onCollisionDetected);
        connect(m_collisionAvoidance.get(), &CollisionAvoidance::collisionResolved,
                this, &DroneControl::onCollisionResolved);
        
        // Connect log messages to our relay signal
        connect(m_collisionAvoidance.get(), &CollisionAvoidance::logMessage,
                this, &DroneControl::logMessage);
                
        qDebug() << "CollisionAvoidance oluşturuldu";
    }
    
    // Mevcut hedefler ve formasyonlar varsa güncelle
    if (!drones.isEmpty() && m_currentFormation != FormationType::NONE) {
        updateFormationTargets();
    }
}

void DroneControl::onFormationDeviation(int droneId, double distance)
{
    setStatusMessage(QString("Drone %1 formasyondan %.2f metre saptı").arg(droneId).arg(distance));
    emit formationDeviation(droneId, distance);
}

void DroneControl::onFormationCorrectionStarted()
{
    setStatusMessage("Formasyon düzeltme işlemi başladı");
    emit formationCorrectionStarted();
}

void DroneControl::onFormationCorrectionCompleted()
{
    setStatusMessage("Formasyon düzeltme tamamlandı");
    emit formationCorrectionCompleted();
}

void DroneControl::onCollisionDetected(int drone1Id, int drone2Id, double distance)
{
    setStatusMessage(QString("UYARI: Drone %1 ve %2 arasında çarpışma riski! Mesafe: %.2fm")
                    .arg(drone1Id).arg(drone2Id).arg(distance));
    emit collisionDetected(drone1Id, drone2Id, distance);
}

void DroneControl::onCollisionResolved(int drone1Id, int drone2Id)
{
    setStatusMessage(QString("Çarpışma riski çözüldü: Drone %1 ve %2")
                    .arg(drone1Id).arg(drone2Id));
    emit collisionResolved(drone1Id, drone2Id);
}

bool DroneControl::armAndTakeoff()
{
    if (!m_swarm || m_swarm->size() == 0) {
        setStatusMessage("Sürüde drone bulunamadı");
        emit commandExecuted(false, "Sürüde drone bulunamadı");
        return false;
    }
    
    auto drones = m_swarm->getDrones();
    setStatusMessage("Dronelar kalkış için hazırlanıyor...");
    
    std::vector<std::future<void>> futures;
    for (auto& drone : drones) {
        futures.push_back(std::async(std::launch::async, [&drone]() {
            arm_and_takeoff(drone);
        }));
    }
    
    // Tüm droneların kalkış komutlarını tamamlamasını bekle
    for (auto& fut : futures) {
        fut.get();
    }
    
    setStatusMessage("Tüm dronelar havada");
    emit commandExecuted(true, "Tüm dronelar için kalkış komutu gönderildi");
    
    return true;
}

bool DroneControl::setFormation(int formationType, int durationSeconds)
{
    FormationType targetFormation = static_cast<FormationType>(formationType);
    handle_formation_change(targetFormation, durationSeconds);
    return true;
}

void DroneControl::handle_formation_change(FormationType targetFormation, int duration)
{
    if (!m_swarm || m_swarm->size() == 0) {
        setStatusMessage("Sürüde drone bulunamadı");
        emit commandExecuted(false, "Sürüde drone bulunamadı");
        return;
    }
    
    if (targetFormation == m_currentFormation && targetFormation != FormationType::NONE) {
        setStatusMessage("Dronelar zaten bu formasyonda");
        emit commandExecuted(true, "Dronelar zaten bu formasyonda");
        return;
    }
    
    auto drones = m_swarm->getDrones();
    
    // Lider drone'u belirle
    int leaderId = m_swarm->getLeaderId();
    if (leaderId < 0 || leaderId >= drones.size()) {
        leaderId = 0;
    }
    
    // Referans pozisyonunu lider drondan al
    auto ref_telemetry = m_swarm->getTelemetry(leaderId + 1);
    if (!ref_telemetry) {
        setStatusMessage("Lider drone telemetrisi alınamadı");
        emit commandExecuted(false, "Lider drone telemetrisi alınamadı");
        return;
    }
    
    double ref_lat = ref_telemetry->m_position.latitude_deg;
    double ref_lon = ref_telemetry->m_position.longitude_deg;
    float ref_alt = ref_telemetry->m_position.absolute_altitude_m;
    
    // Mesafe değerini derece cinsine çevir (yaklaşık olarak)
    const double meter_to_degree = 1.0 / 111000.0;
    
    // Düz çizgi formasyonu için offset değeri
    const double lat_offset = m_formationDistance * meter_to_degree;
    
    // V formasyonları için açı değerini radyana çevir
    const double angle_rad = v_formation_angle * M_PI / 180.0;
    
    // V formasyonları için offset değerleri
    const double v_lat_offset = m_formationDistance * meter_to_degree * cos(angle_rad / 2);
    const double v_lon_offset = m_formationDistance * meter_to_degree * sin(angle_rad / 2);
    
    std::vector<std::future<void>> futures;
    
    // İniş komutu mu
    if (targetFormation == FormationType::NONE) {
        setStatusMessage("İniş komutu alındı");
        
        // Eğer düz çizgi formasyonundaysa, doğrudan iniş yap
        if (m_currentFormation == FormationType::STRAIGHT_LINE) {
            for (int i = 0; i < drones.size(); ++i) {
                double target_lat, target_lon;
                land_straight_line_formation(i, ref_lat, ref_lon, lat_offset, target_lat, target_lon);
                
                auto drone = drones[i];
                futures.push_back(std::async(std::launch::async, 
                    [drone, target_lat, target_lon, ref_alt]() {
                        drone->action->goto_location(target_lat, target_lon, ref_alt, 0.0f);
                    }
                ));
            }
            for (auto& fut : futures) fut.get();
        }
        
        // Tüm droneları indir
        landAllDrones();
        return;
    }
    
    setStatusMessage("Hedef formasyon hazırlanıyor: " + QString::number(static_cast<int>(targetFormation)));
    
    // Tüm hedef pozisyonları hesapla
    std::map<int, std::tuple<double, double, float>> targetPositions;
    std::map<int, std::tuple<double, double, float>> adjustedPositions;
    
    for (int i = 0; i < drones.size(); i++) {
        if (!drones[i]->system) continue;
        
        int droneId = static_cast<int>(drones[i]->system->get_system_id());
        double target_lat = ref_lat;
        double target_lon = ref_lon;
        float target_alt = ref_alt;
        
        // Hedef pozisyonu formasyon tipine göre hesapla
        switch (targetFormation) {
            case FormationType::STRAIGHT_LINE:
                target_lat = ref_lat + (i * lat_offset); // Latitude boyunca düzenle
                break;
                
            case FormationType::REVERSE_V:
                calculate_reverse_v_position(
                    i, ref_lat, ref_lon, ref_alt,
                    v_lat_offset, v_lon_offset,
                    target_lat, target_lon, target_alt
                );
                break;
                
            case FormationType::NORMAL_V:
                calculate_v_position(
                    i, ref_lat, ref_lon, ref_alt,
                    v_lat_offset, v_lon_offset,
                    target_lat, target_lon, target_alt
                );
                break;
                
            default:
                break;
        }
        
        targetPositions[droneId] = std::make_tuple(target_lat, target_lon, target_alt);
    }
    
    // Formation verifier'ı geçici olarak durdur
    bool wasVerifierRunning = false;
    if (m_formationVerifier && m_formationVerifier->isRunning()) {
        m_formationVerifier->stop();
        wasVerifierRunning = true;
    }
    
    // Çarpışma kontrolü yap ve gerekirse yolları ayarla
    bool collisionsDetected = false;
    if (m_collisionAvoidance) {
        collisionsDetected = m_collisionAvoidance->processFormationTransition(targetPositions, adjustedPositions);
    } else {
        adjustedPositions = targetPositions;
    }
    
    // Hareket önceliği sıralaması yap
    // Önce hedefine yakın olanların hareket etmesini sağla
    std::vector<int> droneOrder;
    for (int i = 0; i < drones.size(); i++) {
        droneOrder.push_back(i);
    }
    
    // Droneları hedeflerine uzaklıklarına göre sırala
    std::sort(droneOrder.begin(), droneOrder.end(), 
        [&drones, &targetPositions, this](int a, int b) {
            if (!drones[a]->system || !drones[b]->system || 
                !drones[a]->telemetry_manager || !drones[b]->telemetry_manager) {
                return false;
            }
            
            int droneIdA = static_cast<int>(drones[a]->system->get_system_id());
            int droneIdB = static_cast<int>(drones[b]->system->get_system_id());
            
            auto posA = drones[a]->telemetry_manager->telemetryData().m_position;
            auto posB = drones[b]->telemetry_manager->telemetryData().m_position;
            
            auto targetPosA = targetPositions.find(droneIdA);
            auto targetPosB = targetPositions.find(droneIdB);
            
            if (targetPosA == targetPositions.end() || targetPosB == targetPositions.end()) {
                return false;
            }
            
            double distA = GeoUtils::calculateDistance3D(
                posA.latitude_deg, posA.longitude_deg, posA.absolute_altitude_m,
                std::get<0>(targetPosA->second), std::get<1>(targetPosA->second), std::get<2>(targetPosA->second)
            );
            
            double distB = GeoUtils::calculateDistance3D(
                posB.latitude_deg, posB.longitude_deg, posB.absolute_altitude_m,
                std::get<0>(targetPosB->second), std::get<1>(targetPosB->second), std::get<2>(targetPosB->second)
            );
            
            return distA < distB; // Hedefine daha yakın olan önce hareket etsin
        });
    
    // İlk aşama: Eğer çarpışma tespit edildiyse, droneları güvenli geçici pozisyonlara taşı
    if (collisionsDetected) {
        setStatusMessage("Çarpışma önleme sistemi aktif - güvenli ara pozisyonlara hareket ediliyor");
        
        // Droneları ayarlanmış pozisyonlara taşı
        for (int idx : droneOrder) {
            auto& drone = drones[idx];
            if (!drone->system || !drone->action) continue;
            
            int droneId = static_cast<int>(drone->system->get_system_id());
            auto adjustedPosIter = adjustedPositions.find(droneId);
            if (adjustedPosIter == adjustedPositions.end()) continue;
            
            auto adjustedPos = adjustedPosIter->second;
            
            futures.push_back(std::async(std::launch::async,
                [drone, adjustedPos]() {
                    drone->action->goto_location(
                        std::get<0>(adjustedPos),
                        std::get<1>(adjustedPos),
                        std::get<2>(adjustedPos),
                        0.0f
                    );
                }
            ));
            
            // Her drone hareketi arasında kısa bir gecikme ekle
            std::this_thread::sleep_for(std::chrono::milliseconds(150));
        }
        
        // Tüm droneların hareket etmesini bekle
        for (auto& fut : futures) {
            fut.get();
        }
        
        futures.clear();
        
        // Droneların ara pozisyonlara ulaşmasını bekle
        setStatusMessage("Dronelar güvenli ara pozisyonlara gidiyor...");
        std::this_thread::sleep_for(std::chrono::seconds(4));
    }
    
    // İkinci aşama: Hedef pozisyonlara hareket et
    setStatusMessage("Dronelar hedef formasyon pozisyonlarına gidiyor");
    
    // Droneları hedef pozisyonlara taşı
    for (int idx : droneOrder) {
        auto& drone = drones[idx];
        if (!drone->system || !drone->action) continue;
        
        int droneId = static_cast<int>(drone->system->get_system_id());
        auto targetPosIter = targetPositions.find(droneId);
        if (targetPosIter == targetPositions.end()) continue;
        
        auto targetPos = targetPosIter->second;
        
        futures.push_back(std::async(std::launch::async,
            [drone, targetPos]() {
                drone->action->goto_location(
                    std::get<0>(targetPos),
                    std::get<1>(targetPos),
                    std::get<2>(targetPos),
                    0.0f
                );
            }
        ));
        
        // Her drone hareketi arasında kısa bir gecikme ekle
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }
    
    // Tüm droneların hareket etmesini bekle
    for (auto& fut : futures) {
        fut.get();
    }
    
    // Mevcut formasyon tipini güncelle
    m_currentFormation = targetFormation;
    emit currentFormationChanged();
    
    // Formation verifier'ı güncelle ve yeniden başlat
    updateFormationTargets();
    if (wasVerifierRunning && m_formationVerifier) {
        m_formationVerifier->start();
    }
    
    setStatusMessage("Formasyon başarıyla uygulandı: " + QString::number(static_cast<int>(m_currentFormation)));
    emit commandExecuted(true, "Formasyon başarıyla uygulandı");
    
    // Eğer süre belirtildiyse ve bir görev çalışmıyorsa, o süre kadar bekleyip iniş yap
    if (duration > 0 && !m_isMissionRunning) {
        setStatusMessage(QString("Dronelar %1 saniye boyunca bu formasyonda kalacak").arg(duration));
        
        // Yeni thread içinde bekleme işlemini yap
        std::thread([this, duration]() {
            // Her saniye geri sayım göster
            for (int i = duration; i > 0; i--) {
                setStatusMessage(QString("Kalan süre: %1 saniye").arg(i));
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            
            setStatusMessage("Formasyon süresi doldu. Dronelar iniş yapıyor...");
            
            // UI thread üzerinden iniş komutunu çalıştır
            QMetaObject::invokeMethod(this, "landAllDrones", Qt::QueuedConnection);
        }).detach();
    }
}

bool DroneControl::changeAltitude(float newAltitude)
{
    if (!m_swarm || m_swarm->size() == 0) {
        setStatusMessage("Sürüde drone bulunamadı");
        emit commandExecuted(false, "Sürüde drone bulunamadı");
        return false;
    }
    
    if (newAltitude < 2.0f) {
        newAltitude = 2.0f; // Minimum güvenli yükseklik
        setStatusMessage("Uyarı: Çok düşük irtifa! Minimum 2m olarak ayarlandı.");
    }
    else if (newAltitude > 100.0f) {
        newAltitude = 100.0f; // Maksimum izin verilen yükseklik
        setStatusMessage("Uyarı: Çok yüksek irtifa! Maksimum 100m olarak ayarlandı.");
    }
    
    m_formationAltitude = newAltitude;
    setStatusMessage("İrtifa değeri " + QString::number(m_formationAltitude) + "m olarak ayarlandı");
    
    auto drones = m_swarm->getDrones();
    std::vector<std::future<void>> futures;
    
    // Her drone için mevcut konumunu al ve sadece yüksekliği değiştir
    for (auto& drone : drones) {
        if (!drone || !drone->action || !drone->telemetry_manager) continue;
        
        auto telemetry = drone->telemetry_manager->telemetryData();
        
        futures.push_back(std::async(std::launch::async, [drone, newAltitude, telemetry]() {
            auto result = drone->action->goto_location(
                telemetry.m_position.latitude_deg,
                telemetry.m_position.longitude_deg,
                telemetry.m_homePosition.absolute_altitude_m + newAltitude,
                0.0f // Heading değişmeyecek
            );
            
            if (result != Action::Result::Success) {
                qDebug() << "Drone" << drone->system->get_system_id() 
                      << "için yükseklik değiştirme başarısız!";
            }
        }));
    }
    
    // Tüm drone'ların hareket etmeye başlamasını bekle
    for (auto& fut : futures) {
        fut.get();
    }
    
    // Formation verifier'ı güncelle
    updateFormationTargets();
    
    emit commandExecuted(true, "İrtifa değeri güncellendi");
    return true;
}

bool DroneControl::changeFormationDistance(double newDistance)
{
    if (newDistance <= 0.5) {
        newDistance = 0.5; // Minimum güvenli mesafe
        setStatusMessage("Uyarı: Çok küçük mesafe değeri! Minimum 0.5m olarak ayarlandı.");
    } 
    else if (newDistance > 10.0) {
        newDistance = 10.0; // Maksimum izin verilen mesafe
        setStatusMessage("Uyarı: Çok büyük mesafe değeri! Maksimum 10m olarak ayarlandı.");
    }
    
    m_formationDistance = newDistance;
    setStatusMessage("Formasyon mesafesi " + QString::number(m_formationDistance) + "m olarak ayarlandı");
    
    // Eğer aktif bir formasyon varsa, mevcut formasyonu yeni mesafe değeriyle güncelle
    if (m_currentFormation != FormationType::NONE) {
        // Geçici olarak formation verifier'ı durdur
        bool wasVerifierRunning = false;
        if (m_formationVerifier && m_formationVerifier->isRunning()) {
            m_formationVerifier->stop();
            wasVerifierRunning = true;
        }
        
        // Mevcut formasyonu yeniden oluştur
        FormationType currentType = m_currentFormation;
        handle_formation_change(currentType);
        
        // Formation verifier'ı tekrar başlat
        if (wasVerifierRunning && m_formationVerifier) {
            m_formationVerifier->start();
        }
    }
    
    emit commandExecuted(true, "Formasyon mesafesi güncellendi");
    return true;
}

bool DroneControl::landAllDrones()
{
    if (!m_swarm || m_swarm->size() == 0) {
        setStatusMessage("Sürüde drone bulunamadı");
        emit commandExecuted(false, "Sürüde drone bulunamadı");
        return false;
    }
    
    auto drones = m_swarm->getDrones();
    setStatusMessage("Tüm dronelar iniş yapıyor...");
    
    std::vector<std::future<void>> futures;
    for (auto& drone : drones) {
        if (drone && drone->action) {
            futures.push_back(std::async(std::launch::async, [drone]() {
                auto result = drone->action->land();
                if (result != Action::Result::Success) {
                    qDebug() << "İniş komutu başarısız!";
                }
            }));
        }
    }
    
    // Tüm droneların iniş komutunu tamamlamasını bekle
    for (auto& fut : futures) {
        fut.get();
    }
    
    m_currentFormation = FormationType::NONE;
    emit currentFormationChanged();
    setStatusMessage("Tüm dronelar iniş yaptı");
    emit commandExecuted(true, "Tüm dronelar için iniş komutu gönderildi");
    
    return true;
}

void DroneControl::checkFormationStatus()
{
    // Durum kontrolü burada yapılabilir
    // Şu an için basit bir durum güncellemesi
    if (m_swarm && m_swarm->size() > 0) {
        if (m_currentFormation != FormationType::NONE) {
            // setStatusMessage("Aktif formasyon: " + QString::number(static_cast<int>(m_currentFormation)));
        }
    }
}

void DroneControl::updateFormationTargets()
{
    // Formasyon verifier için hedef pozisyonları güncelle
    if (!m_formationVerifier || !m_swarm || m_swarm->size() == 0) {
        return;
    }
    
    // Lider dronu belirle
    auto drones = m_swarm->getDrones();
    int leaderId = m_swarm->getLeaderId();
    if (leaderId < 0 || leaderId >= drones.size()) {
        leaderId = 0;
    }
    
    // Referans pozisyonunu lider drondan al
    auto ref_telemetry = m_swarm->getTelemetry(leaderId + 1);
    if (!ref_telemetry) {
        return;
    }
    
    double ref_lat = ref_telemetry->m_position.latitude_deg;
    double ref_lon = ref_telemetry->m_position.longitude_deg;
    float ref_alt = ref_telemetry->m_position.absolute_altitude_m;
    
    // Mesafe değerini derece cinsine çevir
    const double meter_to_degree = 1.0 / 111000.0;
    
    // Düz çizgi formasyonu için offset değeri
    const double lat_offset = m_formationDistance * meter_to_degree;
    
    // V formasyonları için açı değerini radyana çevir
    const double angle_rad = v_formation_angle * M_PI / 180.0;
    
    // V formasyonları için offset değerleri
    const double v_lat_offset = m_formationDistance * meter_to_degree * cos(angle_rad / 2);
    const double v_lon_offset = m_formationDistance * meter_to_degree * sin(angle_rad / 2);
    
    std::map<int, std::tuple<double, double, float>> targetPositions;
    
    // Mevcut formasyon tipine göre hedef pozisyonları hesapla
    switch (m_currentFormation) {
        case FormationType::STRAIGHT_LINE: {
            for (int i = 0; i < drones.size(); ++i) {
                if (!drones[i]->system) continue;
                
                int droneId = static_cast<int>(drones[i]->system->get_system_id());
                double target_lat = ref_lat + (i * lat_offset); // Latitude boyunca düzenle
                targetPositions[droneId] = std::make_tuple(target_lat, ref_lon, ref_alt);
            }
            break;
        }
        
        case FormationType::REVERSE_V: {
            for (int i = 0; i < drones.size(); ++i) {
                if (!drones[i]->system) continue;
                
                int droneId = static_cast<int>(drones[i]->system->get_system_id());
                double target_lat, target_lon;
                float target_alt;
                
                calculate_reverse_v_position(
                    i, ref_lat, ref_lon, ref_alt,
                    v_lat_offset, v_lon_offset,
                    target_lat, target_lon, target_alt
                );
                
                targetPositions[droneId] = std::make_tuple(target_lat, target_lon, target_alt);
            }
            break;
        }
        
        case FormationType::NORMAL_V: {
            for (int i = 0; i < drones.size(); ++i) {
                if (!drones[i]->system) continue;
                
                int droneId = static_cast<int>(drones[i]->system->get_system_id());
                double target_lat, target_lon;
                float target_alt;
                
                calculate_v_position(
                    i, ref_lat, ref_lon, ref_alt,
                    v_lat_offset, v_lon_offset,
                    target_lat, target_lon, target_alt
                );
                
                targetPositions[droneId] = std::make_tuple(target_lat, target_lon, target_alt);
            }
            break;
        }
        
        default:
            // No formation or unhandled formation type
            return;
    }
    
    // Formation verifier'a hedef pozisyonları ayarla
    if (m_formationVerifier) {
        m_formationVerifier->setTargetPositions(targetPositions);
    }
}

void DroneControl::setStatusMessage(const QString& message)
{
    m_statusMessage = getTimeString() + message;
    emit statusMessageChanged();
}

QString DroneControl::getTimeString() const
{
    QDateTime now = QDateTime::currentDateTime();
    return QString("[%1] ").arg(now.toString("hh:mm:ss"));
}

bool DroneControl::generateTargetPositions(FormationType targetFormation, 
                                         std::map<int, std::tuple<double, double, float>>& targetPositions)
{
    auto drones = m_swarm->getDrones();
    if (drones.isEmpty()) {
        return false;
    }
    
    // Lider dronu belirle, yoksa ilk dronu kullan
    int leaderId = m_swarm->getLeaderId();
    if (leaderId < 0 || leaderId >= drones.size()) {
        leaderId = 0;
    }
    
    // Referans pozisyonunu lider drondan al
    auto ref_telemetry = m_swarm->getTelemetry(leaderId + 1);
    if (!ref_telemetry) {
        return false;
    }
    
    double ref_lat = ref_telemetry->m_position.latitude_deg;
    double ref_lon = ref_telemetry->m_position.longitude_deg;
    float ref_alt = ref_telemetry->m_position.absolute_altitude_m;
    
    // Mesafe değerini derece cinsine çevir (yaklaşık olarak)
    // 1 derece yaklaşık 111 km (111000 m) olduğundan, metre cinsinden mesafeyi dereceye çevirmek için:
    const double meter_to_degree = 1.0 / 111000.0;
    
    // Düz çizgi formasyonu için offset değeri
    const double lat_offset = m_formationDistance * meter_to_degree;
    
    // V formasyonları için açı değerini radyana çevir
    const double angle_rad = v_formation_angle * M_PI / 180.0;
    
    // V formasyonları için offset değerleri
    const double v_lat_offset = m_formationDistance * meter_to_degree * cos(angle_rad / 2);
    const double v_lon_offset = m_formationDistance * meter_to_degree * sin(angle_rad / 2);
    
    // Her drone için hedef pozisyonu hesapla
    for (int i = 0; i < drones.size(); i++) {
        if (!drones[i] || !drones[i]->system) continue;
        
        int droneId = static_cast<int>(drones[i]->system->get_system_id());
        double target_lat = ref_lat;
        double target_lon = ref_lon;
        float target_alt = m_formationAltitude + ref_telemetry->m_homePosition.absolute_altitude_m;
        
        // Hedef pozisyonu formasyon tipine göre hesapla
        switch (targetFormation) {
            case FormationType::STRAIGHT_LINE:
                target_lat = ref_lat + (i * lat_offset);
                break;
                
            case FormationType::REVERSE_V:
                calculate_reverse_v_position(
                    i, ref_lat, ref_lon, target_alt,
                    v_lat_offset, v_lon_offset,
                    target_lat, target_lon, target_alt
                );
                break;
                
            case FormationType::NORMAL_V:
                calculate_v_position(
                    i, ref_lat, ref_lon, target_alt,
                    v_lat_offset, v_lon_offset,
                    target_lat, target_lon, target_alt
                );
                break;
                
            default:
                continue;
        }
        
        targetPositions[droneId] = std::make_tuple(target_lat, target_lon, target_alt);
    }
    
    return true;
}

bool DroneControl::addMissionToQueue(int formationType, int durationSeconds)
{
    if (durationSeconds < 0) {
        durationSeconds = 0;
    }
    
    FormationType type = static_cast<FormationType>(formationType);
    m_missionQueue.push_back(FormationMission(type, durationSeconds));
    
    emit missionQueueChanged();
    emit logMessage(QString("Görev kuyruğuna eklendi: Formasyon %1, Süre %2 sn")
                   .arg(formationType).arg(durationSeconds), "info");
    
    return true;
}

bool DroneControl::clearMissionQueue()
{
    if (m_isMissionRunning) {
        emit logMessage("Görev çalışırken kuyruk temizlenemez!", "warning");
        return false;
    }
    
    m_missionQueue.clear();
    emit missionQueueChanged();
    emit logMessage("Görev kuyruğu temizlendi", "info");
    
    return true;
}

bool DroneControl::executeMissionQueue()
{
    if (m_missionQueue.empty()) {
        emit logMessage("Görev kuyruğu boş!", "error");
        return false;
    }
    
    if (m_isMissionRunning) {
        emit logMessage("Bir görev zaten çalışıyor!", "warning");
        return false;
    }
    
    // Reset mission state
    m_currentMissionIndex = -1;
    m_isMissionRunning = true;
    
    // Start executing the first mission
    QMetaObject::invokeMethod(this, [this]() {
        executeMissionAtIndex(0);
    }, Qt::QueuedConnection);
    
    emit logMessage(QString("Görev dizisi başlatılıyor. Toplam görev: %1").arg(m_missionQueue.size()), "info");
    return true;
}

void DroneControl::executeMissionAtIndex(int index)
{
    if (index >= m_missionQueue.size() || !m_isMissionRunning) {
        // All missions completed or manually stopped
        m_isMissionRunning = false;
        m_currentMissionIndex = -1;
        
        emit allMissionsCompleted();
        emit logMessage("Tüm görevler tamamlandı", "info");
        return;
    }
    
    m_currentMissionIndex = index;
    auto& mission = m_missionQueue[index];
    
    emit missionStarted(index, m_missionQueue.size(), 
                       static_cast<int>(mission.formationType), mission.durationSeconds);
                       
    emit logMessage(QString("Görev %1/%2 başlatılıyor: Formasyon %3, Süre %4 sn")
                   .arg(index + 1)
                   .arg(m_missionQueue.size())
                   .arg(static_cast<int>(mission.formationType))
                   .arg(mission.durationSeconds), "info");
    
    // Execute the formation without automatic landing, regardless of duration
    handle_formation_change(mission.formationType, 0);
    
    // Schedule the next mission after this one's duration (or immediately if 0)
    int nextDelay = mission.durationSeconds > 0 ? mission.durationSeconds * 1000 : 5000;
    
    QTimer::singleShot(nextDelay, this, [this, index]() {
        emit missionCompleted(index, m_missionQueue.size());
        executeMissionAtIndex(index + 1);
    });
}