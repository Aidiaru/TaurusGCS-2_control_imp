#include <FormationManager.h>
#include <Formation.h>
#include <GeoUtils.h>

#include <QDebug>

#include <future>
#include <thread>

using namespace std::chrono_literals;

FormationManager::FormationManager(Swarm* swarm, QObject *parent)
    : QObject(parent), m_swarm(swarm), m_currentFormation(FormationType::NONE)
{
}

FormationManager::~FormationManager()
{
}

bool FormationManager::setFormation(int formationType)
{
    FormationType targetFormation = static_cast<FormationType>(formationType);
    return executeFormation(targetFormation);
}

bool FormationManager::executeFormation(FormationType type)
{
    if (!m_swarm || m_swarm->size() == 0) {
        emit formationCommandExecuted(false, "Sürüde drone bulunamadı");
        return false;
    }
    
    auto drones = m_swarm->getDrones();
    
    // Lider dronu belirle, yoksa ilk dronu kullan
    int leaderId = m_swarm->getLeaderId();
    if (leaderId < 0 || leaderId >= drones.size()) {
        leaderId = 0;
    }
    
    // Referans pozisyonunu lider drondan al
    auto ref_telemetry = m_swarm->getTelemetry(leaderId + 1);
    if (!ref_telemetry) {
        emit formationCommandExecuted(false, "Lider drone telemetrisi alınamadı");
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
    
    std::vector<std::future<void>> futures;
    
    // Her drone için hedef pozisyonu hesapla ve hareket komutunu gönder
    for (int i = 0; i < drones.size(); i++) {
        if (!drones[i] || !drones[i]->action) continue;
        
        double target_lat = ref_lat;
        double target_lon = ref_lon;
        float target_alt = m_formationAltitude;
        
        // Hedef pozisyonu formasyon tipine göre hesapla
        switch (type) {
            case FormationType::STRAIGHT_LINE:
                target_lat = ref_lat + (i * lat_offset);
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
                // Tanımlanmamış formasyon tipi
                continue;
        }
        
        // drone hareketi için async kullan
        futures.push_back(std::async(std::launch::async,
            [drone=drones[i], target_lat, target_lon, target_alt]() {
                drone->action->goto_location(target_lat, target_lon, target_alt, 0.0f);
            }
        ));
        
        // Dronelar arasına kısa bir gecikme ekle
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }
    
    // Tüm droneların hareketinin tamamlanmasını bekle
    for (auto& fut : futures) {
        fut.get();
    }
    
    // Formasyon tipini güncelle
    if (m_currentFormation != type) {
        m_currentFormation = type;
        emit currentFormationChanged();
    }
    
    emit formationCommandExecuted(true, "Formasyon başarıyla uygulandı");
    return true;
}

bool FormationManager::changeAltitude(float newAltitude)
{
    if (!m_swarm || m_swarm->size() == 0) {
        emit formationCommandExecuted(false, "Sürüde drone bulunamadı");
        return false;
    }
    
    if (newAltitude < 2.0f) {
        newAltitude = 2.0f; // Minimum güvenli yükseklik
    }
    else if (newAltitude > 100.0f) {
        newAltitude = 100.0f; // Maksimum izin verilen yükseklik
    }
    
    m_formationAltitude = newAltitude;
    qDebug() << "Yeni irtifa değeri:" << m_formationAltitude << "metre";
    
    // Eğer aktif bir formasyon varsa, mevcut formasyonu yeni yükseklikle güncelle
    if (m_currentFormation != FormationType::NONE) {
        return executeFormation(m_currentFormation);
    }
    
    return true;
}

bool FormationManager::changeFormationDistance(double newDistance)
{
    if (newDistance <= 0.5) {
        newDistance = 0.5; // Minimum güvenli mesafe
        qDebug() << "Uyarı: Çok küçük mesafe değeri! Minimum 0.5 metre olarak ayarlanıyor.";
    } 
    else if (newDistance > 10.0) {
        newDistance = 10.0; // Maksimum izin verilen mesafe
        qDebug() << "Uyarı: Çok büyük mesafe değeri! Maksimum 10 metre olarak ayarlanıyor.";
    }
    
    m_formationDistance = newDistance;
    qDebug() << "Formasyon mesafesi" << m_formationDistance << "metre olarak ayarlandı.";
    
    // Eğer aktif bir formasyon varsa, mevcut formasyonu yeni mesafe değeriyle güncelle
    if (m_currentFormation != FormationType::NONE) {
        return executeFormation(m_currentFormation);
    }
    
    return true;
}

bool FormationManager::landAll()
{
    if (!m_swarm || m_swarm->size() == 0) {
        emit formationCommandExecuted(false, "Sürüde drone bulunamadı");
        return false;
    }
    
    auto drones = m_swarm->getDrones();
    
    // Eğer düz çizgi formasyonundaysak, droneların düzgün bir şekilde inmesi için
    // önceden düz çizgi formasyonuna geçiş yapabilirsiniz
    if (m_currentFormation != FormationType::STRAIGHT_LINE) {
        executeFormation(FormationType::STRAIGHT_LINE);
        std::this_thread::sleep_for(3s); // Formasyonun tamamlanmasını bekle
    }
    
    std::vector<std::future<void>> futures;
    for (auto& drone : drones) {
        if (drone && drone->action) {
            futures.push_back(std::async(std::launch::async, [&drone]() {
                drone->action->land();
            }));
        }
    }
    
    // Tüm droneların iniş komutunu tamamlamasını bekle
    for (auto& fut : futures) {
        fut.get();
    }
    
    m_currentFormation = FormationType::NONE;
    emit currentFormationChanged();
    emit formationCommandExecuted(true, "Tüm dronelar iniş komutunu aldı");
    
    return true;
} 