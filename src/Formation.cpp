#include <Formation.h>

#include <future>
#include <chrono>
#include <cmath>
#include <QDebug>
#include <thread>

using namespace std::chrono_literals;
using namespace mavsdk;

// Basit formasyon fonksiyonu
void goto_formation(std::shared_ptr<Drone> drone, double target_lat, double target_lon, float target_alt) {
    if (!drone || !drone->action) {
        qDebug() << "Drone veya action nesnesi bulunamadı!";
        return;
    }

    Action::Result result = drone->action->goto_location(
        target_lat,
        target_lon,
        target_alt,
        0.0f
    );

    if (result != Action::Result::Success) {
        qDebug() << "Konum ayarlama başarısız!";
        return;
    }

    // Formasyon geçişleri için bekleme süresini kısalttık
    std::this_thread::sleep_for(3s);
}

// Reverse V formasyonu için pozisyon hesaplama
void calculate_reverse_v_position(size_t index, double ref_lat, double ref_lon, float ref_alt,
                                double lat_offset, double lon_offset,
                                double& target_lat, double& target_lon, float& target_alt) {
    if (index == 0) {
        // Referans drone Reverse V'nin ucunda (^)
        target_lat = ref_lat;
        target_lon = ref_lon;
        target_alt = ref_alt;
        return;
    }

    // Reverse V formasyonu için, droneları referans drone ile uçta bir V oluşturacak şekilde konumlandırıyoruz
    // Tek indisler sol kola, çift indisler sağ kola gider
    int layer = (index + 1) / 2;  // Referans drone'dan ne kadar uzakta
    bool is_left_branch = (index % 2 == 1);  // Tek indisler sola gider
    
    // Reverse V için, dronelar geriye doğru (azalan enlem) ve dışarı doğru hareket eder
    target_lat = ref_lat - (layer * lat_offset);  // Referanstan geriye doğru
    
    // Sol kol batıya (azalan boylam), sağ kol doğuya (artan boylam) hareket eder
    target_lon = ref_lon + (is_left_branch ? -lon_offset * layer : lon_offset * layer);
    
    // Tüm dronelar aynı yükseklikte
    target_alt = ref_alt;
}

// Normal V formasyonu için pozisyon hesaplama
void calculate_v_position(size_t index, double ref_lat, double ref_lon, float ref_alt,
                         double lat_offset, double lon_offset,
                         double& target_lat, double& target_lon, float& target_alt) {
    if (index == 0) {
        // Referans drone normal V'nin ucunda
        target_lat = ref_lat;
        target_lon = ref_lon;
        target_alt = ref_alt;
        return;
    }

    // Normal V formasyonu için, droneları referans drone ile altta bir V oluşturacak şekilde konumlandırıyoruz
    // Tek indisler sol kola, çift indisler sağ kola gider
    int layer = (index + 1) / 2;  // Referans drone'dan ne kadar uzakta
    bool is_left_branch = (index % 2 == 1);  // Tek indisler sola gider
    
    // V için, dronelar ileri doğru (artan enlem) ve dışarı doğru hareket eder
    target_lat = ref_lat + (layer * lat_offset);  // Referanstan ileri doğru
    
    // Sol kol batıya (azalan boylam), sağ kol doğuya (artan boylam) hareket eder
    target_lon = ref_lon + (is_left_branch ? -lon_offset * layer : lon_offset * layer);
    
    // Tüm dronelar aynı yükseklikte
    target_alt = ref_alt;
}

// İniş için düz çizgi formasyonu (dikey bazlı) hesaplama
void land_straight_line_formation(size_t index, double ref_lat, double ref_lon, double lat_offset,
                                 double& target_lat, double& target_lon) {
    target_lon = ref_lon;  // Aynı boylamı koru
    
    if (index == 0) {
        target_lat = ref_lat;
        return;
    }
    
    // Düz çizgi formasyonunda dikey pozisyonu hesapla (lat_offset kullanarak)
    target_lat = ref_lat + (index * lat_offset);
} 