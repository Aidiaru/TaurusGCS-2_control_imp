#include <GeoUtils.h>
#include <cmath>

// Yerküre yarıçapı ve derece-radyan dönüşüm sabitleri
namespace {
    constexpr double EARTH_RADIUS_M = 6371000.0;
    constexpr double DEG_TO_RAD = M_PI / 180.0;
    constexpr double RAD_TO_DEG = 180.0 / M_PI;
}

namespace GeoUtils {

double calculateDistance3D(double lat1, double lon1, float alt1, 
                          double lat2, double lon2, float alt2) {
    // Dereceleri radyana çevirme
    double lat1_rad = lat1 * DEG_TO_RAD;
    double lon1_rad = lon1 * DEG_TO_RAD;
    double lat2_rad = lat2 * DEG_TO_RAD;
    double lon2_rad = lon2 * DEG_TO_RAD;

    // Haversine formülü ile 2D coğrafi mesafeyi hesaplama
    double dlat = lat2_rad - lat1_rad;
    double dlon = lon2_rad - lon1_rad;
    double a = std::sin(dlat/2) * std::sin(dlat/2) + 
               std::cos(lat1_rad) * std::cos(lat2_rad) * 
               std::sin(dlon/2) * std::sin(dlon/2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
    double horizontal_distance = EARTH_RADIUS_M * c;
    
    // Yükseklik farkını da ekleyerek 3D mesafeyi hesaplama
    double vertical_distance = alt2 - alt1;
    double distance_3d = std::sqrt(horizontal_distance * horizontal_distance + 
                                  vertical_distance * vertical_distance);
    
    return distance_3d;
}

void geoToCartesian(double lat, double lon, float alt, 
                   double& x, double& y, double& z) {
    // Coğrafi koordinatları kartezyen (ECEF) koordinatlarına dönüştürme
    double lat_rad = lat * DEG_TO_RAD;
    double lon_rad = lon * DEG_TO_RAD;
    
    // ECEF koordinatlarını hesaplama
    double r = EARTH_RADIUS_M + alt;
    x = r * std::cos(lat_rad) * std::cos(lon_rad);
    y = r * std::cos(lat_rad) * std::sin(lon_rad);
    z = r * std::sin(lat_rad);
}

void cartesianToGeo(double x, double y, double z, 
                   double& lat, double& lon, float& alt) {
    // Kartezyen (ECEF) koordinatlarını coğrafi koordinatlara dönüştürme
    double p = std::sqrt(x*x + y*y);
    lon = std::atan2(y, x) * RAD_TO_DEG;
    lat = std::atan2(z, p) * RAD_TO_DEG;
    
    // Yükseklik hesaplama (yaklaşık)
    alt = static_cast<float>(std::sqrt(x*x + y*y + z*z) - EARTH_RADIUS_M);
}

} // namespace GeoUtils 